---
title: Chapter 5 - Nodes, Topics, Services, and Actions
description: Deep dive into ROS 2's core communication abstractions, practical patterns, design decisions, and failure modes for distributed robotics systems.
difficulty: Intermediate
category: Part 2 - ROS 2
keywords: nodes, topics, services, actions, communication patterns, design, callbacks, subscriptions
---

# Chapter 5: Nodes, Topics, Services, and Actions

## 1. Chapter Overview

Chapter 4 introduced ROS 2's architecture and communication patterns at a conceptual level. This chapter shifts focus to *design decisions and practical patterns*—the choices engineers face when building real systems.

When should you use a topic vs. a service? How do you design node boundaries? What happens when a subscriber is slower than the publisher's data rate? How do you coordinate actions across multiple nodes?

This chapter answers these questions through detailed analysis of each communication pattern, real-world design tradeoffs, and failure scenarios. By the end, you'll be able to architect complex robotic systems by choosing the right pattern for each data flow.

---

## 2. Learning Objectives

After completing this chapter, you will understand:

- The **node lifecycle** and how nodes register/discover each other in ROS 2
- **Design principles** for decomposing robotic systems into nodes
- **Topic subscription patterns**: async callbacks, polling, synchronization across multiple topics
- **Buffering and flow control**: message queues, depth settings, handling slow subscribers
- **Service design**: request/reply semantics, error handling, timeout behavior
- **Action design**: goal states, feedback, cancellation, and completion
- **Synchronization challenges**: how to combine data from multiple topics with different frequencies
- **Failure detection and recovery**: watchdogs, heartbeats, and health monitoring
- **Scaling to multi-robot systems**: topics as shared namespaces, parameter hierarchies
- **Trade-off analysis**: latency vs. throughput, reliability vs. bandwidth, complexity vs. robustness

---

## 3. Core Concepts

### 3.1 Node Lifecycle and Discovery

A ROS 2 node goes through the following stages:

```
1. UNCONFIGURED (created, not yet initialized)
    ↓
2. CONFIGURING (declaring parameters, initializing subscriptions)
    ↓
3. INACTIVE (configured, not yet executing)
    ↓
4. ACTIVATING (preparing for main loop)
    ↓
5. ACTIVE (main spin loop; processing messages)
    ↓
6. DEACTIVATING (preparing shutdown)
    ↓
7. INACTIVE (execution stopped)
    ↓
8. UNCONFIGURED (cleaned up)
```

In practice, most nodes go: unconfigured → configuring → inactive → activating → active → deactivating → unconfigured. Errors can transition to ERROR state.

**Discovery:** When a node starts, it advertises itself on the network. Other nodes detect it via DDS multicast. Within milliseconds, all nodes on the network know about each other—no central master required. This is a **key difference from ROS 1**, which required a master server.

---

### 3.2 Node Design Principles

Decomposing a system into nodes is an *art*. Guidelines:

#### Rule 1: One Concern Per Node
A node should have a single responsibility. Examples:

- **Camera driver node:** Reads from USB camera; publishes frames to `/camera/image`
- **Object detector node:** Subscribes to `/camera/image`; publishes detections to `/detections`
- **Gripper controller node:** Subscribes to `/gripper/command`; publishes gripper state to `/gripper/state`

This enables **reusability** (swap camera driver without changing detector) and **debugging** (test each node independently).

#### Rule 2: Minimize Latency-Critical Dependencies
If node A depends on low-latency data from node B, prefer pub/sub (asynchronous) over services (blocking). Services introduce extra latency from request serialization, network transit, processing, response transit.

**Example (Wrong):**
```
Control loop (100 Hz)
    └─> Service call to pose estimator (20 Hz) for latest pose
    └─> Service call blocks; control can't run
    └─> Effective control frequency: 20 Hz (bottleneck)
```

**Example (Right):**
```
Pose estimator (20 Hz)
    └─> Publishes /pose topic
Control loop (100 Hz)
    ├─> Subscribes to /pose
    ├─> Uses last-received pose; never blocks
    └─> Runs at full 100 Hz
```

#### Rule 3: Use Services for Rare, Configuration-Like Interactions
Services are appropriate for:
- Setting parameters that don't change frequently
- One-off queries (e.g., "what is the current calibration?")
- Commanding discrete state changes (e.g., "enable servo")

**Example (Good):**
```
Application node → Service call: /camera/set_exposure(value=100)
Camera driver responds: { success: true }
```

#### Rule 4: Isolate Hard Real-Time Critical Operations
If a node handles hard real-time control (e.g., motor control at 1 kHz), isolate it from non-real-time operations (e.g., learning, large computations). Running inference (10-100 ms latency) in the same thread as real-time control (1 ms deadlines) will cause deadline misses.

**Solution:** Separate nodes:
- Real-time control node: subscribes to low-level commands, publishes to motor driver
- Perception/learning node: subscribes to sensor data, publishes higher-level goals

Communication between them uses pub/sub (asynchronous) with loose deadlines.

---

### 3.3 Topics and Subscriptions in Detail

#### Data Flow: Publisher to Subscriber

```
Publisher Node                   DDS Layer                  Subscriber Node
─────────────                   ─────────                  ──────────────

self.pub.publish(msg) ─────────→ [Serialize]                    
                                    ↓
                                 [Multicast/Unicast on network]
                                    ↓
                            [Deserialize into ROS msg type]
                                    ↓
            ← ─ ─ ─ ─ ─ ─ ─ ─ ─ Call callback(msg) ← ← ← ← 
                                    ↑
                          subscriber_callback(msg) {}
                          (executes in node's executor)
```

**Latency sources:**
1. **Serialization:** ~0.1-0.5 ms (convert Python object to bytes)
2. **Network transit:** 1-10 ms (depends on network, packet size, multicast overhead)
3. **Deserialization:** ~0.1-0.5 ms
4. **Callback queuing:** depends on node's executor; typically <1 ms
5. **Callback execution:** depends on callback code; can be 1 ms to seconds

Total: 2-20 ms typical for local network. Can stretch to 100+ ms over congested WiFi or WAN.

#### Buffering: Depth and History Policies

When a subscriber receives messages, they are queued. The **depth** parameter controls queue size.

```
Publisher publishes at 1000 Hz
Subscriber callback is slow (processes each message in 50 ms)

Queue depth = 1:
  T=0 ms: Msg 1 arrives; callback starts
  T=0 ms: Msg 2 arrives; DROPPED (queue full)
  T=0 ms: Msg 3 arrives; DROPPED
  ...
  T=50 ms: Callback finishes; processes Msg 1
  T=50 ms: Msg 51 is waiting; callback processes it
  Result: Many messages dropped; subscriber lags behind

Queue depth = 100:
  T=0 ms: Msgs 1-100 queued
  T=50 ms: Callback processes Msg 1; Msg 101 queued
  T=100 ms: Callback processes Msg 2; Msg 151 queued
  ...
  T=5000 ms: Callback finally processes Msg 1; queue still has 50 msgs
  Result: Constant lag; subscriber always behind
```

**Tuning depth:**
- **Streaming data (lidar, camera):** Small depth (1-5). You want fresh data, not stale queued data.
- **Configuration/state (goals, setpoints):** Larger depth (10-20). Ensure nothing gets dropped.
- **Critical safety data (emergency stop):** Unlimited depth + RELIABLE QoS. Never drop.

#### Synchronizing Multiple Topics

A common pattern: subscribe to sensor A and sensor B; process them together when both arrive.

**Naive approach (Wrong):**
```python
class MyNode(Node):
    def __init__(self):
        self.latest_camera = None
        self.latest_imu = None
    
    def camera_callback(self, msg):
        self.latest_camera = msg
        # Process immediately with latest IMU
        if self.latest_imu:
            self.process(self.latest_camera, self.latest_imu)
    
    def imu_callback(self, msg):
        self.latest_imu = msg
        # Process immediately with latest camera
        if self.latest_camera:
            self.process(self.latest_camera, self.latest_imu)
```

**Problem:** If camera arrives at T=0 and IMU at T=100ms, camera callback processes with IMU from T=-100ms (very stale). Data is misaligned temporally.

**Better approach (Message Filters):**

ROS 2 provides `message_filters` library for synchronization:

```python
from message_filters import Synchronizer, TimeSynchronizer
import message_filters

class MyNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        
        # Synchronize with time threshold (messages within 100 ms are considered simultaneous)
        sync = message_filters.TimeSynchronizer([self.camera_sub, self.imu_sub], queue_size=10)
        sync.registerCallback(self.on_sync)
    
    def on_sync(self, camera_msg, imu_msg):
        # Both messages have similar timestamps
        self.process(camera_msg, imu_msg)
```

This waits for matching timestamps before calling the callback, ensuring temporal alignment.

---

### 3.4 Services: Synchronous Request/Reply

A service defines a **request message** and **response message**.

**Example: Set camera exposure**

Define in `camera_interface.srv`:
```
# Request
int32 exposure_time_ms
---
# Response
bool success
string error_message
```

Server (camera driver):
```python
def set_exposure_callback(self, request, response):
    try:
        self.camera.set_exposure(request.exposure_time_ms)
        response.success = True
        response.error_message = ""
    except Exception as e:
        response.success = False
        response.error_message = str(e)
    return response

self.srv = self.create_service(SetExposure, '/camera/set_exposure', self.set_exposure_callback)
```

Client (application):
```python
client = self.create_client(SetExposure, '/camera/set_exposure')

# Wait for service to be available
while not client.wait_for_service(timeout_sec=5.0):
    self.get_logger().info('Service not available; waiting...')

request = SetExposure.Request()
request.exposure_time_ms = 50

# Blocking call
future = client.call_async(request)
rclpy.spin_until_future_complete(self, future)
response = future.result()

if response.success:
    self.get_logger().info('Exposure set successfully')
else:
    self.get_logger().error(f'Failed: {response.error_message}')
```

**Failure modes:**
1. **Service not available:** Client waits indefinitely (or times out). Solution: Check service availability before calling.
2. **Slow service:** Response takes several seconds. Client blocks. Solution: Use async call if you can't wait.
3. **Network partition:** Client and server on different networks. Service never found. Solution: Configure DDS to use unicast or adjust network discovery.

---

### 3.5 Actions: Goal-Oriented Long-Running Tasks

An action is more complex than a service. It consists of:

1. **Goal message:** What the server should do
2. **Feedback message:** Periodic updates while executing
3. **Result message:** Final outcome

**Example: Grasping action**

Define in `gripper_interface.action`:
```
# Goal
int32 object_id
float32 desired_grip_force
---
# Result
bool success
float32 final_grip_force
---
# Feedback
float32 progress          # 0 to 1
float32 current_grip_force
string status_message
```

Server (gripper action server):
```python
def execute_callback(self, goal_handle):
    request = goal_handle.request
    feedback = GraspObject.Feedback()
    
    for i in range(100):
        if goal_handle.is_cancel_requested():
            goal_handle.canceled()
            return GraspObject.Result()  # Cancelled
        
        # Simulate grasping
        feedback.progress = i / 100.0
        feedback.current_grip_force = i * 0.5  # Increase force
        feedback.status_message = f"Grasping... {feedback.progress*100:.0f}%"
        goal_handle.publish_feedback(feedback)
        
        time.sleep(0.01)  # Simulate work
    
    result = GraspObject.Result()
    result.success = True
    result.final_grip_force = 50.0
    goal_handle.succeed()
    return result
```

Client (manipulation planner):
```python
client = self.create_action_client(GraspObject, '/gripper/grasp')

# Wait for server
while not client.wait_for_server(timeout_sec=5.0):
    self.get_logger().info('Action server not available...')

goal = GraspObject.Goal()
goal.object_id = 42
goal.desired_grip_force = 30.0

# Non-blocking send goal
send_goal_future = client.send_goal_async(goal, feedback_callback=self.feedback_callback)

# In main loop, check if goal is done
send_goal_future.add_done_callback(self.goal_response_callback)

def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected')
        return
    
    # Goal accepted; wait for result (non-blocking)
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self.get_result_callback)

def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(f'Progress: {feedback.progress*100:.0f}%')

def get_result_callback(self, future):
    result = future.result()
    self.get_logger().info(f'Grasp completed: {result.result.success}')
```

**Key advantages over services:**
1. **Preemption:** Client can cancel goal mid-execution
2. **Feedback:** Periodic updates; client knows progress
3. **Asynchronous:** Client doesn't block; can do other work while goal executes
4. **Long operations:** Designed for multi-second tasks

---

### 3.6 Topic Namespace Design for Multi-Robot Systems

As systems scale to multiple robots, naming conventions become critical.

**Flat namespace (Bad):**
```
/camera/image
/lidar/scan
/motor/command
/pose
/detections
```

If you have 10 robots, how do you distinguish? Collision.

**Hierarchical namespace (Good):**
```
/robot1/camera/image
/robot1/lidar/scan
/robot1/motor/command
/robot1/pose
/robot2/camera/image
/robot2/lidar/scan
/robot2/motor/command
/robot2/pose
```

**Even better (Parameter-driven):**

ROS 2 uses **remapping** to reassign topics at launch:

```bash
# Launch robot1 with custom namespace
ros2 run my_package camera_driver_node --ros-args -r /camera/image:=/robot1/camera/image

# Or in launch file:
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='camera_driver_node',
            name='camera',
            namespace='robot1'  # Prefix all topics with /robot1
        ),
        Node(
            package='my_package',
            executable='detector_node',
            name='detector',
            namespace='robot1'
        ),
    ])
```

This way, you write nodes once; namespacing is handled at launch time. A single detector node works for all robots.

---

## 4. System Architecture: Multi-Node Coordination

Here's a complete example for a mobile manipulator system:

```
┌─────────────────────────────────────────────────┐
│        Sensor Drivers (Low-level nodes)         │
├─────────────────────────────────────────────────┤
│ camera_driver          lidar_driver             │
│ pub: /camera/image    pub: /lidar/scan         │
│ svc: /camera/config   svc: /lidar/set_params  │
└──────────┬──────────────────────────┬──────────┘
           │                          │
           ↓ (Image topic)            ↓ (PointCloud2 topic)
┌────────────────────────────────────────────────┐
│      Perception Nodes (Medium-level)           │
├────────────────────────────────────────────────┤
│ detector_node          slam_node               │
│ sub: /camera/image     sub: /lidar/scan        │
│ pub: /detections       pub: /pose              │
│ pub: /bounding_boxes   pub: /map               │
└──────────┬─────────────────────────┬───────────┘
           │ (/detections)           │ (/pose)
           │                         │
           ↓                         ↓
┌──────────────────────────────────────────────────┐
│     Planning Nodes (High-level decision)         │
├──────────────────────────────────────────────────┤
│    manipulation_planner                         │
│    sub: /detections, /pose                      │
│    action: /gripper/grasp (long-running)        │
│    action: /arm/move_to (goal-oriented)         │
│    pub: /trajectory (goal trajectory)           │
└──────────┬─────────────────────────────────────┘
           │ (/trajectory)
           │
           ↓
┌──────────────────────────────────────────────────┐
│     Control Nodes (Real-time execution)         │
├──────────────────────────────────────────────────┤
│    trajectory_controller (100 Hz)               │
│    sub: /trajectory                             │
│    service call: /gripper/set_position          │
│    pub: /joint_commands (motor commands)        │
└──────────┬─────────────────────────────────────┘
           │ (/joint_commands)
           │
           ↓
┌──────────────────────────────────────────────────┐
│     Hardware Layer (Motor drivers, etc.)        │
├──────────────────────────────────────────────────┤
│    motor_driver (actuate hardware)              │
└──────────────────────────────────────────────────┘
```

**Communication breakdown:**
- Sensor drivers → Perception (pub/sub) – asynchronous streaming
- Perception → Planning (pub/sub) – asynchronous state updates
- Planning → Control (action or pub/sub) – depends on timing criticality
- Control → Hardware (service or direct I/O) – usually direct to avoid latency

---

## 5. Practical Implementation Outline

### 5.1 Choosing Between Topic, Service, and Action

| Question | Answer | Pattern |
|----------|--------|---------|
| **Is this data continuous/streaming?** | Yes | Topic (pub/sub) |
| **Is this a one-off request?** | Yes | Service |
| **Is this long-running (>1 second)?** | Yes | Action |
| **Does client need to wait for response?** | Yes | Service |
| **Does client need progress updates?** | Yes | Action |
| **Can the request be cancelled mid-execution?** | Yes | Action |
| **High throughput (>100 Hz)?** | Yes | Topic |
| **Is low latency critical (<10 ms)?** | Yes | Topic (avoid service) |

### 5.2 Multi-Node Debugging Workflow

When a system fails, use ROS 2 CLI tools to diagnose:

```bash
# List all nodes
ros2 node list

# Check what a specific node publishes/subscribes
ros2 node info /detector_node

# Monitor a topic
ros2 topic echo /detections

# Check topic publisher/subscriber count
ros2 topic info /detections

# Call a service
ros2 service call /camera/set_exposure SetExposure "{exposure_time_ms: 50}"

# Send action goal and monitor
ros2 action send_goal /gripper/grasp GraspObject "{object_id: 42}"

# Profile message frequency
ros2 topic hz /detections
```

### 5.3 Deployment: Single Machine vs. Distributed

**Single machine (development):**
- All nodes on one process
- Fast multicast-based discovery
- Simple debugging

**Distributed (production):**
- Nodes on different machines (different robots, different computers)
- Requires network connectivity between machines
- May need to configure DDS for specific network interfaces

```bash
# On robot1 (192.168.1.10)
export ROS_DOMAIN_ID=1  # Isolate traffic
ros2 run my_package detector_node

# On robot2 (192.168.1.11)
export ROS_DOMAIN_ID=1  # Same domain; can see robot1
ros2 run my_package grasp_node

# Now grasp_node on robot2 can subscribe to /detections from robot1's detector
```

---

## 6. Role of AI Agents

### Integrating Learned Policies

A learned policy (e.g., vision model or RL policy) is just another ROS 2 node. Example: learned grasping policy.

```python
class GraspPolicyNode(Node):
    def __init__(self):
        super().__init__('grasp_policy')
        
        # Subscribe to perception outputs
        self.detections_sub = self.create_subscription(
            Detections,
            '/detections',
            self.detections_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1, reliability=BEST_EFFORT)
        )
        
        # Publish grasp goals to action server
        self.grasp_client = self.create_action_client(GraspObject, '/gripper/grasp')
        
        # Load learned policy (neural network)
        self.policy = torch.jit.load('grasp_policy.pt')
    
    def detections_callback(self, detections_msg):
        # Convert ROS message to tensor
        images = [self.msg_to_tensor(d.image) for d in detections_msg.detections]
        
        # Run inference (may take 10-100 ms)
        with torch.no_grad():
            grasp_proposals = self.policy(images)  # Output: grasp poses, confidence
        
        # Select best grasp
        best_grasp = grasp_proposals[0]
        
        # Send to gripper action server
        goal = GraspObject.Goal()
        goal.object_id = best_grasp.object_id
        goal.desired_grip_force = best_grasp.force
        
        self.grasp_client.send_goal_async(goal)
```

**Key considerations:**
1. **Callback shouldn't block:** If inference takes 100 ms, run in background thread
2. **Asynchronous integration:** Use action client (non-blocking) not service call (blocking)
3. **Handle failures gracefully:** What if inference fails? What if action server is down?

---

## 7. Common Mistakes & Pitfalls

### 7.1 Blocking on Service Calls in Main Loop

**Mistake:**
```python
def timer_callback(self):  # Called at 100 Hz
    response = self.service_client.call(request)  # BLOCKS
    # If service takes 100 ms, timer misses 9 callbacks
```

**Solution:** Use async service calls or offload to thread.

---

### 7.2 Ignoring Message Synchronization

**Mistake:** Subscribing to camera and IMU separately; processing with arbitrarily old data.

**Solution:** Use `message_filters.TimeSynchronizer` to align timestamps.

---

### 7.3 Unbounded Message Queue Growth

**Mistake:** Publisher sends 1000 messages/sec; subscriber processes 10 messages/sec. Queue grows indefinitely.

**Solution:** Tune queue depth. Accept that you'll drop old messages. Use QoS KEEP_LAST with small depth.

---

### 7.4 Service/Action Timeout Silent Failures

**Mistake:** Call action goal; don't check if it was accepted or completed.

**Solution:** Always check goal_handle and wait for result.

---

### 7.5 Namespace Collisions in Multi-Robot Systems

**Mistake:** All robots publish to `/camera/image`. Both cameras collide; subscribers get mixed data.

**Solution:** Use namespaces: `/robot1/camera/image`, `/robot2/camera/image`.

---

### 7.6 Wrong QoS Profile Mismatch

**Mistake:** Use default QoS; doesn't match publisher's QoS. No connection.

**Solution:** Explicitly set QoS on subscriber to match publisher.

---

## 8. Summary

### Key Takeaways

1. **Nodes are independent processes** communicating via ROS 2 middleware. Design with clear boundaries.

2. **Topics are for streaming asynchronous data**; use when you don't care about ordering or want fresh data (lidar, camera).

3. **Services are synchronous request/reply**; use for configuration, queries, state changes.

4. **Actions are goal-oriented asynchronous tasks**; use for long-running operations with feedback and cancellation.

5. **Synchronizing multiple topics requires message_filters**; don't assume data arrives in sync.

6. **Queue depth controls buffering**; small depth for fresh data (sensors), large depth for critical data (goals).

7. **Namespace hierarchies are essential for multi-robot systems**; design early.

8. **Real-time constraints require careful node design**: avoid blocking, isolate perception from control, use appropriate frequencies.

9. **AI agents integrate as specialized nodes** that subscribe to perception and publish goals/actions.

10. **Debugging requires ROS 2 CLI tools**: `ros2 topic echo`, `ros2 service call`, `ros2 action send_goal`.

---

### Self-Assessment Checklist

- ✓ Design a multi-node ROS 2 system for a given application (identify nodes, topics, services, actions)
- ✓ Explain pub/sub vs. service vs. action trade-offs
- ✓ Synchronize data from multiple topics using message_filters
- ✓ Configure QoS profiles for specific communication needs
- ✓ Debug failed pub/sub connections using ROS 2 CLI tools
- ✓ Implement a service server and client
- ✓ Implement an action server and client
- ✓ Design namespace hierarchies for multi-robot systems
- ✓ Identify and fix blocking operations in node callbacks
- ✓ Integrate learned policies as ROS 2 nodes

---

## 9. RAG-Seed Questions

### Foundational

**Q1: What is the difference between a topic and a service?**
A: Topics are asynchronous publish/subscribe for streaming data; publisher doesn't wait. Services are synchronous request/reply; client blocks. Use topics for continuous data (sensors); services for one-off requests.

**Q2: When should you use an action instead of a service?**
A: Actions are for long-running tasks (>1 second) requiring feedback and cancellation. Services are for quick request/reply. Actions provide progress updates; services don't. Grasping is an action; setting camera exposure is a service.

**Q3: What does message queue depth mean?**
A: Depth is the number of messages buffered before old ones are dropped. Depth=1 keeps only the latest message (fresh data); depth=100 keeps 100 messages (handles slow subscribers). Choose based on tolerance for old data.

**Q4: How do you handle messages from multiple topics arriving at different frequencies?**
A: Use `message_filters.TimeSynchronizer` to wait for matching timestamps. Don't process with arbitrarily old data from other topics.

**Q5: What is QoS, and why does it matter?**
A: QoS (Quality of Service) policies control reliability (BEST_EFFORT vs. RELIABLE), durability (VOLATILE vs. TRANSIENT_LOCAL), and deadlines. Critical messages (goals) need RELIABLE; sensor streams (lidar) tolerate BEST_EFFORT.

---

### Intermediate

**Q6: Design a ROS 2 system for a mobile manipulator with the following: camera driver, object detector, manipulation planner, trajectory controller, gripper driver. Specify what should be a node, what should be a topic, and what should be an action.**
A: Nodes: camera_driver, detector, planner, controller, gripper_driver. Topics: /camera/image (camera→detector), /detections (detector→planner), /pose (from SLAM), /trajectory (planner→controller), /gripper/state (gripper→controller). Actions: /gripper/grasp (goal-oriented). Services: /gripper/set_position (config). This mirrors the 5-layer architecture from PART 1.

**Q7: Your control loop runs at 100 Hz but depends on a perception system that runs at 20 Hz. How do you structure this?**
A: Decouple: perception publishes to /perception/output at 20 Hz. Control loop subscribes (with KEEP_LAST depth=1) and runs at 100 Hz independently using the latest perception result. Never block control waiting for perception.

**Q8: A subscriber is slower than the publisher's data rate. What happens, and how do you fix it?**
A: Messages queue up to queue depth. If depth is small, old messages drop. If depth is large, subscriber lags behind. Fix by: (1) Increase QoS depth for critical data, (2) Reduce queue depth for sensor streams (tolerate drops), (3) Speed up subscriber callback.

**Q9: How would you debug why a service call never completes?**
A: (1) Check if service exists: `ros2 service list`. (2) Check service type: `ros2 service info /service_name`. (3) Call manually: `ros2 service call /service_name ServiceType`. (4) Check server logs: did it receive the request? Is callback stuck? (5) Increase timeout on client. (6) Check network connectivity if service is remote.

**Q10: Design a namespace hierarchy for a fleet of 5 robots, each with camera, lidar, and arm. How do you launch them with minimal code duplication?**
A: Use namespaces: /robot1, /robot2, etc. Write one launch file with parameters: ```python Node(namespace='robot1'), Node(namespace='robot2'), ... ``` or loop: ```for i in range(5): nodes.append(Node(namespace=f'robot{i}'))```. All topic names get robot-specific prefixes automatically.

---

### Advanced

**Q11: Implement a ROS 2 node that subscribes to two image topics with different publish rates, synchronizes them by timestamp, and publishes detections. Handle the case where one camera is faster than the other.**
A: Use `message_filters.TimeSynchronizer([sub1, sub2], slop=0.05)` (50 ms time window). Filter will wait for messages within slop time of each other before calling callback. If one camera is 30 Hz and other is 20 Hz, synchronizer outputs at ~20 Hz (limited by slower camera). See Chapter 5, Section 3.3 for example code.

**Q12: You want to integrate a learned policy (runs inference in 50 ms) with a 100 Hz control loop. The policy should run in parallel without blocking control. Design the ROS 2 architecture.**
A: Two nodes: (1) Control loop node (100 Hz): subscribes to /policy_output (KEEP_LAST), publishes motor commands without blocking. (2) Policy node (separate thread or process, 20 Hz): subscribes to /sensor_data, runs inference (50 ms), publishes /policy_output. Use action client (non-blocking) for state-changing commands. Control loop never waits for policy; it uses latest available output.

**Q13: How would you implement a watchdog to detect and recover from a dead perception node?**
A: (1) Have perception node publish heartbeat to /perception/heartbeat at regular interval (e.g., 10 Hz). (2) Control node subscribes; if no heartbeat for >500 ms, trigger failover (use last-known state or fail safely). Alternative: use DDS liveliness policy with deadline; DDS notifies when publisher disappears.

**Q14: Your multi-robot system has 10 robots, each publishing to /camera/image. How do you prevent collisions and ensure each robot gets the right images?**
A: Use hierarchical namespaces: each robot launches its nodes with namespace='robotN'. Launch file applies namespace prefix automatically. So robot1's camera publishes to /robot1/camera/image; robot2 to /robot2/camera/image. Perception nodes also launched with namespace; they subscribe to /camera/image, which resolves to /robotN/camera/image for each robot.

**Q15: Compare the latency implications of these architectures for a time-critical control task: (A) Tight coupling (all in one node), (B) ROS 2 pub/sub with default QoS, (C) ROS 2 pub/sub with tuned QoS (BEST_EFFORT, small queue). When would you choose each?**
A: (A) Monolith: <1 ms latency but brittle (one failure breaks everything); use for hard real-time. (B) Default pub/sub: 5-20 ms latency (serialization, network, queuing); good for most robotics. (C) Tuned QoS: 2-10 ms latency (low overhead); use for time-critical but distributed systems. For mobile robots, (B) is typical. For low-latency humanoid control, use (A) or (C).

---

**Total Questions:** 15  
**Difficulty distribution:** 5 foundational, 5 intermediate, 5 advanced  

---

**Ready to proceed to Chapter 6?**
