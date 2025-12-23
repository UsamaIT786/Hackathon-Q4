---
title: Chapter 4 - ROS 2 Architecture and Middleware
description: Comprehensive overview of ROS 2 architecture, middleware, publish/subscribe patterns, services, actions, QoS, and design principles for distributed robotics systems.
difficulty: Intermediate
category: Part 2 - ROS 2
keywords: ["ROS 2", "middleware", "architecture", "publish/subscribe", "DDS", "QoS", "services", "actions", "distributed systems"]
---

# Chapter 4: ROS 2 Architecture and Middleware

## 1. Chapter Overview

ROS 2 is the de facto standard middleware for modern robotics systems, enabling distributed computation, standardized communication, and hardware abstraction. Unlike PART 1, which established *conceptual foundations*, this chapter focuses on the *technical infrastructure* that makes Physical AI systems practical at scale.

ROS 2's architecture is built on three core insights:

1. **Distribution is necessary.** Real robotic systems require multiple processes (sensor drivers, perception algorithms, planning, control) running in parallel on diverse hardware (edge devices, GPUs, embedded systems). Monolithic designs don't scale.

2. **Communication patterns matter.** Different problems require different communication semantics. Sometimes you need fast, asynchronous updates (sensor streaming); other times, you need reliable request/reply (configuration). ROS 2 standardizes these patterns.

3. **Middleware should be transparent.** Developers should focus on robotics problems, not communication protocol implementation. ROS 2 abstracts the underlying Data Distribution Service (DDS) middleware layer.

This chapter answers:
- **What is ROS 2?** (positioning and core components)
- **How does its architecture enable Physical AI?** (distributed perception, planning, control)
- **What communication patterns does it provide?** (topics, services, actions)
- **How do I ensure reliability and real-time performance?** (QoS, DDS)

---

## 2. Learning Objectives

After completing this chapter, you will understand:

- The **definition of ROS 2** and how it differs from ROS 1
- ROS 2's **layered architecture** (application, client library, middleware, DDS, OS)
- The **publisher/subscriber pattern** and when to use it vs. other communication models
- **Services** and **actions** as synchronous and asynchronous request/reply mechanisms
- **Quality of Service (QoS) policies** and how they affect reliability, latency, and bandwidth
- **Data Distribution Service (DDS)** as the underlying middleware layer
- How ROS 2 enables **distributed robotics systems** (multi-node, multi-machine coordination)
- **Design principles** for ROS 2 systems operating under real-time constraints
- **Common architectural patterns** (sensor drivers, perception stacks, planning nodes, control loops)
- **Scaling considerations** for multi-robot systems and large deployments

---

## 3. Core Concepts

### 3.1 What is ROS 2?

**ROS 2** (Robot Operating System 2) is a middleware framework and development ecosystem for building robotic systems. It provides:

1. **Communication layer:** A standardized way for independent processes (nodes) to exchange data.
2. **Hardware abstraction:** Standard interfaces for sensors (cameras, lidar, IMUs), actuators (motors), and compute platforms.
3. **Tooling:** Build systems (Colcon), package management, debugging utilities (CLI tools, visualization).
4. **Development language support:** C++, Python, Java, and others through multiple client libraries.
5. **Real-time capabilities:** QoS policies and scheduling mechanisms for deterministic execution.

ROS 2 is **not**:
- A single monolithic library (it's a collection of packages and standards)
- An operating system (it runs on top of Linux, Windows, macOS)
- A replacement for application logic (it handles communication, not decision-making)
- A guarantee of real-time behavior (this must be designed by engineers)

**Positioned in PART 1 context:**
- PART 1 established that Physical AI systems must integrate perception, reasoning, and actuation.
- ROS 2 is the *practical infrastructure* for this integration in production systems.
- ROS 2 alone doesn't implement intelligence; rather, it enables intelligent agents (neural networks, planners, controllers) to exchange information reliably.

---

### 3.2 Comparison: ROS 1 vs. ROS 2

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| **Middleware** | Custom XML-RPC + TCP/IP | DDS (standardized) |
| **Master process** | Required (single point of failure) | Decentralized (no master) |
| **Real-time support** | Limited; soft real-time only | Full support for hard real-time |
| **Multi-machine** | Works, but fragile; requires network reliability | Robust; built for distributed systems |
| **DDS security** | None | Built-in encryption, authentication |
| **Performance** | Adequate for most robots; high latency variance | Optimized for predictable latency |
| **Release cycle** | LTS: 5 years; aggressive deprecation | LTS: 3 years per release; gradual migration |
| **Current adoption** | Decreasing; legacy systems | Standard for new development (2023+) |

**For PART 1 concepts:** ROS 2's distributed architecture directly enables the perception-decision-action loop discussed in Chapter 1 and the multi-layer architecture from Chapter 2.

---

### 3.3 Layered Architecture of ROS 2

ROS 2 consists of five layers, each responsible for specific concerns:

```
┌─────────────────────────────────────────┐
│    Application Layer (Your Code)         │
│  (Nodes: perception, planning, control) │
└─────────────────────────────────────────┘
                    ↑↓
┌─────────────────────────────────────────┐
│   Client Library Layer (rclpy, rclcpp)  │
│  (Node, Publisher, Subscriber APIs)    │
└─────────────────────────────────────────┘
                    ↑↓
┌─────────────────────────────────────────┐
│  Middleware Interface Layer (rcl)        │
│  (Abstracts DDS, manages subscriptions) │
└─────────────────────────────────────────┘
                    ↑↓
┌─────────────────────────────────────────┐
│   DDS Layer (Fast DDS, Connext, etc.)   │
│  (Actual communication protocol)        │
└─────────────────────────────────────────┘
                    ↑↓
┌─────────────────────────────────────────┐
│  OS/Network Layer (Linux, UDP/TCP)      │
│  (Sockets, kernel buffers)              │
└─────────────────────────────────────────┘
```

Each layer hides complexity from the one above. Developers write application code without worrying about DDS internals; the client library (rclpy, rclcpp) handles that automatically.

---

### 3.4 Communication Patterns in ROS 2

ROS 2 provides four primary communication patterns, each optimized for different use cases:

#### 3.4.1 Publisher/Subscriber (Asynchronous, One-to-Many)

**Pattern:** A publisher sends messages to a named *topic*; any number of subscribers listen.

**Characteristics:**
- **Asynchronous:** Publisher doesn't wait for subscriber response; fire-and-forget
- **Decoupled:** Publishers and subscribers don't know about each other
- **One-to-many:** One publisher can reach multiple subscribers; one subscriber receives from multiple publishers
- **Latency:** Low and predictable (just network transit time)
- **Throughput:** High (can stream data at >100 Hz)

**Example (Perception Loop):**
```
Camera driver → /camera/image topic
              ↓
         Object detector node
              ↓
         /detection topic
              ↓
         Planning node (subscribes to detections)
```

**When to use:** Continuous sensor data (camera frames, lidar scans), state updates, status broadcasts.

**ROS 2 specifics:** Implemented via DDS publish/subscribe; QoS policies control reliability and history.

---

#### 3.4.2 Services (Synchronous, Request/Reply)

**Pattern:** A client sends a request to a service; the service processes and replies.

**Characteristics:**
- **Synchronous:** Client blocks waiting for response
- **One-to-one:** One client per request; one service server
- **Bidirectional:** Client sends data; server replies with result
- **Latency:** Higher than pub/sub (includes processing time)
- **Semantics:** Request-reply; guaranteed delivery

**Example (Configuration):**
```
Application node (client)
    ↓ (service call: SetIntegrationTime)
Camera driver (service server)
    ↓ (response: success=true, new_value=50ms)
Application node receives response
```

**When to use:** Configuration changes, one-off computations, queries (e.g., "get robot state").

**ROS 2 specifics:** Defined with `.srv` files; client library provides blocking and async call variants.

---

#### 3.4.3 Actions (Goal-Oriented, Asynchronous)

**Pattern:** A client sends a *goal* to an action server; the server provides periodic *feedback* and a final *result*.

**Characteristics:**
- **Asynchronous with feedback:** Client doesn't block; receives periodic updates
- **Preemptible:** Client can cancel goal mid-execution
- **State machine:** Server tracks goal state (accepted, executing, succeeded, failed, cancelled)
- **Latency:** Higher than pub/sub; includes execution time + feedback overhead
- **Use case:** Long-running tasks (e.g., navigation, grasping)

**Example (Grasping Action):**
```
Manipulation planner (client)
    ↓ (send goal: GraspObject(target_id=42))
Gripper action server (server)
    ↓ (feedback: progress=0.5, current_force=2.5N)
    ↓ (feedback: progress=0.8, current_force=3.2N)
    ↓ (result: success=true, final_force=3.0N)
Manipulation planner receives result
```

**When to use:** Navigation, manipulation, multi-step processes, anything requiring feedback and cancellation.

**ROS 2 specifics:** Defined with `.action` files; built on top of pub/sub and services.

---

#### 3.4.4 Parameters (Mutable Configuration)

**Pattern:** A node exposes configurable parameters that can be read, written, or monitored.

**Characteristics:**
- **Read/write at runtime:** No restart needed to change parameters
- **Type-aware:** Parameters have types (int, double, string, array)
- **Optional monitoring:** Nodes can monitor parameters and react to changes
- **Persistence:** Parameters can be saved and loaded from YAML files

**Example (Control Gains):**
```
Node loads parameter: /controller/kp = 1.0
    ↓ (launch.yaml: sets kp=2.0)
    ↓ (node detects change, updates PID controller)
Node now uses /controller/kp = 2.0 (no restart)
```

**When to use:** Tuning (gains, thresholds), feature flags, switching between behaviors.

**ROS 2 specifics:** Built into every node via `node.declare_parameter()` and `node.get_parameter()`.

---

### 3.5 Quality of Service (QoS)

**QoS policies** control the *reliability guarantees*, *latency characteristics*, and *durability* of communication. They are crucial for real-time robotics where failure modes matter.

#### QoS Dimensions:

| Policy | Options | Meaning | Example |
|--------|---------|---------|---------|
| **Reliability** | BEST_EFFORT, RELIABLE | Loss tolerance | Lidar scans: BEST_EFFORT (OK to drop); navigation goals: RELIABLE |
| **Durability** | TRANSIENT_LOCAL, VOLATILE | Latecomer gets history? | Robot state: TRANSIENT_LOCAL (new subscribers catch up); streaming: VOLATILE |
| **History** | KEEP_LAST, KEEP_ALL | Buffer size | KEEP_LAST(depth=10): keep 10 newest messages |
| **Deadline** | Liveliness timeout | How long to wait for message? | 100ms deadline: must get message within 100ms or treat as stale |
| **Liveliness** | AUTOMATIC, MANUAL_BY_TOPIC | Heartbeat strategy | Detect dead publishers; trigger failover |

#### Real-World QoS Tuning Example:

**Scenario:** Mobile robot's lidar driver publishes point clouds.

```python
# Lidar topics are high-volume, tolerant of dropped frames
lidar_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # OK to drop
    durability=QoSDurabilityPolicy.VOLATILE,       # No history
    history=HistoryPolicy.KEEP_LAST,               # Keep last N
    depth=5,                                        # Keep last 5
    deadline=Duration(seconds=0.1)                 # Must arrive within 100ms
)
self.lidar_sub = self.create_subscription(
    PointCloud2,
    'scan',
    self.lidar_callback,
    qos_profile=lidar_qos
)

# Navigation goal is critical; must not be lost
goal_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,     # No drops
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,# Latecomers see it
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
self.goal_sub = self.create_subscription(
    NavGoal,
    'goal',
    self.goal_callback,
    qos_profile=goal_qos
)
```

**QoS and real-time:** Tight deadlines (small values) reduce latency but require good network; loose deadlines tolerate congestion.

---

### 3.6 Data Distribution Service (DDS)

DDS is the **standardized middleware protocol** underlying ROS 2. Understanding DDS helps troubleshoot communication issues.

**Key DDS Concepts:**

1. **Decentralized discovery:** Nodes automatically discover each other on the network (via multicast); no central master needed.
2. **Topic-based communication:** Publishers and subscribers connect via topic names; DDS handles routing.
3. **Quality of Service:** DDS standardizes QoS policies across implementations.
4. **Multiple implementations:** Fast DDS, RTI Connext, OpenDDS (ROS 2 can run on any DDS implementation).
5. **Security profiles:** DDS supports encryption, authentication, access control.

**DDS in ROS 2 stack:**
- ROS 2 client libraries (rclpy, rclcpp) expose DDS features through simple APIs
- Engineers rarely interact with DDS directly; it's abstracted away
- Advanced users tune DDS configurations for performance/security

**Example:** When you create a publisher in rclpy, behind the scenes, the DDS layer:
1. Advertises the topic on the network
2. Waits for matching subscribers
3. Establishes multicast or unicast connections
4. Handles message serialization, ordering, and delivery

---

## 4. System Architecture

### Multi-Node ROS 2 System for Mobile Manipulation

Below is a realistic architecture integrating PART 1 concepts (perception, reasoning, actuation) with ROS 2 communication patterns:

```
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR LAYER (Drivers)                   │
├─────────────────────────────────────────────────────────────┤
│ Camera Driver     Lidar Driver      Motor Driver             │
│     ↓                 ↓                  ↓                   │
│ /camera/image   /lidar/scan         /odom                   │
│   (pub/sub)       (pub/sub)          (pub/sub)              │
└──────────┬──────────┬────────────────────┬──────────────────┘
           │          │                    │
           ↓          ↓                    ↓
┌─────────────────────────────────────────────────────────────┐
│               PERCEPTION LAYER                              │
├─────────────────────────────────────────────────────────────┤
│ Object Detector          Localization Node                  │
│   Subscribe: /camera     Subscribe: /lidar, /odom           │
│   Publish: /detections   Publish: /pose                     │
│   (Topic: async updates) (Topic: async updates)             │
└──────────┬──────────────────────────┬──────────────────────┘
           │                          │
           ↓                          ↓
           /detections                /pose (global state)
           │
           ↓
┌─────────────────────────────────────────────────────────────┐
│               PLANNING LAYER                                │
├─────────────────────────────────────────────────────────────┤
│          Manipulation Planner (Action Server)               │
│   Subscribe: /detections, /pose                             │
│   Service: /gripper/set_position                            │
│   Action: /gripper/grasp (with feedback)                    │
│   Publish: /arm/trajectory (goal trajectory)                │
└──────────┬──────────────────────────────────────────────────┘
           │
           ↓
           /arm/trajectory
           │
           ↓
┌─────────────────────────────────────────────────────────────┐
│               CONTROL LAYER                                 │
├─────────────────────────────────────────────────────────────┤
│     Trajectory Controller (High-frequency loop)             │
│   Subscribe: /arm/trajectory                                │
│   Call: /gripper/set_position (service)                     │
│   Publish: /arm/command (motor commands @ 100 Hz)           │
└──────────┬──────────────────────────────────────────────────┘
           │
           ↓
           Motor Driver Hardware
```

**Mapping to PART 1 Architecture:**
- **Sensor layer** = PART 1 layer 5 (hardware) + layer 4 (drivers)
- **Perception layer** = PART 1 layer 3 (perception)
- **Planning layer** = PART 1 layer 2 (reasoning/planning)
- **Control layer** = PART 1 layer 2 (control)
- **Middleware** = PART 1 layer 4 (ROS 2)

Each layer is a **ROS 2 node** or **collection of nodes**. Communication between layers uses pub/sub, services, or actions depending on latency and reliability requirements.

---

## 5. Practical Implementation Outline

### 5.1 Setting Up a ROS 2 Workspace

ROS 2 development happens in a **workspace** directory structure:

```
~/ros2_ws/                          # Workspace root
├── src/                            # Source code
│   ├── my_perception_pkg/
│   │   ├── src/
│   │   │   └── detector_node.py
│   │   ├── resource/
│   │   ├── package.xml             # Package metadata
│   │   └── setup.py
│   ├── my_control_pkg/
│   │   ├── src/
│   │   │   └── controller_node.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── ...
├── build/                          # Build artifacts (auto-generated)
├── install/                        # Installed packages (auto-generated)
├── log/                            # Build logs (auto-generated)
└── colcon.lock                     # Dependency lock file
```

**Setup command:**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build                        # Builds all packages
source install/setup.bash           # Activate workspace
ros2 node list                      # Verify ROS 2 is working
```

### 5.2 Multi-Node Communication Design

When designing a ROS 2 system, ask:

1. **What data flows between processes?** → Topics/subscriptions
2. **What requires request/reply?** → Services
3. **What is goal-oriented and long-running?** → Actions
4. **What are latency requirements?** → QoS policy tuning
5. **What can tolerate failure?** → BEST_EFFORT pub/sub
6. **What must never be lost?** → RELIABLE pub/sub + TRANSIENT_LOCAL durability

**Design decision matrix:**

| Communication Need | Pattern | Reason |
|---|---|---|
| Streaming sensor data (camera, lidar) | Topic (pub/sub) | High throughput, asynchronous, BEST_EFFORT OK |
| Robot state update (pose, velocity) | Topic (pub/sub) | High frequency (10-100 Hz), asynchronous |
| Configuration change (PID gains) | Service or Parameter | Synchronous, one-off, low frequency |
| Robot navigation goal | Action | Goal-oriented, long-running, needs feedback |
| Gripper close command | Service or Action | Action better (feedback: grip progress; cancellation: abort grasp) |
| Emergency stop | Topic + Parameter | Topic broadcast (low latency); Parameter monitors (persistent) |

### 5.3 Technology Stack Example: Mobile Manipulator

A typical ROS 2 stack for a mobile manipulator (e.g., Boston Dynamics Spot):

| Layer | Component | Technology |
|-------|-----------|-----------|
| **Hardware** | Base motors, arm, gripper, IMU, camera, lidar | Custom drivers |
| **Drivers** | ROS 2 nodes publishing raw sensor/actuator data | Python or C++ drivers using rclpy/rclcpp |
| **Perception** | Object detection, visual odometry, SLAM | PyTorch + ROS 2 nodes, or Isaac Sim perception |
| **Planning** | Motion planning, grasp selection, navigation | MoveIt 2 (motion planning framework built on ROS 2) |
| **Control** | Low-level trajectory tracking, joint control | Custom C++ control nodes (real-time capable) |
| **Middleware** | Communication, coordination, timing | ROS 2 with DDS (Fast DDS by default) |
| **Tools** | Visualization, debugging, logging | rviz2 (built into ROS 2); rosbag2 for recording |

---

## 6. Role of AI Agents

### How AI Agents Integrate with ROS 2

In PART 1, we introduced AI agents as components that make decisions. In ROS 2, agents manifest as **specialized nodes** that:

1. **Subscribe to perception topics** (camera frames, object detections, state estimates)
2. **Run inference or planning** (neural networks, planners, reinforcement learning policies)
3. **Publish or send goals** (action goals, trajectory commands, service requests)

**Example: Learned Grasping Policy**

```
Input: /camera/image, /detections
  ↓
[Vision encoder (CLIP)]
  ↓ (extract visual features)
[Grasp policy (neural network)]
  ↓ (outputs: grasp pose, gripper width)
[ROS 2 action client]
  ↓ (sends goal to /gripper/grasp action server)
Output: Gripper executes grasp
```

The policy node is a **regular ROS 2 node**; its internal logic (NN weights, optimization) is irrelevant to ROS 2. ROS 2's job is to route the inputs (images, detections) and outputs (goals) reliably.

### Challenges: Latency and Determinism

**Challenge 1: Inference latency.** Neural network forward passes take 10-100+ ms. If control frequency is 100 Hz (10 ms cycle), inference can't happen every cycle. Solution: Decouple perception (lower frequency) from control (high frequency).

```
Perception (20 Hz): camera → detection NN → /detections topic
Control (100 Hz): subscribe to /detections (already computed), send motor commands
```

**Challenge 2: Asynchronous updates.** ROS 2 is fundamentally asynchronous (messages arrive when they arrive). Real-time control needs deterministic timing. Solution: Use DDS QoS deadlines and tune QoS for predictable behavior.

---

## 7. Common Mistakes & Pitfalls

### 7.1 Overcomplicating System Architecture

**Mistake:** Creating one massive ROS 2 node that handles perception, planning, and control in a single process.

**Why it fails:**
- Hard to debug (everything mixed together)
- Can't reuse components (tight coupling)
- Single process failure brings down entire system
- Can't distribute across machines

**Solution:** Decompose into multiple nodes, each responsible for one concern. Use ROS 2's pub/sub to connect them.

---

### 7.2 Ignoring QoS Mismatches

**Mistake:** Publisher publishes with BEST_EFFORT; subscriber expects RELIABLE. No data received.

**Why it fails:**
- QoS must be compatible between publisher and subscriber
- RELIABLE publisher + BEST_EFFORT subscriber works (subscriber downgrades to best-effort)
- BEST_EFFORT publisher + RELIABLE subscriber fails (incompatible; no connection)

**Detection:** `ros2 topic info /topic_name` shows pub/sub count. If it's 0, check QoS compatibility.

**Solution:** Define QoS requirements per topic during design. Document them in package README.

---

### 7.3 Tight Coupling via Direct Dependencies

**Mistake:** Node A directly imports and calls functions from Node B (instead of using pub/sub).

**Why it fails:**
- Can't run nodes on separate machines
- Can't replace Node B without recompiling Node A
- Failure in B crashes A

**Solution:** Use ROS 2 communication primitives (pub/sub, services, actions). Think of nodes as independent processes.

---

### 7.4 Blocking Callbacks in Subscriber

**Mistake:** Subscriber callback does heavy computation (e.g., 2-second inference).

```python
def callback(self, msg):
    # WRONG: Blocks the entire node for 2 seconds
    features = run_nn_inference(msg.data)  # 2s
    self.publish_result(features)
```

**Why it fails:**
- While callback runs, other subscribers/service servers/timers are blocked
- If QoS deadline is 100ms, the 2s callback violates it
- System becomes unresponsive

**Solution:** Offload heavy computation to a separate thread or process.

```python
def callback(self, msg):
    # Queue message for processing in background thread
    self.processing_queue.put(msg)

def processing_thread(self):
    while True:
        msg = self.processing_queue.get()
        features = run_nn_inference(msg.data)  # 2s in background
        self.publish_result(features)

# In __init__: Start processing thread
```

See Chapter 6 (Python development) for proper threading patterns.

---

### 7.5 No Graceful Shutdown Handling

**Mistake:** Node doesn't handle Ctrl+C; resources (files, network connections) leak.

**Why it fails:**
- ROS 2 nodes may be interrupted at any time
- Without proper signal handling, hardware gets stuck in bad state
- Gripper might stay in "open" state; motor might continue spinning

**Solution:** Register signal handlers and clean up on shutdown.

```python
def main():
    node = MyNode()
    
    def shutdown_handler(sig, frame):
        print("Shutting down...")
        node.cleanup()  # Close files, stop motors, etc.
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, shutdown_handler)  # Ctrl+C
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

### 7.6 Message Type Mismatch or Incompatible APIs

**Mistake:** Publisher publishes `std_msgs/String`; subscriber expects `std_msgs/Float64`.

**Why it fails:**
- Types must match exactly; no automatic conversion
- Node will crash or silently fail to deserialize

**Detection:** `ros2 topic info /topic_name --verbose` shows exact type.

**Solution:** Design message types early. Document them in package README. Use custom `.msg` files for complex data.

---

## 8. Summary

### Key Takeaways

1. **ROS 2 is distributed middleware**, not a single library. It enables nodes to communicate reliably across processes and machines.

2. **Publish/subscribe is the foundation.** Topics allow asynchronous, decoupled communication ideal for sensor streaming and state updates.

3. **Services and actions** provide synchronous and goal-oriented alternatives for request/reply and long-running tasks.

4. **Quality of Service (QoS) policies** control reliability and latency. Tuning QoS is essential for real-time robotics.

5. **Decentralized architecture** (DDS) eliminates single points of failure. Nodes discover each other automatically.

6. **Layered design** (sensors → perception → planning → control) maps naturally to ROS 2 nodes and topics.

7. **Real-time execution** requires careful design: avoid blocking callbacks, use appropriate QoS deadlines, and decompose long-running tasks.

8. **Composition over monoliths.** Multiple small, focused nodes beat one giant node.

9. **ROS 2 is infrastructure**, not intelligence. It doesn't make decisions; it routes data. AI agents run as specialized nodes on top.

10. **DDS is the engine**, but you rarely interact with it directly. Client libraries (rclpy, rclcpp) abstract it away.

---

### Self-Assessment Checklist

After completing this chapter, verify you can:

- ✓ Explain the five layers of ROS 2 architecture
- ✓ Describe pub/sub, services, actions, and when to use each
- ✓ Define QoS and its four key policies (reliability, durability, history, deadline)
- ✓ Articulate why ROS 2 uses DDS instead of custom communication
- ✓ Sketch a multi-node ROS 2 system for a given application
- ✓ Identify QoS incompatibilities between publisher and subscriber
- ✓ Design node boundaries (what should be one node vs. multiple nodes)
- ✓ List three common ROS 2 mistakes and how to avoid them
- ✓ Explain how AI agents (learned policies, planners) integrate with ROS 2
- ✓ Understand real-time constraints in ROS 2 systems

---

## 9. RAG-Seed Questions

### Foundational (0-1 year robotics experience)

**Q1: What is ROS 2?**
A: ROS 2 (Robot Operating System 2) is middleware that enables communication between independent processes (nodes) in robotic systems. It provides publish/subscribe patterns for streaming data, services for request/reply, and actions for goal-oriented tasks. It abstracts underlying DDS (Data Distribution Service) middleware.

**Q2: How does ROS 2 differ from ROS 1?**
A: ROS 2 uses DDS (decentralized) instead of ROS Master (single point of failure); supports real-time via QoS policies; has better security; and enables multi-machine systems more robustly. ROS 1 is legacy; new systems use ROS 2.

**Q3: What are the three main communication patterns in ROS 2?**
A: (1) Publish/subscribe (pub/sub) for asynchronous, one-to-many updates; (2) Services for synchronous request/reply; (3) Actions for goal-oriented tasks with feedback. Topics stream data; services are one-off; actions are long-running.

**Q4: What is a ROS 2 node?**
A: A node is a lightweight independent process that publishes or subscribes to topics, provides or calls services, or implements action servers/clients. Nodes communicate via ROS 2's middleware layer.

**Q5: What does QoS stand for, and why does it matter?**
A: QoS (Quality of Service) specifies reliability, latency, durability, and deadlines for communication. It matters because some messages can tolerate loss (lidar scans: BEST_EFFORT) while others must never be lost (navigation goals: RELIABLE).

---

### Intermediate (1-3 years robotics experience)

**Q6: Explain the difference between publish/subscribe and services.**
A: Pub/sub is asynchronous; publisher doesn't wait for response. Services are synchronous; client blocks until server replies. Use pub/sub for continuous updates; services for request/reply (e.g., "get current state" or "set configuration").

**Q7: When would you use an action instead of a service?**
A: Actions are for long-running, preemptible tasks with feedback. Examples: navigation (takes seconds; client wants periodic updates on progress), grasping (feedback: grip force), trajectory execution (feedback: current joint angles). Services block the client; actions don't.

**Q8: What is DDS, and why does ROS 2 use it?**
A: DDS (Data Distribution Service) is a standardized middleware protocol for pub/sub communication. ROS 2 uses it because it's decentralized (no master), supports real-time QoS, has security features, and is vendor-independent. ROS 2 sits atop DDS.

**Q9: How do QoS profiles affect real-time robotics?**
A: Strict deadlines (e.g., 10 ms) force fast message delivery but can fail under network congestion. Loose deadlines tolerate delay but may miss real-time control windows. RELIABLE messages have higher latency than BEST_EFFORT. Tune QoS based on application requirements.

**Q10: What is a QoS mismatch, and how does it break systems?**
A: QoS mismatch occurs when pub/sub compatibility requirements aren't met. Example: BEST_EFFORT publisher + RELIABLE subscriber = no connection (incompatible). Solution: Align QoS between publishers and subscribers. Check with `ros2 topic info --verbose`.

---

### Advanced (3+ years robotics experience)

**Q11: Design a ROS 2 architecture for a mobile manipulator (base + arm + gripper) that satisfies real-time control requirements.**
A: Decompose into nodes: sensor drivers (camera, lidar, IMU), localization (publishes /pose), object detection (subscribes /camera, publishes /detections), manipulation planner (action server /grasp), trajectory controller (subscribes /trajectory, publishes /joint_commands at 100 Hz). Use BEST_EFFORT + loose deadlines for perception; RELIABLE + tight deadlines for commands. This mapping mirrors PART 1's 5-layer architecture.

**Q12: A control loop runs at 100 Hz but depends on a perception system that takes 200 ms (running inference). How would you integrate these in ROS 2 without violating latency requirements?**
A: Decouple perception from control. Run perception at lower frequency (5-10 Hz); publish results to /detections topic. Control loop subscribes to /detections (which is already computed) and runs at 100 Hz independently. Perception doesn't block control; they run asynchronously with different frequencies.

**Q13: Your gripper action server occasionally times out when the network is congested. How would you debug and fix this?**
A: (1) Check QoS deadline on action topic: `ros2 action info /gripper/grasp --verbose`. (2) Increase deadline if too tight. (3) Use DDS middleware profiling to identify packet loss. (4) Consider reducing message frequency or payload size. (5) Isolate gripper communication on dedicated network segment if possible.

**Q14: What are the latency implications of RELIABLE vs. BEST_EFFORT pub/sub, and when would you choose each?**
A: RELIABLE adds acknowledgment overhead (~10-50 ms extra per round-trip); BEST_EFFORT is faster but drops packets. Use RELIABLE for critical data (navigation goals, safety commands); BEST_EFFORT for high-frequency sensor streams (lidar scans that arrive every 10 ms—if one drops, the next one is coming soon). Mixed strategies: perception topics BEST_EFFORT; planning inputs RELIABLE.

**Q15: How would you ensure a ROS 2 system remains responsive even if one node blocks for several seconds?**
A: (1) Use multithreading in blocking nodes; don't block the main spin loop. (2) Implement separate thread pools for I/O-bound tasks (file I/O, network calls). (3) Use executors (single-threaded, multi-threaded, multi-process) to isolate node execution. (4) Monitor callback execution time; implement watchdog timers to detect stuck nodes. (5) Design node boundaries so one failure doesn't cascade (isolation).

---

**Total Questions:** 15  
**Difficulty distribution:** 5 foundational, 5 intermediate, 5 advanced  
**Concepts covered:** Architecture, communication patterns, QoS, DDS, real-time design, multi-node systems, debugging, agent integration  
**Cross-references:** PART 1 (system architecture, perception-action loops), PART 2 Chapters 5-6 (implementation), PART 3 (simulation integration)

---

**Ready to proceed to Chapter 5?**
