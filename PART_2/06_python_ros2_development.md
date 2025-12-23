---
title: Chapter 6 - Python-Based ROS 2 Development with rclpy
description: Practical guide to writing production-quality ROS 2 nodes in Python using rclpy, including initialization, spin loops, threading models, parameters, and safe shutdown.
difficulty: Intermediate
category: Part 2 - ROS 2
keywords: Python, rclpy, ROS 2 development, nodes, callbacks, threading, parameters, lifecycle
---

# Chapter 6: Python-Based ROS 2 Development with rclpy

## 1. Chapter Overview

Chapters 4-5 established *conceptual understanding* of ROS 2. This chapter shifts to *hands-on development*. You will write Python ROS 2 nodes that are production-ready: safe, responsive, debuggable, and maintainable.

Python is the most popular choice for ROS 2 development because it enables rapid iteration while remaining powerful enough for perception, planning, and control. The `rclpy` library provides Python bindings to ROS 2's core APIs.

This chapter covers:
- **Node initialization** and lifecycle management
- **Spin models** (single-threaded, multi-threaded, multi-process executors)
- **Callback design** (avoiding blocking, handling errors)
- **Parameter management** (declaring, reading, monitoring changes)
- **Safe shutdown** (signal handling, resource cleanup)
- **Common patterns** (node composition, reusable components)
- **Testing and debugging** (unit tests, logging, profiling)

---

## 2. Learning Objectives

After completing this chapter, you will understand:

- How to **initialize a ROS 2 node** in Python with rclpy
- The **node lifecycle** (initialization, inactive, active, shutdown)
- **Spin models** and when to use each (single-threaded, multi-threaded, multi-process)
- **Callback execution** and how to avoid blocking
- **Parameter declaration and reading** at node startup
- **Dynamic parameter updates** and monitoring parameter changes
- **Error handling** in callbacks and services
- **Graceful shutdown** and resource cleanup
- **Composition**: Writing reusable node classes; launching multiple nodes
- **Testing strategies** for ROS 2 nodes in Python
- **Profiling and optimization** for performance-critical nodes

---

## 3. Core Concepts

### 3.1 Basic Node Structure

Every ROS 2 Python node follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        
        # Create subscriptions, publishers, services, timers here
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            qos_profile=10  # Queue depth
        )
        
        # Timer for periodic tasks
        self.timer = self.create_timer(0.1, self.timer_callback)  # Called every 100 ms
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
    
    def timer_callback(self):
        self.get_logger().info('Timer fired')

def main(args=None):
    rclpy.init(args=args)
    
    node = MyNode()
    rclpy.spin(node)  # Enter main loop; blocks until shutdown
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key components:**
1. **Class inherits from `Node`:** All ROS 2 functionality available
2. **`__init__` initializes subscriptions, publishers, timers:** Happens once at startup
3. **`rclpy.init()` initializes ROS 2 context globally**
4. **`rclpy.spin()` enters the main event loop:** Callbacks are invoked as messages arrive
5. **Cleanup on shutdown:** Destroy node, shutdown ROS 2

---

### 3.2 Spin Models

The **executor** determines how callbacks are scheduled. ROS 2 provides three options:

#### 3.2.1 Single-Threaded Executor

```python
rclpy.spin(node)  # Default; single-threaded executor
```

**How it works:**
- One thread processes all callbacks sequentially
- Callbacks block each other
- Simple; low overhead

**Example execution timeline:**
```
T=0 ms: Callback A starts (100 ms task)
T=100 ms: Callback B can now run (was waiting)
T=110 ms: Callback B completes
T=120 ms: Next message arrives; new callback runs
```

**Good for:**
- Low-complexity nodes
- No heavy computation in callbacks
- Deterministic ordering matters

**Bad for:**
- Blocking I/O in callbacks (file reads, network requests)
- Multiple subscribers with different latencies
- Real-time constraints

---

#### 3.2.2 Multi-Threaded Executor

```python
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

**How it works:**
- Thread pool (default 4 threads) processes callbacks concurrently
- Callbacks can run in parallel
- More complex; potential for race conditions

**Example execution timeline:**
```
T=0 ms: Callback A starts on thread 1 (100 ms task)
T=5 ms: Callback B arrives on thread 2 (5 ms task)
T=10 ms: Callback B completes (no blocking)
T=100 ms: Callback A completes
```

**Good for:**
- Multiple independent subscribers
- I/O-bound operations in callbacks
- High throughput systems

**Bad for:**
- Shared state between callbacks (race conditions without locks)
- Hard real-time (preemption overhead)

---

#### 3.2.3 Multi-Process Executor

```python
executor = rclpy.executors.MultiProcessExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

**How it works:**
- Separate OS process per node
- True parallelism (on multi-core machines)
- Full isolation; no shared state

**Good for:**
- Nodes that must be isolated (one crash doesn't affect others)
- Mixing C++ and Python nodes (different processes)
- Hard real-time control with soft real-time perception

**Bad for:**
- Overhead of inter-process communication
- Complex coordination

---

### 3.3 Parameter Management

Parameters are runtime configuration values. ROS 2 makes them *readable at any time*, *writable during execution* (no restart needed), and *monitorable* (you can detect changes).

#### Declaring Parameters

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameter with default value
        self.declare_parameter('kp', 1.0)  # Default: 1.0
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('joint_name', 'shoulder')  # String parameter
        
        # Read initial value
        kp = self.get_parameter('kp').value
        self.get_logger().info(f'KP = {kp}')
```

#### Dynamic Parameter Updates

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        self.declare_parameter('kp', 1.0)
        
        # Set up callback for parameter changes
        self.add_on_set_parameters_callback(self.on_parameter_change)
    
    def on_parameter_change(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f'KP updated to {self.kp}')
        
        # Must return SetParametersResult
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)
```

#### Setting Parameters at Launch or Runtime

```bash
# At launch
ros2 run my_package my_node --ros-args -p kp:=2.0

# At runtime (change without restart)
ros2 param set /my_node kp 3.0

# View all parameters
ros2 param list /my_node
ros2 param get /my_node kp
```

---

### 3.4 Callback Design and Blocking

A callback is executed when a message arrives (for subscriptions) or timer fires (for timers).

#### Good: Non-Blocking Callback

```python
def callback(self, msg):
    # Quick processing; no blocking I/O
    self.counter += 1
    self.last_msg = msg
    self.get_logger().info(f'Msg #{self.counter}')  # Logging is fast
```

**Execution time:** <1 ms. Next message can be processed immediately.

#### Bad: Blocking Callback

```python
def callback(self, msg):
    # WRONG: Blocks for several seconds
    data = requests.get('https://example.com/api')  # Network request; 500+ ms
    result = process_image(msg.data)  # Inference; 100+ ms
    
    # While callback runs, other callbacks are stuck
```

**Execution time:** 500+ ms. If messages arrive at 100 Hz (every 10 ms), queue fills up and messages are dropped.

#### Solution: Off-Load Heavy Work

```python
import threading
from queue import Queue

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        self.work_queue = Queue()
        
        # Create subscription
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.quick_callback,
            qos_profile=1  # Keep only latest
        )
        
        # Start background thread for heavy work
        self.worker_thread = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_thread.start()
    
    def quick_callback(self, msg):
        # Just queue the message; return quickly
        self.work_queue.put(msg)
    
    def worker_loop(self):
        # This runs in background thread; doesn't block main loop
        while True:
            msg = self.work_queue.get()  # Blocks until message available
            
            # Heavy work here (inference, I/O)
            result = self.run_inference(msg)  # 100 ms
            
            # Publish result
            self.publish_result(result)
```

**Execution:**
- Callback (main thread): <1 ms (just queuing)
- Heavy work (background thread): 100 ms (doesn't block callback)
- Result published from background thread: asynchronous

---

### 3.5 Graceful Shutdown and Signal Handling

When a node receives Ctrl+C, it must clean up resources.

#### Default Behavior (Usually Sufficient)

```python
def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

#### Custom Cleanup (For Complex Resources)

```python
import signal

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Open resources that need cleanup
        self.camera = cv2.VideoCapture(0)
        self.model = load_model('model.pt')
    
    def cleanup(self):
        self.get_logger().info('Shutting down gracefully...')
        self.camera.release()
        self.model.to('cpu')  # Move model off GPU
        # Other cleanup

def main():
    rclpy.init()
    node = MyNode()
    
    def signal_handler(sig, frame):
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # SIGTERM
    
    rclpy.spin(node)
```

---

### 3.6 Common ROS 2 Node Patterns

#### Pattern 1: Sensor Driver

```python
class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        
        self.camera = cv2.VideoCapture(0)
        self.publisher = self.create_publisher(Image, 'camera/image', qos_profile=10)
        
        # Publish at fixed frequency
        self.timer = self.create_timer(1/30, self.publish_frame)  # 30 Hz
    
    def publish_frame(self):
        ret, frame = self.camera.read()
        if ret:
            msg = cv2_to_ros(frame)
            self.publisher.publish(msg)
```

#### Pattern 2: Perception Node

```python
class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.detect_callback,
            qos_profile=1  # Only latest frame
        )
        self.publisher = self.create_publisher(Detections, 'detections', qos_profile=10)
        
        self.model = YOLO('yolov8n.pt')  # Load once; reuse
    
    def detect_callback(self, msg):
        frame = ros_to_cv2(msg)
        
        # Quick detection (model cached)
        detections = self.model(frame)
        
        # Publish
        output = self.detections_to_ros(detections)
        self.publisher.publish(output)
```

#### Pattern 3: Action Server

```python
from rclpy.action import ActionServer

class GraspServer(Node):
    def __init__(self):
        super().__init__('grasp_server')
        
        self.action_server = ActionServer(
            self,
            GraspObject,
            '/gripper/grasp',
            self.execute_grasp
        )
    
    async def execute_grasp(self, goal_handle):
        request = goal_handle.request
        
        for i in range(100):
            if goal_handle.is_cancel_requested():
                return GraspObject.Result()
            
            feedback = GraspObject.Feedback()
            feedback.progress = i / 100.0
            goal_handle.publish_feedback(feedback)
            
            # Do actual grasping work
            await asyncio.sleep(0.01)
        
        result = GraspObject.Result()
        result.success = True
        goal_handle.succeed()
        return result
```

---

## 4. System Architecture

A typical Python ROS 2 system for perception-based control:

```
┌──────────────────┐
│  Camera Driver   │ (Python node)
│  opencv_python  │
│  → /camera/image│
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ Object Detector  │ (Python + PyTorch)
│  YOLO model     │
│  → /detections  │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ Planner Node     │ (Python)
│  → /goal_pose    │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ Control Node     │ (C++ for real-time)
│  PID loops       │
│  → /motor_cmd    │
└────────┬─────────┘
         │
         ↓
   Hardware
```

---

## 5. Practical Implementation Outline

### 5.1 Project Structure

```
my_ros2_pkg/
├── setup.py
├── package.xml
├── src/
│   └── my_ros2_pkg/
│       ├── __init__.py
│       ├── nodes.py (Define node classes)
│       └── utils.py (Helper functions)
├── launch/
│   └── system.launch.py (Launch multiple nodes)
└── test/
    └── test_nodes.py (Unit tests)
```

### 5.2 Writing and Testing

```python
# src/my_ros2_pkg/nodes.py

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        
        self.subscription = self.create_subscription(Image, 'camera/image', self.callback, qos_profile=1)
        self.publisher = self.create_publisher(Detections, 'detections', qos_profile=10)
        
        self.model = load_model()
    
    def callback(self, msg):
        detections = self.model(msg)
        self.publisher.publish(detections)

# test/test_nodes.py

import unittest
from my_ros2_pkg.nodes import PerceptionNode

class TestPerceptionNode(unittest.TestCase):
    def test_initialization(self):
        node = PerceptionNode()
        self.assertIsNotNone(node)
        node.destroy_node()
    
    def test_callback(self):
        node = PerceptionNode()
        
        # Create test image message
        test_image = Image()
        test_image.data = b'\x00' * 1000
        
        # Call callback
        node.callback(test_image)
        
        node.destroy_node()
```

---

## 6. Role of AI Agents

AI agents in ROS 2 (learned policies, planners) are nodes that:

1. **Subscribe to perception** (camera, detections, state)
2. **Run inference/planning** (neural network, optimization)
3. **Publish goals or commands** (action goals, topic messages)

### Example: Learned Manipulation Policy

```python
class ManipulationPolicy(Node):
    def __init__(self):
        super().__init__('manipulation_policy')
        
        # Subscribe to perception
        self.detections_sub = self.create_subscription(
            Detections,
            'detections',
            self.on_detections,
            qos_profile=1
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'pose',
            self.on_pose,
            qos_profile=1
        )
        
        # Action client for grasping
        self.grasp_client = self.create_action_client(
            GraspObject,
            '/gripper/grasp'
        )
        
        # Load policy model
        self.policy = torch.jit.load('policy.pt')
        
        self.current_detections = None
        self.current_pose = None
    
    def on_detections(self, msg):
        self.current_detections = msg
        self.maybe_act()
    
    def on_pose(self, msg):
        self.current_pose = msg
        self.maybe_act()
    
    def maybe_act(self):
        if self.current_detections is None or self.current_pose is None:
            return
        
        # Prepare input for policy
        state = self.prepare_state(self.current_detections, self.current_pose)
        
        # Run policy (inference)
        with torch.no_grad():
            action = self.policy(state)  # 10-100 ms
        
        # Convert action to goal
        goal = GraspObject.Goal()
        goal.object_id = action.object_id.item()
        
        # Send goal (non-blocking)
        self.grasp_client.send_goal_async(goal)
```

---

## 7. Common Mistakes & Pitfalls

### 7.1 Blocking the Main Spin Loop

**Mistake:** Heavy computation in callback.

**Solution:** Use threading or background tasks.

---

### 7.2 Not Declaring Parameters Before Using

**Mistake:** Call `get_parameter()` without declaring first; returns None.

**Solution:** Always `declare_parameter()` before `get_parameter()`.

---

### 7.3 Resource Leaks on Shutdown

**Mistake:** Open files/cameras/models but don't close on exit.

**Solution:** Implement cleanup; use signal handlers.

---

### 7.4 Race Conditions in Multi-Threaded Callbacks

**Mistake:** Multiple callback threads modify shared state without locks.

**Solution:** Use threading locks for shared mutable state.

---

### 7.5 Unresponsive Node Due to Wrong Executor

**Mistake:** Use single-threaded executor with blocking I/O callback; node becomes unresponsive.

**Solution:** Choose appropriate executor; off-load blocking work.

---

### 7.6 No Error Handling in Callbacks

**Mistake:** Exception in callback crashes node.

**Solution:** Wrap callbacks with try/except; log errors.

---

## 8. Summary

### Key Takeaways

1. **Every ROS 2 Python node inherits from `Node`** and defines subscriptions, publishers, timers in `__init__`.

2. **`rclpy.spin()` enters the main event loop**; callbacks are invoked as events occur.

3. **Callbacks must be non-blocking** to avoid starving other callbacks. Off-load heavy work to threads.

4. **Parameters enable runtime configuration** without restart. Use `declare_parameter()` and `get_parameter()`.

5. **Spin models** (single-threaded, multi-threaded, multi-process) control callback scheduling. Choose based on workload.

6. **Graceful shutdown** requires signal handling and resource cleanup.

7. **Common patterns** (drivers, perception, planning, control) are reusable templates.

8. **AI agents are nodes that run inference/planning** and interact with ROS 2 via subscriptions and action clients.

9. **Testing is essential**; write unit tests for node logic independent of ROS 2.

10. **Logging is your friend for debugging**; use `self.get_logger().info/warn/error()`.

---

### Self-Assessment Checklist

- ✓ Write a basic Python ROS 2 node with subscriptions and publishers
- ✓ Implement a timer-based periodic task
- ✓ Declare and read ROS 2 parameters
- ✓ Handle parameter changes dynamically
- ✓ Choose an appropriate executor for your node (single/multi-threaded/multi-process)
- ✓ Off-load blocking I/O to background threads
- ✓ Implement graceful shutdown with signal handling
- ✓ Write an action server and action client
- ✓ Synchronize multiple subscriptions using message_filters
- ✓ Test node logic with unit tests

---

## 9. RAG-Seed Questions

### Foundational

**Q1: What is the basic structure of a Python ROS 2 node?**
A: A class inheriting from `Node`, with subscriptions/publishers created in `__init__`, callbacks for subscriptions/timers, and a main function that calls `rclpy.init()`, `rclpy.spin(node)`, then cleanup. See Section 3.1 for template.

**Q2: What does `rclpy.spin()` do?**
A: Enters the main event loop. Callbacks are invoked as messages arrive or timers fire. Blocks until shutdown (Ctrl+C or `rclpy.shutdown()`).

**Q3: What are the three spin models, and when would you use each?**
A: (1) Single-threaded (default): simple, but callbacks block each other. Use for low-complexity. (2) Multi-threaded: callbacks run in parallel. Use for I/O-bound or independent subscribers. (3) Multi-process: separate OS processes. Use for isolation or mixing languages. See Section 3.2.

**Q4: How do you declare and read a ROS 2 parameter?**
A: `self.declare_parameter('name', default_value)` in `__init__`. Read with `self.get_parameter('name').value`. Change at runtime with `ros2 param set /node_name name value` without restart.

**Q5: How do you avoid blocking the main callback loop?**
A: Use threading. Queue incoming messages in callback (fast); process them in background thread (slow). See Section 3.4 for example.

---

### Intermediate

**Q6: Design a Python ROS 2 node that reads camera frames at 30 Hz, runs a neural network inference (100 ms per frame), and publishes detections. Ensure the camera driver doesn't block.**
A: Use multi-threaded executor or single-threaded with background thread. Camera callback quick-queues frames. Background thread runs inference and publishes. Detections publish rate will be ~10 Hz (limited by inference time). See Section 3.4 pattern.

**Q7: How would you monitor a ROS 2 parameter for changes and react to them in real-time?**
A: Implement `add_on_set_parameters_callback()` callback. When parameter changes (via `ros2 param set`), callback is invoked; update internal state. No restart needed. See Section 3.4.

**Q8: Implement a simple action server in Python that takes 10 seconds to execute and provides feedback every second.**
A: Use `ActionServer` with async `execute_callback`. Loop 10 times; publish feedback, sleep 1 second, check `is_cancel_requested()`. See Section 3.6, Pattern 3.

**Q9: What's the difference between `rclpy.spin()` and `executor.spin()`?**
A: `rclpy.spin(node)` uses the default single-threaded executor. `executor = MultiThreadedExecutor(); executor.add_node(node); executor.spin()` uses multi-threaded. More control with explicit executor.

**Q10: How do you safely shut down a Python ROS 2 node that has open resources (camera, GPU model)?**
A: Implement cleanup method; register signal handler for SIGINT (Ctrl+C) and SIGTERM. Handler calls cleanup, destroys node, shuts down ROS 2. See Section 3.5.

---

### Advanced

**Q11: Implement a Python ROS 2 node that subscribes to an action goal, runs a 5-minute optimization algorithm in a background thread, and publishes progress feedback without blocking the node.**
A: Create action server. On goal, queue it and return immediately. Background thread dequeues goals and runs optimization, publishing feedback via action server's feedback mechanism. Main loop never blocks. Implementation similar to Section 3.4 + Section 3.6 pattern 3.

**Q12: You have a camera driver (Python) running at 30 Hz and an inference node (Python/PyTorch) running at 5 Hz. Both use the default single-threaded executor. Why might frames be dropped, and how would you fix it?**
A: Single-threaded executor can't interleave; camera callback blocks during inference callback. Solution: (1) Use multi-threaded executor, or (2) Have camera callback queue frames (non-blocking); inference runs independently in background thread at slower pace. Decouple frame acquisition (fast) from inference (slow).

**Q13: How would you profile a Python ROS 2 node to identify which callbacks are slow and causing latency?**
A: Add timing code in callbacks: `t0 = time.time(); [callback work]; duration = time.time() - t0`. Log duration. Use `ros2 topic hz` to measure message frequency. High latency + low frequency = slow callback. Use cProfile for detailed profiling of Python code.

**Q14: A Python ROS 2 node loads a large neural network model (2 GB) in `__init__`. What are the implications for multi-instance deployments, and how would you mitigate?**
A: Each instance loads 2 GB; 5 instances = 10 GB memory. Mitigation: (1) Load model once in shared memory (difficult in Python), (2) Use model server (separate process) and RPC calls, (3) Quantize model to smaller size, (4) Use GPU (model in VRAM, not main memory).

**Q15: Compare latency of these ROS 2 Python architectures: (A) Single node with multiple subscribers, (B) Multiple nodes communicating via topics, (C) Nodes with shared memory (multiprocessing.Queue). When would you choose each?**
A: (A) Single node: <5 ms latency; simple; monolithic. (B) Multiple nodes: 5-20 ms latency (serialization overhead); distributed; better isolation. (C) Shared memory: 5-10 ms; fast; complex synchronization. Choose (A) for simple perception; (B) for modularity; (C) for high-throughput low-latency in trusted environment.

---

**Total Questions:** 15  
**Difficulty distribution:** 5 foundational, 5 intermediate, 5 advanced  

---

**Ready to proceed to Chapter 7?**
