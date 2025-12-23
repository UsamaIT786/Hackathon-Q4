---
title: Chapter 7 - Agent-to-ROS Communication Patterns
description: Integration of AI agents (learned policies, planners, decision systems) with ROS 2 communication layer, addressing latency, failure modes, and real-time constraints.
difficulty: Advanced
category: Part 2 - ROS 2
keywords: AI agents, policies, planning, ROS 2 integration, latency, real-time, safety, perception-action loops
---

# Chapter 7: Agent-to-ROS Communication Patterns

## 1. Chapter Overview

In PART 1, we introduced **AI agents** as decision-making systems (learned policies, planners, controllers). In Chapters 4-6, we learned how ROS 2 enables distributed communication. This chapter bridges the two: how do AI agents *actually integrate* with ROS 2 systems operating under real-world constraints?

This is where theory meets practice. Real robotics systems must handle:
- **Latency:** Inference takes 50-500 ms; control runs at 100+ Hz
- **Uncertainty:** What if the AI module crashes? What if network fails?
- **Safety:** A bad decision from an AI agent could destroy hardware or injure humans
- **Heterogeneity:** Some agents are learned (NN), others are classical (MPC), others are hybrid

This chapter provides patterns for integrating AI agents robustly.

---

## 2. Learning Objectives

After completing this chapter, you will understand:

- **Types of AI agents** in robotics: learned policies, planners, controllers, hybrid systems
- **Latency decoupling patterns** between perception (slow), planning (medium), control (fast)
- **State representation** for agent inputs and outputs in ROS 2
- **Synchronization between agent nodes and ROS 2** communication layer
- **Failure detection and recovery** (watchdogs, fallbacks, graceful degradation)
- **Safety patterns** (approval queues, limits checking, risk assessment)
- **Testing AI agents** in simulation before hardware
- **Multi-agent coordination** patterns (negotiation, priority, arbitration)
- **Scaling patterns** for deployed agent systems (hot-swapping policies, versioning)
- **Debugging integration failures** (why agent predictions don't translate to actions)

---

## 3. Core Concepts

### 3.1 Types of AI Agents in Robotics

Not all AI agents are created equal. ROS 2 integration patterns depend on agent *type*.

#### 3.1.1 Reactive Learned Policies

A **reactive policy** is a neural network or learned function that maps *current state* to *action*.

```
State (images, joint positions) → Policy NN → Action (joint velocities, gripper commands)
Latency: 10-100 ms
```

**Example: Visual servoing**
```
Input: Camera frame
Policy: ResNet encoder → Dense layers
Output: Linear velocity, angular velocity
Latency: 50 ms

Publishing frequency: 20 Hz (one output every 50 ms)
Control loop: Subscribes to /policy/command at 100 Hz; uses latest action
```

**ROS 2 pattern:**
```python
class VisualServoing(Node):
    def __init__(self):
        self.camera_sub = self.create_subscription(Image, 'camera/image', self.on_image, qos_profile=1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile=10)
        self.policy = load_policy('visual_servoing.pt')
    
    def on_image(self, msg):
        # Slow: inference takes 50 ms
        image = ros_to_tensor(msg)
        with torch.no_grad():
            cmd = self.policy(image)
        
        # Publish (may drop old images due to qos_profile=1)
        twist = Twist()
        twist.linear.x = cmd[0].item()
        twist.angular.z = cmd[1].item()
        self.cmd_pub.publish(twist)
```

**Advantage:** Simple; one-shot mapping from perception to action.

**Disadvantage:** Limited to reactive behavior; can't plan or handle temporal dependencies.

---

#### 3.1.2 Learned Planning Policies

A **planning policy** reasons about **sequences of actions** to achieve a goal. Often based on reinforcement learning (RL) or imitation learning.

```
Goal state + Current state → Planner NN → Action sequence (a_0, a_1, a_2, ...)
Latency: 100-1000 ms (offline planning)
```

**Example: Trajectory planning for manipulation**
```
Input: Current pose, goal pose, object pose
Policy: Transformer or LSTM encoder
Output: Planned joint angles sequence (50 timesteps)
Latency: 200 ms

Then lower-level controller (classical PID) executes trajectory
```

**ROS 2 pattern:**
```python
class ManipulationPlanner(Node):
    def __init__(self):
        self.grasp_goal_sub = self.create_subscription(
            GraspGoal,
            'grasp_goal',
            self.on_goal,
            qos_profile=10  # Don't drop goals
        )
        
        self.traj_action_srv = self.create_action_server(
            ExecuteTrajectory,
            '/arm/execute_trajectory',
            self.execute_trajectory_callback
        )
        
        self.planner = load_policy('trajectory_planner.pt')
    
    def on_goal(self, goal_msg):
        # Plan trajectory (slow)
        state = self.prepare_state(goal_msg)
        with torch.no_grad():
            trajectory = self.planner(state)  # 200 ms
        
        # Send as action goal to lower-level controller
        traj_goal = ExecuteTrajectory.Goal()
        traj_goal.trajectory = trajectory
        
        self.traj_action_client.send_goal_async(traj_goal)
```

**Advantage:** Can reason about future and plan longer horizons.

**Disadvantage:** Requires goal representation; planning time adds latency.

---

#### 3.1.3 Hybrid Agents (Learned + Classical)

**Hybrid agents** combine learned perception with classical planning/control.

```
Camera → CNN encoder → Feature vector
Feature vector + Goal → MPC optimizer → Control input
                          ↓
                    Classical optimization
```

**Example: Learning perception; MPC for control**
```
Vision encoder: CNN that learns visual features (learned)
Planner: Model Predictive Control that optimizes trajectory (classical)
Control: PID loop on motors (classical)
```

**ROS 2 pattern:**
```python
class HybridController(Node):
    def __init__(self):
        self.camera_sub = self.create_subscription(Image, 'camera/image', self.on_image, qos_profile=1)
        self.goal_sub = self.create_subscription(GoalPose, 'goal', self.on_goal, qos_profile=10)
        self.cmd_pub = self.create_publisher(JointCommand, 'joint_commands', qos_profile=10)
        
        self.vision_encoder = load_model('vision_encoder.pt')  # Learned
        self.mpc = MPCPlanner()  # Classical
    
    def on_image(self, msg):
        # Extract features using learned model
        image = ros_to_tensor(msg)
        with torch.no_grad():
            features = self.vision_encoder(image)  # 10 ms
        
        self.current_features = features
    
    def on_goal(self, msg):
        # Plan using classical MPC + learned features
        if self.current_features is None:
            return
        
        # MPC uses learned features + goal + classical optimization
        cmd = self.mpc.solve(
            state=self.current_features,
            goal=msg.goal_pose,
            dt=0.01  # 100 Hz control
        )
        
        joint_msg = JointCommand()
        joint_msg.positions = cmd
        self.cmd_pub.publish(joint_msg)
```

**Advantage:** Combines learning (flexibility) with classical methods (predictability).

**Disadvantage:** Requires expertise in both domains; more complex integration.

---

### 3.2 Latency Decoupling

The central challenge: **Perception is slow (50+ ms); Control is fast (10 ms cycles).** They must not block each other.

#### Pattern 1: Decoupled Perception and Control

```
Perception (20 Hz):
  Image → Inference (50 ms) → /feature topic
  ↓ (every 50 ms)

Control (100 Hz):
  /feature topic (subscribe with KEEP_LAST=1) → PID loop (1 ms) → /command topic
  ↓ (every 10 ms; reuses last feature)

Timeline:
T=0 ms: Feature 1 computed; published
T=10 ms: Control uses Feature 1
T=20 ms: Control uses Feature 1 (same feature; 10 ms old)
T=30 ms: Control uses Feature 1 (20 ms old)
T=40 ms: Control uses Feature 1 (30 ms old)
T=50 ms: Feature 2 computed; published
T=60 ms: Control uses Feature 2
```

**ROS 2 code:**
```python
class DecoupledControlNode(Node):
    def __init__(self):
        # Subscribe with KEEP_LAST=1; don't block on perception
        self.feature_sub = self.create_subscription(
            Features,
            'features',
            self.on_features,
            qos_profile=QoSProfile(history=KEEP_LAST, depth=1)  # Only latest
        )
        
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        self.current_features = None
    
    def on_features(self, msg):
        self.current_features = msg  # Just store; don't block
    
    def control_loop(self):
        if self.current_features is None:
            return
        
        # Use latest features; don't wait for new ones
        cmd = self.pid_control(self.current_features)
        self.cmd_pub.publish(cmd)
```

**Advantage:** Control loop never blocks waiting for perception. Guaranteed 100 Hz.

**Disadvantage:** Control uses stale perception data (up to 50 ms old). Acceptable for many applications.

---

#### Pattern 2: Gated Perception

When control must wait for fresh perception (e.g., safety-critical decisions), use **gated perception**—perception publishes events, control responds.

```
Perception:
  Image → Inference → Safety check → /safety_event topic (only on important events)

Control (event-triggered):
  Waits for /safety_event
  → Action triggered
  → Publishes /command
```

**ROS 2 code:**
```python
class GatedPerceptionControl(Node):
    def __init__(self):
        self.image_sub = self.create_subscription(Image, 'image', self.on_image, qos_profile=1)
        self.safety_event_pub = self.create_publisher(SafetyEvent, 'safety_event', qos_profile=10)
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        self.policy = load_policy('safety_check.pt')
    
    def on_image(self, msg):
        # Inference
        image = ros_to_tensor(msg)
        with torch.no_grad():
            safety_score = self.policy(image)  # 0-1; 1 = unsafe
        
        # Only publish event if safety score crosses threshold
        if safety_score > 0.9:
            event = SafetyEvent()
            event.score = safety_score
            event.action = 'EMERGENCY_STOP'
            self.safety_event_pub.publish(event)
            
            # Immediately command stop
            cmd = Command()
            cmd.velocity = 0.0
            cmd.angular_velocity = 0.0
            self.cmd_pub.publish(cmd)
```

**Advantage:** Perception only interrupts control when necessary.

**Disadvantage:** Requires clear event definition; miss events → unsafe behavior.

---

### 3.3 Failure Handling and Safety

AI agents can fail: inference errors, model hallucinations, network timeouts. ROS 2 systems must be *resilient*.

#### Pattern 1: Watchdog Timer

Detect when agent is stuck or crashed.

```python
class WatchdogAgent(Node):
    def __init__(self):
        self.agent_sub = self.create_subscription(Command, 'agent_command', self.on_agent_command, qos_profile=10)
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        # Watchdog timer; fires if no message received in 1 second
        self.watchdog_timeout = 1.0  # seconds
        self.last_agent_command_time = time.time()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)  # Check every 100 ms
    
    def on_agent_command(self, msg):
        self.last_agent_command_time = time.time()
        
        # Forward command to hardware
        self.cmd_pub.publish(msg)
    
    def watchdog_check(self):
        elapsed = time.time() - self.last_agent_command_time
        
        if elapsed > self.watchdog_timeout:
            # Agent timed out; send safe default command
            self.get_logger().error(f'Agent timeout detected (elapsed={elapsed:.1f}s); sending STOP')
            
            safe_cmd = Command()
            safe_cmd.velocity = 0.0
            self.cmd_pub.publish(safe_cmd)
```

#### Pattern 2: Approval Queue

For critical decisions, require human approval before execution.

```python
class ApprovalAgent(Node):
    def __init__(self):
        self.agent_sub = self.create_subscription(
            Action,
            'agent_action',
            self.on_agent_action,
            qos_profile=10
        )
        
        self.approval_request_pub = self.create_publisher(
            ApprovalRequest,
            'approval_request',
            qos_profile=10
        )
        
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        self.pending_action = None
    
    def on_agent_action(self, action_msg):
        # For safety-critical actions, request approval
        if action_msg.risk_level == 'HIGH':
            self.get_logger().warn(f'High-risk action: {action_msg.description}; requesting approval')
            
            request = ApprovalRequest()
            request.action = action_msg
            request.timestamp = self.get_clock().now()
            self.approval_request_pub.publish(request)
            
            self.pending_action = action_msg
        else:
            # Low-risk; execute immediately
            cmd = self.action_to_command(action_msg)
            self.cmd_pub.publish(cmd)
```

#### Pattern 3: Fallback to Classical Control

If agent fails, fall back to pre-programmed or human control.

```python
class FallbackAgent(Node):
    def __init__(self):
        self.agent_sub = self.create_subscription(Command, 'agent_command', self.on_agent_command, qos_profile=10)
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        self.agent_healthy = True
        self.fallback_controller = SimpleController()  # Classical controller
    
    def on_agent_command(self, msg):
        try:
            # Validate command (check if sensible)
            if not self.validate_command(msg):
                raise ValueError(f'Invalid command: {msg}')
            
            self.cmd_pub.publish(msg)
            self.agent_healthy = True
        
        except Exception as e:
            self.get_logger().error(f'Agent command failed: {e}; using fallback')
            self.agent_healthy = False
            
            # Use fallback controller
            fallback_cmd = self.fallback_controller.compute_command()
            self.cmd_pub.publish(fallback_cmd)
    
    def validate_command(self, cmd):
        # Check if command is within safe bounds
        return (
            abs(cmd.velocity) < 2.0 and  # Max 2 m/s
            abs(cmd.angular_velocity) < 1.0 and  # Max 1 rad/s
            cmd.velocity >= 0.0  # No backward (for safety)
        )
```

---

### 3.4 State Representation and Synchronization

Agents need a *representation of the world state*. This must be synchronized across multiple sensors/nodes.

#### Challenge: Asynchronous Sensor Updates

```
Camera publishes at 30 Hz; Lidar at 10 Hz; IMU at 100 Hz

Agent wants to reason about (camera_frame, lidar_scan, imu_accel)
But they rarely arrive simultaneously.

Naive approach: Use whatever is latest
  → Temporal misalignment (10ms camera + 50ms lidar; 60ms old lidar)

Better approach: Use message_filters to synchronize
  → Wait for matching timestamps
  → Slight additional latency (sync window); guaranteed alignment
```

**ROS 2 code:**
```python
class StateAggregator(Node):
    def __init__(self):
        # Subscriptions with message_filters
        self.camera_sub = message_filters.Subscriber(self, Image, 'camera/image')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, 'lidar/scan')
        self.imu_sub = message_filters.Subscriber(self, Imu, 'imu/data')
        
        # Synchronize with 100 ms time window
        sync = message_filters.TimeSynchronizer(
            [self.camera_sub, self.lidar_sub, self.imu_sub],
            queue_size=10,
            slop=0.1  # 100 ms tolerance
        )
        sync.registerCallback(self.on_synchronized_state)
        
        self.state_pub = self.create_publisher(WorldState, 'world_state', qos_profile=10)
    
    def on_synchronized_state(self, camera_msg, lidar_msg, imu_msg):
        # All three messages have similar timestamps (within slop)
        state = WorldState()
        state.camera = camera_msg
        state.lidar = lidar_msg
        state.imu = imu_msg
        state.timestamp = camera_msg.header.stamp
        
        self.state_pub.publish(state)
```

---

### 3.5 Multi-Agent Coordination

When multiple agents operate on the same robot, they must coordinate to avoid conflicts.

#### Pattern 1: Arbitration

One "referee" node decides which agent gets control.

```python
class Arbitrator(Node):
    def __init__(self):
        self.agent1_sub = self.create_subscription(Command, 'agent1/command', self.on_agent1, qos_profile=10)
        self.agent2_sub = self.create_subscription(Command, 'agent2/command', self.on_agent2, qos_profile=10)
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        self.active_agent = 'agent1'  # Default
        self.agent1_cmd = None
        self.agent2_cmd = None
    
    def on_agent1(self, msg):
        self.agent1_cmd = msg
        
        if self.active_agent == 'agent1':
            self.cmd_pub.publish(msg)
    
    def on_agent2(self, msg):
        self.agent2_cmd = msg
        
        if self.active_agent == 'agent2':
            self.cmd_pub.publish(msg)
    
    def switch_agent(self, new_agent):
        self.get_logger().info(f'Switching from {self.active_agent} to {new_agent}')
        self.active_agent = new_agent
```

#### Pattern 2: Blending

Average or blend commands from multiple agents.

```python
class BlendingAgent(Node):
    def __init__(self):
        self.agent1_sub = self.create_subscription(Command, 'agent1/command', self.on_agent1, qos_profile=10)
        self.agent2_sub = self.create_subscription(Command, 'agent2/command', self.on_agent2, qos_profile=10)
        self.cmd_pub = self.create_publisher(Command, 'command', qos_profile=10)
        
        self.agent1_cmd = None
        self.agent2_cmd = None
        self.blend_timer = self.create_timer(0.01, self.blend_and_publish)  # 100 Hz
    
    def blend_and_publish(self):
        if self.agent1_cmd is None or self.agent2_cmd is None:
            return
        
        # Blend: weighted average
        w1, w2 = 0.6, 0.4
        blended = Command()
        blended.velocity = (
            w1 * self.agent1_cmd.velocity +
            w2 * self.agent2_cmd.velocity
        )
        blended.angular_velocity = (
            w1 * self.agent1_cmd.angular_velocity +
            w2 * self.agent2_cmd.angular_velocity
        )
        
        self.cmd_pub.publish(blended)
```

---

## 4. System Architecture: Agent-Based Control Loop

Complete example integrating learned and classical components:

```
┌──────────────────────┐
│   Camera Driver      │ (30 Hz)
│ → /camera/image      │
└──────────┬───────────┘
           │
           ↓
┌──────────────────────────────────┐
│    Vision Encoder Node           │ (Learned perception)
│    Subscribe: /camera/image      │ (Fast, always running)
│    Publish: /visual_features     │
│    (50 ms inference; 20 Hz output)
└──────────┬──────────────────────┘
           │
           ↓
┌──────────────────────────────────┐
│    Action Node (Policy)          │ (Learned planning)
│    Subscribe: /visual_features   │ (Event-triggered or regular)
│    Action: /gripper/grasp        │ (Non-blocking send goal)
└──────────┬──────────────────────┘
           │
           ↓
┌──────────────────────────────────┐
│    Trajectory Controller Node    │ (Classical control)
│    Subscribe: /grasp_trajectory  │ (High frequency)
│    Publish: /joint_command       │ (100 Hz)
│    (PID loops; <10 ms)           │
└──────────┬──────────────────────┘
           │
           ↓
┌──────────────────────────────────┐
│    Hardware (Motor Drivers)      │
└──────────────────────────────────┘
```

Latency breakdown:
- Image capture: 0 ms (now)
- Vision inference: 50 ms
- Planning: 100 ms
- Control: 10 ms cycle
- **Total latency:** 160 ms from image to motor command

This is typical for robotics; acceptable for many applications but not real-time reactive tasks.

---

## 5. Practical Implementation Outline

### 5.1 Implementing a Learned Policy Node

```python
class LearnedPolicyNode(Node):
    def __init__(self):
        super().__init__('learned_policy')
        
        # Input: Current state (from perception)
        self.state_sub = self.create_subscription(
            State,
            'state',
            self.on_state,
            qos_profile=1  # Only latest
        )
        
        # Output: Action (to controller)
        self.action_pub = self.create_publisher(Action, 'action', qos_profile=10)
        
        # Load model
        self.policy = torch.jit.load('policy.pt')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Inference time tracking (for debugging)
        self.inference_times = []
    
    def on_state(self, msg):
        t0 = time.time()
        
        try:
            # Convert ROS message to tensor
            state_tensor = self.msg_to_tensor(msg).to(self.device)
            
            # Run inference
            with torch.no_grad():
                action_tensor = self.policy(state_tensor)
            
            # Convert back to ROS message
            action_msg = self.tensor_to_msg(action_tensor)
            
            # Publish
            self.action_pub.publish(action_msg)
            
            # Log performance
            inference_time = time.time() - t0
            self.inference_times.append(inference_time)
            
            if len(self.inference_times) >= 100:
                avg_time = sum(self.inference_times) / len(self.inference_times)
                self.get_logger().info(
                    f'Inference: avg={avg_time*1000:.1f}ms, '
                    f'max={max(self.inference_times)*1000:.1f}ms'
                )
                self.inference_times = []
        
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
    
    def msg_to_tensor(self, msg):
        # Convert ROS State message to PyTorch tensor
        return torch.tensor([msg.position, msg.velocity, msg.acceleration])
    
    def tensor_to_msg(self, tensor):
        # Convert PyTorch tensor to ROS Action message
        msg = Action()
        msg.velocity = float(tensor[0])
        msg.angular_velocity = float(tensor[1])
        return msg
```

### 5.2 Monitoring Agent Health

```python
class AgentMonitor(Node):
    def __init__(self):
        self.agent_sub = self.create_subscription(
            Action,
            'agent_action',
            self.on_action,
            qos_profile=10
        )
        
        self.health_pub = self.create_publisher(
            AgentHealth,
            'agent_health',
            qos_profile=10
        )
        
        self.timer = self.create_timer(1.0, self.publish_health)  # Publish health every 1s
        
        self.last_action_time = time.time()
        self.action_count = 0
    
    def on_action(self, msg):
        self.last_action_time = time.time()
        self.action_count += 1
    
    def publish_health(self):
        elapsed = time.time() - self.last_action_time
        
        health = AgentHealth()
        health.timestamp = self.get_clock().now()
        health.is_active = elapsed < 1.0  # Healthy if action published in last 1s
        health.action_frequency = self.action_count  # Actions per second
        
        if not health.is_active:
            health.status = 'INACTIVE'
        else:
            health.status = 'ACTIVE'
        
        self.health_pub.publish(health)
        self.action_count = 0
```

---

## 6. Role of AI Agents (Meta-Discussion)

This chapter directly addresses PART 1's question: "What is the role of AI agents in Physical AI systems?"

**Answer:**
- AI agents (learned policies, planners) are *decision-making components* in larger robotic systems
- They run as ROS 2 nodes; they don't own the system communication layer
- They must be *fast enough* (inference latency) and *safe* (fallback mechanisms)
- Integration requires careful handling of latency mismatches and failure modes

AI agents are powerful but not magical; they must respect real-world constraints (latency, uncertainty, safety).

---

## 7. Common Mistakes & Pitfalls

### 7.1 Ignoring Inference Latency in System Design

**Mistake:** Design control loop assuming perception is instant.

**Solution:** Explicitly model latency; decouple perception from control.

---

### 7.2 No Fallback if Agent Fails

**Mistake:** Agent crashes; whole system crashes.

**Solution:** Implement watchdog, fallback controller, or human takeover.

---

### 7.3 Synchronization Failures

**Mistake:** Agent receives misaligned sensor data (stale or mixed timestamps).

**Solution:** Use message_filters for temporal synchronization.

---

### 7.4 Safety Violations

**Mistake:** Agent outputs impossible commands (negative velocity, extreme accelerations).

**Solution:** Validate all agent outputs before forwarding to hardware.

---

### 7.5 Multi-Agent Conflicts

**Mistake:** Multiple agents publish to same topic; messages collide.

**Solution:** Use arbitration or blending; explicit coordination.

---

### 7.6 Poor Testing Integration

**Mistake:** Test agent in isolation; fails when integrated with ROS 2 system.

**Solution:** Test agent nodes in simulation; use mock ROS 2 interfaces.

---

## 8. Summary

### Key Takeaways

1. **AI agents are ROS 2 nodes** that subscribe to perception, run inference/planning, and publish actions.

2. **Latency decoupling is essential**: Perception (slow, 50-100 ms) and control (fast, 10 ms) must run independently.

3. **Reactive policies** (NN mapping state to action) are simple but limited. **Planning policies** are complex but capable. **Hybrid agents** (learned + classical) combine both.

4. **Synchronization matters**: Use message_filters to align data from multiple sensors.

5. **Failure handling is safety-critical**: Implement watchdogs, fallbacks, and approval gates.

6. **Multi-agent coordination** requires arbitration, blending, or negotiation.

7. **Validation and testing** are crucial before deploying learned models on real hardware.

8. **Monitoring agent health** (inference time, action frequency, output validity) enables debugging.

9. **Safety constraints** (velocity limits, approval gates, risk assessment) prevent bad decisions from reaching hardware.

10. **Documentation and versioning** of agent models enable reproducibility and debugging.

---

### Self-Assessment Checklist

- ✓ Explain three types of AI agents (reactive, planning, hybrid) and ROS 2 integration for each
- ✓ Design a latency-decoupled system where perception (slow) and control (fast) run independently
- ✓ Implement synchronization for multiple asynchronous sensors using message_filters
- ✓ Design a failure recovery system with watchdog and fallback
- ✓ Implement safety validation for agent outputs
- ✓ Handle multi-agent coordination via arbitration or blending
- ✓ Monitor and log agent health (inference time, action frequency)
- ✓ Test agent node in ROS 2 simulation before hardware
- ✓ Implement graceful degradation (agent fails → use fallback)
- ✓ Debug integration failures using ROS 2 CLI tools

---

## 9. RAG-Seed Questions

### Foundational

**Q1: What is an AI agent in the context of ROS 2?**
A: An AI agent is a ROS 2 node that subscribes to perception (camera, state), runs decision logic (neural network, planner), and publishes actions (commands, goals). It's one component of a larger system.

**Q2: Why must perception and control be decoupled?**
A: Perception (inference) is slow (50-100 ms); control loops run fast (10-100 Hz). If control waits for perception, it misses cycles. Solution: Decouple; control uses latest perception asynchronously.

**Q3: What happens if you synchronize perception directly to control (blocking)?**
A: Control loop blocks waiting for perception to finish. If perception takes 100 ms and control needs 10 ms cycles, control can only run at 10 Hz (10x slower). Unacceptable for real-time robotics.

**Q4: How do you handle the case where sensor 1 publishes at 30 Hz and sensor 2 publishes at 10 Hz?**
A: Use `message_filters.TimeSynchronizer` to wait for both to arrive with similar timestamps before calling callback. Output rate limited by slower sensor (~10 Hz).

**Q5: What is a watchdog timer, and why do you need it for AI agents?**
A: A watchdog monitors if agent is producing outputs (e.g., new action every 100 ms). If timeout occurs (no output for 1 second), agent is assumed dead; system triggers fallback (e.g., STOP command).

---

### Intermediate

**Q6: Design a ROS 2 system where a learned policy controls a 100 Hz control loop, but inference takes 200 ms. How would you structure it?**
A: (1) Perception node publishes features at ~5 Hz (every 200 ms inference). (2) Control loop subscribes to features with KEEP_LAST=1. (3) Control runs at 100 Hz using latest features (up to 200 ms old). (4) Inverse kinematics or classical control at high frequency. Learned policy acts as high-level perception; classical control at low level.

**Q7: Implement a safety check for a learned grasping policy: validate that gripper force is within [0, 100 N] before sending to hardware.**
A: ```python
def validate_grasp(self, action):
    if not (0 <= action.grip_force <= 100):
        self.get_logger().error(f'Invalid force: {action.grip_force}; clamping')
        action.grip_force = max(0, min(100, action.grip_force))
    return action
```
Or reject action entirely if out of bounds.

**Q8: Two agents (vision-based and force-control-based) operate on same gripper. How would you coordinate them?**
A: Arbitration (one active at a time), blending (average commands), or negotiation (agents signal preference). Arbitrator node selects which agent's command to execute based on mode (e.g., vision mode → agent1 active, force mode → agent2 active).

**Q9: Your AI agent's perception is correct, but commands don't translate to desired behavior. What could be wrong?**
A: (1) Action representation mismatch (agent outputs velocity; hardware expects position). (2) Latency accumulation (actions stale by the time executed). (3) Unmodeled dynamics (agent assumes instant response; hardware has inertia). (4) Sensor-actuator feedback loop closed at wrong level. Debug by logging intermediate values.

**Q10: How would you test an AI agent node in ROS 2 without deploying to hardware?**
A: Use Gazebo simulation. Instead of real camera/motors, use simulated ones. Agent node runs unmodified; subscribes to /camera (from Gazebo) and publishes /commands (to Gazebo physics). Test logic in safe simulated environment before hardware.

---

### Advanced

**Q11: Design a system where a 100 ms planning policy outputs a trajectory, and a 10 ms control loop executes it. How do you prevent the control loop from outrunning the planner?**
A: Use action server pattern. Policy sends trajectory goal (via action) to controller. Controller executes (follows trajectory point-by-point). Policy publishes new goals as they're ready. Controller never outruns because it's following predefined trajectory. If policy dies, controller finishes current trajectory then stops.

**Q12: Implement a graceful degradation strategy: if learned policy fails, fall back to classical controller. Both should be able to control the robot.**
A: (1) Create controller multiplexer node. (2) Subscribe to learned policy outputs and classical controller outputs. (3) Have a health monitor on learned policy. (4) If learned policy unhealthy, switch multiplexer to classical controller outputs. (5) Classical controller always runs in background. See Section 3.3, Pattern 3 for code.

**Q13: You have a 10-camera perception pipeline (each adds 10 ms latency). Total latency is 100 ms. How would you optimize?**
A: (1) Parallelize (run cameras in parallel threads, not sequential). (2) Sample sparsely (process every 2nd frame from some cameras). (3) Use lightweight models (e.g., mobilenet vs. resnet50). (4) Move to GPU. (5) Accept latency; design control to handle it (feedback-based control corrects for stale perception). Most practical: parallelize + accept latency.

**Q14: Implement monitoring to detect if agent's output distribution shifts (e.g., model drift or adversarial input). What metric would you use?**
A: (1) Track mean and variance of action outputs. (2) If deviation exceeds threshold (e.g., 3 sigma), trigger alert. (3) Use simple statistical tests (Kullback-Leibler divergence if you have reference distribution). (4) Log all actions for post-hoc analysis. Example: if gripper force suddenly spikes to 150 N (was always <50 N), that's drift.

**Q15: A multi-agent system has 3 agents competing for gripper. One agent (vision) is high-confidence, another (force) is low-confidence, third (safety) overrides both. Design priority-based arbitration.**
A: Arbitrator checks in order: (1) If safety agent signals abort, output STOP. (2) Else if vision confidence > threshold, use vision. (3) Else if force confidence > threshold, use force. (4) Else use fallback. Implement as priority queue or explicit if/else logic. Agents publish (action, confidence, priority); arbitrator selects highest priority + highest confidence.

---

**Total Questions:** 15  
**Difficulty distribution:** 5 foundational, 5 intermediate, 5 advanced  

---

**Ready to proceed to Chapter 8 (final chapter)?**
