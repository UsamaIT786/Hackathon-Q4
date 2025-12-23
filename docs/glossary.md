# Technical Glossary
## AI-Native Physical AI & Humanoid Robotics Textbook

**Purpose:** Consistent terminology index for all readers and RAG systems. Each term includes definition, context, and cross-references.

---

## A

**Actuator**
- Definition: A device that converts electrical, pneumatic, or hydraulic energy into mechanical motion.
- Types: DC motors, stepper motors, servo motors, pneumatic cylinders, hydraulic cylinders.
- Context: Chapter 2
- See also: Sensor, Control loop, Morphology

**Algorithm**
- Definition: Step-by-step procedure for solving a problem or achieving a goal.
- In robotics: Perception algorithms (object detection), control algorithms (PID), planning algorithms (pathfinding).
- Context: Throughout
- See also: Neural network, Policy, Reinforcement learning

**Angle of Attack** (in aerodynamics, relevant for aerial robots)
- Definition: Angle between object surface and incoming air flow.
- Context: PART 5+ (aerial vehicles)

**Autonomous**
- Definition: Capable of operating without human input; self-governing.
- Context: Throughout (e.g., "autonomous robot")
- See also: Teleoperation, Human-in-the-loop

---

## B

**Backlash**
- Definition: Mechanical play or slack in a transmission (e.g., gearbox). Causes delayed response to input change.
- Impact: Causes non-linearity in control; reduces precision.
- Measurement: 1-3° typical in cheap gearboxes; &lt;0.1° in high-quality.
- Context: Chapter 2 (Pitfalls)
- See also: Stiffness, Hysteresis

**Bandwidth** (control)
- Definition: Maximum frequency at which a system can respond to input changes.
- Example: Motor with 100 Hz bandwidth can track commands up to 100 Hz; higher frequencies are ignored.
- Context: Chapter 2
- See also: Latency, Control loop frequency

**Behavior**
- Definition: Observable actions of an agent in response to state and goals.
- In robotics: "Wall-following behavior," "grasping behavior"
- Context: Throughout
- See also: Policy, Reactive, Deliberative

**Behavioral Cloning**
- Definition: Learning by imitating expert demonstrations; supervised learning from expert trajectories.
- Method: Collect (state, action) pairs from expert; train neural network to predict action given state.
- Advantage: No reward function needed.
- Limitation: Only learns to mimic expert; doesn't generalize beyond distribution.
- Context: PART 5 (imitation learning)
- See also: Imitation learning, Reinforcement learning

**Bipedal**
- Definition: Moving on two legs.
- Examples: Humanoid robots, humans.
- Context: PART 6 (humanoid robots)
- See also: Quadruped, Locomotion

**Blind Spot**
- Definition: Region not visible to a sensor due to geometry or occlusion.
- Example: A single forward-facing camera cannot see behind the robot; lidar has small vertical gaps.
- Context: Chapter 3 (sensor design)
- See also: Field of view, Sensor fusion

---

## C

**Calibration**
- Definition: Measuring and adjusting a system to match ground truth; eliminating systematic errors.
- Example: Camera calibration (measuring lens distortion), encoder calibration (measuring zero position).
- Context: Throughout
- See also: System identification

**Closed-loop Control**
- Definition: Control system that uses feedback to adjust behavior; measures output, compares to goal, adjusts input.
- Formula: error = goal - actual; command = f(error)
- Opposite: Open-loop (no feedback)
- Context: Chapter 2
- See also: Feedback, PID control, Stable

**Compliance** (mechanical)
- Definition: Property of a system to yield under force; opposite of stiffness.
- Compliant systems: Pneumatic actuators, series-elastic actuators, soft manipulators.
- Advantage: Safe around humans (yield on contact).
- Disadvantage: Less precise (harder to control position).
- Context: Chapter 2
- See also: Stiffness, Force control

**Computation Latency**
- Definition: Time required for a processor to compute a decision/control command.
- Typical: 1-50ms for basic algorithms; 100-500ms for deep learning.
- Context: Chapter 1 (latency budget)
- See also: Latency, Perception latency, System latency

**Control Architecture**
- Definition: Overall design pattern for how sensors feed into control decisions, which generate actions.
- Types: Reactive, deliberative, hybrid.
- Context: Chapter 2 (section 3.5)
- See also: Reactive, Deliberative, Hybrid

**Control Loop**
- Definition: Feedback cycle: sense → decide → act → measure result → repeat.
- Frequency: 10-1000 Hz (depends on task).
- Context: Chapter 1 (core concept); Chapter 2 (detailed)
- See also: Perception-decision-action loop, Closed-loop control

**Control Loop Frequency** (or Update Rate)
- Definition: How many times per second the control loop executes.
- Examples: 10 Hz (slow, high-level planning), 100 Hz (standard robot control), 1000 Hz (fast actuator control).
- Impact: Higher frequency allows faster response but requires lower latency.
- Context: Chapter 2
- See also: Latency, Bandwidth

**CLIP** (Contrastive Language-Image Pre-training)
- Definition: Vision-language model that learns shared representation of images and text.
- Capability: Can classify any object without retraining (zero-shot learning).
- Context: PART 5 (vision-language models)
- See also: Vision-language model, Foundation model

---

## D

**Deliberative**
- Definition: Control architecture that plans ahead using a world model.
- Characteristics: Slower (50-500ms), more flexible, requires state estimation.
- Context: Chapter 2 (section 3.5.2)
- See also: Reactive, Hybrid, Planning

**Degrees of Freedom (DOF)**
- Definition: Number of independent motions a robot can perform.
- Examples: 2D wheeled robot (3 DOF: x, y, θ), 6-DOF arm, 7-DOF redundant arm.
- Context: Chapter 2 (section 3.3)
- See also: Kinematics, Morphology, Redundancy

**Demonstration** (in learning)
- Definition: Example trajectory showing desired behavior; used for imitation learning.
- Format: Sequence of (state, action) pairs collected from human expert or simulator.
- Context: PART 5 (imitation learning)
- See also: Behavioral cloning, Imitation learning

**Deterministic**
- Definition: System behaves identically given same inputs; no randomness.
- Opposite: Stochastic (behavior varies).
- Context: Throughout (especially documentation design)
- See also: Stochastic, Reproducibility

**Digital Twin**
- Definition: Software model of a physical robot system; includes geometry, physics, sensors, actuators.
- Purpose: Enable training and validation before deployment to real hardware.
- Fidelity: Can range from simple geometric models to photorealistic simulations.
- Context: Chapter 3 (section 3.1.2)
- See also: Simulation, Gazebo, Isaac Sim

**Domain Randomization**
- Definition: Deliberately varying simulation parameters during training to increase robustness.
- Examples: Randomize friction (0.2-0.8), lighting (0.5-1.5× brightness), sensor noise (0-5%).
- Purpose: Train policy that generalizes to real hardware (which has different parameters).
- Impact: Typically reduces sim-to-real gap by 10-20%.
- Context: Chapter 3 (section 3.3)
- See also: Sim-to-real transfer, Reality gap, Curriculum learning

**DOF**
- See: Degrees of freedom

**Dynamics**
- Definition: How a system's state changes over time given forces/torques applied.
- Forward dynamics: Given forces, predict resulting motion.
- Inverse dynamics: Given desired motion, compute required forces.
- Context: Chapter 2 (kinematics and dynamics)
- See also: Kinematics, Physics, Control

---

## E

**Embodied Cognition**
- Definition: Principle that intelligence emerges from an agent's interaction with its physical environment.
- Key insight: Morphology, sensors, and actuators fundamentally shape what intelligence is possible.
- Context: Chapter 1 (section 3.2)
- See also: Morphology, Physical AI, Embodied intelligence

**Embodied Intelligence**
- Definition: Intelligence that arises from tight coupling between perception, reasoning, and action.
- Opposite: Disembodied intelligence (pure computation, no physical instantiation).
- Context: Throughout
- See also: Embodied cognition, Physical AI

**Encoder** (position sensor)
- Definition: Sensor that measures joint angle or linear position.
- Types: Rotary encoders (measure angle), linear encoders (measure distance).
- Characteristics: Low latency (`<1ms`), high precision, low cost.
- Context: Chapter 2 (sensors)
- See also: Sensor, Proprioceptive, Feedback

**End-Effector**
- Definition: The "tool" at the end of a robot arm; typically a gripper, but can be any end-use device.
- Context: Chapter 2 (kinematics)
- See also: Gripper, Manipulation, Workspace

**Episode** (in learning)
- Definition: One complete run of a task from start to finish.
- Example: Robot attempts to grasp an object; success or failure = end of episode.
- Context: PART 5+ (reinforcement learning)
- See also: Trajectory, Experience, Training

---

## F

**Field of View (FOV)**
- Definition: Angular extent of space that a sensor can observe.
- Examples: RGB camera ~60° horizontal, fish-eye camera ~180-200°, 2D lidar ~270° horizontal.
- Impact: Larger FOV covers more space but may sacrifice resolution.
- Context: Chapter 2 (sensor design), Chapter 3 (sensor gaps)
- See also: Blind spot, Sensor fusion, Coverage

**Force-Torque Sensor** (F/T sensor, load cell)
- Definition: Sensor that measures forces and torques applied to a point.
- Typical output: 6D (Fx, Fy, Fz, Mx, My, Mz).
- Use case: End-effector force feedback for compliant control, grasp quality assessment.
- Context: Chapter 2 (sensors)
- See also: Tactile sensor, Proprioceptive, Feedback control

**Foundation Model**
- Definition: Large pre-trained neural network (billions of parameters) trained on diverse data.
- Examples: GPT-4, CLIP, Vision Transformers.
- Advantage: Can be fine-tuned for new tasks with minimal data.
- Context: PART 5 (vision-language models)
- See also: Transfer learning, Fine-tuning, Pre-trained

**Forward Kinematics**
- Definition: Computing end-effector position given joint angles.
- Input: Joint angles (θ₁, θ₂, ..., θₙ)
- Output: End-effector position (x, y, z) and orientation
- Context: Chapter 2 (kinematics)
- See also: Inverse kinematics

**Friction**
- Definition: Resistance to motion between two surfaces in contact.
- Models: Coulomb friction (static ≠ kinetic), viscous friction (velocity-dependent), stick-slip.
- Impact: Key parameter affecting robot motion; often a major source of sim-to-real discrepancy.
- Context: Chapter 2, Chapter 3 (reality gap)
- See also: Contact dynamics, Coefficient of friction

---

## G

**Gait** (in locomotion)
- Definition: Pattern of movement for multi-legged robots.
- Examples: Trotting (4 legs), galloping (4 legs), hexapod waving gate.
- Context: PART 6+ (humanoid locomotion)
- See also: Bipedal, Locomotion, Balance

**Gazebo**
- Definition: Open-source physics simulator commonly used for robotics.
- Capabilities: Rigid-body dynamics, sensor simulation, plugin architecture.
- Limitation: Poor at soft-body simulation (cloth, deformable objects).
- Context: Chapter 3 (simulation), PART 3 (deep dive)
- See also: Physics engine, Digital twin, NVIDIA Isaac

**Generalization**
- Definition: Ability of a trained model to perform well on data/conditions not seen during training.
- Goal: High generalization across real-world variation.
- Challenge: Neural networks often memorize training distribution; poor generalization.
- Solutions: Domain randomization, transfer learning, regularization.
- Context: Throughout (especially PART 5+)
- See also: Overfitting, Domain randomization, Transfer learning

**Grasping**
- Definition: Act of gripping and holding an object with a gripper or hand.
- Key challenge: Must determine grasp point, apply appropriate force, avoid slipping.
- Context: PART 4+ (as example task)
- See also: Manipulation, Gripper, Dexterity

**Gripper**
- Definition: End-effector for grasping objects.
- Types: Two-finger parallel gripper, three-finger, five-finger dexterous hand.
- Context: Chapter 2 (actuators), Chapter 3 (as example)
- See also: End-effector, Grasping, Dexterity

---

## H

**Hardware-Software Co-Design**
- Definition: Designing hardware and software simultaneously, each informing the other.
- Principle: Hardware constraints (latency, power, actuator bandwidth) directly constrain algorithm choice.
- Opposite: Hardware-first then software (leads to mismatch).
- Context: Chapter 1 (key distinction), Chapter 2 (design patterns)
- See also: Embodied cognition, Morphology

**Hybrid Architecture**
- Definition: Control system combining reactive and deliberative components.
- Pattern: Deliberative high-level planner + reactive safety layer.
- Advantage: Combines flexibility of planning with speed of reaction.
- Context: Chapter 2 (section 3.5.3)
- See also: Reactive, Deliberative, Safety layer

**Humanoid**
- Definition: Robot with human-like form; typically bipedal, with arms and hands.
- Examples: Boston Dynamics Atlas, Tesla Optimus, iCub.
- Context: PART 6+ (humanoid robots)
- See also: Bipedal, Morphology, Embodied

**Hyperparameter**
- Definition: Parameter of a learning algorithm that is set before training (not learned).
- Examples: Learning rate, discount factor, network architecture.
- Impact: Tuning hyperparameters is critical for training success.
- Context: PART 5+ (reinforcement learning)
- See also: Parameter, Tuning, Training

---

## I

**IMU** (Inertial Measurement Unit)
- Definition: Sensor that measures acceleration and angular velocity.
- Typical output: 6D (ax, ay, az, ωx, ωy, ωz).
- Use case: Orientation estimation, impact detection, balance feedback.
- Limitation: Drifts over time (gyro bias accumulates).
- Context: Chapter 2 (sensors)
- See also: Proprioceptive, Sensor fusion, Orientation estimation

**Imitation Learning**
- Definition: Learning by observing and copying expert behavior.
- Method: Collect expert demonstrations; train neural network to predict expert actions.
- Advantage: No reward function needed; faster than pure RL.
- Limitation: Only learns distribution of expert; cannot exceed expert performance.
- Context: PART 5 (section on learning from demonstrations)
- See also: Behavioral cloning, Reinforcement learning, Transfer learning

**Inference**
- Definition: Running a trained neural network on new data to make predictions.
- Speed: Critical for real-time robotics; must be fast enough to fit control loop latency budget.
- Optimization: Quantization, distillation, edge-optimized architectures.
- Context: Throughout (especially PART 4)
- See also: Neural network, Training, Optimization

**Inverse Kinematics (IK)**
- Definition: Computing joint angles needed to reach a desired end-effector position.
- Challenge: Multiple solutions possible (redundancy); some solutions may be unreachable.
- Methods: Analytical (simple but only works for simple arms), numerical (general but slow), learned (fast but requires training).
- Context: Chapter 2 (section 3.6)
- See also: Forward kinematics, Redundancy, Motion planning

**Isaac Sim** (NVIDIA Isaac)
- Definition: Photorealistic physics simulator for robotics, integrated with PyTorch.
- Advantages: High fidelity, GPU-accelerated, easy integration with learning frameworks.
- Context: PART 4 (deep dive)
- See also: Gazebo, Physics engine, Digital twin

---

## J

**Jitter**
- Definition: Unwanted variation in timing of a system.
- Impact: Network jitter (variable communication delays) causes control instability.
- Context: Chapter 1 (latency), Chapter 3 (reality gap)
- See also: Latency, Real-time, Determinism

---

## K

**Kinematics**
- Definition: Study of motion without considering forces.
- Includes: Forward kinematics (position from angles), inverse kinematics (angles from position).
- Context: Chapter 2 (section 3.6)
- See also: Dynamics, Forward kinematics, Inverse kinematics

---

## L

**Latency**
- Definition: Time delay between an event and system response to it.
- Types: Sensor latency (time to measure), compute latency (time to decide), actuator latency (time to respond).
- Impact: High latency causes system instability and slow response; critical constraint in robotics.
- Budget: Total end-to-end latency must be much less than control period.
- Context: Chapter 1 (core concept), Chapter 2 (analysis), Chapter 3 (reality gap)
- See also: Control loop frequency, Throughput, Real-time

**Lidar** (Light Detection and Ranging)
- Definition: Sensor that measures distance by emitting laser light and timing reflections.
- Output: 3D point cloud (or 2D points for 2D lidar).
- Advantage: Works in darkness, gives 3D geometry, robust to lighting changes.
- Limitation: Sparse (not all points), expensive, consumes power.
- Context: Chapter 2 (sensors)
- See also: Range sensor, Point cloud, 3D perception

**Locomotion**
- Definition: How a robot moves through space.
- Types: Wheeled, legged, tracked, flying, swimming.
- Context: PART 6+ (humanoid locomotion)
- See also: Gait, Balance, Dynamics

---

## M

**Manipulation**
- Definition: Interacting with objects; typically grasping, moving, assembling.
- Key challenges: Grasp stability, force control, precision.
- Context: PART 4+ (as major application area)
- See also: Grasping, End-effector, Dexterity

**Morphology**
- Definition: Physical form, shape, and structure of a robot.
- Impact: Directly constrains what behaviors are learnable; shapes intelligence.
- Principle: Embodied cognition: morphology + environment → intelligence.
- Context: Chapter 1 (embodied cognition), Chapter 2 (design), Chapter 3 (morphological computation)
- See also: Embodied cognition, Degrees of freedom, Hardware-software co-design

**Motion Planning**
- Definition: Computing a collision-free path from current configuration to goal.
- Methods: RRT (sampling-based), grid search (discrete), trajectory optimization (continuous).
- Context: Chapter 2 (section 3.6)
- See also: Path planning, Inverse kinematics, Collision detection

---

## N

**Neural Network**
- Definition: Computational model inspired by biological neurons; learns functions from data.
- Types: Feedforward, convolutional (CNN), recurrent (RNN/LSTM), transformer.
- Use in robotics: Perception (CNNs for vision), control (feedforward for policies), reasoning (transformers for planning).
- Context: Throughout (especially PART 5+)
- See also: Deep learning, Learning, AI

**Noise**
- Definition: Unwanted variation or uncertainty in measurements.
- Types: Thermal noise (random), quantization noise (discrete levels), outliers (rare bad measurements).
- Handling: Filtering (Kalman filter), averaging, outlier rejection.
- Context: Chapter 2 (sensors), Chapter 3 (reality gap)
- See also: Uncertainty, Signal processing, Robustness

---

## O

**Object Detection**
- Definition: Computer vision task of locating and classifying objects in images.
- Output: Bounding box (x, y, width, height) + class label + confidence score.
- Models: YOLO, Faster R-CNN, Detectron2.
- Context: PART 4 (perception)
- See also: Vision, Semantics, Deep learning

**Observability** (partial observability)
- Definition: Degree to which system state can be inferred from observations.
- Full observability: All state variables directly measured.
- Partial observability: Some state hidden; must be inferred.
- Challenge: Real robots have partial observability; must use filtering/estimation.
- Context: Chapter 1 (constraint), Chapter 2 (state estimation)
- See also: Hidden state, State estimation, Sensor fusion

**Optical Flow**
- Definition: Apparent motion of brightness patterns in image sequences.
- Use: Can estimate robot motion from video; enables visual odometry.
- Context: PART 4 (vision)
- See also: Visual odometry, Ego-motion, Computer vision

---

## P

**Path Planning**
- Definition: Computing a collision-free path for a robot to follow.
- Distinction from motion planning: Path = geometric, motion = with timing and dynamics.
- Algorithms: RRT, Dijkstra, A*.
- Context: Chapter 2
- See also: Motion planning, Collision avoidance, Navigation

**PID Control**
- Definition: Feedback control law with three terms: proportional, integral, derivative.
- Formula: u = Kp·e + Ki∫e dt + Kd·de/dt
- Use: Standard for joint-level control in robots.
- Context: Chapter 2 (section 3.4.3)
- See also: Feedback control, Stability, Control law

**Physics Engine**
- Definition: Software that simulates physical interactions (gravity, contact, friction).
- Examples: Gazebo (ODE), NVIDIA Isaac (PhysX), PyBullet.
- Tradeoff: Speed vs. accuracy; faster engines less accurate.
- Context: Chapter 3 (simulation)
- See also: Digital twin, Simulation, Dynamics

**Policy** (in learning)
- Definition: Mapping from state to action; the "learned behavior."
- Representation: Neural network, decision tree, lookup table.
- Training: Reinforcement learning or imitation learning.
- Deployment: Run policy on real robot to perform task.
- Context: PART 5+ (learning)
- See also: Behavior, Control law, Reinforcement learning

**Proprioception**
- Definition: Sensing one's own internal state (joint angles, orientation, velocity).
- Sensors: Encoders, IMU, force-torque sensors.
- Importance: Essential for control feedback; often overlooked.
- Context: Chapter 2 (sensors)
- See also: Exteroception, Sensor, Feedback

---

## Q

**QoS** (Quality of Service)
- Definition: Guarantees or expectations about communication performance.
- Parameters: Latency, jitter, bandwidth, reliability.
- Robotics context: ROS 2 allows tuning QoS for different priorities (real-time vs. best-effort).
- Context: PART 2 (ROS 2)
- See also: Communication, Middleware, Real-time

---

## R

**RAG** (Retrieval-Augmented Generation)
- Definition: AI technique of retrieving relevant documents, then using them to generate responses.
- Use case: Textbook optimization for chatbot interaction.
- Context: Throughout (documentation designed for RAG)
- See also: Semantic search, Knowledge retrieval, Chatbot

**Reality Gap**
- Definition: Difference in behavior between simulated and real robots.
- Sources: Physics inaccuracy, sensor modeling errors, latency differences.
- Magnitude: Typically 5-30% performance drop.
- Solutions: Domain randomization, system identification, real-world fine-tuning.
- Context: Chapter 3 (core topic)
- See also: Sim-to-real transfer, Domain randomization, Validation

**Real-Time**
- Definition: System that must respond within guaranteed time bounds.
- Hard real-time: Must never miss deadline (safety-critical).
- Soft real-time: Prefer to meet deadline but can tolerate occasional misses.
- Robotics: Most robot control is soft real-time (occasional deadline miss is OK).
- Context: Chapter 2, Chapter 3
- See also: Determinism, Latency, Jitter

**Reactive**
- Definition: Control architecture with direct sensor-to-action mapping; no planning or memory.
- Characteristics: Ultra-low latency (&lt;10ms), simple, limited flexibility.
- Use case: Reflexes (obstacle avoidance, emergency stop).
- Context: Chapter 2 (section 3.5.1)
- See also: Deliberative, Hybrid, Control architecture

**Redundancy** (mechanical)
- Definition: Having more degrees of freedom than minimum needed for task.
- Example: 7-DOF arm (can reach any point/orientation with infinite configurations).
- Advantage: Can avoid obstacles, optimize for comfort/power.
- Disadvantage: Computational complexity of inverse kinematics.
- Context: Chapter 2 (kinematics)
- See also: Degrees of freedom, Inverse kinematics

**Reinforcement Learning (RL)**
- Definition: Learning by interacting with environment and receiving reward signals.
- Methods: Q-learning, policy gradient, actor-critic.
- Advantage: Can learn novel behaviors not in training data.
- Disadvantage: Requires many interactions; sample-inefficient.
- Context: PART 5+ (learning)
- See also: Policy, Reward, Training

**Reward Function**
- Definition: Scalar score indicating quality of action/outcome; guides reinforcement learning.
- Design: Critical; poor reward function leads to unintended behaviors.
- Example: For grasping: +1 if grasp stable for 2s, -1 if object drops, -0.01 per time step (encourage speed).
- Context: PART 5 (RL)
- See also: Objective function, Loss function, Reinforcement learning

**Robot**
- Definition: Physical agent capable of sensing, reasoning, and acting in the world.
- Distinction from other machines: Robots are autonomous (not just scripted).
- Context: Throughout
- See also: Autonomous, Agent, Physical AI

**ROS 2** (Robot Operating System 2)
- Definition: Middleware framework for building distributed robot software.
- Components: Publisher/subscriber messaging, services, parameters, QoS.
- Purpose: Abstracts hardware; enables modular software development.
- Context: PART 2 (deep dive)
- See also: Middleware, Distributed systems, Hardware abstraction

---

## S

**Safety**
- Definition: Property that system does not cause harm to humans or itself.
- In learning: Challenge because neural networks are opaque; hard to verify safety properties.
- Solutions: Analytical safety layer, human-in-the-loop, constraint verification.
- Context: Throughout (especially Chapter 1, Chapter 2 pitfalls, Chapter 3 validation)
- See also: Verification, Reliability, Constraint

**Semantic Segmentation**
- Definition: Computer vision task of labeling each pixel with a class.
- Output: Dense pixel-wise predictions (not just bounding boxes).
- Use case: Understanding scene structure (sky, ground, obstacles).
- Context: PART 4 (perception)
- See also: Object detection, Vision, Deep learning

**Semantics** (in language and vision)
- Definition: Meaning; what things refer to.
- Language semantics: What words/sentences mean.
- Visual semantics: What objects/scenes are in an image.
- Context: PART 5+ (vision-language models)
- See also: Grounding, Vision-language, Language understanding

**Sensor**
- Definition: Device that measures environmental state or internal state.
- Types: Exteroceptive (environment) vs. proprioceptive (self).
- Characteristics: Latency, accuracy, noise, FOV, power consumption.
- Context: Chapter 2 (section 3.1)
- See also: Actuator, Perception, Proprioceptive

**Sensor Fusion**
- Definition: Combining measurements from multiple sensors to improve accuracy/robustness.
- Example: Using encoder + IMU + visual odometry together to estimate robot position (better than any single sensor).
- Method: Kalman filter, particle filter, neural network.
- Context: Chapter 2 (design), Chapter 3 (robustness)
- See also: Filtering, Robustness, Redundancy

**Sensorimotor**
- Definition: Integration of sensing and motor control; perception-action coupling.
- Context: Chapter 1 (embodied cognition)
- See also: Embodied, Perception-action

**Sim-to-Real Transfer** (or Sim-to-Real)
- Definition: Process of training in simulation and deploying to real hardware.
- Challenge: Reality gap causes performance drop.
- Solutions: Domain randomization, system identification, real-world fine-tuning.
- Context: Chapter 3 (core topic)
- See also: Reality gap, Domain randomization, Validation

**Simulation** (in robotics)
- Definition: Software model of a physical robot system; enables testing without hardware.
- Components: Geometry (CAD), physics engine (dynamics), sensors, actuators.
- Advantages: Cost, safety, parallelization, data generation.
- Limitations: Approximate; always has reality gap.
- Context: Chapter 3 (section 3.1)
- See also: Digital twin, Physics engine, Gazebo, Isaac Sim

**SLAM** (Simultaneous Localization and Mapping)
- Definition: Algorithm that builds a map while localizing within it.
- Use case: Mobile robots exploring unknown environments.
- Methods: EKF-SLAM, GraphSLAM, visual SLAM.
- Context: PART 2+ (navigation)
- See also: Localization, Mapping, Navigation

**Stability** (control)
- Definition: System does not diverge; bounded error remains bounded.
- Criterion: All eigenvalues of closed-loop system have negative real part.
- Importance: Unstable system diverges or oscillates uncontrollably.
- Context: Chapter 2 (control)
- See also: Feedback control, Eigenvalue, Convergence

**State**
- Definition: Complete description of a system at a point in time.
- Example: Robot state = (position, velocity, gripper_state).
- Notation: Usually vector **x** or s.
- Context: Throughout
- See also: Observable state, Hidden state, Observation

**State Estimation**
- Definition: Inferring hidden state from noisy observations.
- Method: Kalman filter (linear), extended Kalman filter (nonlinear), particle filter (general).
- Importance: Real sensors are noisy; estimation combines multiple measurements.
- Context: Chapter 2
- See also: Filtering, Sensor fusion, Observability

**Stiffness** (mechanical)
- Definition: Resistance to deformation under force.
- Opposite: Compliance (ability to yield).
- Impact: Stiff manipulators are precise but unsafe around humans; compliant are safe but less precise.
- Context: Chapter 2
- See also: Compliance, Force control

**Stochastic**
- Definition: Random; behavior varies even with same inputs.
- Opposite: Deterministic.
- Use in robotics: Policies with exploration randomness; domain randomization.
- Context: Throughout
- See also: Deterministic, Probability, Randomness

**System Identification**
- Definition: Measuring real-world parameters to improve simulation accuracy.
- Methods: Least squares fitting, Kalman filter, neural networks.
- Purpose: Reduce reality gap by making simulator match real system.
- Context: Chapter 3 (section 3.4)
- See also: Calibration, Parameter estimation, Reality gap

---

## T

**Tactile Sensor**
- Definition: Sensor that measures touch, pressure, or slip.
- Use case: Gripper feedback (is object secure?), impact detection.
- Context: Chapter 2 (sensors)
- See also: Force-torque sensor, Proprioceptive, Contact

**Torque**
- Definition: Rotational force; tendency to cause rotation.
- Formula: τ = r × F (cross product of radius and force).
- Unit: Newton-meter (N⋅m).
- Context: Chapter 2 (actuators, control)
- See also: Force, Stiffness, Dynamics

**Trajectory**
- Definition: Sequence of states over time; path through state space.
- In robotics: Complete episode from start to goal (e.g., picking up an object).
- Context: Throughout
- See also: Path, Episode, Motion planning

**Transfer Learning**
- Definition: Using knowledge from one task to improve learning on a different but related task.
- Example: Pre-trained vision model on ImageNet → fine-tune on robot vision task.
- Advantage: Reduces need for labeled data and training time.
- Context: PART 5 (learning)
- See also: Fine-tuning, Foundation model, Generalization

**Transformer**
- Definition: Neural network architecture using self-attention; excellent for sequences and multimodal data.
- Advantage: Can attend to any part of input; parallelizable; good for language and vision.
- Context: PART 5 (vision-language models)
- See also: Attention, Neural network, Vision-language

---

## U

**Uncertainty**
- Definition: Lack of perfect knowledge about state or outcomes.
- Types: Aleatoric (irreducible randomness), epistemic (can be reduced with more data).
- Handling: Probabilistic models, uncertainty estimation, robust control.
- Context: Chapter 1 (constraint), Chapter 3 (reality gap)
- See also: Noise, Randomness, Robustness

**Underdetermined**
- Definition: System has more unknowns than equations; multiple solutions.
- Example: Inverse kinematics with redundant arm (7 equations, ∞ solutions).
- Context: Chapter 2 (kinematics)
- See also: Redundancy, Inverse kinematics

---

## V

**Validation**
- Definition: Testing that a system meets requirements.
- In sim-to-real: Testing policies on real hardware to verify performance.
- Opposite: Verification (checking that system was built correctly).
- Context: Chapter 3 (section on validation and deployment)
- See also: Testing, Verification, Deployment

**Velocity** (linear and angular)
- Definition: Rate of change of position (linear) or orientation (angular).
- Units: m/s (linear), rad/s (angular).
- Importance: Fundamental for motion control and safety.
- Context: Chapter 2, Chapter 3
- See also: Acceleration, Kinematics, Dynamics

**Vision** (computer vision)
- Definition: Extracting information from images/video.
- Tasks: Object detection, segmentation, pose estimation, optical flow.
- Context: PART 4 (perception), PART 5 (vision-language models)
- See also: Camera, Deep learning, Perception

**Vision-Language Model (VLM)**
- Definition: Neural network that understands both images and text.
- Examples: CLIP, GPT-4V, Flamingo.
- Capability: Describe images in text; answer questions about images; zero-shot classification.
- Context: PART 5 (section on VLMs)
- See also: Foundation model, Transfer learning, Multimodal

**Visual Servoing**
- Definition: Using camera feedback to control robot motion (move robot so target stays in desired image location).
- Advantage: Robust to small modeling errors.
- Disadvantage: Requires fast camera update rates; sensitive to occlusion.
- Context: PART 4 (control)
- See also: Feedback control, Vision, Control

---

## W

**Workspace**
- Definition: Set of all positions/orientations the end-effector can reach.
- Impact: Larger workspace = more flexibility; typically requires more DOF.
- Context: Chapter 2 (design)
- See also: Degrees of freedom, Reachability, Kinematics

---

## X-Z

**Zero-Shot Learning**
- Definition: Performing a task without seeing any examples of that specific task.
- Example: CLIP can classify objects it never saw during training (generalization).
- Advantage: Don't need to train on every possible class.
- Context: PART 5 (foundation models)
- See also: Transfer learning, Generalization, Few-shot learning

---

## Acronyms and Abbreviations

| Acronym | Expansion | Context |
|---------|-----------|---------|
| **AI** | Artificial Intelligence | Throughout |
| **CAD** | Computer-Aided Design | Chapter 3 (simulation setup) |
| **CNN** | Convolutional Neural Network | PART 4+ (vision) |
| **DOF** | Degrees of Freedom | Chapter 2 |
| **DQN** | Deep Q-Network | PART 5 (RL) |
| **EKF** | Extended Kalman Filter | PART 2+ (filtering) |
| **FOV** | Field of View | Chapter 2 |
| **F/T** | Force/Torque | Chapter 2 |
| **IK** | Inverse Kinematics | Chapter 2 |
| **IMU** | Inertial Measurement Unit | Chapter 2 |
| **LLM** | Large Language Model | PART 5, 6 (language) |
| **ML** | Machine Learning | Throughout |
| **MPC** | Model Predictive Control | PART 2+ (control) |
| **NN** | Neural Network | PART 5+ |
| **PID** | Proportional-Integral-Derivative | Chapter 2 |
| **PPO** | Proximal Policy Optimization | PART 5 (RL) |
| **RL** | Reinforcement Learning | PART 5+ |
| **ROS** | Robot Operating System | PART 2 |
| **SLAM** | Simultaneous Localization and Mapping | PART 2+ |
| **VLM** | Vision-Language Model | PART 5 |

---

**Glossary Status:** Complete for PART 1

**Last Updated:** 2025-12-21

**Next Update:** As additional PARTS are completed, add terminology from those chapters.

