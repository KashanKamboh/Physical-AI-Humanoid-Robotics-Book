---
sidebar_position: 2
---

# Capstone Execution Guide

## Overview

This guide provides detailed instructions for completing the capstone project: "The Autonomous Humanoid". The capstone integrates all modules into a complete system that demonstrates the full pipeline from voice commands to robotic action execution.

## Capstone Submission Checklist

### Technical Requirements
- [ ] Complete ROS 2 workspace with all modules integrated
- [ ] Working voice-to-action pipeline
- [ ] Autonomous navigation system functional
- [ ] Object recognition and manipulation capabilities
- [ ] Simulation environment with humanoid model
- [ ] All 13 chapters implemented with required artifacts

### Documentation Requirements
- [ ] System architecture diagram
- [ ] Integration guide
- [ ] Troubleshooting documentation
- [ ] Performance evaluation report
- [ ] Video demonstration (≤90 seconds)

### Validation Requirements
- [ ] All modules tested independently
- [ ] Integrated system validated in simulation
- [ ] Performance metrics collected
- [ ] Safety checks completed

## Hardware Connection Architecture

### ROS 2 Communication Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Voice Input   │────│   NLP/Planning  │────│  Task Manager   │
│   (Microphone)  │    │    System       │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                       │
                    ┌─────────────────┐    ┌───────────▼───────────┐
                    │  Navigation     │────│   Perception System │
                    │    System       │    │                     │
                    └─────────────────┘    └─────────────────────┘
                                                       │
                    ┌─────────────────┐    ┌───────────▼───────────┐
                    │ Manipulation    │────│   Robot Controller  │
                    │    System       │    │                     │
                    └─────────────────┘    └─────────────────────┘
                                                       │
                    ┌─────────────────┐    ┌───────────▼───────────┐
                    │   Simulation    │◀───│   Hardware Layer    │
                    │   Environment   │    │ (Real/Simulated)    │
                    └─────────────────┘    └─────────────────────┘
```

### Key ROS 2 Topics and Services

**Voice Processing:**
- `/voice/command` (std_msgs/String) - Voice command input
- `/voice/transcript` (std_msgs/String) - Transcription output

**Task Planning:**
- `/task/goal` (move_base_msgs/MoveBaseActionGoal) - Navigation goals
- `/task/object_request` (std_msgs/String) - Object recognition requests

**Navigation:**
- `/move_base/goal` (geometry_msgs/PoseStamped) - Navigation commands
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) - Localization

**Perception:**
- `/camera/rgb/image_raw` (sensor_msgs/Image) - RGB camera feed
- `/camera/depth/image_raw` (sensor_msgs/Image) - Depth camera feed
- `/detected_objects` (vision_msgs/Detection2DArray) - Object detections

**Manipulation:**
- `/arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory) - Arm control
- `/gripper_controller/command` (control_msgs/GripperCommand) - Gripper control

## ROS Topics List

### Core Navigation Topics
- `/cmd_vel` - Velocity commands for base movement
- `/odom` - Odometry data
- `/scan` - Laser scan data
- `/map` - Static map
- `/move_base_simple/goal` - Simple navigation goal

### Perception Topics
- `/camera/color/image_raw` - RGB image data
- `/camera/depth/image_raw` - Depth image data
- `/camera/color/camera_info` - Camera calibration data
- `/yolo/detections` - Object detection results
- `/aruco/poses` - ArUco marker poses

### Manipulation Topics
- `/joint_states` - Current joint positions
- `/arm_controller/command` - Arm joint commands
- `/gripper_controller/command` - Gripper commands
- `/tcp_pose` - Tool center point pose

### System Monitoring Topics
- `/rosout` - System logging
- `/tf` - Transform tree
- `/tf_static` - Static transforms

## Model Asset References

### Robot Models
- **Humanoid Model**: `models/humanoid.urdf.xacro`
- **Base Link**: `base_link` (main reference frame)
- **Camera Link**: `camera_link` (RGB-D sensor)
- **Arm Joints**: `joint_1` through `joint_7` (7-DOF manipulator)
- **End Effector**: `ee_link` (gripper attachment point)

### Environment Models
- **Simulation World**: `worlds/autonomous_humanoid.world`
- **Navigation Map**: `maps/autonomous_humanoid.yaml`
- **Objects**: `models/objects/` (cubes, cylinders, custom objects)

### Sensor Models
- **RGB-D Camera**: Intel Realsense D435 equivalent
- **IMU**: Inertial measurement unit
- **Force/Torque Sensor**: At end-effector for manipulation

## Implementation Steps

### Step 1: Voice Command Integration
1. Set up speech recognition using Whisper or similar
2. Integrate with ROS 2 via a voice node
3. Parse commands and convert to task specifications
4. Test voice recognition accuracy in various environments

### Step 2: Task Planning Pipeline
1. Implement cognitive planning system
2. Create task decomposition algorithms
3. Integrate with ROS action servers
4. Validate task execution sequences

### Step 3: Navigation Integration
1. Configure Nav2 for the humanoid platform
2. Set up costmaps and planners
3. Implement obstacle avoidance
4. Test navigation in simulation environment

### Step 4: Perception System
1. Set up camera calibration
2. Implement object detection pipeline
3. Integrate with ROS 2 vision nodes
4. Test object recognition accuracy

### Step 5: Manipulation System
1. Configure robot arm controllers
2. Implement grasp planning
3. Set up end-effector control
4. Test manipulation in simulation

### Step 6: System Integration
1. Connect all subsystems
2. Implement error handling
3. Test complete pipeline
4. Optimize performance

## Demo Recording Instructions

### Video Requirements
- **Duration**: Maximum 90 seconds
- **Resolution**: Minimum 720p
- **Format**: MP4 or MOV
- **Content**: Complete demonstration of VOICE ⟶ PLAN ⟶ NAVIGATE ⟶ RECOGNIZE OBJECT ⟶ MANIPULATE

### Recording Setup
1. **Camera Position**: Overhead view showing robot and environment
2. **Audio**: Clear recording of voice commands
3. **Screen Capture**: ROS 2 visualization showing system state
4. **Multiple Angles**: Include close-ups of manipulation tasks

### Demo Scenarios
1. **Basic Navigation**: Move to specified location
2. **Object Recognition**: Identify and locate specific object
3. **Manipulation**: Pick up and move object
4. **Complex Task**: Multi-step task combining all capabilities

### Performance Metrics to Capture
- **Command Response Time**: Time from voice input to action initiation
- **Navigation Accuracy**: Position accuracy when reaching goals
- **Object Recognition Rate**: Percentage of correctly identified objects
- **Manipulation Success Rate**: Successful grasp and manipulation attempts
- **System Reliability**: Percentage of tasks completed without errors

## Troubleshooting Guide

### Common Issues and Solutions

**Voice Recognition Problems:**
- Issue: Poor recognition accuracy
- Solution: Check audio input levels, reduce background noise, use better microphone

**Navigation Failures:**
- Issue: Robot getting stuck or taking inefficient paths
- Solution: Check costmap parameters, verify map quality, adjust planner settings

**Perception Issues:**
- Issue: Poor object detection or false positives
- Solution: Adjust detection thresholds, improve lighting, verify camera calibration

**Manipulation Failures:**
- Issue: Failed grasp attempts
- Solution: Improve grasp planning, check gripper calibration, adjust approach angles

### Debugging Commands

**Check ROS 2 System Status:**
```bash
ros2 node list
ros2 topic list
ros2 service list
```

**Monitor System Performance:**
```bash
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot
```

**Check System Logs:**
```bash
ros2 run rqt_console rqt_console
```

## Evaluation Rubric

### Base Book (100 pts)
- [ ] All 13 chapters completed with required content (50 pts)
- [ ] Technical accuracy and APA citations (25 pts)
- [ ] Docusaurus site fully functional (25 pts)

### Agent-based Intelligence (+50 pts)
- [ ] Voice recognition and NLP integration (25 pts)
- [ ] AI-driven task planning (25 pts)

### Personalized Experience (+50 pts)
- [ ] User profile and preference tracking (25 pts)
- [ ] Adaptive content delivery (25 pts)

### Auto-translate URDU (+50 pts)
- [ ] Urdu translation functionality (25 pts)
- [ ] Cultural adaptation of content (25 pts)

### Total Possible Score: 250 Points