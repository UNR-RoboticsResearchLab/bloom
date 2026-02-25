# bloom_node - Bloom Robot Web Service Integration

## Overview
ROS2 package providing web service integration, state management,
and behavior coordination for bloom social robot.

## Components

- **WebServiceClient**: HTTP client with connection pooling, thread pooling, retry logic
- **StateManager**: Robot state machine with ROS2 integration
- **BehaviorCoordinator**: Priority-based behavior arbitration with mutual exclusion
- **ConfigurationManager**: Thread-safe runtime configuration

## Building

To build the package, run the following commands from the root of your ROS2 workspace:

```bash
colcon build --packages-select bloom_node
```

## Configuration

The `ConfigurationManager` allows you to set parameters at runtime. You can use ROS2 parameters or a configuration file. For example, to set a parameter via the command line:

```bash
ros2 param set /bloom_node some_parameter value
```

## ROS Interface

The `bloom_node` provides the following ROS2 interfaces:


## Examples
## Troubleshooting