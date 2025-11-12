# Cube Sorter Robot

A pick and place demo built on the Viam platform using computer vision and motion planning to automate a uFactory Lite6 robot arm, vacuum gripper, and RealSense depth camera.

## Features

- Computer vision-based object detection using segmentation
- Coordinate frame transformations (camera frame to world frame)
- Motion planning with constraints for precise arm control
- Vacuum gripper control for pick and place operations
- Interactive user confirmation prompts during operation

## Prerequisites

- Go 1.16 or higher
- Access to a Viam robot instance with:
  - uFactory Lite6 robot arm
  - RealSense depth camera
  - Vacuum gripper
  - Vision segmentation service configured

## Model devrel:cube-sorter:cube-sorter

The business logic service for sorting cubes into bowls, or other arbitrary pick and place tasks.

### Configuration
The following attribute template can be used to configure this model:

```json
{
    "arm_name": <string>,
    "camera_name": <string>,
    "gripper_name": <string>,
    "segmenter_name": <string>,
    "start_pose": <string>,
    "place_pose": <string>,
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `arm_name` | string  | Required  | Name of the arm component to use in motion planning. |
| `camera_name` | string | Required  | Name of the camera component providing point cloud data. |
| `gripper_name` | string | Required  | Name of the gripper component to use in picking. |
| `segementer_name` | string | Required  | Name of the vision service providing point cloud objects. |
| `start_pose` | string | Required  | Name of the `arm-position-saver` switch component providing the start position of the arm. |
| `place_pose` | string | Required  | Name of the `arm-position-saver` switch component providing the placement position for the picked cube. |
| `motion` | string | Optional  | Name of the motion service to use for planning. Defaults to `"builtin"` |

#### Example Configuration

```json
{
  "camera_name": "realsense-cam",
  "gripper_name": "vacuum_gripper",
  "segmenter_name": "segment-detections",
  "start_pose": "home-pose",
  "place_pose": "black-cube-placement",
  "arm_name": "lite6-arm"
}
```

### DoCommand

#### Start

Run through the pick and place routine: detect objects -> get pose for first object -> pick up object -> place in configured pose -> return to start pose

```json
{
  "command": "start"
}
```

#### Reset

Stop any current action, open the gripper, and return to start position.

```json
{
  "command": "reset"
}
```

## Development

### 1. Clone the Repository

```bash
git clone <repository-url>
cd cube-sorter
```

### 2. Install Dependencies

```bash
go mod tidy
```

See the [Developer Guide](./DEVELOPER_GUIDE.md) for more information about building and running the module.


## How It Works

1. **Initialization**: Connects to the Viam robot and initializes all components (arm, camera, gripper, vision service)
2. **Home Position**: Moves the arm to a safe home position
3. **Object Detection**: Uses the camera and vision segmentation service to detect objects
4. **Coordinate Transformation**: Transforms detected object coordinates from camera frame to world frame
5. **Motion Planning**: Plans and executes motion with constraints to approach the object
6. **Pick**: Lowers to the object and activates the vacuum gripper
7. **Place**: Lifts the object and moves to a predefined drop location
8. **Return Home**: Returns the arm to the home position


## Safety Notes

- Ensure the workspace is clear before running
- Emergency stop should be readily accessible on the physical robot

## License

[Apache 2.0](./LICENSE)
