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

## Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd cube-sorter
```

### 2. Install Dependencies

```bash
go mod tidy
```

### 3. Configure Environment Variables

Create a `.env` file in the project root with your Viam robot credentials:

```bash
VIAM_ROBOT_ADDRESS=your-robot-address.viam.cloud
VIAM_ENTITY_ID=your-entity-id
VIAM_API_KEY=your-api-key
```

You can find these credentials in your Viam robot's settings page.

**Note:** The `.env` file is ignored by git to keep your credentials secure.

Alternatively, you can set these as system environment variables:

```bash
export VIAM_ROBOT_ADDRESS=your-robot-address.viam.cloud
export VIAM_ENTITY_ID=your-entity-id
export VIAM_API_KEY=your-api-key
```

### 4. Build and Run

```bash
# Build the application
go build

# Run the application
./cube-sorter
```

Or run directly without building:

```bash
go run .
```

## How It Works

1. **Initialization**: Connects to the Viam robot and initializes all components (arm, camera, gripper, vision service)
2. **Home Position**: Moves the arm to a safe home position
3. **Object Detection**: Uses the camera and vision segmentation service to detect objects
4. **Coordinate Transformation**: Transforms detected object coordinates from camera frame to world frame
5. **Motion Planning**: Plans and executes motion with constraints to approach the object
6. **Pick**: Lowers to the object and activates the vacuum gripper
7. **Place**: Lifts the object and moves to a predefined drop location
8. **Return Home**: Returns the arm to the home position

The application includes user confirmation prompts at critical steps for safety.

## Project Structure

- `main.go` - Main application logic and robot control flow
- `utils.go` - Utility functions for environment loading and geometric calculations
- `.env` - Environment variables (not tracked in git)

## Development

### Run Tests

```bash
go test ./...
```

### Update Dependencies

```bash
go mod tidy
```

## Configuration

The application uses several hardcoded parameters that can be adjusted in `main.go`:

- **Z-axis heights**: 150mm (approach), 90mm (pickup), 200mm (lift)
- **Motion constraints**: 5mm linear tolerance, 1° orientation tolerance
- **End effector orientation**: OZ=-1, Theta=-180 (pointing downward)
- **Timing delays**: 500ms-3000ms between operations

## Safety Notes

- The application picks the first detected object only
- User confirmation is required before critical operations
- Ensure the workspace is clear before running
- Emergency stop should be readily accessible on the physical robot

## License

TBD
