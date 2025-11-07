package main

import (
	"bufio"
	"context"
	"fmt"
	"os"
	"strings"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

// waitForConfirmation prompts the user for confirmation before proceeding
func waitForConfirmation(prompt string) bool {
	reader := bufio.NewReader(os.Stdin)
	fmt.Printf("%s (yes/no): ", prompt)
	response, err := reader.ReadString('\n')
	if err != nil {
		return false
	}
	response = strings.TrimSpace(strings.ToLower(response))
	return response == "yes" || response == "y"
}

func main() {
	logger := logging.NewLogger("client")

	// Load environment variables from .env file
	if err := loadEnv(".env"); err != nil {
		logger.Warnf("Failed to load .env file: %v", err)
	}

	// Get credentials from environment variables
	robotAddress := os.Getenv("VIAM_ROBOT_ADDRESS")
	entityID := os.Getenv("VIAM_ENTITY_ID")
	apiKey := os.Getenv("VIAM_API_KEY")

	if robotAddress == "" || entityID == "" || apiKey == "" {
		logger.Fatal("Missing required environment variables: VIAM_ROBOT_ADDRESS, VIAM_ENTITY_ID, VIAM_API_KEY")
	}

	machine, err := client.New(
		context.Background(),
		robotAddress,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			entityID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	defer machine.Close(context.Background())
	logger.Info("Resources:")
	logger.Info(machine.ResourceNames())

	// home pose switch
	homePose, err := toggleswitch.FromProvider(machine, "home-pose")
	if err != nil {
		logger.Error(err)
		return
	}

	dropPose, err := toggleswitch.FromProvider(machine, "black-cube-placement")
	if err != nil {
		logger.Error(err)
		return
	}

	// vacuum_gripper
	vacuumGripper, err := gripper.FromProvider(machine, "vacuum_gripper")
	if err != nil {
		logger.Error(err)
		return
	}

	// lite6-arm
	lite6, err := arm.FromProvider(machine, "lite6-arm")
	if err != nil {
		logger.Error(err)
		return
	}

	// realsense-cam
	realsenseCam, err := camera.FromProvider(machine, "realsense-cam")
	if err != nil {
		logger.Error(err)
		return
	}
	realsenseCamReturnValue, err := realsenseCam.Properties(context.Background())
	if err != nil {
		logger.Error(err)
		return
	}
	logger.Infof("realsense-cam Properties return value: %+v", realsenseCamReturnValue)

	// obstacles
	// obstaclesService, err := vision.FromProvider(machine, "obstacles")
	// if err != nil {
	// 	logger.Error(err)
	// 	return
	// }
	segmenter, err := vision.FromProvider(machine, "segment-detections")
	if err != nil {
		logger.Error(err)
		return
	}

	motionService, err := motion.FromProvider(machine, "builtin")
	if err != nil {
		logger.Error(err)
		return
	}

	logger.Info("Going to home position")
	err = homePose.SetPosition(context.Background(), 2, nil)
	if err != nil {
		logger.Error(err)
		return
	}

	time.Sleep(time.Millisecond * 500)

	logger.Info("Getting objects from camera")
	objs, err := segmenter.GetObjectPointClouds(context.Background(), "", nil)
	if err != nil {
		logger.Error(err)
		return
	}

	obj := objs[0]
	objVec := obj.Geometry.Pose().Point()
	logger.Info("Detected object vector: ", objVec)

	objectInCameraFrame := spatialmath.NewPoseFromPoint(objVec)

	cameraFramePose := referenceframe.NewPoseInFrame("realsense-cam", objectInCameraFrame)

	logger.Info("Object in camera frame: ", cameraFramePose.Pose().Point())

	worldFramePose, err := machine.TransformPose(context.Background(), cameraFramePose, "world", nil)

	worldFramePoint := worldFramePose.Pose().Point()
	logger.Info("Object pose in the world: ", worldFramePoint.X, worldFramePoint.Y, worldFramePoint.Z)

	// obstacles := []*referenceframe.GeometriesInFrame{}
	// obstacles = append(obstacles, referenceframe.NewGeometriesInFrame("world", []spatialmath.Geometry{obj.Geometry}))

	// logger.Infof("add object as obstacle %v", obj.Geometry)

	// worldState, err := referenceframe.NewWorldState(obstacles, nil)
	// if err != nil {
	// 	return
	// }

	// md := obj.MetaData()
	// objCenter := md.Center()

	// p := GetApproachPoint(objCenter, 25, &spatialmath.OrientationVectorDegrees{OZ: -1})
	worldFramePoint.Z = 150

	goToPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(worldFramePoint, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: -180}))

	logger.Infof("trying to move to %v", goToPose.Pose())

	// Confirmation before approaching the object
	if !waitForConfirmation("Proceed to approach object?") {
		logger.Info("Operation cancelled by user")
		return
	}

	err = vacuumGripper.Open(context.Background(), nil)
	if err != nil {
		logger.Error(err)
		return
	}

	constraints := motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{
				LineToleranceMm:          5,
				OrientationToleranceDegs: 1,
			},
		},
	}
	_, err = motionService.Move(context.Background(), motion.MoveReq{
		ComponentName: lite6.Name().ShortName(),
		Destination:   goToPose,
		Constraints:   &constraints,
	})

	if err != nil {
		logger.Error(err)
		return
	}

	time.Sleep(time.Second * 1)

	// Confirmation before approaching the object
	if !waitForConfirmation("Proceed to lower to object?") {
		logger.Info("Operation cancelled by user")
		logger.Info("Going to home position")
		err = homePose.SetPosition(context.Background(), 2, nil)
		if err != nil {
			logger.Error(err)
			return
		}
		return
	}

	worldFramePoint.Z = 90
	goToPose = referenceframe.NewPoseInFrame("world", spatialmath.NewPose(worldFramePoint, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: -180}))

	_, err = motionService.Move(context.Background(), motion.MoveReq{
		ComponentName: lite6.Name().ShortName(),
		Destination:   goToPose,
		Constraints:   &constraints,
	})

	if err != nil {
		logger.Error(err)
		return
	}

	time.Sleep(time.Second * 1)

	// Confirmation before grabbing the object
	if !waitForConfirmation("Proceed to grab object with gripper?") {
		logger.Info("Grab operation cancelled by user")
		logger.Info("Going to home position")
		err = homePose.SetPosition(context.Background(), 2, nil)
		if err != nil {
			logger.Error(err)
			return
		}
		return
	}

	_, err = vacuumGripper.Grab(context.Background(), nil)
	if err != nil {
		logger.Error(err)
		return
	}

	time.Sleep(time.Millisecond * 1500)

	worldFramePoint.Z = 200
	goToPose = referenceframe.NewPoseInFrame("world", spatialmath.NewPose(worldFramePoint, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: -180}))

	_, err = motionService.Move(context.Background(), motion.MoveReq{
		ComponentName: lite6.Name().ShortName(),
		Destination:   goToPose,
		Constraints:   &constraints,
	})

	if err != nil {
		logger.Error(err)
		return
	}

	logger.Info("Going to drop position")
	err = dropPose.SetPosition(context.Background(), 2, nil)
	if err != nil {
		logger.Error(err)
		return
	}

	time.Sleep(time.Second * 3)

	err = vacuumGripper.Open(context.Background(), nil)
	if err != nil {
		logger.Error(err)
		return
	}

	time.Sleep(time.Millisecond * 1500)

	logger.Info("Going to home position")
	err = homePose.SetPosition(context.Background(), 2, nil)
	if err != nil {
		logger.Error(err)
		return
	}
}
