package cube_sorter

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var Sorter = resource.NewModel("devrel", "cube-sorter", "cube-sorter")
var constraints = motionplan.Constraints{
	LinearConstraint: []motionplan.LinearConstraint{
		{
			LineToleranceMm:          5,
			OrientationToleranceDegs: 1,
		},
	},
}

func init() {
	resource.RegisterService(generic.API, Sorter,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newSorter,
		},
	)
}

type Config struct {
	Arm       string `json:"arm_name"`
	Cam       string `json:"camera_name"`
	Gripper   string `json:"gripper_name"`
	Segmenter string `json:"segmenter_name"`

	StartPose string `json:"start_pose"`
	PlacePose string `json:"place_pose"`

	Motion string `json:"motion_service,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	deps := []string{cfg.Arm, cfg.Cam, cfg.Gripper, cfg.Segmenter, cfg.StartPose, cfg.PlacePose}

	if cfg.Motion == "" {
		cfg.Motion = motion.Named("builtin").String()
	}
	deps = append(deps, cfg.Motion)

	return deps, nil, nil
}

type sorter struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()

	arm       arm.Arm
	cam       camera.Camera
	gripper   gripper.Gripper
	segmenter vision.Service
	startPose toggleswitch.Switch
	placePose toggleswitch.Switch
	motion    motion.Service
	client    robot.Robot
}

func newSorter(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	if conf.Motion == "" {
		conf.Motion = "builtin"
	}

	return NewSorter(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSorter(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	robotClient, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)

	if err != nil {
		return nil, err
	}

	armDep, err := arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, err
	}

	gripperDep, err := gripper.FromProvider(deps, conf.Gripper)
	if err != nil {
		return nil, err
	}

	cam, err := camera.FromProvider(deps, conf.Cam)
	if err != nil {
		return nil, err
	}

	startPose, err := toggleswitch.FromProvider(deps, conf.StartPose)
	if err != nil {
		return nil, err
	}

	placePose, err := toggleswitch.FromProvider(deps, conf.PlacePose)
	if err != nil {
		return nil, err
	}

	segmenter, err := vision.FromProvider(deps, conf.Segmenter)
	if err != nil {
		return nil, err
	}

	motionSvc, err := motion.FromProvider(deps, conf.Motion)
	if err != nil {
		return nil, err
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &sorter{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,

		client:    robotClient,
		arm:       armDep,
		cam:       cam,
		gripper:   gripperDep,
		segmenter: segmenter,
		motion:    motionSvc,
		startPose: startPose,
		placePose: placePose,
	}
	return s, nil
}

func (s *sorter) Name() resource.Name {
	return s.name
}

func (s *sorter) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "start":
		err := s.Start()
		return map[string]any{"success": err == nil}, err

	case "reset":
		err := s.Reset()
		return map[string]any{"success": err == nil}, err
	}
	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (s *sorter) Start() error {
	ctx := s.cancelCtx

	s.logger.Info("Going to start position")
	err := s.startPose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 500)

	s.logger.Info("Getting objects from camera")
	objs, err := s.segmenter.GetObjectPointClouds(ctx, "", nil)
	if err != nil {
		return err
	}

	if len(objs) == 0 {
		return fmt.Errorf("No objects found to sort")
	}

	waypoints := []any{}

	obj := objs[0]
	objVec := obj.Geometry.Pose().Point()
	s.logger.Info("Detected object at point: ", objVec)

	objInCamFrame := spatialmath.NewPoseFromPoint(objVec)
	camFramePose := referenceframe.NewPoseInFrame(s.cam.Name().ShortName(), objInCamFrame)

	s.logger.Info("Object in camera frame: ", camFramePose.Pose().Point())

	worldFramePose, err := s.client.TransformPose(ctx, camFramePose, "world", nil)
	worldFramePoint := worldFramePose.Pose().Point()
	s.logger.Info("Object pose in the world: ", worldFramePoint.X, worldFramePoint.Y, worldFramePoint.Z)

	// set approach height
	worldFramePoint.Z = 150

	goToPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(worldFramePoint, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: -180}))

	s.logger.Infof("trying to move to %v", goToPose.Pose())

	plan := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{
			s.arm.Name().ShortName(): goToPose,
		},
		referenceframe.FrameSystemInputs{},
	)

	waypoints = append(waypoints, plan.Serialize())

	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return err
	}

	// drop to pick height
	worldFramePoint.Z = 90
	goToPose = referenceframe.NewPoseInFrame("world", spatialmath.NewPose(worldFramePoint, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: -180}))

	_, err = s.motion.Move(context.Background(), motion.MoveReq{
		ComponentName: s.arm.Name().ShortName(),
		Destination:   goToPose,
		Constraints:   &constraints,
		Extra: map[string]any{
			"waypoints": waypoints,
		},
	})

	if err != nil {
		return nil
	}

	time.Sleep(time.Second * 1)

	_, err = s.gripper.Grab(ctx, nil)
	if err != nil {
		return nil
	}

	time.Sleep(time.Millisecond * 250)

	// raise to place movement height
	worldFramePoint.Z = 200
	goToPose = referenceframe.NewPoseInFrame("world", spatialmath.NewPose(worldFramePoint, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: -180}))

	_, err = s.motion.Move(context.Background(), motion.MoveReq{
		ComponentName: s.arm.Name().ShortName(),
		Destination:   goToPose,
		Constraints:   &constraints,
		Extra: map[string]any{
			"waypoints": waypoints,
		},
	})

	if err != nil {
		return err
	}

	s.logger.Info("Going to place position")
	err = s.placePose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 2000)

	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return nil
	}

	time.Sleep(time.Millisecond * 500)

	s.logger.Info("Returning to start position")
	err = s.startPose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	return nil
}

func (s *sorter) Reset() error {
	ctx := s.cancelCtx

	s.logger.Info("Stopping arm")
	err := s.arm.Stop(ctx, nil)
	if err != nil {
		return err
	}

	s.logger.Info("Opening gripper")
	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return err
	}

	s.logger.Info("Returning to start pose")
	err = s.startPose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	return nil
}

func (s *sorter) Close(ctx context.Context) error {
	err := s.client.Close(ctx)
	if err != nil {
		return err
	}

	s.cancelFunc()
	return nil
}
