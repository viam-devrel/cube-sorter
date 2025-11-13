package cube_sorter

import (
	"context"
	"fmt"
	"slices"
	"sync"
	"time"

	"golang.org/x/sync/errgroup"

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
	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/objectdetection"

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

const PICK_HEIGHT = 90

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

	StartPose string            `json:"start_pose"`
	Placement map[string]string `json:"placement"`

	Motion string `json:"motion_service,omitempty"`

	GripperLength float64 `json:"gripper_length,omitempty"`
	CubeHeight    float64 `json:"cube_height,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	deps := []string{cfg.Arm, cfg.Cam, cfg.Gripper, cfg.Segmenter, cfg.StartPose}

	if cfg.Motion == "" {
		cfg.Motion = motion.Named("builtin").String()
	}
	deps = append(deps, cfg.Motion)

	for _, pose := range cfg.Placement {
		deps = append(deps, pose)
	}

	return deps, nil, nil
}

type sorter struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()

	// config
	gripperLength float64
	cubeHeight    float64

	// service deps
	arm        arm.Arm
	cam        camera.Camera
	gripper    gripper.Gripper
	segmenter  vision.Service
	startPose  toggleswitch.Switch
	placePoses map[string]toggleswitch.Switch
	motion     motion.Service
	client     robot.Robot

	// state
	status          string
	detectedObjects []DetectedObject
	mu              sync.RWMutex
}

func newSorter(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	if conf.Motion == "" {
		conf.Motion = "builtin"
	}

	if conf.GripperLength == 0.0 {
		conf.GripperLength = 60.0
	}
	if conf.CubeHeight == 0.0 {
		conf.CubeHeight = 30.0
	}

	return NewSorter(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSorter(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	robotClient, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)

	if err != nil {
		return nil, err
	}

	logger.Info("Following placement poses: ", conf.Placement)

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

	placePoses := map[string]toggleswitch.Switch{}
	for key, pose := range conf.Placement {
		poseSwitch, err := toggleswitch.FromProvider(deps, pose)
		if err != nil {
			return nil, err
		}
		placePoses[key] = poseSwitch
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

		client:        robotClient,
		arm:           armDep,
		cam:           cam,
		gripper:       gripperDep,
		segmenter:     segmenter,
		motion:        motionSvc,
		startPose:     startPose,
		placePoses:    placePoses,
		gripperLength: conf.GripperLength,
		cubeHeight:    conf.CubeHeight,

		status: "idle",
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

	case "get_detected_objects":
		err := s.GetDetectedObjects(ctx)
		serializedObjs := s.serializeDetectedObjects()
		return map[string]any{"success": err == nil, "objects": serializedObjs}, err

	case "pick_object":
		selectedObjectLabel, ok := cmd["label"].(string)
		if !ok {
			return nil, fmt.Errorf("pick_object command requires 'label' string parameter")
		}
		err := s.PickObject(ctx, selectedObjectLabel)
		if err == nil {
			s.removeDetectedObjectFromList(selectedObjectLabel)
		}
		return map[string]any{"success": err == nil}, err

	case "get_status":
		status, err := s.Status()
		serializedObjs := s.serializeDetectedObjects()
		return map[string]any{"success": err == nil, "status": status, "detected_objects": serializedObjs}, err
	}

	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (s *sorter) Start() error {
	ctx := s.cancelCtx

	err := s.GetDetectedObjects(ctx)
	if err != nil {
		return err
	}

	for _, obj := range s.detectedObjects {
		err = s.PickObject(ctx, obj.Label)
		if err != nil {
			return err
		}
		s.removeDetectedObjectFromList(obj.Label)
	}

	s.mu.Lock()
	clear(s.detectedObjects)
	s.mu.Unlock()

	return nil
}

func (s *sorter) GetDetectedObjects(ctx context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	clear(s.detectedObjects)

	s.logger.Info("Going to start position")
	err := s.startPose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 500)

	g, ctx := errgroup.WithContext(ctx)
	results := map[string]any{}

	s.logger.Info("Getting objects from camera")

	g.Go(func() error {
		objs, err := s.segmenter.GetObjectPointClouds(ctx, "", nil)
		if err != nil {
			return err
		}
		results["objects"] = objs
		return nil
	})

	g.Go(func() error {
		dets, err := s.segmenter.DetectionsFromCamera(ctx, "", nil)
		if err != nil {
			return err
		}
		results["detections"] = dets
		return nil
	})

	if err = g.Wait(); err != nil {
		return err
	}

	objs := results["objects"].([]*viz.Object)
	if len(objs) == 0 {
		return fmt.Errorf("No objects found to sort")
	}

	dets := results["detections"].([]objectdetection.Detection)

	for _, obj := range objs {
		label := obj.Geometry.Label()
		for _, det := range dets {
			if det.Label() == label {
				s.detectedObjects = append(s.detectedObjects, DetectedObject{Label: label, Object: *obj, Detection: det})
				break
			}
		}
	}
	s.status = "objects_detected"

	return nil
}

func (s *sorter) PickObject(ctx context.Context, selectedObjectLabel string) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	var selectedObj *viz.Object
	for _, obj := range s.detectedObjects {
		if obj.Label == selectedObjectLabel {
			selectedObj = &obj.Object
			break
		}
	}
	if selectedObj == nil {
		return fmt.Errorf("Unable to find matching object with label %s", selectedObjectLabel)
	}

	objLabel := selectedObj.Geometry.Label()
	armName := s.arm.Name().ShortName()

	objInWorld, err := s.client.TransformPointCloud(ctx, selectedObj, s.cam.Name().ShortName(), "world")
	if err != nil {
		return err
	}
	objMd := objInWorld.MetaData()
	pickPoint := objMd.Center()
	pickPoint.Z = objMd.MaxZ + s.gripperLength

	s.logger.Infof("Pick point: %v", pickPoint)

	pickPoint.Z = s.cubeHeight + s.gripperLength

	// set approach height
	approachPoint := pickPoint
	approachPoint.Z += 100

	currentPose, err := s.motion.GetPose(ctx, armName, "world", nil, nil)
	movingOrientation := spatialmath.OrientationVectorDegrees{
		OZ:    -1,
		Theta: currentPose.Pose().Orientation().OrientationVectorDegrees().Theta,
	}

	approachPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(approachPoint, &movingOrientation))

	s.logger.Infof("trying to move to %v", approachPose.Pose())

	plan := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{
			s.arm.Name().ShortName(): approachPose,
		},
		referenceframe.FrameSystemInputs{},
	)

	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return err
	}

	// drop to pick height
	pickPose := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(pickPoint, &movingOrientation))

	s.status = "picking"

	_, err = s.motion.Move(context.Background(), motion.MoveReq{
		ComponentName: armName,
		Destination:   pickPose,
		Constraints:   &constraints,
		Extra: map[string]any{
			"waypoints": []any{plan.Serialize()},
		},
	})

	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 500)

	_, err = s.gripper.Grab(ctx, nil)
	if err != nil {
		return nil
	}

	time.Sleep(time.Millisecond * 250)

	// raise to place movement height
	pickupPoint := pickPoint
	pickupPoint.Z += 150

	err = s.arm.MoveToPosition(ctx, spatialmath.NewPose(pickupPoint, &movingOrientation), nil)
	if err != nil {
		return err
	}

	s.status = "placing"

	placePose, ok := s.placePoses[objLabel]
	if !ok {
		return fmt.Errorf("Unable to find pose to place %s", objLabel)
	}
	s.logger.Infof("Going to place position for %s", objLabel)
	err = placePose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 1000)

	err = s.gripper.Open(ctx, nil)
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 250)

	s.logger.Info("Returning to start position")
	err = s.startPose.SetPosition(ctx, 2, nil)
	if err != nil {
		return err
	}

	s.status = "idle"

	return nil
}

func (s *sorter) Status() (string, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.status, nil
}

func (s *sorter) Reset() error {
	s.mu.Lock()
	s.status = "resetting"
	s.mu.Unlock()

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

	s.mu.Lock()
	s.status = "idle"
	clear(s.detectedObjects)
	s.mu.Unlock()

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

func (s *sorter) removeDetectedObjectFromList(selectedObjectLabel string) {
	s.mu.Lock()
	defer s.mu.Unlock()

	var objIdx int
	for idx, obj := range s.detectedObjects {
		if obj.Label == selectedObjectLabel {
			objIdx = idx
			break
		}
	}
	s.detectedObjects = slices.Delete(s.detectedObjects, objIdx, objIdx+1)
}

func (s *sorter) serializeDetectedObjects() []any {
	s.mu.RLock()
	defer s.mu.RUnlock()
	serializedObjs := []any{}
	for _, obj := range s.detectedObjects {
		serializedObjs = append(serializedObjs, obj.Serialize())
	}
	return serializedObjs
}
