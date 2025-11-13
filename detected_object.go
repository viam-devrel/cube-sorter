package cube_sorter

import (
	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/objectdetection"
)

type DetectedObject struct {
	Label     string
	Object    viz.Object
	Detection objectdetection.Detection
}

func (do DetectedObject) Serialize() map[string]any {
	boundingBox := do.Detection.BoundingBox()
	result := map[string]any{
		"label": do.Label,
		"box": map[string]any{
			"xMin": boundingBox.Min.X,
			"yMin": boundingBox.Min.Y,
			"xMax": boundingBox.Max.X,
			"yMax": boundingBox.Max.Y,
		},
	}
	return result
}
