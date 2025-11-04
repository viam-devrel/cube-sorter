package main

import (
	"bufio"
	"math"
	"os"
	"strings"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// loadEnv reads a .env file and sets environment variables
func loadEnv(filename string) error {
	file, err := os.Open(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())

		// Skip empty lines and comments
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}

		// Split on first '=' sign
		parts := strings.SplitN(line, "=", 2)
		if len(parts) == 2 {
			key := strings.TrimSpace(parts[0])
			value := strings.TrimSpace(parts[1])
			os.Setenv(key, value)
		}
	}

	return scanner.Err()
}

func GetApproachPoint(p r3.Vector, deltaLinear float64, o *spatialmath.OrientationVectorDegrees) r3.Vector {
	d := math.Pow((o.OX*o.OX)+(o.OY*o.OY)+(o.OZ*o.OZ), .5)

	xLinear := (o.OX * deltaLinear / d)
	yLinear := (o.OY * deltaLinear / d)
	zLinear := (o.OZ * deltaLinear / d)

	approachPoint := r3.Vector{
		X: p.X - xLinear,
		Y: p.Y - yLinear,
		Z: p.Z - zLinear,
	}

	return approachPoint
}
