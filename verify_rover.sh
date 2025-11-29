#!/bin/bash
# Quick verification script for Mars Rover model

echo "=========================================="
echo "Mars Curiosity Rover Model Verification"
echo "=========================================="
echo ""

# Check model directory
if [ -d "MarsRover" ]; then
    echo "✓ MarsRover directory exists"
else
    echo "✗ MarsRover directory not found"
    exit 1
fi

# Check required files
echo ""
echo "Checking required files:"
files=("model.config" "model.sdf" "mars_rover.dae")
for file in "${files[@]}"; do
    if [ -f "MarsRover/$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file missing"
    fi
done

# Check textures
echo ""
echo "Checking textures:"
texture_count=$(ls MarsRover/*.png 2>/dev/null | wc -l)
echo "  Found $texture_count texture files"

# Check world file
echo ""
echo "Checking world file:"
if grep -q "MarsRover" MyModel.world; then
    echo "  ✓ Mars Rover included in MyModel.world"
else
    echo "  ✗ Mars Rover not in world file"
fi

echo ""
echo "=========================================="
echo "Model is ready for Gazebo simulation!"
echo "Run: ./launch_model.sh"
echo "=========================================="
