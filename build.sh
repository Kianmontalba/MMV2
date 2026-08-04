#!/bin/bash
# MMV2 GDExtension Build Script
# Usage: ./build.sh [platform] [target] [arch]

PLATFORM=${1:-"linux"}
TARGET=${2:-"template_debug"}
ARCH=${3:-"x86_64"}

echo "========================================"
echo "Building MMV2 GDExtension"
echo "Platform: $PLATFORM"
echo "Target: $TARGET"
echo "Arch: $ARCH"
echo "========================================"

# Check if godot-cpp exists
if [ ! -d "godot-cpp" ]; then
    echo "Error: godot-cpp submodule not found!"
    echo "Run: git submodule add https://github.com/godotengine/godot-cpp.git"
    exit 1
fi

# Build
scons platform=$PLATFORM target=$TARGET arch=$ARCH -j$(nproc)

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo "Output: demo/bin/"
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
