#!/bin/bash

# ORION Robot Development Workflow Script
# This script provides a complete workflow for developing and testing the ORION robot

# Configuration
WORKSPACE_ROOT="$HOME/Documents/ORION/orion_ws"
PACKAGE_NAME="orion_description"
LAUNCH_FILE="display.launch.py"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Function to check if we're in the right directory
check_workspace() {
    if [[ ! -d "$WORKSPACE_ROOT" ]]; then
        print_error "Workspace directory $WORKSPACE_ROOT does not exist!"
        exit 1
    fi

    if [[ ! -d "$WORKSPACE_ROOT/src/$PACKAGE_NAME" ]]; then
        print_error "Package $PACKAGE_NAME not found in $WORKSPACE_ROOT/src/"
        exit 1
    fi
}

# Function to build the package
build_package() {
    print_step "Building $PACKAGE_NAME package..."

    cd "$WORKSPACE_ROOT"

    if [[ "$1" == "fast" ]]; then
        print_status "Performing fast rebuild (incremental build)..."
        colcon build --packages-select $PACKAGE_NAME --continue-on-error
    else
        print_status "Performing full rebuild..."
        colcon build --packages-select $PACKAGE_NAME
    fi

    if [[ $? -ne 0 ]]; then
        print_error "Build failed!"
        exit 1
    fi

    print_status "Build completed successfully!"
}

# Function to source the workspace
source_workspace() {
    print_step "Sourcing workspace..."

    cd "$WORKSPACE_ROOT"

    if [[ -f "install/setup.bash" ]]; then
        source install/setup.bash
        print_status "Workspace sourced successfully!"
    else
        print_error "setup.bash not found! Make sure the workspace is built."
        exit 1
    fi
}

# Function to launch the robot
launch_robot() {
    print_step "Launching robot in RViz2..."

    cd "$WORKSPACE_ROOT"

    # Check if ROS 2 is available
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS 2 is not sourced or installed!"
        exit 1
    fi

    print_status "Starting launch: ros2 launch $PACKAGE_NAME $LAUNCH_FILE"
    ros2 launch $PACKAGE_NAME $LAUNCH_FILE
}

# Function to show usage
show_usage() {
    echo "ORION Robot Development Workflow Script"
    echo ""
    echo "Usage:"
    echo "  $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build          - Build the package and source workspace"
    echo "  launch         - Launch the robot (assumes workspace is built)"
    echo "  rebuild        - Full rebuild of the package"
    echo "  fast           - Fast incremental rebuild"
    echo "  dev            - Development mode: fast rebuild + launch"
    echo "  full           - Full workflow: build + launch"
    echo "  help           - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 dev         # Fast rebuild and launch (use after xacro changes)"
    echo "  $0 full        # Complete rebuild and launch"
    echo "  $0 build       # Just build the package"
    echo ""
    echo "Workflow recommendations:"
    echo "  - Use 'dev' after editing xacro files (fast rebuild)"
    echo "  - Use 'full' when changing CMakeLists.txt or adding new files"
    echo "  - Use 'build' when you only want to build without launching"
}

# Main script logic
main() {
    local command="$1"

    case "$command" in
        "build")
            check_workspace
            build_package
            source_workspace
            print_status "Package built and workspace sourced. Ready to launch!"
            ;;

        "launch")
            check_workspace
            source_workspace
            launch_robot
            ;;

        "rebuild")
            check_workspace
            build_package "full"
            source_workspace
            print_status "Full rebuild completed!"
            ;;

        "fast")
            check_workspace
            build_package "fast"
            source_workspace
            print_status "Fast rebuild completed!"
            ;;

        "dev")
            print_status "Starting development workflow (fast rebuild + launch)..."
            check_workspace
            build_package "fast"
            source_workspace
            launch_robot
            ;;

        "full")
            print_status "Starting full workflow (complete rebuild + launch)..."
            check_workspace
            build_package "full"
            source_workspace
            launch_robot
            ;;

        "help"|"-h"|"--help"|"")
            show_usage
            ;;

        *)
            print_error "Unknown command: $command"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"