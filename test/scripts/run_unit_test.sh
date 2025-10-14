#!/bin/bash

# SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Script to build, test, and collect coverage for all unit test components
# Usage: run_unit_test.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
EXEC_UNIT_TEST="$SCRIPT_DIR/exec_unit_test.sh"

# Define unit test components (add new components here)
COMPONENTS=(
    "sample"
    # Add other components here as they are created
    # "public/component/libcamera_image"
    # "public/component/inference_property_converter"
)

print_usage() {
    echo "Usage: $0"
    echo ""
    echo "This script will:"
    echo "  1. Build all unit test components"
    echo "  2. Run all unit tests"
    echo "  3. Collect unified coverage data for all components"
    echo ""
    echo "Components configured:"
    for component in "${COMPONENTS[@]}"; do
        echo "  - $component"
    done
    echo ""
    echo "To add new components, edit the COMPONENTS array in this script."
    echo ""
    echo "For individual component operations, use exec_unit_test.sh directly:"
    echo "  ./exec_unit_test.sh build <component>"
    echo "  ./exec_unit_test.sh test <component>"
    echo "  ./exec_unit_test.sh collect # Unified coverage for all"
}

# Validate exec_unit_test.sh exists
if [ ! -f "$EXEC_UNIT_TEST" ]; then
    echo "Error: exec_unit_test.sh not found at '$EXEC_UNIT_TEST'"
    exit 1
fi

# Parse arguments
if [ $# -gt 0 ]; then
    echo "Error: This script does not accept arguments"
    print_usage
    exit 1
fi

# Function to check if component directory exists and has CMakeLists.txt
check_component() {
    local component="$1"
    local component_path="$TEST_ROOT/$component"
    
    if [ ! -d "$component_path" ]; then
        echo "Warning: Component directory '$component_path' does not exist. Skipping."
        return 1
    fi
    
    if [ ! -f "$component_path/CMakeLists.txt" ]; then
        echo "Warning: CMakeLists.txt not found in '$component_path'. Skipping."
        return 1
    fi
    
    return 0
}

# Function to build and test all components
build_and_test_all() {
    echo "Starting build and test for all components..."
    
    local failed_components=()
    
    for component in "${COMPONENTS[@]}"; do
        echo ""
        echo "========================================"
        echo "Processing component: $component"
        echo "========================================"
        
        if ! check_component "$component"; then
            failed_components+=("$component (skipped)")
            continue
        fi
        
        # Build component
        echo "Building $component..."
        if ! "$EXEC_UNIT_TEST" build "$component"; then
            echo "Error: Failed to build $component"
            failed_components+=("$component (build failed)")
            continue
        fi
        
        # Test component
        echo "Testing $component..."
        if ! "$EXEC_UNIT_TEST" test "$component"; then
            echo "Error: Failed to test $component"
            failed_components+=("$component (test failed)")
            continue
        fi
        
        echo "Successfully completed build and test for $component"
    done
    
    # Report results
    if [ ${#failed_components[@]} -eq 0 ]; then
        echo ""
        echo "========================================"
        echo "All components built and tested successfully!"
        echo "========================================"
        return 0
    else
        echo ""
        echo "========================================"
        echo "Build and test completed with some issues:"
        for failed in "${failed_components[@]}"; do
            echo "  - $failed"
        done
        echo "========================================"
        return 1
    fi
}

# Function to collect unified coverage report
collect_unified_coverage() {
    echo ""
    echo "========================================"
    echo "Collecting unified coverage report..."
    echo "========================================"
    
    # Call exec_unit_test.sh to create unified coverage report
    # This will automatically collect coverage from all components that have been tested
    echo "Creating unified coverage report for all tested components..."
    if ! "$EXEC_UNIT_TEST" collect; then
        echo "Error: Failed to create unified coverage report"
        exit 1
    fi
    
    echo ""
    echo "========================================"
    echo "Unified coverage collection completed!"
    echo "========================================"
    echo "Coverage reports available at: $TEST_ROOT/coverage/html/index.html"
}
# Main execution
build_and_test_all
collect_unified_coverage

echo ""
echo "Operation completed successfully"
