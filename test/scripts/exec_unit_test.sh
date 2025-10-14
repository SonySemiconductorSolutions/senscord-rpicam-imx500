#!/bin/bash

# SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Script to build, test, and collect coverage for unit tests
# Usage: exec_unit_test.sh [build|test|collect] <directory_path>

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

print_usage() {
    echo "Usage: $0 [build|test] <directory_path>"
    echo "       $0 collect [directory_path]"
    echo ""
    echo "Commands:"
    echo "  build    - Build the unit tests for the specified directory"
    echo "  test     - Run the unit tests for the specified directory"
    echo "  collect  - Collect coverage data and generate HTML report"
    echo ""
    echo "Arguments:"
    echo "  directory_path - Directory path relative to test/"
    echo "                   (optional for collect command - omit for unified coverage)"
    echo ""
    echo "Examples:"
    echo "  $0 build sample               # Build sample tests"
    echo "  $0 test sample                # Run sample tests"
    echo "  $0 collect sample             # Collect coverage for sample tests"
    echo "  $0 collect                    # Collect unified coverage for all test directories"
}

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    print_usage
    exit 1
fi

COMMAND="$1"
TARGET_DIR="${2:-}"

# Validate command
case "$COMMAND" in
    build|test)
        if [ -z "$TARGET_DIR" ]; then
            echo "Error: directory_path is required for '$COMMAND' command"
            print_usage
            exit 1
        fi
        ;;
    collect)
        # TARGET_DIR is optional for collect command
        ;;
    *)
        echo "Error: Invalid command '$COMMAND'"
        print_usage
        exit 1
        ;;
esac

# Validate target directory
if [ -z "$TARGET_DIR" ]; then
    # No directory specified: unified coverage collection for test root directory
    TARGET_PATH="$TEST_ROOT"
else
    TARGET_PATH="$TEST_ROOT/$TARGET_DIR"
fi

if [ ! -d "$TARGET_PATH" ]; then
    echo "Error: Target directory '$TARGET_PATH' does not exist"
    exit 1
fi

# For build and test commands, check if CMakeLists.txt exists and setup build directory
if [ "$COMMAND" != "collect" ]; then
    if [ ! -f "$TARGET_PATH/CMakeLists.txt" ]; then
        echo "Error: CMakeLists.txt not found in '$TARGET_PATH'"
        exit 1
    fi
    
    BUILD_DIR="$TARGET_PATH/build"
    # Change to target directory
    cd "$TARGET_PATH"
fi

case "$COMMAND" in
    build)
        echo "Building unit tests for $TARGET_DIR..."
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR"
        cmake .. -DCMAKE_BUILD_TYPE=Debug
        make -j$(nproc)
        echo "Build completed successfully"
        ;;
        
    test)
        echo "Running unit tests for $TARGET_DIR..."
        if [ ! -d "$BUILD_DIR" ] || [ ! -f "$BUILD_DIR/Makefile" ]; then
            echo "Error: Build directory not found or not built."
            echo "Please run 'build' command first: $0 build $TARGET_DIR"
            exit 1
        fi
        
        cd "$BUILD_DIR"
        
        # Run tests
        ctest --output-on-failure
        echo "Tests completed successfully"
        ;;
        
    collect)
        echo "Collecting coverage (target: ${TARGET_DIR:-all})"
        
        # Output directory & report title
        COVERAGE_DIR="$TARGET_PATH/coverage"
        REPORT_TITLE="${TARGET_DIR:+${TARGET_DIR} Unit Test Coverage Report}"
        : "${REPORT_TITLE:=senscord-rpicam-imx500 - Unified Unit Test Coverage Report}"
        mkdir -p "$COVERAGE_DIR"

        # Coverage files
        COMBINED_INFO="$COVERAGE_DIR/combined_coverage.info"
        RAW_INFO="$COVERAGE_DIR/raw.info"

        # Capture & filter
        if ! lcov --capture --directory "$TARGET_PATH" --rc lcov_branch_coverage=1 --output-file "$RAW_INFO"; then
            echo "Error: Failed capture"; exit 1; fi

        if ! lcov --remove "$RAW_INFO" \
                '/usr/*' '*/gtest/*' '*/gmock/*' '*_gtest.cc' '*_mock.cc' '*_mock.h' \
                --output-file "$COMBINED_INFO" --quiet; then
            echo "Error: Failed filter"; rm -f "$RAW_INFO"; exit 1; fi
        rm -f "$RAW_INFO"

        # Generate HTML report
        if ! genhtml "$COMBINED_INFO" \
                --output-directory "$COVERAGE_DIR/html" \
                --title "$REPORT_TITLE" \
                --show-details \
                --legend \
                --function-coverage \
                --branch-coverage; then
            echo "Error: Failed to generate HTML report"; exit 1; fi
        
        echo "Coverage OK: HTML=$COVERAGE_DIR/html/index.html info=$COMBINED_INFO"
        ;;
esac

echo "Operation completed successfully"
