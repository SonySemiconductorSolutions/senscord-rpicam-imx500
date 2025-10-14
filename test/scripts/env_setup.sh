#!/bin/bash
# SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
#
# SPDX-License-Identifier: Apache-2.0

echo "======================================================================"
echo "              Environment Setup for Unit Tests                       "
echo "======================================================================"

# Update package list
echo "Updating package list..."
apt-get update

# Install essential build tools
echo "Installing essential build tools..."
apt-get install -y \
    build-essential \
    cmake \
    make \
    gcc \
    g++ \
    pkg-config \
    libtool \
    autoconf \
    automake

# Install coverage tools
echo "Installing coverage tools..."
apt-get install -y \
    lcov \
    gcovr

# Install Google Test and Google Mock
echo "Installing Google Test and Google Mock..."
apt-get install -y \
    libgtest-dev \
    libgmock-dev \
    googletest

# Install threading support
echo "Installing threading support..."
apt-get install -y \
    libpthread-stubs0-dev

# Install additional development tools
echo "Installing additional development tools..."
apt-get install -y \
    git \
    curl \
    wget \
    unzip \
    zip

# Install system libraries that may be needed
echo "Installing system libraries..."
apt-get install -y \
    libc6-dev \
    linux-libc-dev

# Create symbolic links for gtest if needed (for older Ubuntu versions)
if [ ! -f /usr/lib/libgtest.a ] && [ -d /usr/src/googletest ]; then
    echo "Building Google Test from source..."
    cd /usr/src/googletest
    cmake .
    make
    cp lib/*.a /usr/lib/
    cd -
fi

# Verify installations
echo "======================================================================"
echo "                    Verifying Installations                          "
echo "======================================================================"

# Update PATH to ensure installed tools are available
export PATH="/usr/bin:/usr/local/bin:/bin:$PATH"

echo "Checking CMake version:"
cmake --version

echo "Checking GCC version:"
gcc --version

echo "Checking LCOV version:"
if command -v lcov &> /dev/null; then
    lcov --version
else
    echo "Warning: lcov not found in PATH. Checking installation..."
    if [ -f /usr/bin/lcov ]; then
        echo "lcov found at /usr/bin/lcov"
        /usr/bin/lcov --version
    else
        echo "Error: lcov installation may have failed"
        apt-get install -y --reinstall lcov
        lcov --version
    fi
fi

echo "Checking if Google Test is available:"
if pkg-config --exists gtest; then
    echo "Google Test found via pkg-config"
    pkg-config --modversion gtest
else
    echo "Google Test not found via pkg-config, checking libraries..."
    if [ -f /usr/lib/libgtest.a ] || [ -f /usr/lib/x86_64-linux-gnu/libgtest.a ]; then
        echo "Google Test library found"
    else
        echo "Warning: Google Test library not found"
    fi
fi

echo "Checking if Google Mock is available:"
if pkg-config --exists gmock; then
    echo "Google Mock found via pkg-config"
    pkg-config --modversion gmock
else
    echo "Google Mock not found via pkg-config, checking libraries..."
    if [ -f /usr/lib/libgmock.a ] || [ -f /usr/lib/x86_64-linux-gnu/libgmock.a ]; then
        echo "Google Mock library found"
    else
        echo "Warning: Google Mock library not found"
    fi
fi

# Set environment variables
echo "======================================================================"
echo "                  Setting Environment Variables                      "
echo "======================================================================"

# Ensure PATH includes all necessary directories
export PATH="/usr/bin:/usr/local/bin:/bin:$PATH"
export CMAKE_PREFIX_PATH="/usr:$CMAKE_PREFIX_PATH"
export PKG_CONFIG_PATH="/usr/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"

# Create a temporary PATH file for GitHub Actions to source
if [ "$GITHUB_ACTIONS" = "true" ]; then
    echo "export PATH=\"/usr/bin:/usr/local/bin:/bin:\$PATH\"" > /tmp/path_setup.sh
    echo "PATH setup script created for GitHub Actions"
fi

echo "Environment variables set:"
echo "PATH=$PATH"
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH"

echo "======================================================================"
echo "              Environment Setup Completed Successfully               "
echo "======================================================================"

# Create a simple test to verify the environment
echo "Creating environment verification test..."
cat > /tmp/test_env.cpp << 'EOF'
#include <gtest/gtest.h>
#include <gmock/gmock.h>

TEST(EnvironmentTest, BasicTest) {
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
EOF

echo "Compiling environment verification test..."
if g++ -std=c++17 -o /tmp/test_env /tmp/test_env.cpp -lgtest -lgmock -lpthread; then
    echo "Environment verification test compiled successfully"
    if /tmp/test_env; then
        echo "Environment verification test passed"
    else
        echo "Warning: Environment verification test failed"
    fi
    rm -f /tmp/test_env /tmp/test_env.cpp
else
    echo "Warning: Failed to compile environment verification test"
    echo "This may indicate missing dependencies"
fi

echo "Environment setup script completed."
