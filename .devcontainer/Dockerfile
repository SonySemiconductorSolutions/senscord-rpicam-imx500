# DevContainer for Unit Tests
FROM ubuntu:24.04

# Define build arguments and environment variables for maintainability
ARG CMAKE_VERSION=3.31.6
ARG NODE_VERSION=20.x
ARG ALLURE_VERSION=2.34.1

# Set non-interactive mode, timezone, and locale
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    CMAKE_VERSION=${CMAKE_VERSION} \
    NODE_VERSION=${NODE_VERSION} \
    ALLURE_VERSION=${ALLURE_VERSION}

# Install all apt packages in one layer to optimize caching and size
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        # Build essentials
        build-essential \
        pkg-config \
        gcc-13 \
        g++-13 \
        # Network and download tools
        curl \
        wget \
        ca-certificates \
        gnupg \
        # Version control
        git \
        # Utilities
        unzip \
        sudo \
        # Locale support
        locales \
        # Testing frameworks
        libgtest-dev \
        libgmock-dev \
        # Perl modules for LCOV 2.1
        libcapture-tiny-perl \
        libdatetime-perl \
        libtimedate-perl \
        libgd-perl \
        libjson-xs-perl \
        # Java runtime (required for Allure)
        openjdk-11-jre \
        # Image processing libraries (required for JPEG compression in rpicam_app_adapter.cpp)
        libjpeg-dev \
        # JSON library (required for rpicam_app_adapter JSON parsing)
        nlohmann-json3-dev \
        # Boost library (required for rpicam-apps command-line options)
        libboost-program-options-dev \
        # XML library (required for senscord OSAL)
        libxml2-dev \
        # MessagePack library (required for senscord serialization)
        libmsgpack-dev \
        # Development tools (optional but useful)
        gdb \
        tree \
    # Set GCC alternatives
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 \
    # Configure locale
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    # Clean up apt cache
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install CMake from source using environment variable
WORKDIR /usr/local/src
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz \
    && tar -xzf cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz -C /usr/local --strip-components=1 \
    && rm cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz

# Install LCOV 2.1 from source (for exception branch exclusion support)
RUN wget -q https://github.com/linux-test-project/lcov/releases/download/v2.1/lcov-2.1.tar.gz \
    && tar -xzf lcov-2.1.tar.gz \
    && cd lcov-2.1 \
    && make install PREFIX=/usr/local \
    && cd .. \
    && rm -rf lcov-2.1 lcov-2.1.tar.gz

# Install Node.js and Allure in one layer using environment variables
RUN curl -fsSL https://deb.nodesource.com/setup_${NODE_VERSION} | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && npm install -g allure-commandline@${ALLURE_VERSION} \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Configure the default ubuntu user (UID=1000, GID=1000) with sudo privileges
# Ubuntu 24.04 base image already has this user, so we just configure it
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set working directory
WORKDIR /workspace

# Switch to ubuntu user
USER ubuntu

# Configure bash environment for test development
RUN echo "# Auto-navigate to test directory" >> ~/.bashrc \
    && echo "cd /workspace/test/scripts" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# Use LCOV 2.1 from /usr/local/bin" >> ~/.bashrc \
    && echo 'export PATH="/usr/local/bin:$PATH"' >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# Unit Test Environment Welcome Message" >> ~/.bashrc \
    && echo "echo '📋 Unit Test Environment Ready!'" >> ~/.bashrc \
    && echo "echo '📂 Current directory: test/scripts'" >> ~/.bashrc \
    && echo "echo '🚀 Run ./run_unit_test.sh to execute all tests'" >> ~/.bashrc \
    && echo "echo '🧹 Run ./clean_test_artifacts.sh to clean up all test artifacts'" >> ~/.bashrc \
    && echo "echo ''" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# Convenient aliases for unit testing" >> ~/.bashrc \
    && echo "alias test-run='./run_unit_test.sh'" >> ~/.bashrc \
    && echo "alias test-clean='./clean_test_artifacts.sh'" >> ~/.bashrc \
    && echo "alias test-build='./exec_unit_test.sh build sample'" >> ~/.bashrc \
    && echo "alias ll='ls -alF'" >> ~/.bashrc \
    && echo "alias la='ls -A'" >> ~/.bashrc \
    && echo "alias l='ls -CF'" >> ~/.bashrc

# Verify installations
RUN echo "=== Verifying installations ===" \
    && echo "Build tools:" \
    && cmake --version | head -1 \
    && gcc --version | head -1 \
    && echo "LCOV: $(lcov --version | head -n 1)" \
    && echo "Test frameworks:" \
    && echo "Google Test: Source available in /usr/src/gtest ($(ls /usr/src/gtest/ | wc -l) files)" \
    && echo "Allure dependencies:" \
    && echo "Node.js: $(node --version)" \
    && echo "Java: $(java -version 2>&1 | head -1)" \
    && echo "Allure: $(allure --version 2>/dev/null | head -1)"

# Set default command
CMD ["/bin/bash"]