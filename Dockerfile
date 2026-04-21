FROM ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl \
    python3-pip python3-catkin-tools \
    libeigen3-dev libfmt-dev \
    coinor-libipopt-dev \
    gfortran \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-interactive-markers \
    ros-noetic-controller-manager \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "numpy>=1.19.5" scipy

# ── OSQP (from bundled source) ────────────────────────────────────────────────
COPY dependecies/osqp /tmp/osqp
RUN cmake -S /tmp/osqp -B /tmp/osqp-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/osqp-build --target install -j$(nproc) && \
    rm -rf /tmp/osqp /tmp/osqp-build && \
    cp /usr/local/include/osqp/* /usr/local/include/

# ── OsqpEigen (from bundled source) ───────────────────────────────────────────
COPY dependecies/osqp-eigen /tmp/osqp-eigen
RUN cmake -S /tmp/osqp-eigen -B /tmp/osqp-eigen-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/osqp-eigen-build --target install -j$(nproc) && \
    rm -rf /tmp/osqp-eigen /tmp/osqp-eigen-build

# ── CasADi (from bundled source) ──────────────────────────────────────────────
COPY dependecies/casadi /tmp/casadi
RUN cmake -S /tmp/casadi -B /tmp/casadi-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DWITH_PYTHON=OFF \
        -DWITH_EXAMPLES=OFF \
        -DWITH_IPOPT=ON && \
    cmake --build /tmp/casadi-build --target install -j$(nproc) && \
    rm -rf /tmp/casadi /tmp/casadi-build

# ── Copy project ──────────────────────────────────────────────────────────────
WORKDIR /workspace
COPY . .

# ── fmt from bundled source ───────────────────────────────────────────────────
RUN cmake -S dependecies/fmt-8.1.1 -B /tmp/fmt-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DFMT_TEST=OFF && \
    cmake --build /tmp/fmt-build --target install -j$(nproc) && \
    rm -rf /tmp/fmt-build

# ── Sophus from bundled source ────────────────────────────────────────────────
RUN cmake -S dependecies/Sophus-main-1.x -B /tmp/sophus-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/sophus-build --target install -j$(nproc) && \
    rm -rf /tmp/sophus-build

# ── Build ROS workspace ───────────────────────────────────────────────────────
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd pinocchio_sim_ws && \
    rm -rf build devel && \
    catkin_make -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -50" || \
    (docker_build_log=$(find /workspace/pinocchio_sim_ws/build -name '*.log' 2>/dev/null | head -1) && \
    cat $docker_build_log 2>/dev/null; exit 1)

# Source workspace on container start
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/pinocchio_sim_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "/usr/local/lib" >> /etc/ld.so.conf.d/local.conf && \
    ldconfig

CMD ["/bin/bash"]
