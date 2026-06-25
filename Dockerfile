FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    pkg-config \
    # OpenCV 4.5 (satisfies >= 4.4 requirement)
    libopencv-dev \
    # Eigen3
    libeigen3-dev \
    # Pangolin dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libpython3-dev \
    libboost-dev \
    libboost-thread-dev \
    libboost-filesystem-dev \
    # Virtual framebuffer for headless Pangolin
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Build Pangolin from source
RUN git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin && \
    cmake -S /opt/Pangolin -B /opt/Pangolin/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /opt/Pangolin/build -j$(nproc) && \
    cmake --install /opt/Pangolin/build && \
    ldconfig

# Clone ORB_SLAM3_Gamma_Correction and build
RUN git clone https://github.com/cheyawelewa/ORB_SLAM3_Gamma_Correction.git /ORB_SLAM3

WORKDIR /ORB_SLAM3

RUN cd Thirdparty/DBoW2 && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && \
    cd /ORB_SLAM3/Thirdparty/g2o && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && \
    cd /ORB_SLAM3/Thirdparty/Sophus && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

RUN cd Vocabulary && tar -xf ORBvoc.txt.tar.gz

RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Start a virtual display so Pangolin doesn't crash on headless machines.
# Pass --no-viewer to the ORB_SLAM3 example to skip the GUI entirely.
ENV DISPLAY=:1
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
