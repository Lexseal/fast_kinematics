FROM quay.io/pypa/manylinux2014_x86_64

# basic development tools
RUN yum update -y && yum install -y wget git && yum upgrade -y && yum clean all
WORKDIR /workspace

# get oh my zsh
RUN yum install -y zsh && \
    sh -c -y "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# get cuda
RUN yum-config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel7/x86_64/cuda-rhel7.repo && \
    yum clean all && \
    yum -y install cuda-toolkit-12-4 && \
    echo 'export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}' >> ~/.zshrc && \
    echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.zshrc

# Eigen
RUN git clone --single-branch -b 3.4.0 --depth 1 https://gitlab.com/libeigen/eigen.git && \
    cd eigen && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /workspace/eigen

# tinyxml2
RUN git clone --single-branch -b 10.0.0 --depth 1 https://github.com/leethomason/tinyxml2.git && \
    cd tinyxml2 && mkdir build && cd build && \
    cmake -DCMAKE_CXX_FLAGS='-D_GLIBCXX_USE_CXX11_ABI=0' -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /workspace/tinyxml2

# console_bridge
RUN git clone --single-branch -b 1.0.2 --depth 1 https://github.com/ros/console_bridge.git && \
    cd console_bridge && mkdir build && cd build && \
    cmake -DCMAKE_CXX_FLAGS='-D_GLIBCXX_USE_CXX11_ABI=0' -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /workspace/console_bridge

# urdfdom_headers
RUN git clone --single-branch -b 1.1.1 --depth 1 https://github.com/ros/urdfdom_headers.git && \
    cd urdfdom_headers && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    rm -rf /workspace/urdfdom_headers

# urdfdom
RUN git clone --single-branch -b 4.0.0 --depth 1 https://github.com/ros/urdfdom.git && \
    # modify the CMakeLists.txt to build static library
    cd urdfdom/urdf_parser && sed -i 's/SHARED/STATIC/' CMakeLists.txt && \
    cd .. && mkdir build && cd build && \
    cmake -DCMAKE_CXX_FLAGS='-D_GLIBCXX_USE_CXX11_ABI=0' -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /workspace/urdfdom

# get fast_kinematics
RUN git clone https://github.com/Lexseal/fast_kinematics.git
