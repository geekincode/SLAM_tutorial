# 多阶段构建：编译阶段
FROM ubuntu:22.04

# 避免在安装过程中出现交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 使用国内镜像加速（针对中国大陆用户）：
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list

# 安装必要的工具和依赖
RUN apt-get update && apt-get install -y --no-install-recommends\
    git \
    wget \
    curl \
    unzip \
    cmake \
    g++ \
    ccache \
    tree \
    vim \
    libeigen3-dev \
    libsuitesparse-dev \
    qtdeclarative5-dev \
    libqglviewer-dev-qt5 \
    libceres-dev \
    libopencv-dev \
    libboost1.74-all-dev \
    # X11显示支持
    libx11-dev \
    libxt-dev \
    libgl1-mesa-glx \
    libxrender1 \
    libxext6 \
    xvfb \
    # 添加中文字体和locale支持
    locales \
    fonts-wqy-microhei \
    language-pack-zh-hans \
    # && rm -rf /var/lib/apt/lists/* \
    # 配置中文locale
    && locale-gen zh_CN.UTF-8 \
    && update-locale LANG=zh_CN.UTF-8 LC_ALL=zh_CN.UTF-8

# 设置环境变量
ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0


# 配置 git 网络设置来解决 TLS 连接问题
RUN git config --global http.version HTTP/1.1 \
    && git config --global http.sslVerify false \
    && git config --global https.sslVerify false

# 为Vim预配置中文支持
RUN echo "set encoding=utf-8" >> ~/.vimrc \
    && echo "set fileencodings=utf-8,chinese,latin1" >> ~/.vimrc \
    && echo "set langmenu=zh_CN.UTF-8" >> ~/.vimrc \
    && echo "language messages zh_CN.UTF-8" >> ~/.vimrc

# Sophus 库
RUN cd /home && \
    git clone https://github.com/strasdat/Sophus.git && \
    # 仅docker需要去掉sudo
    sed -i 's/sudo //g' Sophus/scripts/install_ubuntu_deps_incl_ceres.sh && \
    # 给所有 apt-get install 加 -y
    sed -i '/^ *#/!s/apt-get install/apt-get -y install/g' Sophus/scripts/install_ubuntu_deps_incl_ceres.sh && \
    sed -i 's|https://ceres-solver.googlesource.com/ceres-solver|https://github.com/ceres-solver/ceres-solver.git|g' Sophus/scripts/install_ubuntu_deps_incl_ceres.sh && \
    # 修改CMakeLists.txt中的最低版本要求
    sed -i 's/cmake_minimum_required(VERSION [0-9.]\+)/cmake_minimum_required(VERSION 3.16)/' Sophus/CMakeLists.txt && \
    cd Sophus && \
    export DEBIAN_FRONTEND=noninteractive && \
    bash scripts/install_ubuntu_deps_incl_ceres.sh && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j1 && \
    make install

RUN cd /home && \
    git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

# 更新动态链接库缓存
RUN ldconfig

# 设置工作目录
WORKDIR /home/slam_tutorial

# 复制项目文件
COPY . .

# 默认命令
CMD ["/bin/bash"]