ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}-ros-core

ENV http_proxy   "http://proxy.cohort-study.com:43890"
ENV https_proxy  "http://proxy.cohort-study.com:43890"
ENV all_proxy    "socks5://proxy.cohort-study.com:43567"

#RUN sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirrors.aliyun.com/g' /etc/apt/sources.list
RUN sed -i 's/http:\/\/archive.ubuntu.com/https:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list

RUN apt-get update -q && apt-get dist-upgrade -q -y
RUN apt-get install -y lsb-release wget software-properties-common build-essential

RUN bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"
RUN bash -c "$(wget -O - https://apt.kitware.com/kitware-archive.sh)"

RUN apt-get update -q && \
	apt-get dist-upgrade -q -y && \
	apt-get install -y \
	clang-15 clang-tools-15 clang-15-doc libclang-common-15-dev libclang-15-dev libclang1-15 clang-format-15 python3-clang-15 clangd-15 clang-tidy-15 \
	libc++-15-dev libc++abi-15-dev
RUN apt-get install -y ccache ninja-build cmake vim git gdb gdbserver

# if distro is galactic, dont install generate-parameter-library


RUN apt-get install -y \
	ros-${ROS_DISTRO}-ros-base \
	ros-${ROS_DISTRO}-control-msgs \
	ros-${ROS_DISTRO}-controller-interface \
	ros-${ROS_DISTRO}-realtime-tools \
	ros-${ROS_DISTRO}-controller-manager-msgs \
	ros-${ROS_DISTRO}-example-interfaces

RUN apt-get install -y \
	python3-pip python3-vcstool python3-colcon-common-extensions

RUN apt-get install -y python3-colcon-mixin
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
	colcon mixin update default

#unset 
ENV http_proxy   ""
ENV https_proxy  ""
ENV all_proxy ""

RUN apt-get install -y libopencv-dev libaravis-dev gdb

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# proxy
# update-alternatives clang++ clang++-15
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-15 100 
RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-15 100
#RUN python3 -m pip install yapf mypy -i https://mirrors.cloud.tencent.com/pypi/simple
RUN python3 -m pip install yapf mypy setuptools==58.2.0 -i https://mirrors.cloud.tencent.com/pypi/simple


RUN echo "export http_proxy=http://proxy.cohort-study.com:43890" >> ~/.bashrc
RUN echo "export https_proxy=http://proxy.cohort-study.com:43890" >> ~/.bashrc
RUN echo "export all_proxy=http://proxy.cohort-study.com:43890" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
