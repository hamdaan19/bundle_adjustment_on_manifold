# Pull ubuntu base image
FROM ubuntu:20.04

# Update
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get install -y

# Install dependencies
RUN apt-get install -y unzip && apt-get install -y wget && apt-get install -y build-essential checkinstall && apt-get install -y sudo
RUN apt-get install -y openssh-server usbutils nano libssl-dev
RUN apt-get install -y git tar

WORKDIR /software

# Installing CMake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz && \
    tar -zxvf cmake-3.20.0.tar.gz && cd cmake-3.20.0 && \
    ./configure && \
    make && sudo make install 

# Installing Eigen
RUN git clone https://gitlab.com/libeigen/eigen.git --branch 3.4.0 --single-branch && \ 
    cd eigen && mkdir build && cd build && \
    cmake .. -DEIGEN_DEFAULT_TO_ROW_MAJOR=$ROW_MAJOR_DEFAULT && \
    sudo make install && cd ../..

# Installing Ceres Dependencies
RUN sudo apt-get install -y libgoogle-glog-dev

#Installing Ceres Solver
RUN git clone https://github.com/ceres-solver/ceres-solver.git --branch 2.2.0 --single-branch && \
    cd ceres-solver && mkdir build && cd build && \
    cmake .. && \
    sudo make install && cd ../..

#Installing Sophus Dependencies
RUN sudo apt-get install -y libfmt-dev

# Installing Sophus
RUN git clone https://github.com/strasdat/Sophus.git --branch 1.22.10 --single-branch && \
    cd Sophus && mkdir build && cd build && \
    cmake .. && \
    make && sudo make install && cd ../..  

RUN sudo apt-get install -y net-tools

# Installing GTK
RUN sudo apt-get update && sudo apt install -y libgtk2.0-dev pkg-config

# Installing OpenCV and OpenCV contrib
RUN git clone https://github.com/opencv/opencv_contrib.git --branch 4.10.0 --single-branch && \ 
    git clone https://github.com/opencv/opencv.git --branch 4.10.0 --single-branch && \
    cd opencv && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D WITH_QT=OFF -DWITH_GTK_2_x=ON .. && \
    make && sudo make install && cd ../..  

# Installing zsh
RUN sudo apt-get install -y zsh
# Installing Oh-My-Zsh
# Install Oh my zsh
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" -- \
    -p git \
    -p docker-compose \
    -p docker \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting 

RUN sudo apt-get update && sudo apt-get install -qqy x11-apps

WORKDIR /home/bundle_adjustment_on_manifold
CMD ["/bin/zsh"]