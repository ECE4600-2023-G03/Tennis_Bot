from ros:humble

RUN apt update && \
    apt install -y \
    ros-humble-rviz2

RUN apt update && \
    apt install -y \
    python3-pip \
    wget \
    vim \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-teleop-twist-keyboard \
    xclip \
    less \
    net-tools \
    gdb     


RUN pip3 install Jetson.GPIO
RUN pip3 install opencv-python depthai 
RUN pip3 install --upgrade setuptools==58.2.0

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

# install dependancies for ORB-SLAM 2
RUN apt-get install -y \
	python3-dev \
	python3-pip \
	python3-numpy \
	software-properties-common \					
	python3-rosdep \
	openssh-client \			
	libgtk-3-dev \
	libglew-dev \
	libgl1-mesa-dev \
	pkg-config \
	libpython2.7-dev \
	ffmpeg \
	libboost-dev \
	libboost-system-dev \
	libcanberra-gtk-module \
	software-properties-common

RUN apt-get update && apt-get install --yes python3-rosinstall-generator


RUN git clone -b 3.2 --single-branch https://gitlab.com/libeigen/eigen.git

RUN git clone https://github.com/stevenlovegrove/Pangolin.git

RUN git clone https://github.com/opencv/opencv.git

RUN mkdir -p ./ws/src && cd ./ws/src
RUN git clone https://github.com/mirellameelo/ORB_SLAM2_ROS_2.git
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git src/vision_opencv
RUN git clone https://github.com/ros2/message_filters src/message_filters


# #ORB SLAM 2 build
# RUN git clone https://github.com/raulmur/ORB_SLAM2.git
# RUN cd $./ORB_SLAM2
# RUN rm CMakeLists.txt
# RUN cp $HOME/ORB_SLAM2_ROS_2/src/CMakeLists.txt .
# RUN rm $HOME/ORB_SLAM2_ROS_2/src/CMakeLists.txt
# RUN cd Thirdparty/DBoW2
# RUN mkdir build && cd build
# RUN cmake .. -DCMAKE_BUILD_TYPE=Release
# RUN make -j
# RUN cd ../../g2o
# RUN mkdir build && cd build
# RUN cmake .. -DCMAKE_BUILD_TYPE=Release
# RUN make -j
# RUN cd ../../../
# RUN cd Vocabulary
# RUN tar -xf ORBvoc.txt.tar.gz
# # Configuring and building ORB_SLAM2
# RUN cd ..
# RUN mkdir build && cd build
# RUN cmake .. \
# 	-DROS_BUILD_TYPE=Release \
# 	-DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
# 	-DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"
# RUN make -j
# RUN make install


# RUN echo "source $HOME/ros2_sdk/install/setup.sh" >> ~/.bashrc
# RUN echo "source $HOME/ws/install/local_setup.sh" >> ~/.bashrc
# RUN echo "export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:~/ORB_SLAM2/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

