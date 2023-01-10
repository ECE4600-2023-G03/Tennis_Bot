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
