FROM moveit/moveit2:humble-release

ENV TERM=xterm-256color
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc

RUN apt update && \ 
    apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control \
    ros-humble-rqt ros-humble-rqt-moveit \
    ros-humble-rqt-common-plugins -y

RUN apt update && \
    apt install tmux vim python3-pip -y

WORKDIR /home/rahul/mtc_ws

RUN apt update && \
    rosdep update && \
    git clone -b humble https://github.com/moveit/moveit_task_constructor.git src/ && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

WORKDIR /home/rahul
 
