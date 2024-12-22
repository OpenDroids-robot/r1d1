FROM moveit/moveit2:humble-release

ENV TERM=xterm-256color
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc
RUN apt update
RUN apt install ros-humble-gazebo-ros -y
RUN apt install ros-humble-gazebo-ros2-control -y
RUN apt install ros-humble-rqt -y
RUN apt install ros-humble-rqt-moveit -y
RUN apt install tmux -y
RUN apt install vim -y
RUN apt install python3-pip
