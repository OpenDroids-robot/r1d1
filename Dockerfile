FROM moveit/moveit2:humble-release

ENV TERM=xterm-256color
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc

RUN apt update && \
    apt install ros-humble-ros-gz \
                ros-humble-ign-ros2-control \
                ros-humble-rqt \
                ros-humble-rqt-common-plugins \
                ros-humble-rqt-moveit -y
RUN apt install tmux vim python3-pip -y
WORKDIR /home/rahul
