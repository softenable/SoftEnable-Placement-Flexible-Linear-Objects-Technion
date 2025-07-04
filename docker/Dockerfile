ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

WORKDIR /ws

# Dev container arguments
ARG USERNAME=ros
ARG UID=1000
ARG GID=${UID} 


RUN apt-get update && apt-get -y install sudo nano gdb udev curl git ssh

COPY ./docker/packages.txt /var/packages.txt
# hadolint ignore=DL3009
RUN apt-get update && xargs apt-get install -y --no-install-recommends < /var/packages.txt && rm /var/packages.txt

# Create new user and home directory
# hadolint ignore=DL3004
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

#setup pinocchio cuda libraries # https://stack-of-tasks.github.io/pinocchio/download.html
COPY ./docker/setup_pinocchio.bash /var/setup_pinocchio.bash
RUN chmod +x /var/setup_pinocchio.bash && /var/setup_pinocchio.bash

 # Add additional packages or dependencies here
RUN sudo apt update && sudo apt install -y python3-pip
COPY ./docker/requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt
 # Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /ws/

USER ${USERNAME}

COPY . /ws/

# Install ROS dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths src -y --ignore-src && colcon build --base-path src --symlink-install

# Set up the entrypoint
COPY /docker/entrypoint.sh /
  
ENTRYPOINT [ "/entrypoint.sh" ]
