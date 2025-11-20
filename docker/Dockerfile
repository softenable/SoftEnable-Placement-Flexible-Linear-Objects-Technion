# syntax=docker/dockerfile:1.7
ARG ROS_DISTRO=humble

FROM registry.gitlab.ocado.tech/external-collaborations/he-robotics/common/he-base-docker-images/he-ros-humble-base:1.0.1
# ---- Base setup ----
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /ws

# Dev container args
ARG USERNAME=ros
ARG UID=1000
ARG GID=${UID}

# Create user/group if missing (base may already provide it) and ensure sudo access
# hadolint ignore=DL3008
RUN if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
      groupadd --gid "${GID}" "${USERNAME}" && \
      useradd --uid "${GID}" --gid "${UID}" --create-home "${USERNAME}" --shell /bin/bash; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ---- System packages ----
COPY docker/packages.txt /var/packages.txt
# hadolint ignore=DL3009
RUN --mount=type=secret,id=netrc,target=/etc/apt/auth.conf.d/ocado.conf \
    --mount=type=secret,id=HE_APT_GPG,target=/etc/apt/trusted.gpg.d/he_apt_gpg \
    gpg --dearmor /etc/apt/trusted.gpg.d/he_apt_gpg \
    && ls -l /etc/apt/trusted.gpg.d \
    && echo "deb https://nexus.ocado.tech/repository/apt-jammy-internal/ jammy main" | tee /etc/apt/sources.list.d/ocado.list \
    && apt-get update && xargs apt-get install -y --no-install-recommends < /var/packages.txt && rm /var/packages.txt \
    && rm /etc/apt/sources.list.d/ocado.list && rm /etc/apt/trusted.gpg.d/he_apt_gpg.gpg


# ---- Python deps ----
COPY ./docker/requirements.txt requirements.txt
RUN --mount=type=secret,id=netrc,uid=${UID},target=/root/.netrc \
    uv pip install --system -r requirements.txt

# ---- Additional setup ----
#setup pinocchio cuda libraries # https://stack-of-tasks.github.io/pinocchio/download.html
COPY ./docker/setup_pinocchio.bash /var/setup_pinocchio.bash
RUN chmod +x /var/setup_pinocchio.bash && /var/setup_pinocchio.bash

# ---- Workspace (source) ----
COPY --chown=${UID}:${GID} . /ws
RUN chown ${UID}:${GID} /ws


# ---- Build workspace ----
USER ${USERNAME}

ARG ROS_PKG=displacement_planner
COPY --chown=${UID}:${GID} --chmod=0755 docker/colcon_build.sh /tmp/colcon_build.sh
RUN /tmp/colcon_build.sh "${ROS_PKG}" && rm -f /tmp/colcon_build.sh

COPY --chmod=0755 docker/entrypoint.sh /entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]