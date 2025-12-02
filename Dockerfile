ARG FROM_IMAGE=ros:humble-ros-base
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# Common runtime/build deps
FROM $FROM_IMAGE AS base
ARG OVERLAY_WS

# overwrite defaults to persist minimal cache
RUN rosdep update --rosdistro $ROS_DISTRO && \
  cat <<EOF > /etc/apt/apt.conf.d/docker-clean && apt-get update
APT::Install-Recommends "false";
APT::Install-Suggests "false";
EOF


RUN rm -rf /var/lib/apt/lists/* && \
  apt-get update && \
  apt-get install -y \
  libeigen3-dev \
  python3-venv \
  python3-pip \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  && rm -rf /var/lib/apt/lists/* 

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV OVERLAY_WS=${OVERLAY_WS}
WORKDIR $OVERLAY_WS

# ----- DEV STAGE -------

FROM base AS dev

ARG OVERLAY_WS
ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN apt-get update && apt-get install -y \
  ros-humble-rviz2 \
  clangd \
  gdb \
  curl \
  ca-certificates \
  fzf \
  ripgrep \
  fd-find \
  xclip \
  nodejs \
  npm \
  build-essential \
  unzip \
  xz-utils && rm -rf /var/lib/apt/lists/* 

# Install Neovim 0.11.5 (Currently installing x86-64)
RUN curl -L https://github.com/neovim/neovim/releases/download/v0.11.5/nvim-linux-x86_64.tar.gz \
  -o /tmp/nvim.tar.gz && \
  tar xzf /tmp/nvim.tar.gz -C /opt && \
  ln -s /opt/nvim-linux-x86_64/bin/nvim /usr/local/bin/nvim && \
  rm /tmp/nvim.tar.gz 

# Create user matching host UID/GID
RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
  && mkdir -p ${OVERLAY_WS} \
  && chown -R ${USERNAME}:${USERNAME} ${OVERLAY_WS} \
  && mkdir -p /home/${USERNAME}/.local/state /home/${USERNAME}/.local/share \
  && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.local 


USER ${USERNAME}

# Clone custom dotfiles for neovim setup
# Cache repo head version to avoid caching in case of changes
ADD https://api.github.com/repos/EmanueleGiacomini/.dotfiles/git/refs/heads/main version.json
RUN git clone https://github.com/EmanueleGiacomini/.dotfiles.git /home/${USERNAME}/dotfiles && \
  mkdir /home/${USERNAME}/.config/ && \
  ln -s /home/${USERNAME}/dotfiles/.config/nvim /home/${USERNAME}/.config/nvim

COPY --chown=${USERNAME}:${USERNAME} scripts/update_lazyvim.lua /tmp/

RUN /usr/local/bin/nvim --headless -c "luafile /tmp/update_lazyvim.lua" -c "qall" \ 
  && rm /tmp/update_lazyvim.lua

RUN mkdir -p ${OVERLAY_WS}/src ${OVERLAY_WS}/build ${OVERLAY_WS}/install ${OVERLAY_WS}/log 

WORKDIR ${OVERLAY_WS}


# ----- BUILD STAGE ------

FROM base AS builder

# Repo with multiple packages goes under src/mad-icp
COPY . src/mad-icp

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build \
    --merge-install \
    --symlink-install \
    --base-paths src/mad-icp

# ----- DEPLOYMENT STAGE ------
FROM ros:humble-ros-core AS robot
ARG OVERLAY_WS
ENV OVERLAY_WS=${OVERLAY_WS}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR ${OVERLAY_WS}

COPY --from=builder ${OVERLAY_WS}/install ${OVERLAY_WS}/install

CMD ["/bin/bash"]
