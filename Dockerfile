FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN set -eux; \
    for i in 1 2 3 4 5; do apt-get update && break || sleep 5; done; \
    apt-get install -y --no-install-recommends \
        build-essential \
        python3-colcon-common-extensions \
        can-utils \
        net-tools \
        iproute2; \
    # Try installing optional ROS socketcan bridge package if available, but don't fail the build if it's not
    apt-get install -y --no-install-recommends ros-humble-can-msgs || true; \
    rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash developer
USER developer
WORKDIR /ws

CMD ["bash"]
