FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-2f516bb AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/
RUN echo "yaml file:///main_ws/src/packaging/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/52-fogsw-module.list
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-2f516bb

ENTRYPOINT /entrypoint.sh

RUN apt-get update && \
	apt-get install -y --no-install-recommends \
        libsurvive \
        ros-galactic-angles \
        ros-galactic-geographic-msgs \
        libatlas3-base \
        liblapacke \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-indoor-pos_*_amd64.deb /indoor_pos.deb
RUN dpkg -i /indoor_pos.deb && rm /indoor_pos.deb

