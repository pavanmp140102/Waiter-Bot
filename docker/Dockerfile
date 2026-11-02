# Multi-stage image for Waiter-Bot (see docs/plan/M8_docker.md).
#   base    : ROS 2 Jazzy plus every dependency declared in the package.xml files
#   runtime : base plus the source tree, built with colcon
# Pinned by digest (2026-10-30) so every machine builds from the same base image.
ARG ROS_IMAGE=ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

# Keeps only the package.xml files. The next stage copies from here, so its cache key depends
# on the manifests alone and editing source files does not rerun the dependency install.
FROM ${ROS_IMAGE} AS manifests
COPY src/ /src/
RUN find /src -type f ! -name package.xml -delete \
    && find /src -mindepth 1 -type d -empty -delete

FROM ${ROS_IMAGE} AS base
# ARG, not ENV, so the setting applies to the build but does not stay in the final image.
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ws

# Dependencies come first and only from the manifests, so a key missing from a package.xml
# fails the build here instead of going unnoticed.
COPY --from=manifests /src/ /tmp/src_manifests/
RUN echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/90norecommends \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/src_manifests --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /tmp/src_manifests /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# A plain `colcon build` (not --symlink-install): the image tests a real install, where the host's
# development build uses symlinks.
FROM base AS runtime
COPY src/ /ws/src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build
