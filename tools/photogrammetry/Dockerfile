# tools/photogrammetry/Dockerfile — DES-005 D5/D2 wrapper image.
#
# Multi-arch (amd64 ground station + arm64/Jetson Orin companion) wrapper
# around `run_pipeline.py`. This is a SEPARATE image from the repository
# root Dockerfile (the ROS/colcon build used by CI) -- this package is pure
# Python + Docker (it shells out to a second, ODM, container at runtime) and
# has no ROS/C++ dependency, so it does not extend or touch the root image.
#
# Base: `python:3.11-slim` is an official multi-arch manifest (linux/amd64,
# linux/arm64/v8, ...) -- no arch-hardcoded base tag here, so
#   docker buildx build --platform linux/amd64,linux/arm64 \
#       -t <registry>/photogrammetry:<tag> tools/photogrammetry
# builds and pushes both arches from this one Dockerfile (MAP-8: "same
# package/CLI runs on both").
#
# ODM version pin (DES-005 D1): the reconstruction engine itself runs in a
# SEPARATE, already-multi-arch `opendronemap/odm` container invoked at
# runtime by odm_runner.py (docker-in-docker / sibling-container pattern --
# this image's own `docker` CLI below just needs to be able to *launch*
# that container, not bundle it). Keep this tag in lockstep with
# `photogrammetry/odm_runner.py`'s `ODM_IMAGE` constant:
#   opendronemap/odm:3.5.4
#
# Implements: MAP-8
FROM python:3.11-slim

# Pure-Python dependencies only (D2): numpy/shapely/PyYAML/Pillow for the
# check-mode geometry + manifest/CSV I/O, plus the `docker` CLI so this
# container can launch the sibling ODM container unattended (STK-1(d) --
# odm_runner.py never passes -i/-t/-it and always closes stdin).
RUN apt-get update && apt-get install -y --no-install-recommends \
        docker.io \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
        numpy \
        shapely \
        PyYAML \
        Pillow \
        pytest

WORKDIR /opt/photogrammetry
COPY photogrammetry/ ./photogrammetry/
COPY run_pipeline.py verify_dataset.py ./
COPY tests/ ./tests/

ENTRYPOINT ["python3", "/opt/photogrammetry/run_pipeline.py"]
CMD ["--help"]
