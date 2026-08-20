FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libegl1 \
        libgl1 \
        libglfw3 \
        pkg-config \
        python-is-python3 \
        python3 \
        python3-jinja2 \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir \
        matplotlib==3.10.5 \
        mujoco==3.3.5 \
        numpy==2.3.2

ENV MPLBACKEND=Agg
