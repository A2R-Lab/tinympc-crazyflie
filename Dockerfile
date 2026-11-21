FROM ubuntu:24.04

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        bison \
        build-essential \
        ca-certificates \
        cmake \
        flex \
        gcc-arm-none-eabi \
        git \
        libncurses-dev \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        libusb-1.0-0 \
        ninja-build \
        python3 \
        python3-pip \
        python3-venv \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/cfclient \
    && /opt/cfclient/bin/pip install --no-cache-dir cfclient textual typer

ENV PATH="/opt/cfclient/bin:${PATH}"

WORKDIR /workspace
CMD ["/bin/bash"]
