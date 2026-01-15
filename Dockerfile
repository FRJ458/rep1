FROM spgc/duckiebot-base-image:latest

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace

RUN apt install ros-humble-rqt ros-humble-rqt-common-plugins -y
RUN apt-get update && apt-get install -y \
    libx11-xcb1 \
    libxcb1 \
    libxcb-xinerama0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-randr0 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-icccm4 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-shm0 \
    libxcb-util1 \
    libxrender1 \
    libxext6 \
    libsm6 \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
COPY requirements-apt.txt .
COPY requirements-python.txt .
RUN set -e; \
    if [ -s requirements-apt.txt ]; then \
        apt update; \
        xargs -a requirements-apt.txt apt install -y; \
    fi

RUN set -e; \
    if [ -s requirements-python.txt ]; then \
        pip install -r requirements-python.txt; \
    fi
RUN pip3 uninstall -y opencv-python opencv-python-headless
RUN pip3 uninstall -y numpy && pip3 install "numpy<2"
