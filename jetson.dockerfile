# syntax=docker/dockerfile:1.2

FROM stereolabs/zed:5.0-runtime-jetson-jp6.0.0

# Setter opp cloning av github image
# Instalerer git og ssh client
RUN apt-get update && apt-get install -y \
    git \
    openssh-client

# Kopierer ssh key til docker image
RUN mkdir -p /root/.ssh && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

# Bygger med cloning hvis --ssh er lagt til i docker run komandoen
RUN --mount=type=ssh git clone git@github.com:philipahNOV/maze-2025.git /maze

RUN ln -s /maze/jetson/src /maze_jetson
WORKDIR /maze_jetson

VOLUME [ "/usr/local/zed/resources:/usr/local/zed/resources" ]

RUN apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libusb-1.0-0 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir -r requirements.txt

#CMD ["python3", "jetson/src/main.py"]