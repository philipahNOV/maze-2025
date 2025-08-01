
FROM ubuntu:24.04

WORKDIR /maze_jetson

COPY ./requirements_pi.txt .
COPY ./config.yaml .
COPY ./pi pi/

RUN apt-get update && apt-get install -y \
    git \
    libglib2.0-0 \
    python3-tk \
    python3-pip

RUN rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir -r requirements_pi.txt

#CMD ["python3", "jetson/src/main.py"]