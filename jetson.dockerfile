
FROM stereolabs/zed:5.0-runtime-jetson-jp6.0.0

WORKDIR /maze_jetson

COPY ./requirements.txt .
COPY ./config.yaml .
COPY ./jetson jetson/
COPY ./pi pi/

RUN apt-get update && apt-get install -y \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libusb-1.0-0 \
    python3-pip

RUN rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir -r requirements.txt

#CMD ["python3", "jetson/src/main.py"]