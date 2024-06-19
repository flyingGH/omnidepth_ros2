FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update

RUN apt-get install -y git && apt-get install -y python3-pip

RUN mkdir -p ~/omnidepth_ws/src && \
    cd ~/omnidepth_ws/src/ && \
    git clone https://github.com/synapsemobility/omnidepth_ros2.git

# COPY ./requirements.txt .
# RUN pip3 install --no-cache-dir --upgrade -r requirements.txt
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# RUN mkdir -p ~/omnidepth_ws/src/omnidepth_ros2/models
# COPY ./omnidepth_v10.onnx /root/omnidepth_ws/src/omnidepth_ros2/models/