# COMPASS Inference
This repository provides quick setup instructions for deploying COMPASS for inference. While the original [COMPASS repository](https://github.com/NVlabs/COMPASS) does not provide any guide for inference deploymnet, COMPASS is very similar to X-Mobility. Therefore, X-Mobility resources will be used as the backbone for COMPASS deployment.  

## Quick Setup on the Joey

#### 1. Clone the X-MOBILITY repository to your target device running Ubuntu.
```
git clone https://github.com/NVlabs/X-MOBILITY 
```

#### 2. Replace the following files in the X-MOBILITY directory. The files from my repository have been edited to work with COMPASS, following the instructions given [here](https://github.com/NVlabs/X-MOBILITY/pull/14/commits/d25bebf066a71e49b401c0fb7d60f7f26b324bb6). I also changed the ROS2 topic names in `x_mobility_navigator.py` to be compatible with the Joey. 
Download `x_mobility_navigator.launch.py` from this repo. Use it to replace the `X-MOBILITY/ros2_deployment/x_mobility_navigator/launch/x_mobility_navigator.launch.py` file. 

Download `x_mobility_navigator.py` from this repo. Use it to replace the `X-MOBILITY/ros2_deployment/x_mobility_navigator/x_mobility_navigator/x_mobility_navigator.py` file. 

#### 3. Install TensorRT
Ensure the pip Python module is up-to-date and the wheel Python module is installed
```
python3 -m pip install --upgrade pip
python3 -m pip install wheel
```
Install the TensorRT Python wheel
```
python3 -m pip install --upgrade tensorrt
```
Check if installation was successful
```
python3 -c "import tensorrt"
```
Install PyCUDA
```
pip3 install pycuda
```

#### 4. Setup your ROS2 workspace and build the x_mobility_navigator package
Create the ROS2 workspace directory.
```
mkdir -p ~/ros2_ws/src
```
Create a symlink to the x_mobility_navigator ROS2 package in this repository.
```
ln -s <repo root>/ros2_deployment/x_mobility_navigator ~/ros2_ws/src/x_mobility_navigator
```
Build the ROS2 workspace. 
```
cd ~/ros2_ws
```
```
colcon build --symlink-install
```
```
source install/setup.bash
```

#### 5. Download the `compassGeneralistJoey.trt` file [here](https://drive.google.com/file/d/1PwcrBSycGQe3gUwkXYYPnmfPeqgOxEii/view?usp=sharing) and copy it to `/tmp/compass.engine`.

This trt file is the TensorRT engine generated from the COMPASS generalist policy. Camera resolution is 1920x1080 to match the image resolution of the D435i. 

#### 6. Launch COMPASS inference using the prebuilt TensorRT engine
```
ros2 launch x_mobility_navigator x_mobility_navigator.launch.py
```

#### 7. Re-generating the ONNX and TRT files
If needed, regenerate the ONNX file using the instructions found in 
