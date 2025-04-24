# Obtaining user's body status and intention

## Calibrate the realsense D455

**Notice : The realsense D455 must be used with a USB 3.0 cable and interface, otherwise it will not work properly.**

Using ros calibration package:

```
sudo apt-get install ros-noetic-calibration
```

You can also use MATLAB calibration toolbox to calibrate.

## Mediapipe version

## 1. Create a conda envirment

[Mediapipe official website](https://developers.google.com/mediapipe)

```
conda create -n mediapipe python=3.7
conda activate mediapipe
pip install mediapipe 
pip install opencv-python
pip install opencv-contrib-python
pip install -r doc/mediapipe/requirements.txt
```

## 2. Start the program

```
conda activate mediapipe
source ./devel/setup.bash
rosrun realsense intent_mediapipe.py
```

This node will publish the /result and /Pose topics. When the user raises their arm, /result publishs True, and /Pose publishs the position of the user's hip in the camera coordinate system (after calibration it will in the world coordinate).

## Pytorch-openpose version

### 1. Create a conda envirment

[pytorch-openpose](https://github.com/Hzzone/pytorch-openpose)

```
conda create -n pytorch-openpose python=3.7
conda activate pytorch-openpose
pip install -r doc/openpose/requirements.txt
```

## 2. Start the program

```
conda pytorch-openpose
cd src/realsense/scripts
python intent_openpose.py
```

This node will publish the /result and /Pose topics. When the user raises their arm, /result publishs True, and /Pose publishs the position of the user's hip in the camera coordinate system (after calibration it will in the world coordinate).