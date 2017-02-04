# Myo_API
This ROS package is for publish EMG and IMU sensor readings for Myo armband.

The code is written in Python and corrected some of the mistakes in the previous version.

The communication is through wireless Low energy BLuetooth protocal. The link is here https://github.com/thalmiclabs/myo-bluetooth

Moreover, it is a ROS package which can be run as a ros node to generate corresponding ros message.
## Requirements
 - python >=2.6
 - pySerial
 - enum34
 
## Usage
rosrun ros_myo_black myo-black-rawNode.py /dev/ttyACM0
note: PC maybe recognize a different port id.

## Message published:

/myo_black_imu

/myo_black_emg

/myo_black_arm

/myo_black_gest
