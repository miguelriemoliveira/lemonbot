# lemonbot

Drivers for the Lemonbot hardware platform.

## Bringup the system

```
roslaunch lemonbot_bringup all.launch
```

## Control the PTU using the terminal

Bringup the system, see [this section](#bringup-the-system)

and now request a new position

```
rostopic pub /ptu/cmd sensor_msgs/JointState "header:
seq: 0
stamp: {secs: 0, nsecs: 0}
frame_id: ''
name: ['ptu_pan','ptu_tilt']
position: [0.5,0.1]
velocity: [1,1]
effort: [0]"
```

## Single Acquisition Mode

To take a single acquisition just execute the launch file in lemonbot_acquisition `single acquisition` with the specific parameters.

```
roslaunch lemonbot_acquisition single_acquisition.launch min:=-90 max:=90 vel:=10
```

## System calibration

### Calibration files

There are two calibration files, for the extrinsic calibration of both the camera and the laser. These files are located under `lemonbot_calibration/calibration`. They look like:

```yaml
xyz: [0, 0, 0]
rpy: [0, 0, 0]
```

These files are automatically loaded to the `robot_description` using the xacro preprocessor.

# Remote connection through SSH

It it possible to connect to the lemonbot through a ssh connection, using the wifi provided by the router in the mobile station.
The first connection can be made by ssh'ing into the portable computer using the command

```
ssh <ip of the computer> -l lemonbot
```

The username and password are both `lemonbot`.

## Passwordless login

To facilitate the login, we can use a public-private key. To do so, create a key pair in the client machine with the command `ssh-keygen -t rsa` and the copy the public key to the lemonbot computer using the command `ssh-copy-id lemonbot@<ip of the computer>`. Now use the first command to login without a password.

# wstool for dependency management

## Download `lemonbot.rosinstall`

A `.rosinstall` file contains the package dependencies and the versions for all the packages required to use with the lemonbot package, including their repositories (git).

To download the `lemonbot.rosinstall`, use the following command: 

```
wget <lemonbot.rosinstall>
```

## Initialize a `src` folder for the packages.

Then in the same folder as the `.rosinstall` file, clone all the dependencies into the folder `src`:

```
wstool init src lemonbot.rosinstall
```

To update all the repositories, just use the `update` command of wstool:

```
wstool update -t src
```

Note: to execute the command faster, use multiple jobs to parallelize the clone/pull, with the argument `-j8`.

# Calibrations

## Intrinsic calibration of the RGB Camera



## Extrinsic calibration of the RGB Camera to the PTU (hand2eye calibration)

To start the lemonbot in calibration "mode", roslaunch the `lemonbot_calibration/manual_control` and the
`lemonbot_calibration/extrinsic_calibration` and  like so:

```
roslaunch lemonbot_calibration manual_control.launch
roslaunch lemonbot_calibration extrinsic_calibration.launch fiducial_len:=0.093 fiducial_id:=10
```

Then start the calibration client to capture all the required poses and follow it's instructions

```
rosrun lemonbot_calibration calibrate_rgb_camera.py
```
