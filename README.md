# OSVR-Kinect [![Donate](https://nourish.je/assets/images/donate.svg)](http://ko-fi.com/A250KJT)

## Usage

Install the Kinect runtime (v1.8 for Xbox 360 version, v2.0 for Xbox One). Copy the dll to your osvr-plugins-0 folder (the binary should match your OSVR version - the OSVR all-in-one installer is 32-bit).

When you start osvr_server a config window should pop up showing how many bodies are visible to the Kinect sensor, allowing you to choose which body is tracked. You can also recenter the coordinate system and activate seated mode if you're using Kinect 1.

# Tracker alignment

When using a HMD the orientation and position data will likely be misaligned, eg, you are facing forward and leaning forward, but your tracked position instead moves to the side. To correct this, align the orientation tracker with the position tracker's axes and run osvr_reset_yaw on the orientation tracker.

For example, with the OSVR HDK and a Kinect, you would place the HDK in front of the Kinect, pointing towards it, then run

    osvr_reset_yaw.exe --path "/com_osvr_Multiserver/OSVRHackerDevKitPrediction0/semantic/hmd"

## Building

Pre-compiled binaries are available on the [releases page](https://github.com/simlrh/OSVR-Kinect/releases).

An OSVR plugin providing Kinect SDK position and orientation joint tracking, for use with a Kinect for Xbox One, or Kinect for Xbox 360.

    git clone https://github.com/simlrh/OSVR-Kinect
    cd OSVR-Kinect
    git submodule init
    git submodule update

Then follow the standard OSVR plugin build instructions.
