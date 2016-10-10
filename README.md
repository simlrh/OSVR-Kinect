# OSVR-Kinect

## Usage

Install the Kinect runtime (v1.8 for Xbox 360 version, v2.0 for Xbox One). Copy the dll to your osvr-plugins-0 folder (the binary should match your OSVR version - the OSVR all-in-one installer is 32-bit).

When you start osvr_server a config window should pop up showing how many bodies are visible to the Kinect sensor, allowing you to choose which body is tracked. You can also recenter the coordinate system and activate seated mode if you're using Kinect 1.

## Building

Pre-compiled binaries are available on the [releases page](https://github.com/simlrh/OSVR-Kinect/releases).

An OSVR plugin providing Kinect SDK position and orientation joint tracking, for use with a Kinect for Xbox One, or Kinect for Xbox 360.

    git clone https://github.com/simlrh/OSVR-Kinect
    cd OSVR-Kinect
    git submodule init
    git submodule update

Then follow the standard OSVR plugin build instructions.
