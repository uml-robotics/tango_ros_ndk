##Project Tango Native ROS Streaming App
App that streams information captured by the Project Tango sensors over ROS (Pose and Pointcloud for now), using Ekumen Labs' native C++ port of ROS for Android (see https://github.com/ekumenlabs/roscpp_android).

#Build Prerequisites
In order to build, you need to have the Android SDK and NDK installed, along with Android Studio (tested on version 2.1.2), or standalone Gradle. 

#Setup
This app requires both the Tango Client Library and the Tango Support Library, along with the binary distribution of the NDK ROSCPP port, they should be extracted in the specified folders. The Tango Client Library and Support Library (both for the C language) can be downloaded from Google, [here](https://developers.google.com/tango/downloads), current tested version is Tania Borealis.

The NDK ROSCPP port binary distribution can be downloaded from [here](https://github.com/ekumenlabs/roscpp_android/releases), under the Portable distribution section, file `roscpp_android_ndk.tar.gz`.

Finally, some settings need to be changed (Via code for now, will change soon to be settable via the GUI on each run), on the file `tango_ros_ndk/tango_native_ros/tango_native_streaming/src/main/jni/tango_native_streaming.cpp`, the defines at the top, ROS_MASTER_URI, ROS_HOSTNAME, and NAMESPACE need to be set, ROS_MASTER_URI to the address of the ROS MASTER to use (including http:// and the port number), ROS_HOSTNAME to the Tango's IP address, and NAMESPACE to the desired name of the node, and topic namespace. There is also an optional TANGO_PREFIX defined, this, if nonempty, is added as a prefix to the tf names.

#Published Topics
`tango_image_depth` contains the pointcloud captured by the Tango.

#Subscribed Topics
`initial_pose` used to set a known pose. 

#Provided Transforms
1. `map -> odom` Initially a unit transform, changes if initial_pose is used to correct the position to a known one.

2. `odom -> base_link` Tango pose information, as provided by the API from `COORDINATE_FRAME_START_OF_SERVICE` to `COORDINATE_FRAME_DEVICE`

3. `base_link -> tango_pose` Tango's default pose information does not coincide with the pose of a person holding the tablet in a normal orientation, so this transform is provided to coincide with the pose of the user.

4. `base_link -> tango_camera_depth` Provides the depth camera position relative to the device.

5. `base_link -> tango_camera_color` Provides the color camera position relative to the device. (Untested to be correct yet, since color camera information is not being transmitted).

#Future Improvements
- Set configurable options via the user interface instead of code.
- Transmit color images.
- Fuse pointclouds with color camera data to transmit XYZRGB color pointclouds.
- Use ADF for relocating in known areas.