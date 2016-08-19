/*
 * Copyright (c) 2016, University Of Massachusetts Lowell
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University of Massachusetts Lowell nor the names
 * from of its contributors may be used to endorse or promote products
 * derived this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * Author: Carlos Ibarra <clopez@cs.uml.edu>
*/
 
#ifndef TANGO_NATIVE_STREAMING_TANGO_NATIVE_STREAMING_H_
#define TANGO_NATIVE_STREAMING_TANGO_NATIVE_STREAMING_H_

#include <android/log.h>
#include <pthread.h>

//TANGO INCLUDES
#include "tango_client_api.h" 
#include "tango_support_api.h" 

//ROS INCLUDES
#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define LOG_TAG "tango_native_streaming"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace tango_native_streaming {

struct tango_context{
  ros::NodeHandle* nh;
  TangoSupportPointCloudManager *pc_manager;
  pthread_mutex_t* pose_mutex_ptr;
  geometry_msgs::TransformStamped* odom_to_base_ptr;
};

class TangoNativeStreamingApp {
 public:
  unsigned int seq;
  unsigned int map_to_odom_seq;
  unsigned int odom_to_base_seq;
  pthread_t pub_thread;
  pthread_mutex_t pose_mutex;
  ros::Publisher pc_pub;
  ros::Subscriber known_pose_sub;
  sensor_msgs::PointCloud2 pc_msg;
  tango_context ctxt;
  geometry_msgs::TransformStamped map_to_odom;
  geometry_msgs::TransformStamped odom_to_base;
  geometry_msgs::TransformStamped base_to_pose; //tf from tango pose, to pose of person holding tango forward
  geometry_msgs::TransformStamped base_to_depth;
  geometry_msgs::TransformStamped base_to_color;
  tf2_ros::TransformBroadcaster* tf_bcaster;
  tf2_ros::StaticTransformBroadcaster* static_tf_bcaster;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  // Class constructor.
  TangoNativeStreamingApp() : tango_config_(nullptr), tf_bcaster(nullptr), static_tf_bcaster(nullptr), tf_buffer(nullptr), tf_listener(nullptr), map_to_odom_seq(0), odom_to_base_seq(0) {}

  // Class destructor.
  ~TangoNativeStreamingApp() {
    if (tango_config_ != nullptr) {
      TangoConfig_free(tango_config_);
      tango_config_ = nullptr;
    }
    if (tf_bcaster != nullptr)
      delete tf_bcaster;
    if (static_tf_bcaster != nullptr)
      delete static_tf_bcaster;
    if (tf_buffer != nullptr)
      delete tf_buffer;
    if (tf_listener != nullptr)
      delete tf_listener;
  }

  void OnCreate(JNIEnv* env, jobject caller_activity);

  void OnTangoServiceConnected(JNIEnv* env, jobject binder);

  void OnPause();

  void SetCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

 private:
  TangoConfig tango_config_;
};
}  // namespace

#endif  // TANGO_NATIVE_STREAMING_TANGO_NATIVE_STREAMING_H_
