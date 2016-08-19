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


///General TODO: Publish Video, Color Pointcloud, Manage lifecycle on resume

#include <cstdlib>

#include <tango_support_api.h>

#include "tango_native_streaming/tango_native_streaming.h"

#include <tf/transform_datatypes.h>

//TODO: Set these through Java UI instead
//set to ROS_MASTER uri (including http:// and port)
#define ROS_MASTER "http://10.185.0.249:11311"

//set to Tango's IP
#define ROS_IP "10.185.0.224"

//optional, prefixes tf names with it, must either be empty, or end in a forward slash /
#define TANGO_PREFIX "tango_brain_0/"

//namespace for the node, also used as node name, required
#define NAMESPACE "tango_brain_0"

namespace {

// The minimum Tango Core version required from this application.
constexpr int kTangoCoreMinimumVersion = 9377;

void onPointCloudAvailable(void* context, const TangoXYZij* point_cloud) {
  // Number of points in the point cloud.
  int point_cloud_size;
  float average_depth;
  int ret;
  tango_native_streaming::tango_context* ctxt = (tango_native_streaming::tango_context*)context;
  ret = TangoSupport_updatePointCloud(ctxt->pc_manager, point_cloud);
  if (ret != TANGO_SUCCESS)
  {
    LOGE("ERROR UPDATING TANGO MANAGER");
  }
}

void onPoseAvailable(void* context, const TangoPoseData *pose) {
  tango_native_streaming::tango_context* ctxt = (tango_native_streaming::tango_context*)context;
  if (pthread_mutex_trylock(ctxt->pose_mutex_ptr) == 0)
  {
    ctxt->odom_to_base_ptr->transform.translation.x = pose->translation[0];
    ctxt->odom_to_base_ptr->transform.translation.y = pose->translation[1];
    ctxt->odom_to_base_ptr->transform.translation.z = pose->translation[2];
    ctxt->odom_to_base_ptr->transform.rotation.x = pose->orientation[0];
    ctxt->odom_to_base_ptr->transform.rotation.y = pose->orientation[1];
    ctxt->odom_to_base_ptr->transform.rotation.z = pose->orientation[2];
    ctxt->odom_to_base_ptr->transform.rotation.w = pose->orientation[3];
    pthread_mutex_unlock(ctxt->pose_mutex_ptr);
  }
}

void* pub_thread_method(void* arg)
{
    tango_native_streaming::TangoNativeStreamingApp* app;
    app = (tango_native_streaming::TangoNativeStreamingApp*) arg;
    ros::Rate rate(10);
    TangoErrorType ret;
    TangoXYZij* pc_ptr;
    bool new_available;
    while (ros::ok())
    {
        ret = TangoSupport_getLatestPointCloudAndNewDataFlag((app->ctxt).pc_manager, &pc_ptr, &new_available);
        if (ret != TANGO_SUCCESS)
        {
            LOGE("Error retrieving latest pointcloud");
        }
        if (new_available)
        {
            app->pc_msg.width = app->pc_msg.row_step = pc_ptr->xyz_count;
            app->pc_msg.header.seq = app->seq++;
            app->pc_msg.header.stamp = ros::Time::now();
            //LOGI("Header stamp: %f", app->pc_msg.header.stamp.toSec());
            app->pc_msg.data.resize(3* sizeof(float) * pc_ptr->xyz_count);
            memcpy(&app->pc_msg.data[0], (void*)pc_ptr->xyz,  pc_ptr->xyz_count * 3 * sizeof(float));
            app->pc_pub.publish(app->pc_msg);
        }
        pthread_mutex_lock(&(app->pose_mutex));
        app->map_to_odom.header.seq = app->map_to_odom_seq++;
        app->odom_to_base.header.seq = app->odom_to_base_seq++;
        app->map_to_odom.header.stamp = app->odom_to_base.header.stamp = ros::Time::now();
        app->tf_bcaster->sendTransform(app->map_to_odom);
        app->tf_bcaster->sendTransform(app->odom_to_base);
        pthread_mutex_unlock(&(app->pose_mutex));
        ros::spinOnce();
        rate.sleep();
    }
}
}

namespace tango_native_streaming {

//TODO: Run ROS initialization on separate thread so Android does not complain
void TangoNativeStreamingApp::OnCreate(JNIEnv* env, jobject caller_activity) {
  int seq = 0;
  sensor_msgs::PointField x, y, z;
  x.name = "x";
  y.name = "y";
  z.name = "z";
  x.offset = 0;
  y.offset = sizeof(float);
  z.offset = 2.0*sizeof(float);
  x.count = y.count = z.count = 1;
  x.datatype = y.datatype = z.datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.header.frame_id = TANGO_PREFIX"tango_camera_depth";
  pc_msg.height = 1;
  pc_msg.is_bigendian = false;
  pc_msg.is_dense = true;
  pc_msg.point_step = 3 * sizeof(float);
  pc_msg.fields.push_back(x);
  pc_msg.fields.push_back(y);
  pc_msg.fields.push_back(z);
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  pthread_mutex_init(&pose_mutex, NULL);
  int32_t max_point_cloud_elements;
  int ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                                       &max_point_cloud_elements);
  if(ret != TANGO_SUCCESS) {
    LOGE("Failed to query maximum number of point cloud elements.");
  }

  ret = TangoSupport_createPointCloudManager(max_point_cloud_elements, &(ctxt.pc_manager));
  if(ret != TANGO_SUCCESS) {
      LOGE("Failed to create support point cloud manager");
  }

  int argc = 3;
  char *argv[] = {"nothing_important" , "__master:=" ROS_MASTER, "__ip:=" ROS_IP};
  LOGI("GOING TO ROS INIT");

  for(int i = 0; i < argc; i++){
      LOGI("%s", argv[i]);
  }
  ros::init(argc, &argv[0], NAMESPACE);

  LOGI("GOING TO NODEHANDLE");

  // %Tag(ROS_MASTER)%
  std::string master_uri = ros::master::getURI();

  if(ros::master::check()){
      LOGI("ROS MASTER IS UP!");
  } else {
      LOGI("NO ROS MASTER.");
  }
  LOGI("%s", master_uri.c_str());
  ctxt.pose_mutex_ptr = &pose_mutex;
  ctxt.odom_to_base_ptr = &odom_to_base;
  ctxt.nh = new ros::NodeHandle(NAMESPACE);
  tf_bcaster = new tf2_ros::TransformBroadcaster;
  static_tf_bcaster = new tf2_ros::StaticTransformBroadcaster;
  tf_buffer = new tf2_ros::Buffer;
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = TANGO_PREFIX"odom";
  map_to_odom.transform.translation.x = 0;
  map_to_odom.transform.translation.y = 0;
  map_to_odom.transform.translation.z = 0;
  map_to_odom.transform.rotation.x = 0;
  map_to_odom.transform.rotation.y = 0;
  map_to_odom.transform.rotation.z = 0;
  map_to_odom.transform.rotation.w = 1;
  odom_to_base.header.frame_id = TANGO_PREFIX"odom";
  odom_to_base.child_frame_id = TANGO_PREFIX"tango_base_link";
  odom_to_base.transform.translation.x = 0;
  odom_to_base.transform.translation.y = 0;
  odom_to_base.transform.translation.z = 0;
  odom_to_base.transform.rotation.x = 0;
  odom_to_base.transform.rotation.y = 0;
  odom_to_base.transform.rotation.z = 0;
  odom_to_base.transform.rotation.w = 1;
  base_to_pose.header.frame_id = TANGO_PREFIX"tango_base_link";
  base_to_pose.child_frame_id = TANGO_PREFIX"tango_pose";
  base_to_pose.transform.translation.x = 0;
  base_to_pose.transform.translation.y = 0;
  base_to_pose.transform.translation.z = 0;
  base_to_pose.transform.rotation.x = -0.5;
  base_to_pose.transform.rotation.y = 0.5;
  base_to_pose.transform.rotation.z = 0.5;
  base_to_pose.transform.rotation.w = 0.5;
  base_to_depth.header.frame_id = TANGO_PREFIX"tango_base_link";
  base_to_depth.child_frame_id = TANGO_PREFIX"tango_camera_depth";
  base_to_color.header.frame_id = TANGO_PREFIX"tango_base_link";
  base_to_color.child_frame_id = TANGO_PREFIX"tango_camera_color";
  pc_pub = (ctxt.nh)->advertise<sensor_msgs::PointCloud2>("tango_image_depth", 1);
  known_pose_sub = (ctxt.nh)->subscribe("initial_pose", 1, &TangoNativeStreamingApp::SetCurrentPoseCallback, this);
  int version = 0;
  TangoErrorType err = TangoSupport_GetTangoVersion(env, caller_activity, &version);
  if (err != TANGO_SUCCESS || version < kTangoCoreMinimumVersion) {
    LOGE(
        "TangoNativeStreamingApp::OnCreate, Tango Core version is out"
        " of date.");
    std::exit(EXIT_SUCCESS);
  }
}

void TangoNativeStreamingApp::SetCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& known_pose)
{
  LOGI("Will set pose to known");
  bool tf_success = true;
  geometry_msgs::TransformStamped odom_to_pose_tf;
  try{
    odom_to_pose_tf = tf_buffer->lookupTransform(TANGO_PREFIX"odom", TANGO_PREFIX"tango_pose",ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    LOGE("Could not get tf from odom to tango_pose, not setting pose");
    tf_success = false;
  }
  if (tf_success)
  {
    tf::Pose known, maptoodom;
    tf::Vector3 maptoodom_vect;
    tf::Quaternion maptoodom_quat;
    tf::poseMsgToTF(known_pose->pose.pose, known);
    pthread_mutex_lock(&pose_mutex);
    tf::Quaternion odom_to_pose_quat(odom_to_pose_tf.transform.rotation.x, odom_to_pose_tf.transform.rotation.y, odom_to_pose_tf.transform.rotation.z, odom_to_pose_tf.transform.rotation.w);
    tf::Vector3 odom_to_pose_vect(odom_to_pose_tf.transform.translation.x, odom_to_pose_tf.transform.translation.y, odom_to_pose_tf.transform.translation.z);
    tf::Pose odom_to_pose(odom_to_pose_quat, odom_to_pose_vect);
    maptoodom = known * (odom_to_pose.inverse());
    maptoodom_quat = maptoodom.getRotation();
    maptoodom_vect = maptoodom.getOrigin();
    map_to_odom.transform.translation.x = maptoodom_vect.getX();
    map_to_odom.transform.translation.y = maptoodom_vect.getY();
    map_to_odom.transform.translation.z = maptoodom_vect.getZ();
    map_to_odom.transform.rotation.x = maptoodom_quat.getX();
    map_to_odom.transform.rotation.y = maptoodom_quat.getY();
    map_to_odom.transform.rotation.z = maptoodom_quat.getZ();
    map_to_odom.transform.rotation.w = maptoodom_quat.getW();
    pthread_mutex_unlock(&pose_mutex);
  }
}

void TangoNativeStreamingApp::OnTangoServiceConnected(JNIEnv* env, jobject binder) {
  if (TangoService_setBinder(env, binder) != TANGO_SUCCESS) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "TangoService_setBinder error");
    std::exit(EXIT_SUCCESS);
  }

  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "TangoService_getConfig error.");
    std::exit(EXIT_SUCCESS);
  }

  TangoErrorType err =
      TangoConfig_setBool(tango_config_, "config_enable_depth", true);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "config_enable_depth() failed with error code: %d.",
        err);
    std::exit(EXIT_SUCCESS);
  }

  err = TangoService_connectOnXYZijAvailable(onPointCloudAvailable);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "Failed to connect to point cloud callback with error code: %d",
        err);
    std::exit(EXIT_SUCCESS);
  }

  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pair.target = TANGO_COORDINATE_FRAME_DEVICE;

  err = TangoService_connectOnPoseAvailable(1, &pair, onPoseAvailable);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "Failed to connect to point cloud callback with error code: %d",
        err);
    std::exit(EXIT_SUCCESS);
  }

  err = TangoService_connect((void*)&ctxt, tango_config_);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "Failed to connect to the Tango service with error code: %d",
        err);
    std::exit(EXIT_SUCCESS);
  }

  //TODO: Verify correctness of this TF
  TangoPoseData cToIMUPose;
  TangoCoordinateFramePair cToIMUPair;
  cToIMUPair.base = TANGO_COORDINATE_FRAME_IMU;
  cToIMUPair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  TangoService_getPoseAtTime(0.0, cToIMUPair, &cToIMUPose);
  base_to_color.transform.translation.x = cToIMUPose.translation[0];
  base_to_color.transform.translation.y = cToIMUPose.translation[1];
  base_to_color.transform.translation.z = cToIMUPose.translation[2];
  base_to_color.transform.rotation.x = cToIMUPose.orientation[0];
  base_to_color.transform.rotation.y = cToIMUPose.orientation[1];
  base_to_color.transform.rotation.z = cToIMUPose.orientation[2];
  base_to_color.transform.rotation.w = cToIMUPose.orientation[3];

  //Fixed rotation by rotating 90 degrees clockwise over z axis, might not match in other Tango devices?
  tf::Quaternion clockwise90Z = tf::createQuaternionFromYaw(1.57079632679);
  cToIMUPair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
  TangoService_getPoseAtTime(0.0, cToIMUPair, &cToIMUPose);
  base_to_depth.transform.translation.x = cToIMUPose.translation[0];
  base_to_depth.transform.translation.y = cToIMUPose.translation[1];
  base_to_depth.transform.translation.z = cToIMUPose.translation[2];
  base_to_depth.transform.rotation.x = cToIMUPose.orientation[0];
  base_to_depth.transform.rotation.y = cToIMUPose.orientation[1];
  base_to_depth.transform.rotation.z = cToIMUPose.orientation[2];
  base_to_depth.transform.rotation.w = cToIMUPose.orientation[3];
  tf::Quaternion base_to_depth_rotation;
  tf::quaternionMsgToTF(base_to_depth.transform.rotation, base_to_depth_rotation);
  tf::quaternionTFToMsg (base_to_depth_rotation * clockwise90Z, base_to_depth.transform.rotation);
  base_to_depth.header.stamp = base_to_color.header.stamp = base_to_pose.header.stamp = ros::Time::now();
  static_tf_bcaster->sendTransform(base_to_pose);
  static_tf_bcaster->sendTransform(base_to_depth);
  static_tf_bcaster->sendTransform(base_to_color);
  pthread_create(&pub_thread, NULL, pub_thread_method, (void*)this);
}

void TangoNativeStreamingApp::OnPause() {
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();
  LOGI("Shutting down ros");
  ros::shutdown();
  pthread_join(pub_thread, NULL);
  LOGI("Done");
}
}  // namespace 