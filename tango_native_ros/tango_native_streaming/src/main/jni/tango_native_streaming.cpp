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

#include <jni.h>

#include <tango_support_api.h>

#include "tango_native_streaming/tango_native_streaming.h"

#include <tf/transform_datatypes.h>

#include <endian.h>



char *ROS_MASTER_URI_PREFIX = "__master:=",
     *ROS_IP_URI_PREFIX = "__ip:=",
     *TANGO_CAMERA_DEPTH_SUFFIX = "tango_camera_depth",
     *TANGO_CAMERA_COLOR_SUFFIX = "tango_camera_color",
     *TANGO_ODOM_SUFFIX = "odom",
     *TANGO_BASE_LINK_SUFFIX = "tango_base_link",
     *TANGO_POSE_SUFFIX = "tango_pose";

char ros_master[256],
     ros_ip[256],
     tango_prefix[256],
     tango_namespace[256],
     ros_master_uri[256],
     ros_ip_uri[256],
     tango_camera_depth[256],
     tango_camera_color[256],
     tango_odom[256],
     tango_base_link[256],
     tango_pose[256];

     static bool running = false, error = false;

namespace {

// The minimum Tango Core version required from this application.
constexpr int kTangoCoreMinimumVersion = 9377;

void onFrameAvailable(void* context, TangoCameraId id, const TangoImageBuffer *buffer)
{
  if (id == TANGO_CAMERA_COLOR)
  {
    tango_native_streaming::tango_context* ctxt = (tango_native_streaming::tango_context*)context;
    if (!ctxt->image_manager_ready)
    {
      LOGI("Setting up Image Message, received image format is %d", buffer->format);
      ctxt->img_msg_ptr->header.frame_id = tango_camera_color;
      ctxt->img_msg_ptr->height = buffer->height;
      ctxt->img_msg_ptr->width = buffer->width;
      switch (buffer->format)
      {
        case TANGO_HAL_PIXEL_FORMAT_RGBA_8888 : ctxt->img_msg_ptr->encoding = "rgba8";
                                                break;
        case TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP : ctxt->img_msg_ptr->encoding = "rgb8"; //Will convert
                                                break;
        case TANGO_HAL_PIXEL_FORMAT_YV12 : ctxt->img_msg_ptr->encoding = "YV12";
                                                LOGE("Image format is YV21 for which a conversion is not implemented, standard ROS tools wont process the video");
                                                break;
      }
      if (_BYTE_ORDER == _BIG_ENDIAN)
        ctxt->img_msg_ptr->is_bigendian = 1;
      else
        ctxt->img_msg_ptr->is_bigendian = 0;
      ctxt->img_msg_ptr->step = buffer->stride;
      TangoSupport_createImageBufferManager(buffer->format, ctxt->img_msg_ptr->width, ctxt->img_msg_ptr->height, &ctxt->image_manager);
      ctxt->image_manager_ready = true;
    }
    int ret = TangoSupport_updateImageBuffer(ctxt->image_manager, buffer);
    if (ret != TANGO_SUCCESS)
    {
      LOGE("ERROR UPDATING TANGO IMAGE MANAGER");
    }
  }
}

void onPointCloudAvailable(void* context, const TangoPointCloud* point_cloud) {
  tango_native_streaming::tango_context* ctxt = (tango_native_streaming::tango_context*)context;
  int ret = TangoSupport_updatePointCloud(ctxt->pc_manager, point_cloud);
  if (ret != TANGO_SUCCESS)
  {
    LOGE("ERROR UPDATING TANGO MANAGER");
  }
  else
  {
    LOGI("SUCCESSFULLY UPDATED TANGO MANAGER");
  }
}


//Taken from stackoverflow: https://stackoverflow.com/questions/12469730/confusion-on-yuv-nv21-conversion-to-rgb
void decodeYUV420SP(uint8_t rgb[], uint8_t yuv420sp[], int width, int height) {

  int frameSize = width * height;

  int ii = 0;
  int ij = 0;
  int di = +1;
  int dj = +1;

  int a = 0;
  for (int i = 0, ci = ii; i < height; ++i, ci += di) {
       for (int j = 0, cj = ij; j < width; ++j, cj += dj) {
            int y = (0xff & ((int) yuv420sp[ci * width + cj]));
            int v = (0xff & ((int) yuv420sp[frameSize + (ci >> 1) * width + (cj & ~1) + 0]));
            int u = (0xff & ((int) yuv420sp[frameSize + (ci >> 1) * width + (cj & ~1) + 1]));
            y = y < 16 ? 16 : y;

            int r = (int) (1.164f * (y - 16) + 1.596f * (v - 128));
            int g = (int) (1.164f * (y - 16) - 0.813f * (v - 128) - 0.391f * (u - 128));
            int b = (int) (1.164f * (y - 16) + 2.018f * (u - 128));

            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);

            rgb[a] = r;
            a++;
            rgb[a] = g;
            a++;
            rgb[a] = b;
            a++;
       }
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
    TangoPointCloud* pc_ptr;
    TangoImageBuffer* img_ptr;
    bool new_available;
    bool img_size_computed = false;
    int size;
    running = true;
    while (running && !error)
    {
        if (app == NULL)
            LOGE("APP IS NULL");
        if ((app->ctxt).pc_manager == NULL)
            LOGE("pc_manager IS NULL");
        ret = TangoSupport_getLatestPointCloudAndNewDataFlag((app->ctxt).pc_manager, &pc_ptr, &new_available);
        if (ret != TANGO_SUCCESS)
        {
            LOGE("Error %d retrieving latest pointcloud", ret);
        }
        if (new_available)
        {
            app->pc_msg.width = app->pc_msg.row_step = pc_ptr->num_points;
            app->pc_msg.header.seq = app->seq++;
            app->pc_msg.header.stamp = ros::Time::now();
            //LOGI("Header stamp: %f", app->pc_msg.header.stamp.toSec());
            int size = 4 * sizeof(float) * pc_ptr->num_points;
            app->pc_msg.data.resize(size);
            memcpy(&app->pc_msg.data[0], (void*)pc_ptr->points, size);
            app->pc_pub.publish(app->pc_msg);
        }
        if (app->ctxt.image_manager_ready)
        {
            ret = TangoSupport_getLatestImageBufferAndNewDataFlag((app->ctxt).image_manager, &img_ptr, &new_available);
            if (ret != TANGO_SUCCESS)
            {
                LOGE("Error retrieving latest color image");
            }
            if (new_available)
            {
                if (!img_size_computed)
                {
                    int bytes_per_pixel;
                    if (img_ptr->format ==  TANGO_HAL_PIXEL_FORMAT_RGBA_8888)
                    {
                        bytes_per_pixel = 4;
                    }
                    else //means it is NV21 or YV12
                    {
                        bytes_per_pixel = 3;
                    }
                    size = bytes_per_pixel * sizeof(uint8_t) * img_ptr->height * img_ptr->width;
                    app->img_msg.data.resize(size);
                    img_size_computed = true;
                }
                app->img_msg.header.seq = app->img_seq++;
                app->img_msg.header.stamp = ros::Time::now();
                if (img_ptr->format ==  TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP)
                {
                    //TODO: Use openGLES to make conversion faster?
                    decodeYUV420SP(&app->img_msg.data[0], &img_ptr->data[0], img_ptr->width, img_ptr->height);
                }
                else{
                    memcpy(&app->img_msg.data[0], (void*)img_ptr->data, size);
                }
                app->img_pub.publish(app->img_msg);
            }
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
    LOGI("pub_thread stopping");
    return NULL;
}
}

namespace tango_native_streaming {

void TangoNativeStreamingApp::OnCreate(JNIEnv* env, jobject caller_activity)
{
  LOGI("Starting...");
}

void TangoNativeStreamingApp::SetCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& known_pose)
{
  LOGI("Will set pose to known");
  bool tf_success = true;
  geometry_msgs::TransformStamped odom_to_pose_tf;
  try{
    odom_to_pose_tf = tf_buffer->lookupTransform(tango_odom, tango_pose,ros::Time(0));
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
  if (!error)
  {
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

  err = TangoConfig_setInt32(tango_config_, "config_depth_mode", TANGO_POINTCLOUD_XYZC);
    if (err != TANGO_SUCCESS) {
      LOGE("Setting pointcloud mode to XYZc failed with error code: %d.", err);
      std::exit(EXIT_SUCCESS);
    }

  int32_t max_point_cloud_elements;
  int ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                                         &max_point_cloud_elements);
  if(ret != TANGO_SUCCESS) {
    LOGE("Failed to query maximum number of point cloud elements, with error %d", ret);
  }
  else
  {
    LOGI("Max Point Cloud elements = %d", max_point_cloud_elements);
  }
  ret = TangoSupport_createPointCloudManager(max_point_cloud_elements, &(ctxt.pc_manager));
  if(ret != TANGO_SUCCESS) {
      LOGE("Failed to create support point cloud manager");
  }

  err = TangoService_connectOnPointCloudAvailable(onPointCloudAvailable);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "TangoNativeStreamingApp::OnTangoServiceConnected,"
        "Failed to connect to point cloud callback with error code: %d",
        err);
    std::exit(EXIT_SUCCESS);
  }

  err = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
  if (err != TANGO_SUCCESS) {
      LOGE( "Failed to enable color camera with error code: %d", err);
      std::exit(EXIT_SUCCESS);
  }

  err = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, (void*)&ctxt, onFrameAvailable);

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
  if (!error)
  {
    pthread_create(&pub_thread, NULL, pub_thread_method, (void*)this);
  }
  }
}

void TangoNativeStreamingApp::OnPause() {
  running = false;
  LOGI("Stopping pub_thread");
  pthread_join(pub_thread, NULL);
  LOGI("pub_thread stopped");
  // TODO figure out a way to stop the ros node such that it can then be reinitalized to connect to a different master
  // right now can only be paused, and ros settings cannot be changed unless the app is cleared from memory and relaunched.
  if (ros::ok()) {
    LOGI("Shutting down ros");
    //(ctxt.nh)->shutdown();
    delete ctxt.nh;
    ros::shutdown();
    LOGI("ros stopped");
  }
  //if (!error) {
    LOGI("DISCONNECTING FROM TANGO SERVICE");
      TangoConfig_free(tango_config_);
      tango_config_ = nullptr;
      TangoService_disconnect();
    LOGI("TANGO SERVICE DISCONNECTED");
  //}
  LOGI("Done");
}

//TODO: Run ROS initialization on separate thread so Android does not complain
void TangoNativeStreamingApp::OnResume(JNIEnv* env, jobject caller_activity) {

   LOGI("resuming...");

   jclass thisClass = (*env).GetObjectClass(caller_activity);

   jfieldID fidMaster = (*env).GetStaticFieldID(thisClass, "ros_master_jstr", "Ljava/lang/String;"),
            fidRosIp = (*env).GetStaticFieldID(thisClass, "ros_ip_jstr", "Ljava/lang/String;"),
            fidPrefix = (*env).GetStaticFieldID(thisClass, "tango_prefix_jstr", "Ljava/lang/String;"),
            fidNamespace = (*env).GetStaticFieldID(thisClass, "namespace_jstr", "Ljava/lang/String;");
            //fidError = (*env).GetFieldID(thisClass, "nativeError", "Z");

   jstring js_ros_master = (jstring)env->GetStaticObjectField(thisClass, fidMaster),
           js_ros_ip = (jstring)env->GetStaticObjectField(thisClass, fidRosIp),
           js_tango_prefix = (jstring)env->GetStaticObjectField(thisClass, fidPrefix),
           js_tango_namespace = (jstring)env->GetStaticObjectField(thisClass, fidNamespace);

   const char *in_ros_master = env->GetStringUTFChars(js_ros_master, NULL),
              *in_ros_ip = env->GetStringUTFChars(js_ros_ip, NULL),
              *in_tango_prefix = env->GetStringUTFChars(js_tango_prefix, NULL),
              *in_tango_namespace = env->GetStringUTFChars(js_tango_namespace, NULL);

   strcpy(ros_master, in_ros_master);
   env->ReleaseStringUTFChars(js_ros_master, in_ros_master);

   strcpy(ros_ip, in_ros_ip);
   env->ReleaseStringUTFChars(js_ros_ip, in_ros_ip);

   strcpy(tango_prefix, in_tango_prefix);
   env->ReleaseStringUTFChars(js_tango_prefix, in_tango_prefix);

   strcpy(tango_namespace, in_tango_namespace);
   env->ReleaseStringUTFChars(js_tango_namespace, in_tango_namespace);

   strcpy(ros_master_uri, ROS_MASTER_URI_PREFIX);
   strcat(ros_master_uri, ros_master);
   strcpy(ros_ip_uri, ROS_IP_URI_PREFIX);
   strcat(ros_ip_uri, ros_ip);
   strcpy(tango_camera_depth, tango_prefix);
   strcat(tango_camera_depth, TANGO_CAMERA_DEPTH_SUFFIX);
   strcpy(tango_camera_color, tango_prefix);
   strcat(tango_camera_color, TANGO_CAMERA_COLOR_SUFFIX);
   strcpy(tango_odom, tango_prefix);
   strcat(tango_odom, TANGO_ODOM_SUFFIX);
   strcpy(tango_base_link, tango_prefix);
   strcat(tango_base_link, TANGO_BASE_LINK_SUFFIX);
   strcpy(tango_pose, tango_prefix);
   strcat(tango_pose, TANGO_POSE_SUFFIX);

  int seq = 0;
  sensor_msgs::PointField x, y, z, c;
  x.name = "x";
  y.name = "y";
  z.name = "z";
  c.name = "c";
  x.offset = 0;
  y.offset = sizeof(float);
  z.offset = 2.0*sizeof(float);
  c.offset = 3.0*sizeof(float);
  x.count = y.count = z.count = c.count = 1;
  x.datatype = y.datatype = z.datatype = c.datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.header.frame_id = tango_camera_depth;
  pc_msg.height = 1;
  pc_msg.is_bigendian = false;
  pc_msg.is_dense = true;
  pc_msg.point_step = 4 * sizeof(float);
  pc_msg.fields.push_back(x);
  pc_msg.fields.push_back(y);
  pc_msg.fields.push_back(z);
  pc_msg.fields.push_back(c);
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
  else
  {
      LOGI("Successfully created point cloud manager, with %d max point cloud elements", max_point_cloud_elements);
      LOGI("PC Manager address = %d", &(ctxt.pc_manager));
  }

  int argc = 3;
  char *argv[] = {"nothing_important" , ros_master_uri, ros_ip_uri};
  LOGI("GOING TO ROS INIT");

  for(int i = 0; i < argc; i++){
      LOGI("%s", argv[i]);
  }


  ros::init(argc, &argv[0], tango_namespace);

  LOGI("GOING TO NODEHANDLE");

  // %Tag(ROS_MASTER)%
  std::string master_uri = ros::master::getURI();

  if(ros::master::check())
  {
      LOGI("ROS MASTER IS UP!");

      LOGI("%s", master_uri.c_str());
      ctxt.pose_mutex_ptr = &pose_mutex;
      ctxt.odom_to_base_ptr = &odom_to_base;
      ctxt.nh = new ros::NodeHandle(tango_namespace);
      ctxt.image_manager_ready = false;
      tf_bcaster = new tf2_ros::TransformBroadcaster;
      static_tf_bcaster = new tf2_ros::StaticTransformBroadcaster;
      tf_buffer = new tf2_ros::Buffer;
      tf_listener = new tf2_ros::TransformListener(*tf_buffer);
      map_to_odom.header.frame_id = "map";
      map_to_odom.child_frame_id = tango_odom;
      map_to_odom.transform.translation.x = 0;
      map_to_odom.transform.translation.y = 0;
      map_to_odom.transform.translation.z = 0;
      map_to_odom.transform.rotation.x = 0;
      map_to_odom.transform.rotation.y = 0;
      map_to_odom.transform.rotation.z = 0;
      map_to_odom.transform.rotation.w = 1;
      odom_to_base.header.frame_id = tango_odom;
      odom_to_base.child_frame_id = tango_base_link;
      odom_to_base.transform.translation.x = 0;
      odom_to_base.transform.translation.y = 0;
      odom_to_base.transform.translation.z = 0;
      odom_to_base.transform.rotation.x = 0;
      odom_to_base.transform.rotation.y = 0;
      odom_to_base.transform.rotation.z = 0;
      odom_to_base.transform.rotation.w = 1;
      base_to_pose.header.frame_id = tango_base_link;
      base_to_pose.child_frame_id = tango_pose;
      base_to_pose.transform.translation.x = 0;
      base_to_pose.transform.translation.y = 0;
      base_to_pose.transform.translation.z = 0;
      base_to_pose.transform.rotation.x = -0.5;
      base_to_pose.transform.rotation.y = 0.5;
      base_to_pose.transform.rotation.z = 0.5;
      base_to_pose.transform.rotation.w = 0.5;
      base_to_depth.header.frame_id = tango_base_link;
      base_to_depth.child_frame_id = tango_camera_depth;
      base_to_color.header.frame_id = tango_base_link;
      base_to_color.child_frame_id = tango_camera_color;
      pc_pub = (ctxt.nh)->advertise<sensor_msgs::PointCloud2>("tango_image_depth", 1);
      img_pub = (ctxt.nh)->advertise<sensor_msgs::Image>("tango_image_color", 1);
      known_pose_sub = (ctxt.nh)->subscribe("initial_pose", 1, &TangoNativeStreamingApp::SetCurrentPoseCallback, this);
      int version = 0;
      TangoErrorType err = TangoSupport_GetTangoVersion(env, caller_activity, &version);
      if (err != TANGO_SUCCESS || version < kTangoCoreMinimumVersion) {
        LOGE(
            "TangoNativeStreamingApp::OnCreate, Tango Core version is out"
            " of date.");
        std::exit(EXIT_SUCCESS);
      }
  } else {
        LOGI("NO ROS MASTER.");
        //std::exit(EXIT_SUCCESS);
        jfieldID fidErr = env->GetFieldID(thisClass, "nativeError", "Z");
        jboolean err = true;
        env->SetBooleanField(caller_activity, fidErr, err);
        error = true;
        return;
  }
}
}  // namespace