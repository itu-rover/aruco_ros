/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/

/**
 * @file simple_double.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;
ros::Publisher pose_pub1;
ros::Publisher pose_pub2;
ros::Publisher pose_pub3;
ros::Publisher pose_pub4;
std::string child_name1;
std::string child_name2;
std::string child_name3;
std::string child_name4;
std::string parent_name;

double marker_size1;
double marker_size2;
double marker_size3;
double marker_size4;
int marker_id1;
int marker_id2;
int marker_id3;
int marker_id4;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;
  if (cam_info_received)
  {
    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      if (normalizeImageIllumination)
      {
        ROS_WARN("normalizeImageIllumination is unimplemented!");
//        cv::Mat inImageNorm;
//        pal_vision_util::dctNormalization(inImage, inImageNorm, dctComponentsToRemove);
//        inImage = inImageNorm;
      }

      // detection results will go into "markers"
      markers.clear();
      std::map<int, double> marker_sizes = {{marker_id1, marker_size1}, {marker_id2, marker_size2}, {marker_id3, marker_size3}, {marker_id4, marker_size4}};
    
      // ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_sizes, false, false);
      // for each marker, draw info and its boundaries in the image
      for (unsigned int i = 0; i < markers.size(); ++i)
      {
        // only publishing the selected marker
        if (markers[i].id == marker_id1)
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name1));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
          pose_pub1.publish(poseMsg);
        }
        else if (markers[i].id == marker_id2)
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name2));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
          pose_pub2.publish(poseMsg);
        }
        else if (markers[i].id == marker_id3)
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name3));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
          pose_pub3.publish(poseMsg);
        }
        else if (markers[i].id == marker_id4)
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name4));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
          pose_pub4.publish(poseMsg);
        }

        // but drawing all the detected markers
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
      }

      // paint a circle in the center of the image
      cv::circle(inImage, cv::Point(inImage.cols / 2, inImage.rows / 2), 4, cv::Scalar(0, 255, 0), 1);

      // draw a 3D cube in each marker if there is 3D info
      if (camParam.isValid())
      {
        for (unsigned int i = 0; i < markers.size(); ++i)
        {
          aruco::CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
        }
      }

      if (image_pub.getNumSubscribers() > 0)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if (debug_pub.getNumSubscribers() > 0)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }

      ROS_DEBUG("runtime: %f ms", 1000 * (cv::getTickCount() - ticksBefore) / cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

void reconf_callback(aruco_ros::ArucoThresholdConfig &config, std::uint32_t level)
{
  mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
  normalizeImageIllumination = config.normalizeImage;
  dctComponentsToRemove = config.dctComponentsToRemove;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
  f_ = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(f_);

  normalizeImageIllumination = false;

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

  cam_info_received = false;
  image_pub = it.advertise("result", 1);
  debug_pub = it.advertise("debug", 1);
  pose_pub1 = nh.advertise<geometry_msgs::Pose>("pose", 100);
  pose_pub2 = nh.advertise<geometry_msgs::Pose>("pose2", 100);
  pose_pub3 = nh.advertise<geometry_msgs::Pose>("pose3", 100);
  pose_pub4 = nh.advertise<geometry_msgs::Pose>("pose4", 100);

  nh.param<double>("marker_size1", marker_size1, 0.05);
  nh.param<double>("marker_size2", marker_size2, 0.05);
  nh.param<double>("marker_size3", marker_size3, 0.05);
  nh.param<double>("marker_size4", marker_size4, 0.05);
  nh.param<int>("marker_id1", marker_id1, 582);
  nh.param<int>("marker_id2", marker_id2, 26);
  nh.param<int>("marker_id3", marker_id3, 3);
  nh.param<int>("marker_id4", marker_id4, 7);
  nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
  nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if (dctComponentsToRemove == 0)
    normalizeImageIllumination = false;
  nh.param<std::string>("parent_name", parent_name, "");
  nh.param<std::string>("child_name1", child_name1, "");
  nh.param<std::string>("child_name2", child_name2, "");
  nh.param<std::string>("child_name3", child_name3, "");
  nh.param<std::string>("child_name4", child_name4, "");

  if (parent_name == "" || child_name1 == "" || child_name2 == "" || child_name3 == "" || child_name4 == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }

  // ROS_INFO("ArUco node started with marker size of %f meters and marker ids to track: %d, %d", marker_size, marker_id1,
  //          marker_id2);
  // ROS_INFO("ArUco node will publish pose to TF with (%s, %s) and (%s, %s) as (parent,child).", parent_name.c_str(),
  //          child_name1.c_str(), parent_name.c_str(), child_name2.c_str());

  ros::spin();
}
