/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <apriltags_ros/apriltag_detector.h>

#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <pcl-1.7/pcl/common/copy_point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>

namespace apriltags_ros
{

  ///////////////////////////////////////////////////////////////////////////////
  AprilTagDetector::AprilTagDetector (ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : it_(nh),
    node_ (nh),
    stereo_cloud_ptr_ (new pcl::PointCloud<pcl::PointXYZ> ())
  {
    // Parse all the AprilTags descriptions
    XmlRpc::XmlRpcValue april_tag_descriptions;

    if (!pnh.getParam("tag_descriptions", april_tag_descriptions))
    {
      ROS_WARN("No april tags specified");
    }
    else
    {
      try
      {
        descriptions_ = parse_tag_descriptions(april_tag_descriptions);
      }
      catch(XmlRpc::XmlRpcException e)
      {
        ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
      }
    }

    // Set the sensor frame
    if (!pnh.getParam("sensor_frame_id", sensor_frame_id_))
    {
      //sensor_frame_id_ = "camera_rgb_optical_frame";
      sensor_frame_id_ = "multisense/right_camera_optical_frame";
    }

    // AprilTags tag codes (36h11) and tag detector
    AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;
    tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(tag_codes));

    // Subscribers
    image_sub_ = it_.subscribeCamera("/multisense/right/image_rect", 1, &AprilTagDetector::imageCb, this);
    stereo_sub_ = node_.subscribe("/multisense/organized_image_points2", 1, &AprilTagDetector::stereoCB, this);

    //image_sub_ = it_.subscribeCamera("/camera/rgb/image_rect_color", 1, &AprilTagDetector::imageCb, this);
    //stereo_sub_ = node_.subscribe("/camera/depth_registered/points", 1, &AprilTagDetector::stereoCB, this);

    // Publishers
    image_pub_ = it_.advertise("tag_detections_image", 1);
    detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
  }


  ///////////////////////////////////////////////////////////////////////////////
  AprilTagDetector::~AprilTagDetector ()
  {
    // Subscribers
    image_sub_.shutdown();
    stereo_sub_.shutdown();
  }


  ///////////////////////////////////////////////////////////////////////////////
  void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg,
                                 const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    detections_ = tag_detector_->extractTags(gray);
    ROS_DEBUG("%d tag detected", (int)detections_.size());

    if(!sensor_frame_id_.empty())
      cv_ptr->header.frame_id = sensor_frame_id_;

    // camera intrinsic params
    fx_ = cam_info->K[0];
    fy_ = cam_info->K[4];
    px_ = cam_info->K[2];
    py_ = cam_info->K[5];
    cam_width_ = cam_info->width;
    cam_height_ = cam_info->height;

    BOOST_FOREACH(AprilTags::TagDetection detection, detections_)
    {
      std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);

      if(description_itr == descriptions_.end())
      {
        ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
        continue;
      }

      detection.draw(cv_ptr->image);

      AprilTagDescription description = description_itr->second;
      tag_size = description.size();
    }

    image_pub_.publish (cv_ptr->toImageMsg());

  }

  ////////////////////////////////////////////////////////////////////////////////
  void
  AprilTagDetector::stereoCB (const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    geometry_msgs::PoseArray tag_pose_array;
    tag_pose_array.header = cv_ptr->header;

    BOOST_FOREACH(AprilTags::TagDetection detection, detections_)
    {
      std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);

      if(description_itr == descriptions_.end())
      {
        ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
        continue;
      }

      AprilTagDescription description = description_itr->second;
      tag_size = description.size();

      std::cout << "tag size: " << tag_size << std::endl;
      std::pair<float,float> p_[4] = detection.p;
      std::pair<float,float> cxy_ = detection.cxy;
      cx = cxy_.first;
      cy = cxy_.second;
      p_ul = p_[0].first;
      std::cout << "p: " << p_[0].first << std::endl;

      std::cout << "cxy_stereoCB: " << std::ceil(cx) << " , " << std::ceil(cy) << std::endl;
      pcl::fromROSMsg(*msg, *stereo_cloud_ptr_);
      std::cout << "Size: " << stereo_cloud_ptr_->points.size() << std::endl;
      pcl::PointXYZ p;
      if (stereo_cloud_ptr_->points.size())
      {
        std::cout << "Pointcloud z distance: " << stereo_cloud_ptr_->points[std::ceil(cx)*cam_width_+std::ceil(cy)].z << std::endl;
        p = stereo_cloud_ptr_->points[std::ceil(cy)*cam_width_+std::ceil(cx)];
      }

      Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx_, fy_, px_, py_);
      Eigen::Matrix3d rot = transform.block(0,0,3,3);
      Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

      geometry_msgs::PoseStamped tag_pose;
      tag_pose.pose.position.x = p.x;//transform(0,3);
      tag_pose.pose.position.y = p.y;//transform(1,3);
      tag_pose.pose.position.z = p.z;//transform(2,3);
      std::cout << "x,y,z: " << transform(0,3) << "," << transform(1,3) << "," << transform(2,3) << std::endl;
      tag_pose.pose.orientation.x = rot_quaternion.x();
      tag_pose.pose.orientation.y = rot_quaternion.y();
      tag_pose.pose.orientation.z = rot_quaternion.z();
      tag_pose.pose.orientation.w = rot_quaternion.w();
      tag_pose.header = cv_ptr->header;
      tag_pose_array.poses.push_back(tag_pose.pose);

      AprilTagDetection tag_detection;
      tag_detection.pose = tag_pose;
      tag_detection.id = detection.id;
      tag_detection.size = tag_size;
      tag_detection_array.detections.push_back(tag_detection);

      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF (tag_pose, tag_transform);
      tf_pub_.sendTransform (tf::StampedTransform (tag_transform,
                                                   tag_transform.stamp_,
                                                   tag_transform.frame_id_,
                                                   description.frame_name()));
    }

    // Publishers
    detections_pub_.publish (tag_detection_array);
    pose_pub_.publish (tag_pose_array);
  }

  ///////////////////////////////////////////////////////////////////////////////
  std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions (XmlRpc::XmlRpcValue& tag_descriptions)
  {
    std::map<int, AprilTagDescription> descriptions;
    ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < tag_descriptions.size(); ++i)
    {
      XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
      ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      int id = (int)tag_description["id"];
      double size = (double)tag_description["size"];

      std::string frame_name;
      if(tag_description.hasMember("frame_id"))
      {
        ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
        frame_name = (std::string)tag_description["frame_id"];
      }
      else
      {
        std::stringstream frame_name_stream;
        frame_name_stream << "tag_" << id;
        frame_name = frame_name_stream.str();
      }

      AprilTagDescription description(id, size, frame_name);
      ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
      descriptions.insert(std::make_pair(id, description));
    }

    return descriptions;
  }
}