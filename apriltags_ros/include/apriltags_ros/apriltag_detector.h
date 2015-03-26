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

#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>

#include <AprilTags/TagDetector.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>


namespace apriltags_ros
{
  class AprilTagDescription
  {
    public:
      AprilTagDescription (int id, double size, std::string &frame_name):
        id_ (id),
        size_(size),
        frame_name_(frame_name)
      {};

      double size () {return size_;}
      int id () {return id_;} 
      std::string& frame_name () {return frame_name_;}

    private:
      int id_;
      double size_;
      std::string frame_name_;
  };


  class AprilTagDetector
  {
    public:
      AprilTagDetector (ros::NodeHandle& nh, ros::NodeHandle& pnh);
      ~AprilTagDetector ();

    private:
      /** \brief Image and camera info callback function. */
      void imageCb (const sensor_msgs::ImageConstPtr& msg,
                    const sensor_msgs::CameraInfoConstPtr& cam_info);

      /** \brief Cloud callback function. */
      void stereoCB (const sensor_msgs::PointCloud2ConstPtr& msg);

      std::map<int, AprilTagDescription> parse_tag_descriptions (XmlRpc::XmlRpcValue& april_tag_descriptions);

      /** \brief Point Inclusion in Polygon.  W. Randolph Franklin implementation. */
      bool
      pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);

    private:
      /** \brief The ROS node that grabbers are subscribed at. */
      ros::NodeHandle node_;

      /** \brief Image subscriber. */
      image_transport::CameraSubscriber image_sub_;

      /** \brief Stereo camera subscriber. */
      ros::Subscriber stereo_sub_;

      /** \brief Sensor's frame id. */
      std::string sensor_frame_id_;

      /** \brief AprilTags descriptions. */
      std::map<int, AprilTagDescription> descriptions_;

      /** \brief Tag detections vector. */
      std::vector<AprilTags::TagDetection> detections_;

      std::map<int, AprilTagDescription>::const_iterator description_itr;

      /** \brief OpenCV image. */
      cv_bridge::CvImagePtr cv_ptr;

      /** \brief Image transport. */
      image_transport::ImageTransport it_;

      /** \brief */
      boost::shared_ptr<AprilTags::TagDetector> tag_detector_;

      /** \brief */
      AprilTagDetectionArray tag_detection_array;

      /** \brief */
      double tag_size;

      // Publishers
      tf::TransformBroadcaster tf_pub_;
      image_transport::Publisher image_pub_;
      ros::Publisher detections_pub_;
      ros::Publisher pose_pub_;
      ros::Publisher marker_pub;

      /** \brief Apriltag 4 corners and the center. */
      float cx, cy;
      int p_ul_, p_ur_, p_ll_, p_lr_;

      /** \brief Camera's intrinsic parameters. */
      double fx_, fy_, px_, py_;
      int cam_width_, cam_height_;

      /** \brief The latest processed point cloud from the stereo camera. */
      const pcl::PointCloud<pcl::PointXYZ>::Ptr stereo_cloud_ptr_;
  };
}

#endif