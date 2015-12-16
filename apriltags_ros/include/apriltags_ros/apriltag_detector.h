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

// ROS headers
#include <ros/ros.h>

// SENSOR_MSGS headers
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// MESSAGE_FILTERS headers
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// GEOMETRY_MSGS headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

// BOOST headers
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

// PCL headers
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/features/feature.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_device.h>
#include <pcl/io/openni_camera/openni_device_kinect.h>
#include <pcl/io/openni_camera/openni_device_primesense.h>
#include <pcl/io/openni_camera/openni_device_xtion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

// CV_BRIDGE headers
#include <cv_bridge/cv_bridge.h>

// VISUALIZATION_MSGS headers
#include <visualization_msgs/MarkerArray.h>

// IMAGE_TRANSPORT headers
#include <image_transport/image_transport.h>

// TF headers
#include <tf/transform_broadcaster.h>

// XmlRpc headers
#include <XmlRpcException.h>

// AprilTags headers
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

// APRILTAG_ROS headers
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>

namespace apriltags_ros
{
  class AprilTagDescription
  {
    /** \brief AprilTag description class. */
    public:
      /** \brief Constructor. */
      AprilTagDescription (int id, double size, std::string &frame_name):
        id_ (id),
        size_(size),
        frame_name_(frame_name)
      {};

      /** \brief Returns the size of the tag. */
      double
      size () {return size_;}

      /** \brief Returns the id of the tag. */
      int
      id () {return id_;}

      /** \brief Returns the frame name of the tag. */
      std::string&
      frame_name () {return frame_name_;}

    private:
      /** \brief Tag id. */
      int id_;

      /** \brief Tag size. */
      double size_;

      /** \brief Tag frame name. */
      std::string frame_name_;
  };

  class AprilTagDetector
  {
    /** \brief AprilTag detector class. */
    public:
      /** \brief Constructor. */
      AprilTagDetector (ros::NodeHandle& nh, ros::NodeHandle& pnh);

      /** \brief Destructor. */
      ~AprilTagDetector ();

      /** \brief Point cloud callback function. */
      void
      pointCloudCallback (const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);

      /** \brief RGB image callback function. */
      void
      rgbImageCallback (const sensor_msgs::ImageConstPtr& rgb_image_msg);

      /** \brief RGB image and depth callback function. */
      void imageDepthImageCB (const boost::shared_ptr<openni_wrapper::Image>&image,
                              const boost::shared_ptr<openni_wrapper::DepthImage>&depth_image,
                              float constant);

    private:
      /** \brief Point Inclusion in Polygon (W. R. Franklin impl). */
      bool
      pnpoly (int nvert, float *vertx, float *verty, float testx, float testy);

      /** \brief XML tag descriptor parser. */
      std::map<int, AprilTagDescription>
      parse_tag_descriptions (XmlRpc::XmlRpcValue& april_tag_descriptions);

    protected:
      /** \brief PCL openni grabber. */
      pcl::OpenNIGrabber::Ptr openni_grabber_;

      /** \brief Whether to visualize the AprilTag images. */
      bool show_apriltags_image_;

      /** \brief Whether to visualize the AprilTag points. */
      bool show_apriltags_points_;

    private:
      /** \brief The ROS node that grabbers are subscribed at. */
      ros::NodeHandle node_;

      /** \brief Sensor's frame id. */
      std::string sensor_frame_id_;

      /** \brief Point cloud and image subscribers. */
      ros::Subscriber point_cloud_sub_, image_sub_;

      /** \brief AprilTags descriptions. */
      std::map<int, AprilTagDescription> descriptions_;

      /** \brief Tag detections vector. */
      std::vector<AprilTags::TagDetection> detections_;

      /** \brief Description iterator. */
      std::map<int, AprilTagDescription>::const_iterator description_itr_;

      /** \brief OpenCV images for the RGB and the Stereo input. */
      cv_bridge::CvImagePtr image_cv_ptr_, stereo_cv_ptr_;

      /** \brief Image transport. */
      image_transport::ImageTransport it_;

      /** \brief Tag detector object. */
      boost::shared_ptr<AprilTags::TagDetector> tag_detector_;

      /** \brief Array of tag detections. */
      AprilTagDetectionArray tag_detection_array_;

      /** \brief Tag size (in m). */
      double tag_size_;

      /** \brief Publishers */
      image_transport::Publisher image_pub_;
      tf::TransformBroadcaster tf_pub_;
      ros::Publisher marker_pub_;

      /** \brief Apriltag 4 corners and the center. */
      float cu_, cv_;
      int p_ul_, p_ur_, p_ll_, p_lr_;

      /** \brief Camera's intrinsic parameters. */
      double fx_, fy_, px_, py_;
      int cam_width_, cam_height_;

      /** \brief The latest processed point cloud from the stereo camera. */
      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stereo_cloud_ptr_;

      /** \brief The RGB image coming from the point cloud. */
      sensor_msgs::Image stereo_image_;

      /** \brief Whether to use PCLOpenniGrabber. */
      bool use_pclopennigrabber_;

      /** \brief Whether to use the RGB image grabber. */
      bool use_rgb_image_grabber_;
  };
}

#endif