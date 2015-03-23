#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <AprilTags/TagDetector.h>
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
      void imageCb (const sensor_msgs::ImageConstPtr& msg,
                    const sensor_msgs::CameraInfoConstPtr& cam_info);

      void stereoCB (const sensor_msgs::PointCloud2ConstPtr& msg);

      std::map<int, AprilTagDescription> parse_tag_descriptions (XmlRpc::XmlRpcValue& april_tag_descriptions);

    private:
      /** \brief The ROS node that grabbers are subscribed at. */
      ros::NodeHandle node_;

      /** \brief Image subscriber. */
      image_transport::CameraSubscriber image_sub_;

      /** \brief Stereo camera subscriber. */
      ros::Subscriber stereo_sub_;

      std::map<int, AprilTagDescription> descriptions_;
      std::string sensor_frame_id_;
      image_transport::ImageTransport it_;

      tf::TransformBroadcaster tf_pub_;
      boost::shared_ptr<AprilTags::TagDetector> tag_detector_;

      // Publishers
      image_transport::Publisher image_pub_;
      ros::Publisher detections_pub_;
      ros::Publisher pose_pub_;

      /** \brief Apriltag 4 corners and the center. */
      float p_ul;
      float cx, cy;

      /** \brief The latest processed point cloud from the stereo camera. */
      pcl::PointCloud<pcl::PointXYZ>::Ptr stereo_cloud_ptr_;
  };
}

#endif