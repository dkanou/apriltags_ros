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

using namespace pcl;
using namespace pcl::console;
using namespace openni_wrapper;
using pcl::OpenNIGrabber;

/** \brief MACRO for framerate printing. */
#define FPS_CALC(_WHAT_) \
do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
  { \
    std::cout << "Average framerate("<< _WHAT_ << "): " \
    << double(count)/double(now - last) << " Hz" <<  std::endl; \
    count = 0; \
    last = now; \
  } \
}while(false)

namespace apriltags_ros
{
  ///////////////////////////////////////////////////////////////////////////////
  AprilTagDetector::AprilTagDetector (ros::NodeHandle& nh, ros::NodeHandle& pnh):
    it_ (nh), node_ (nh),
    use_pclopennigrabber_ (false), use_rgb_image_grabber_ (false),
    show_apriltags_image_ (true), show_apriltags_points_ (true),
    stereo_cloud_ptr_ (new pcl::PointCloud<pcl::PointXYZRGBA> ()),
    fx_ (525.0), fy_ (525.0), px_ (319.5), py_ (239.5), cam_width_ (640)
  {
    // Parse all the AprilTags descriptions
    XmlRpc::XmlRpcValue april_tag_descriptions;

    if (!pnh.getParam ("tag_descriptions", april_tag_descriptions))
    {
      ROS_WARN ("No april tags specified");
    }
    else
    {
      try
      {
        descriptions_ = parse_tag_descriptions (april_tag_descriptions);
      }
      catch (XmlRpc::XmlRpcException e)
      {
        ROS_ERROR_STREAM ("Error loading tag descriptions: "<< e.getMessage ());
      }
    }

    // Set the sensor frame
    if (!pnh.getParam("sensor_frame_id", sensor_frame_id_))
    {
      //sensor_frame_id_ = "/multisense/left_camera_optical_frame";
      sensor_frame_id_ = "camera_rgb_optical_frame";
    }

    // AprilTags tag codes (36h11) and tag detector
    AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;
    tag_detector_= boost::shared_ptr<AprilTags::TagDetector>
      (new AprilTags::TagDetector(tag_codes));

    // Subscribers
    if (use_rgb_image_grabber_)
    {
      image_sub_ = node_.subscribe ("/camera/rgb/image_rect_color", 1,
                                    &AprilTagDetector::rgbImageCallback, this);
    }
    point_cloud_sub_ = node_.subscribe("/camera/depth_registered/points", 1,
                                       &AprilTagDetector::pointCloudCallback, this);

    // Openni
    if (use_pclopennigrabber_)
    {
      std::string device_id ("");
      pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
      pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

      openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
     openni_grabber_ = pcl::OpenNIGrabber::Ptr (new OpenNIGrabber (device_id, depth_mode, image_mode));

      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&image,
                            const boost::shared_ptr<openni_wrapper::DepthImage>&depth_image,
                            float constant)> f =
        boost::bind (&AprilTagDetector::imageDepthImageCB, this, _1, _2, _3);

      openni_grabber_->registerCallback (f);
      openni_grabber_->start ();
    }

    // Publishers
    image_pub_ = it_.advertise("tag_detections_image", 1);
    marker_pub_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }


  ///////////////////////////////////////////////////////////////////////////////
  AprilTagDetector::~AprilTagDetector ()
  {
    // Shutting down the subscribers
    image_sub_.shutdown();
    point_cloud_sub_.shutdown();
  }


  ///////////////////////////////////////////////////////////////////////////////
  void AprilTagDetector::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    FPS_CALC("imageCB");

    try
    {
      image_cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(image_cv_ptr_->image, gray, CV_BGR2GRAY);

    detections_ = tag_detector_->extractTags(gray);
    ROS_DEBUG("%d tag detected", (int)detections_.size());

    if(!sensor_frame_id_.empty())
      image_cv_ptr_->header.frame_id = sensor_frame_id_;

    BOOST_FOREACH(AprilTags::TagDetection detection, detections_)
    {
      description_itr_ = descriptions_.find(detection.id);

      if(description_itr_ == descriptions_.end())
      {
        ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
        continue;
      }

      // Draw image
      if (show_apriltags_image_)
        detection.draw(image_cv_ptr_->image);

      AprilTagDescription description = description_itr_->second;
      tag_size_ = description.size();
    }

    // Publish the AprilTags image
    if (show_apriltags_image_)
      image_pub_.publish (image_cv_ptr_->toImageMsg());
  }

  ////////////////////////////////////////////////////////////////////////////////
  void
  AprilTagDetector::pointCloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    FPS_CALC("pointCloudCallback");

    //pcl::toROSMsg (*msg, stereo_image_);

    // Transform the ros::msg to a pcl::stereo_cloud
    pcl::fromROSMsg(*msg, *stereo_cloud_ptr_);

    /*
    try
    {
      stereo_cv_ptr_ = cv_bridge::toCvCopy(stereo_image_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor (stereo_cv_ptr_->image, gray, CV_BGR2GRAY);
    */

    cv::Mat imageFrame = cv::Mat (stereo_cloud_ptr_->height,
                                  stereo_cloud_ptr_->width,
                                  CV_8UC3); 
    for (int h=0; h<imageFrame.rows; h++) 
    {
      for (int w=0; w<imageFrame.cols; w++) 
      {
        pcl::PointXYZRGBA point = stereo_cloud_ptr_->at(w, h);
        Eigen::Vector3i rgb = point.getRGBVector3i();
        imageFrame.at<cv::Vec3b>(h,w)[0] = rgb[2];
        imageFrame.at<cv::Vec3b>(h,w)[1] = rgb[1];
        imageFrame.at<cv::Vec3b>(h,w)[2] = rgb[0];
      }
    }

    cv::Mat gray;
    cv::cvtColor (imageFrame, gray, CV_BGR2GRAY);

    double last = pcl::getTime ();
    detections_ = tag_detector_->extractTags(gray);
    ROS_DEBUG("%d tag detected", (int)detections_.size());
    double now = pcl::getTime ();
    std::cout << "Detection time: " << now-last << std::endl;

    //if(!sensor_frame_id_.empty())
    //  stereo_cv_ptr_->header.frame_id = sensor_frame_id_;

    // Detect the pose of the detected tags
    geometry_msgs::PoseArray tag_pose_array;
    //tag_pose_array.header = stereo_cv_ptr_->header;

    // For each detected AprilTag
    int seq=0;
    BOOST_FOREACH(AprilTags::TagDetection detection, detections_)
    {
      description_itr_ = descriptions_.find(detection.id);

      if(description_itr_ == descriptions_.end())
      {
        ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
        continue;
      }

      // Draw image
      //if (show_apriltags_image_)
      //  detection.draw(stereo_cv_ptr_->image);

      if (show_apriltags_image_)
        detection.draw (imageFrame);

      AprilTagDescription description = description_itr_->second;
      tag_size_ = description.size();


      // Central and corner points of the AprilTag
      std::pair<float,float> p_[4] = detection.p;
      std::pair<float,float> cxy_ = detection.cxy;
      cu_ = cxy_.first; //cols
      cv_ = cxy_.second; //rows

      float vertx[4], verty[4];
      verty[0] = p_[0].first;
      verty[1] = p_[1].first;
      verty[2] = p_[2].first;
      verty[3] = p_[3].first;
      float min_verty = std::min (verty[0], std::min (verty[1], std::min (verty[2], verty[3])));
      float max_verty = std::max (verty[0], std::max (verty[1], std::max (verty[2], verty[3])));
      float thres_verty = 0.25*(max_verty-min_verty);
      vertx[0] = p_[0].second;
      vertx[1] = p_[1].second;
      vertx[2] = p_[2].second;
      vertx[3] = p_[3].second;
      float min_vertx = std::min (vertx[0], std::min (vertx[1], std::min (vertx[2], vertx[3])));
      float max_vertx = std::max (vertx[0], std::max (vertx[1], std::max (vertx[2], vertx[3])));
      float thres_vertx = 0.25*(max_vertx-min_vertx);

      

      // Center point (in the cloud)
      pcl::PointXYZRGBA p;

      // Centroid point (in the cloud)
      Eigen::Vector4f centroid;

      if (stereo_cloud_ptr_->points.size())
      {
        // Indices of the point to consider the centroid
        std::vector<int> indices;
        for (float i=min_vertx+thres_vertx; i<max_vertx-thres_vertx; i++)
          for (float j=min_verty+thres_verty; j<max_verty-thres_verty; j++)
            if (pnpoly (4, vertx, verty, i, j))
            {
              pcl::PointXYZRGBA p_tmp = stereo_cloud_ptr_->points[static_cast<int>(i)*cam_width_ + static_cast<int>(j)];
              if (!std::isnan(p_tmp.x))
                indices.push_back (static_cast<int>(i)*cam_width_ + static_cast<int>(j));
            }

        // Find the centroid
        pcl::compute3DCentroid (*stereo_cloud_ptr_, indices, centroid);

        // Center point of AprilTag point cloud
        p = stereo_cloud_ptr_->points[(std::floor(cv_)*cam_width_) + std::floor(cu_)];

      }

      // Tag Pose
      Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size_, fx_, fy_, px_, py_);
      Eigen::Matrix3d rot = transform.block(0,0,3,3);
      Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

      geometry_msgs::PoseStamped tag_pose;

      // AprilTag poisition
      if (std::isinf(centroid(0)) || std::isnan(centroid(0)))
      {
        tag_pose.pose.position.x = transform(0,3);
        tag_pose.pose.position.y = transform(1,3);
        tag_pose.pose.position.z = transform(2,3);
      }
      else
      {
        tag_pose.pose.position.x = centroid(0); //p.x
        tag_pose.pose.position.y = centroid(1); //p.y
        tag_pose.pose.position.z = centroid(2); //p.z
      }

      // AprilTag orientation
      tag_pose.pose.orientation.x = rot_quaternion.x();
      tag_pose.pose.orientation.y = rot_quaternion.y();
      tag_pose.pose.orientation.z = rot_quaternion.z();
      tag_pose.pose.orientation.w = rot_quaternion.w();

      tag_pose.header.seq = seq;
      tag_pose.header.frame_id = sensor_frame_id_;
      tag_pose.header.stamp = ros::Time::now();
      //stereo_cv_ptr_->header;
      tag_pose_array.poses.push_back(tag_pose.pose);

      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF (tag_pose, tag_transform);
      tf_pub_.sendTransform (tf::StampedTransform (tag_transform,
                                                   tag_transform.stamp_,
                                                   tag_transform.frame_id_,
                                                   description.frame_name()));

      // Center point Marker
      if (show_apriltags_points_)
      {
        visualization_msgs::Marker center_point;
        center_point.header.seq = seq;
        center_point.header.frame_id = sensor_frame_id_;
        center_point.header.stamp = ros::Time::now();
        center_point.ns = "AprilTag_point";
        center_point.action = visualization_msgs::Marker::ADD;
        center_point.type = visualization_msgs::Marker::POINTS;
        center_point.color.g = 1.0f;
        center_point.color.a = 1.0;
        center_point.pose.orientation.w = 1.0;
        center_point.scale.x = 0.008;
        center_point.scale.y = 0.008;
        center_point.scale.z = 0.008;
        center_point.id = 0;
        geometry_msgs::Point cntr_point;
        cntr_point.x = centroid(0);
        cntr_point.y = centroid(1);
        cntr_point.z = centroid(2);
        center_point.points.push_back (cntr_point);

        pcl::PointXYZRGBA p_tmp;

        for (float i=min_vertx+thres_vertx; i<max_vertx-thres_vertx; i++)
        {
          for (float j=min_verty+thres_verty; j<max_verty-thres_verty; j++)
          {
            if (pnpoly (4, vertx, verty, i, j))
            {
              p_tmp = stereo_cloud_ptr_->points[static_cast<int>(i)*cam_width_ + static_cast<int>(j)];
              if (!std::isnan(p_tmp.x))
              {
              cntr_point.x = p_tmp.x;
              cntr_point.y = p_tmp.y;
              cntr_point.z = p_tmp.z;
              center_point.points.push_back (cntr_point);
              }
            }
          }
        }
        marker_pub_.publish (center_point);
      }

      // Unique seq number
      seq++;
    }

    // Publish the AprilTags image
    //if (show_apriltags_image_)
    //  image_pub_.publish (stereo_cv_ptr_->toImageMsg());
  }


  ///////////////////////////////////////////////////////////////////////////////
  std::map<int, AprilTagDescription>
  AprilTagDetector::parse_tag_descriptions (XmlRpc::XmlRpcValue& tag_descriptions)
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


  ////////////////////////////////////////////////////////////////////////////////
  bool
  AprilTagDetector::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
  {
    int i, j;
    bool c = false;
    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
      if (((verty[i]>testy) != (verty[j]>testy)) &&
          (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]))
        c = !c;
    }
    return c;
  }

  ///////////////////////////////////////////////////////////////////////////////
  void AprilTagDetector::imageDepthImageCB (const boost::shared_ptr<openni_wrapper::Image>&image,
                                            const boost::shared_ptr<openni_wrapper::DepthImage>&depth_image,
                                            float constant)
  {
    FPS_CALC ("imageDepthImageCB");

    cv::Mat frameRGB = cv::Mat(image->getHeight(),image->getWidth(), CV_8UC3);

    image->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
    cv::Mat frameBGR;
    cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);
    //TBD
  }
}

