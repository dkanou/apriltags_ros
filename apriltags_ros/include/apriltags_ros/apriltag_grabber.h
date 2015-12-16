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

#ifndef APRILTAG_GRABBER_H
#define APRILTAG_GRABBER_H

// ROS headers
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni2_grabber.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace pcl::io::openni2;

namespace apriltags_ros
{
  /** \brief Sync policies for synchronizing sensor_msgs */
  class AprilTagGrabber
  {
    /** \brief AprilTag grabber class. */
    public:
      /** \brief Constructor. */
      AprilTagGrabber (ros::NodeHandle& nh, ros::NodeHandle& pnh);

      /** \brief Destructor. */
      ~AprilTagGrabber ();

      /** \brief Stereo cloud callback function. */
      void imageDepthImageCB (const Image::Ptr &image,
                              const DepthImage::Ptr &depth_image);

      //void imageDepthImageCB (const boost::shared_ptr<openni_wrapper::Image>&image,
      //                        const boost::shared_ptr<openni_wrapper::DepthImage>&depth_image,
      //                        float constant);

    protected:
      pcl::OpenNIGrabber::Ptr grabber;

    private:
      /** \brief The ROS node that grabbers are subscribed at. */
      ros::NodeHandle node_;

      /** \brief Sensor's frame id. */
      std::string sensor_frame_id_;

      /** \brief OpenCV image. */
      cv_bridge::CvImagePtr cv_ptr;
  };
}
#endif