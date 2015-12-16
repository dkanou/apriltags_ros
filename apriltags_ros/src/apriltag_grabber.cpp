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

//
#include <pcl/io/openni_camera/openni_device.h>
#include <pcl/io/openni_camera/openni_device_kinect.h>
#include <pcl/io/openni_camera/openni_device_primesense.h>
#include <pcl/io/openni_camera/openni_device_xtion.h>
#include <pcl/common/time.h>

#include <apriltags_ros/apriltag_grabber.h>

using namespace pcl::io::openni2;
//using namespace openni_wrapper;
using namespace pcl;
//using pcl::OpenNIGrabber;

namespace apriltags_ros
{
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

  ///////////////////////////////////////////////////////////////////////////////
  AprilTagGrabber::AprilTagGrabber (ros::NodeHandle& nh, ros::NodeHandle& pnh):
    node_ (nh)
  {
    // Set the sensor frame
    if (!pnh.getParam("sensor_frame_id", sensor_frame_id_))
    {
      sensor_frame_id_ = "camera_rgb_optical_frame";
    }

    // Openni
    /*
    std::string device_id ("");
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
    {
      for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx)
      {
        std::cout << "Device: " << deviceIdx + 1
                << ", vendor: " << driver.getVendorName(deviceIdx)
                << ", product: " << driver.getProductName(deviceIdx)
                << ", connected: " << driver.getBus(deviceIdx)
                << " @ " << driver.getAddress(deviceIdx)
                << ", serial number: \'"
                << driver.getSerialNumber(deviceIdx)
                << "\'" << std::endl;
      }
    }
    else
    {
      std::cout << "There are NO connected devices" << std::endl;
    }

    grabber = pcl::OpenNIGrabber::Ptr (new OpenNIGrabber (device_id, depth_mode, image_mode));

    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&image,
                              const boost::shared_ptr<openni_wrapper::DepthImage>&depth_image,
                              float constant)> f =
      boost::bind (&AprilTagGrabber::imageDepthImageCB, this, _1, _2, _3);

    grabber->registerCallback (f);
    grabber->start ();
    */

    //Openni2
    /*
    std::string device_id ("");
    pcl::io::OpenNI2Grabber::Mode depth_mode =
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
    pcl::io::OpenNI2Grabber::Mode image_mode =
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;

    boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager =
      pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
    if (deviceManager->getNumOfConnectedDevices () > 0)
    {
      std::cout << "There are connected devices" << std::endl;
      boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device =
        deviceManager->getAnyDevice ();
      std::cout << "Device ID not set, using default device: "
                << device->getStringID ()
                << std::endl;
    }
    else
    {
      std::cout << "There are NO connected devices" << std::endl;
    }
    */
    //pcl::io::OpenNI2Grabber grabber (device_id, depth_mode, image_mode);

    //boost::function<void (const Image::Ptr &image,
    //                      const DepthImage::Ptr &depth_image)> f =
    //  boost::bind (&AprilTagGrabber::imageDepthImageCB, this,  _1, _2);

    //grabber.registerCallback (f);
    //grabber.start ();
  }


  ///////////////////////////////////////////////////////////////////////////////
  AprilTagGrabber::~AprilTagGrabber ()
  {
    // Shutting down the subscribers
  }

  void AprilTagGrabber::imageDepthImageCB (const Image::Ptr &image, const DepthImage::Ptr &depth_image)
  {
    std::cout << "Hey you! ImageDepthImage" << std::endl;
  }

  ///////////////////////////////////////////////////////////////////////////////
  /*
  void AprilTagGrabber::imageDepthImageCB (const boost::shared_ptr<openni_wrapper::Image>&image,
                                           const boost::shared_ptr<openni_wrapper::DepthImage>&depth_image,
                                           float constant)
  {
    FPS_CALC ("imageDepthImageCB");
    std::cout << "Hey you! ImageDepthImage" << std::endl;
    cv::Mat frameRGB = cv::Mat(image->getHeight(),image->getWidth(), CV_8UC3);

    image->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
    cv::Mat frameBGR;
    cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);
  }
  */
}