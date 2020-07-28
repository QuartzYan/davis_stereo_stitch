#ifndef DAVIS_STEREO_STITCH_H
#define DAVIS_STEREO_STITCH_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Bool.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include<opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace cv;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> imageSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<dvs_msgs::EventArray,dvs_msgs::EventArray> eventSyncPolicy;

class DavisStereoStitch
{
public:
  DavisStereoStitch();
  ~DavisStereoStitch();

private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Publisher event_pub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher event_image_pub_;
  image_transport::Publisher left_event_image_pub_, right_event_image_pub_;
  
  ros::Subscriber stitch_calibration_sub_;
  ros::Subscriber left_camera_info_sub_, right_camera_info_sub_;

  message_filters::Subscriber<sensor_msgs::Image>* left_image_sub_ ;  
  message_filters::Subscriber<sensor_msgs::Image>* right_image_sub_;
  message_filters::Synchronizer<imageSyncPolicy>* image_sync_;

  message_filters::Subscriber<dvs_msgs::EventArray>* left_event_sub_ ;  
  message_filters::Subscriber<dvs_msgs::EventArray>* right_event_sub_;
  message_filters::Synchronizer<eventSyncPolicy>* event_sync_;

  // callbacks
  void cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& left_image, const sensor_msgs::Image::ConstPtr& right_image);
  void eventCallback(const dvs_msgs::EventArray::ConstPtr& left_event, const dvs_msgs::EventArray::ConstPtr& right_event);
  void findStitchCallback(const std_msgs::Bool::ConstPtr& msg);

  sensor_msgs::CameraInfo camera_info_left_, camera_info_right_;
  bool got_camera_info_left_, got_camera_info_right_;
  cv::Mat left_camera_matrix_, left_dist_coeffs_;
  cv::Mat right_camera_matrix_, right_dist_coeffs_;
  cv::Mat homography_;

  bool find_stitch_once_;
  bool stitch_calibration_mode_;
};

#endif //DAVIS_STEREO_STITCH_H