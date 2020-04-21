#ifndef DAVIS_STEREO_STITCH_H
#define DAVIS_STEREO_STITCH_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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
  image_transport::Publisher image_pub_;
  ros::Publisher event_pub_;

  message_filters::Subscriber<sensor_msgs::Image>* left_image_sub_ ;  
  message_filters::Subscriber<sensor_msgs::Image>* right_image_sub_;
  message_filters::Synchronizer<imageSyncPolicy>* image_sync_;

  message_filters::Subscriber<dvs_msgs::EventArray>* left_event_sub_ ;  
  message_filters::Subscriber<dvs_msgs::EventArray>* right_event_sub_;
  message_filters::Synchronizer<eventSyncPolicy>* event_sync_;

  void imageCallback(const sensor_msgs::Image::ConstPtr& left_image, const sensor_msgs::Image::ConstPtr& right_image);
  void eventCallback(const dvs_msgs::EventArray::ConstPtr& left_event, const dvs_msgs::EventArray::ConstPtr& right_event);

};

#endif //DAVIS_STEREO_STITCH_H