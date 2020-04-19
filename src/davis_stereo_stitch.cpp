#include "davis_stereo_stitch.h"

DavisStereoStitch::DavisStereoStitch()
{
  left_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/davis_left/image_raw", 1);
  right_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/davis_right/image_raw", 1);

  image_sync_ = new message_filters::Synchronizer<imageSyncPolicy>(imageSyncPolicy(10), *left_image_sub_, *right_image_sub_);
  image_sync_->registerCallback(boost::bind(&DavisStereoStitch::imageCallback, this, _1, _2));

  left_event_sub_ = new message_filters::Subscriber<dvs_msgs::EventArray>(nh_, "/davis_left/events", 1);
  right_event_sub_ = new message_filters::Subscriber<dvs_msgs::EventArray>(nh_, "/davis_right/events", 1);

  event_sync_ = new message_filters::Synchronizer<eventSyncPolicy>(eventSyncPolicy(10), *left_event_sub_, *right_event_sub_);
  event_sync_->registerCallback(boost::bind(&DavisStereoStitch::eventCallback, this, _1, _2));

  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("stitch_image", 1);
  event_pub_ = nh_.advertise<dvs_msgs::EventArray>("stitch_events", 1);
}

DavisStereoStitch::~DavisStereoStitch()
{
  delete left_image_sub_, right_image_sub_, image_sync_;
  delete left_event_sub_, right_event_sub_, event_sync_;
  image_pub_.shutdown();
}

void DavisStereoStitch::imageCallback(const sensor_msgs::Image::ConstPtr &left_image, const sensor_msgs::Image::ConstPtr &right_image)
{
  cv_bridge::CvImagePtr left_cv_ptr, right_cv_ptr;

  try
  {
    left_cv_ptr = cv_bridge::toCvCopy(right_image);
    right_cv_ptr = cv_bridge::toCvCopy(left_image);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat left_cv_image, right_cv_image;

  cv::cvtColor(left_cv_ptr->image, left_cv_image, CV_RGB2BGR);
  cv::cvtColor(right_cv_ptr->image, right_cv_image, CV_RGB2BGR);

  //构造特征提取器，这里使用orb特征点
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  //提取特征
  std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
  orb->detect(left_cv_image, left_keypoints);
  orb->detect(right_cv_image, right_keypoints);

  //构造描述子提取器
  cv::Ptr<cv::DescriptorExtractor> descriptor = orb;
  //提取描述子
  cv::Mat left_descriptors, right_descriptors;
  descriptor->compute(left_cv_image, left_keypoints, left_descriptors);
  descriptor->compute(right_cv_image, right_keypoints, right_descriptors);

  //构造匹配器
  cv::BFMatcher matcher(cv::NORM_L2, true);
  //匹配描述子
  std::vector<cv::DMatch> matches;
  matcher.match(left_descriptors, right_descriptors, matches);

  std::vector<cv::Point2f> left_selPoints, right_selPoints;
  //std::vector<int> left_pointIndexes, right_pointIndexes;
  for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
  {
    left_selPoints.push_back(left_keypoints.at(it->queryIdx).pt);
    right_selPoints.push_back(right_keypoints.at(it->trainIdx).pt);
  }

  std::vector<uchar> inliers(left_selPoints.size(), 0);
  cv::Mat homography;
  homography = findHomography(left_selPoints, right_selPoints, inliers, CV_FM_RANSAC, 1.0);

  //根据RANSAC重新筛选匹配
  std::vector<cv::DMatch> outMatches;
  std::vector<uchar>::const_iterator itIn = inliers.begin();
  std::vector<cv::DMatch>::const_iterator itM = matches.begin();
  for (; itIn != inliers.end(); ++itIn, ++itM)
  {
    if (*itIn)
    {
      outMatches.push_back(*itM);
    }
  }

  //画出匹配图像
  //cv::Mat matchImage;
  //cv::drawMatches(left_cv_image, left_keypoints, right_cv_image, right_keypoints, outMatches, matchImage, 255, 255);

  //拼接
  int d = 20; //渐入渐出融合宽度
  cv::Mat result;
  cv::warpPerspective(left_cv_image, result, homography, cv::Size(2 * left_cv_image.cols - d, left_cv_image.rows)); //Size设置结果图像宽度，宽度裁去一部分，d可调

  cv::Mat half(result, cv::Rect(0, 0, right_cv_image.cols - d, right_cv_image.rows));
  right_cv_image(cv::Range::all(), cv::Range(0, right_cv_image.cols - d)).copyTo(half);
  for (int i = 0; i < d; i++)
  {
    result.col(right_cv_image.cols - d + i) = (d - i) / (float)d * right_cv_image.col(right_cv_image.cols - d + i) + i / (float)d * result.col(right_cv_image.cols - d + i);
  }

  cv_bridge::CvImage out_cv_image;
  result.copyTo(out_cv_image.image);
  out_cv_image.encoding = "bgr8";
  image_pub_.publish(out_cv_image.toImageMsg());
}

void DavisStereoStitch::eventCallback(const dvs_msgs::EventArray::ConstPtr &left_event, const dvs_msgs::EventArray::ConstPtr &right_event)
{
  event_pub_.publish(left_event);
}