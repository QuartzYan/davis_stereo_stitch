#include "davis_stereo_stitch.h"

DavisStereoStitch::DavisStereoStitch()
  :find_stitch_once_(false)
{
  //init param
  got_camera_info_left_ = false;
  got_camera_info_right_ = false;

  if (nh_.hasParam("stitch_find_mode"))
  {
    nh_.param<bool>("stitch_find_mode", stitch_calibration_mode_, false);
    if (stitch_calibration_mode_)
    {
      ROS_INFO("stitch find mode!!!");
    }
  }
  
  std::vector<double> param_homography{0.4916559814809626, 0.1455768474611172, 150.7872720548855,
                                      -0.2305138272924719, 0.9529974437006833, 12.4090961153593,
                                      -0.001774090006639621, 0.0003216603110495318, 1};

  if (nh_.hasParam("homography"))
  {
    XmlRpc::XmlRpcValue _list;
    nh_.getParam("homography", _list);
    ROS_ASSERT(_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(_list.size() == 9);
    for (int i = 0; i < _list.size(); ++i)
    {
      // Read as string to handle no decimals and scientific notation
      std::ostringstream ostr;
      ostr << _list[i];
      std::istringstream istr(ostr.str());
      istr >> param_homography[i];
    }
  }
  else
  {
    ROS_ERROR("NO Homography param!!!!!");
  }

  homography_ = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
  for (int i = 0; i < (homography_.rows * homography_.cols); ++i)
  {
    homography_.at<double>(i) = param_homography[i];
  }

  std::cout << "homography:\n" << homography_ << std::endl;

  //init Subscriber
  stitch_calibration_sub_ = nh_.subscribe("find_stitch_once", 1, &DavisStereoStitch::findStitchCallback, this);

  left_camera_info_sub_ = nh_.subscribe("/davis_left/camera_info", 1, &DavisStereoStitch::cameraInfoLeftCallback, this);
  right_camera_info_sub_ = nh_.subscribe("/davis_right/camera_info", 1, &DavisStereoStitch::cameraInfoRightCallback, this);

  left_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/davis_left/image_raw", 1);
  right_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/davis_right/image_raw", 1);

  image_sync_ = new message_filters::Synchronizer<imageSyncPolicy>(imageSyncPolicy(10), *left_image_sub_, *right_image_sub_);
  image_sync_->registerCallback(boost::bind(&DavisStereoStitch::imageCallback, this, _1, _2));

  left_event_sub_ = new message_filters::Subscriber<dvs_msgs::EventArray>(nh_, "/davis_left/events", 1);
  right_event_sub_ = new message_filters::Subscriber<dvs_msgs::EventArray>(nh_, "/davis_right/events", 1);

  event_sync_ = new message_filters::Synchronizer<eventSyncPolicy>(eventSyncPolicy(10), *left_event_sub_, *right_event_sub_);
  event_sync_->registerCallback(boost::bind(&DavisStereoStitch::eventCallback, this, _1, _2));

  //inint Publisher
  event_pub_ = nh_.advertise<dvs_msgs::EventArray>("stitch_events", 1);
  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("stitch_image", 1);
  event_image_pub_ = it_.advertise("stitch_event_image", 1);
  left_event_image_pub_ = it_.advertise("left_event_image", 1);
  right_event_image_pub_ = it_.advertise("right_event_image", 1);
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
    left_cv_ptr = cv_bridge::toCvCopy(left_image);
    right_cv_ptr = cv_bridge::toCvCopy(right_image);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat left_cv_image_rd, right_cv_image_rd;
  cv::Mat left_cv_image, right_cv_image;

  cv::cvtColor(left_cv_ptr->image, left_cv_image_rd, CV_RGB2BGR);
  cv::cvtColor(right_cv_ptr->image, right_cv_image_rd, CV_RGB2BGR);

  if(got_camera_info_left_ && got_camera_info_right_)
  {
    cv::undistort(left_cv_image_rd, left_cv_image, left_camera_matrix_, left_dist_coeffs_);
    cv::undistort(right_cv_image_rd, right_cv_image, right_camera_matrix_, right_dist_coeffs_);
  }
  else
  {
    ROS_WARN("NO Camera Info!!!!! Please calibrate first!!!!!");
    return;
  }

  if (!((left_cv_image.cols == camera_info_left_.width) && (left_cv_image.rows == camera_info_left_.height)))
    return;
  
  if (!((right_cv_image.cols == camera_info_right_.width) && (right_cv_image.rows == camera_info_right_.height)))
    return;
  
  if (stitch_calibration_mode_ && find_stitch_once_)
  {
    //构造特征提取器，这里使用orb特征点
    cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SIFT::create(1000);
    std::vector<cv::KeyPoint> left_keypoints, right_keypoints;   //特征点
    cv::Mat left_descriptors, right_descriptors;                 //描述子
    //提取特征，描述子
    ptrFeature2D->detectAndCompute(left_cv_image, cv::noArray(), left_keypoints, left_descriptors);
    ptrFeature2D->detectAndCompute(right_cv_image, cv::noArray(), right_keypoints, right_descriptors);

    //构造匹配器,使用欧氏距离和交叉匹配策略进行图像匹配
    cv::BFMatcher matcher(cv::NORM_L2, true);
    //匹配描述子
    std::vector<cv::DMatch> matches;
    matcher.match(left_descriptors, right_descriptors, matches);
    std::vector<cv::Point2f> left_selPoints, right_selPoints;
    //std::vector<int> left_pointIndexes, right_pointIndexes;
    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
    {
      float x = left_keypoints[it->queryIdx].pt.x;
      float y = left_keypoints[it->queryIdx].pt.y;
      left_selPoints.push_back(cv::Point2f(x, y));
      x = right_keypoints[it->trainIdx].pt.x;
      y = right_keypoints[it->trainIdx].pt.y;
      right_selPoints.push_back(cv::Point2f(x, y));
    }
    //使用RANSAC算法估算单应矩阵
    std::vector<char> inliers;
    homography_ = cv::findHomography(right_selPoints, left_selPoints, inliers, CV_FM_RANSAC, 1.0);

    std::cout << homography_ << std::endl;
    find_stitch_once_ = false;
  }
  
  //用单应矩阵对图像进行变换
  cv::Mat result;
  try
  {
    cv::warpPerspective(right_cv_image, result, homography_, cv::Size(2*right_cv_image.cols,right_cv_image.rows));
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  //拼接
  cv::Mat half(result, cv::Rect(0, 0, left_cv_image.cols, left_cv_image.rows));
  left_cv_image.copyTo(half);

  cv_bridge::CvImage out_cv_image;
  result.copyTo(out_cv_image.image);
  out_cv_image.encoding = "bgr8";
  image_pub_.publish(out_cv_image.toImageMsg());
}

void DavisStereoStitch::eventCallback(const dvs_msgs::EventArray::ConstPtr &left_event, const dvs_msgs::EventArray::ConstPtr &right_event)
{
  //publish stitch events msg
  dvs_msgs::EventArray stitch_events_msg;
  stitch_events_msg.header.stamp = ros::Time::now();
  stitch_events_msg.header.frame_id = "camera_link";
  stitch_events_msg.height = left_event->height;
  stitch_events_msg.width = 2 * left_event->width;
  for (int i = 0; i < left_event->events.size(); i++)
  {
    stitch_events_msg.events.push_back(left_event->events[i]);
  }
  for (int i = 0; i < right_event->events.size(); i++)
  {
    double v1[3];
    double v2[] = { 0, 0, 1 };
    v2[0] = right_event->events[i].x;
    v2[1] = right_event->events[i].y;
    v2[2] = 1;
    cv::Mat V1 = cv::Mat(3, 1, CV_64FC1, v1);
    cv::Mat V2 = cv::Mat(3, 1, CV_64FC1, v2);
    V1 = homography_ * V2;
    dvs_msgs::Event e;
    e.x = uint16_t(v1[0] / v1[2]);
    e.y = uint16_t(v1[1] / v1[2]);
    e.ts = right_event->events[i].ts;
    e.polarity = right_event->events[i].polarity;
    stitch_events_msg.events.push_back(e);
  }
  event_pub_.publish(stitch_events_msg);

  //renderer events imag
  cv::Mat left_cv_image, right_cv_image;
  cv::Mat left_cv_image_rd, right_cv_image_rd;
  //renderer left events imag
  left_cv_image_rd = cv::Mat(left_event->height, left_event->width, CV_8UC3);
  left_cv_image_rd = cv::Scalar(0,0,0);
  for (int i = 0; i < left_event->events.size(); ++i)
  {
    const int x = left_event->events[i].x;
    const int y = left_event->events[i].y;

    left_cv_image_rd.at<cv::Vec3b>(cv::Point(x, y)) = (
    left_event->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
  }
  //renderer right events imag
  right_cv_image_rd = cv::Mat(right_event->height, right_event->width, CV_8UC3);
  right_cv_image_rd = cv::Scalar(0,0,0);
  for (int i = 0; i < right_event->events.size(); ++i)
  {
    const int x = right_event->events[i].x;
    const int y = right_event->events[i].y;

    right_cv_image_rd.at<cv::Vec3b>(cv::Point(x, y)) = (
    right_event->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
  }

  //undistort images
  if(got_camera_info_left_ && got_camera_info_right_)
  {
    cv::undistort(left_cv_image_rd, left_cv_image, left_camera_matrix_, left_dist_coeffs_);
    cv::undistort(right_cv_image_rd, right_cv_image, right_camera_matrix_, right_dist_coeffs_);
  }
  else
  {
    return;
  }
  //pubilsh renderer lift events imag
  cv_bridge::CvImage out_left_cv_image, out_right_cv_image;
  left_cv_image.copyTo(out_left_cv_image.image);
  out_left_cv_image.encoding = "bgr8";
  left_event_image_pub_.publish(out_left_cv_image.toImageMsg());
  //pubilsh renderer right events imag
  right_cv_image.copyTo(out_right_cv_image.image);
  out_right_cv_image.encoding = "bgr8";
  right_event_image_pub_.publish(out_right_cv_image.toImageMsg());

  //用单应矩阵对图像进行变换
  cv::Mat result;
  try
  {
    cv::warpPerspective(right_cv_image, result, homography_, cv::Size(2*right_cv_image.cols,right_cv_image.rows));
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  cv::Mat half(result, cv::Rect(0, 0, left_cv_image.cols, left_cv_image.rows));
  left_cv_image.copyTo(half);
  //pubilsh renderer stitch events imag
  cv_bridge::CvImage out_cv_image;
  result.copyTo(out_cv_image.image);
  out_cv_image.encoding = "bgr8";
  event_image_pub_.publish(out_cv_image.toImageMsg());
}

void DavisStereoStitch::cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (got_camera_info_left_)
    return;
  
  got_camera_info_left_ = true;
  camera_info_left_ = *msg;
  //std::cout << camera_info_left_ << std::endl;

  left_camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      left_camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i + j * 3];

  left_dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    left_dist_coeffs_.at<double>(i) = msg->D[i];
}

void DavisStereoStitch::cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (got_camera_info_right_)
    return;
  
  got_camera_info_right_ = true;
  camera_info_right_ = *msg;
  //std::cout << camera_info_right_ << std::endl;

  right_camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      right_camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i + j * 3];

  right_dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    right_dist_coeffs_.at<double>(i) = msg->D[i];
}

void DavisStereoStitch::findStitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
  find_stitch_once_ = msg->data;
}