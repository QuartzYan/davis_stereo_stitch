#include "davis_stereo_stitch.h"

DavisStereoStitch::DavisStereoStitch()
{
  got_camera_info_left_ = false;
  got_camera_info_right_ = false;

  homography_ =(Mat_<double>(3,3)<< 0.4916559814809626, 0.1455768474611172, 150.7872720548855,
                                    -0.2305138272924719, 0.9529974437006833, 12.4090961153593,
                                    -0.001774090006639621, 0.0003216603110495318, 1);

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

  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("stitch_image", 1);
  event_image_pub_ = it_.advertise("stitch_event_image", 1);
  left_event_image_pub_ = it_.advertise("left_event_image", 1);
  right_event_image_pub_ = it_.advertise("right_event_image", 1);
  //event_pub_ = nh_.advertise<dvs_msgs::EventArray>("stitch_events", 1);
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
    return;
  }

  if (!((left_cv_image.cols == camera_info_left_.width) && (left_cv_image.rows == camera_info_left_.height)))
  {
    return;
  }
  if (!((right_cv_image.cols == camera_info_right_.width) && (right_cv_image.rows == camera_info_right_.height)))
  {
    return;
  }
  
  
  
  // static int k = 0;
  // //std::cout << homography_ << std::endl;
  //  if(k<5)
  //  {
  //    //构造特征提取器，这里使用orb特征点
  //    cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SIFT::create(1000);
  //    std::vector<cv::KeyPoint> left_keypoints, right_keypoints;   //特征点
  //    cv::Mat left_descriptors, right_descriptors;                 //描述子
  //    //提取特征，描述子
  //    ptrFeature2D->detectAndCompute(left_cv_image, cv::noArray(), left_keypoints, left_descriptors);
  //    ptrFeature2D->detectAndCompute(right_cv_image, cv::noArray(), right_keypoints, right_descriptors);

//     // //基于FLANN的描述符对象匹配
//     // cv::flann::Index flannIndex(right_descriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
//     // cv::Mat matchIndex(left_descriptors.rows, 2, CV_32SC1);
//     // cv::Mat matchDistance(left_descriptors.rows, 2, CV_32FC1);
//     // flannIndex.knnSearch(left_descriptors, matchIndex, matchDistance, 2, cv::flann::SearchParams());//调用K邻近算法
//     // //根据劳氏算法（Lowe's algorithm）选出优秀的匹配
//     // std::vector<cv::DMatch> goodMatches;
//     // std::vector<cv::Point2f> left_selPoints, right_selPoints;
//     // int dd = 0;
//     // for (int i = 0; i < matchDistance.rows; i++)
//     // {
//     //   if (matchDistance.at<float>(i, 0) < 0.6 * matchDistance.at<float>(i, 1))
//     //   {
//     //     cv::DMatch dmatches(i, matchIndex.at<int>(i, 0), matchDistance.at<float>(i, 0));
//     //     goodMatches.push_back(dmatches);
//     //     float x = left_keypoints[dd].pt.x;
//     //     float y = left_keypoints[dd].pt.y;
//     //     left_selPoints.push_back(cv::Point2f(x, y));
//     //     x = right_keypoints[dd].pt.x;
//     //     y = right_keypoints[dd].pt.y;
//     //     right_selPoints.push_back(cv::Point2f(x, y));
//     //     dd++;
//     //   }
//     // }
//     //cv::Mat matchImage;
//     //cv::drawMatches(left_cv_image, left_keypoints, right_cv_image, right_keypoints, goodMatches, matchImage);

//      //构造匹配器,使用欧氏距离和交叉匹配策略进行图像匹配
//      cv::BFMatcher matcher(cv::NORM_L2, true);
//      //匹配描述子
//      std::vector<cv::DMatch> matches;
//      matcher.match(left_descriptors, right_descriptors, matches);
//      std::vector<cv::Point2f> left_selPoints, right_selPoints;
//      //std::vector<int> left_pointIndexes, right_pointIndexes;
//      for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
//      {
//         float x = left_keypoints[it->queryIdx].pt.x;
//         float y = left_keypoints[it->queryIdx].pt.y;
//         left_selPoints.push_back(cv::Point2f(x, y));
//         x = right_keypoints[it->trainIdx].pt.x;
//         y = right_keypoints[it->trainIdx].pt.y;
//         right_selPoints.push_back(cv::Point2f(x, y));
//      }
//      //使用RANSAC算法估算单应矩阵
//      std::vector<char> inliers;
//      homography_ = cv::findHomography(right_selPoints, left_selPoints, inliers, CV_FM_RANSAC, 1.0);
//      cv::Mat matchImage;
//      cv::drawMatches(left_cv_image, left_keypoints, 
//                      right_cv_image, right_keypoints, 
//                      matches, 
//                      matchImage, 
//                      cv::Scalar(255, 255, 255), 
//                      cv::Scalar(255, 255, 255), 
//                      inliers,
//                      2);
//    k++; 
//    std::cout << homography_ << std::endl;
//  }

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
  cv::Mat left_cv_image, right_cv_image;
  cv::Mat left_cv_image_rd, right_cv_image_rd;

  left_cv_image_rd = cv::Mat(left_event->height, left_event->width, CV_8UC3);
  left_cv_image_rd = cv::Scalar(0,0,0);
  for (int i = 0; i < left_event->events.size(); ++i)
  {
    const int x = left_event->events[i].x;
    const int y = left_event->events[i].y;

    left_cv_image_rd.at<cv::Vec3b>(cv::Point(x, y)) = (
    left_event->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
  }

  right_cv_image_rd = cv::Mat(right_event->height, right_event->width, CV_8UC3);
  right_cv_image_rd = cv::Scalar(0,0,0);
  for (int i = 0; i < right_event->events.size(); ++i)
  {
    const int x = right_event->events[i].x;
    const int y = right_event->events[i].y;

    right_cv_image_rd.at<cv::Vec3b>(cv::Point(x, y)) = (
    right_event->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
  }

  //
  if(got_camera_info_left_ && got_camera_info_right_)
  {
    cv::undistort(left_cv_image_rd, left_cv_image, left_camera_matrix_, left_dist_coeffs_);
    cv::undistort(right_cv_image_rd, right_cv_image, right_camera_matrix_, right_dist_coeffs_);
  }
  else
  {
    return;
  }

  cv_bridge::CvImage out_left_cv_image, out_right_cv_image;
  left_cv_image.copyTo(out_left_cv_image.image);
  out_left_cv_image.encoding = "bgr8";
  left_event_image_pub_.publish(out_left_cv_image.toImageMsg());

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