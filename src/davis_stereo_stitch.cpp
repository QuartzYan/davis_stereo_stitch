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
    left_cv_ptr = cv_bridge::toCvCopy(left_image);
    right_cv_ptr = cv_bridge::toCvCopy(right_image);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat left_cv_image, right_cv_image;

  cv::cvtColor(left_cv_ptr->image, left_cv_image, CV_RGB2BGR);
  cv::cvtColor(right_cv_ptr->image, right_cv_image, CV_RGB2BGR);

  // cv::Mat img1 = cv::imread("/home/caster/davis_ws/src/davis_stereo_stitch/1.jpg");
  // cv::Mat img2 = cv::imread("/home/caster/davis_ws/src/davis_stereo_stitch/2.jpg");
  // if (!img1.data || !img2.data)
  // {
  //   std::cout << "read image error!!!" << std::endl;
  // }
  // img1.copyTo(left_cv_image);
  // img2.copyTo(right_cv_image);

  // std::vector<cv::Mat> imgs;
  // imgs.push_back(left_cv_image);
  // imgs.push_back(right_cv_image);
  // // 使用stitch函数进行拼接
  // cv::Stitcher stitcher = cv::Stitcher::createDefault();
	// cv::Mat result;
	// Stitcher::Status status = stitcher.stitch(imgs, result);
	// if (status != Stitcher::OK)
	// {
	// 	std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
	// }

  static int k = 0;
  static cv::Mat homography;
  // homography =(Mat_<double>(3,3)<<1.013560882527164, 0.08807703589756846, 200.4442873161193,
  //                                 0.02047040792555651, 1.043960338698521, -7.608720426590968,
  //                                 2.590589358762006e-05, 9.12028623436831e-05, 1);
  //std::cout << homography << std::endl;

  if(k<5)
  {
    //构造特征提取器，这里使用orb特征点
    cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SIFT::create(800);
    std::vector<cv::KeyPoint> left_keypoints, right_keypoints;   //特征点
    cv::Mat left_descriptors, right_descriptors;                 //描述子

    //提取特征，描述子
    ptrFeature2D->detectAndCompute(left_cv_image, cv::noArray(), left_keypoints, left_descriptors);
    ptrFeature2D->detectAndCompute(right_cv_image, cv::noArray(), right_keypoints, right_descriptors);

    // //基于FLANN的描述符对象匹配
    // cv::flann::Index flannIndex(right_descriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
    // cv::Mat matchIndex(left_descriptors.rows, 2, CV_32SC1);
    // cv::Mat matchDistance(left_descriptors.rows, 2, CV_32FC1);
    // flannIndex.knnSearch(left_descriptors, matchIndex, matchDistance, 2, cv::flann::SearchParams());//调用K邻近算法
    // //根据劳氏算法（Lowe's algorithm）选出优秀的匹配
    // std::vector<cv::DMatch> goodMatches;
    // std::vector<cv::Point2f> left_selPoints, right_selPoints;
    // int dd = 0;
    // for (int i = 0; i < matchDistance.rows; i++)
    // {
    //   if (matchDistance.at<float>(i, 0) < 0.6 * matchDistance.at<float>(i, 1))
    //   {
    //     cv::DMatch dmatches(i, matchIndex.at<int>(i, 0), matchDistance.at<float>(i, 0));
    //     goodMatches.push_back(dmatches);

    //     float x = left_keypoints[dd].pt.x;
    //     float y = left_keypoints[dd].pt.y;
    //     left_selPoints.push_back(cv::Point2f(x, y));

    //     x = right_keypoints[dd].pt.x;
    //     y = right_keypoints[dd].pt.y;
    //     right_selPoints.push_back(cv::Point2f(x, y));

    //     dd++;
    //   }
    // }

    //cv::Mat matchImage;
    //cv::drawMatches(left_cv_image, left_keypoints, right_cv_image, right_keypoints, goodMatches, matchImage);


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
    homography = cv::findHomography(right_selPoints, left_selPoints, inliers, CV_FM_RANSAC, 1.0);

    cv::Mat matchImage;
    cv::drawMatches(left_cv_image, left_keypoints, 
                    right_cv_image, right_keypoints, 
                    matches, 
                    matchImage, 
                    cv::Scalar(255, 255, 255), 
                    cv::Scalar(255, 255, 255), 
                    inliers,
                    2);

 k++; 
 std::cout << homography << std::endl;
}

  //用单应矩阵对图像进行变换
  cv::Mat result;
  cv::warpPerspective(right_cv_image, result, homography, cv::Size(2*right_cv_image.cols,right_cv_image.rows));

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
  event_pub_.publish(left_event);
}

  
  