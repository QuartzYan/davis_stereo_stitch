#include <iostream>
#include <ros/ros.h>
#include "davis_stereo_stitch.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "davis_stereo_stitch");

  DavisStereoStitch davis_stereo_stitch;

  ros::spin();
  return 0;
}
