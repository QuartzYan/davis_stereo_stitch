#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import csv
import time
import rospy 
import std_msgs
from dvs_msgs.msg import Event
from dvs_msgs.msg import EventArray

sub = None
filename = None

def callBack(msg):
  if not rospy.is_shutdown():
    with open(filename, mode='a') as csv_file:
      csv_write = csv.writer(csv_file)
      for e in msg.events:
        data = (e.x, e.y, e.ts.secs, e.ts.nsecs, e.polarity)
        csv_write.writerow(data)
  else:
    pass

def main():
  global sub, filename
  rospy.init_node("record_events_node")
  topic_name = rospy.get_param("~topic_name", default="/stitch_events")
  sub = rospy.Subscriber(topic_name, EventArray, callBack)
  filename = os.path.expanduser('~') + "/" + topic_name.replace('/', '_') + "_" + time.strftime("%Y%m%d%H%M%S", time.localtime()) + ".csv"
  with open(filename, mode='wb') as csv_file:
    fieldnames = ['x', 'y', 'secs', 'nsecs', 'polarity']
    csv_write  = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_write.writeheader()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass