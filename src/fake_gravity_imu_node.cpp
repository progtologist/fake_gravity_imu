/*********************************************************************
* fake_gravity_imu_node.cpp
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, University of Patras
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
*   * Neither the name of University of Patras nor the names of its
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
*
* Authors: Aris Synodinos
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_gravity_imu_node");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
  tf::TransformListener listener;
  geometry_msgs::Vector3Stamped acc;
  std::string source_frame;
  std::string target_frame;
  nh.getParam("source_frame", source_frame);
  nh.getParam("target_frame", target_frame);
  acc.header.frame_id = source_frame;
  acc.vector.z = - 9.81;
  listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(5));
  ros::Rate r(100);
  while(ros::ok()) {
    geometry_msgs::Vector3Stamped acc_tool;
    try {
      listener.transformVector(target_frame, acc, acc_tool);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    sensor_msgs::Imu msg;
    msg.header= acc_tool.header;
    msg.linear_acceleration = acc_tool.vector;
    pub.publish(msg);
    r.sleep();
  }
  return 0;
}
