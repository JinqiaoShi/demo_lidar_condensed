#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double timeOdomBefBA;
double timeOdomAftBA;

double rollRec, pitchRec, yawRec;
double txRec, tyRec, tzRec;



ros::Publisher *voData2PubPointer = NULL;
tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
nav_msgs::Odometry voData2;
tf::StampedTransform voDataTrans2;


void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData)
{
  if (fabs(timeOdomBefBA - timeOdomAftBA) < 0.005) {

    geometry_msgs::Quaternion geoQuat = voData->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(rollRec, pitchRec, yawRec);

    txRec = voData->pose.pose.position.x;
    tyRec = voData->pose.pose.position.y;
    tzRec = voData->pose.pose.position.z;



    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rollRec, pitchRec, yawRec);

    voData2.header.stamp = voData->header.stamp;
    voData2.pose.pose.orientation.x = -geoQuat.y;
    voData2.pose.pose.orientation.y = -geoQuat.z;
    voData2.pose.pose.orientation.z = geoQuat.x;
    voData2.pose.pose.orientation.w = geoQuat.w;
    voData2.pose.pose.position.x = txRec;
    voData2.pose.pose.position.y = tyRec;
    voData2.pose.pose.position.z = tzRec;
    voData2PubPointer->publish(voData2);

    voDataTrans2.stamp_ = voData->header.stamp;
    voDataTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    voDataTrans2.setOrigin(tf::Vector3(txRec, tyRec, tzRec));
    tfBroadcaster2Pointer->sendTransform(voDataTrans2);
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle nh;

  ros::Subscriber voDataSub = nh.subscribe<nav_msgs::Odometry> 
                              ("/cam_to_init", 1, voDataHandler);


  ros::Publisher voData2Pub = nh.advertise<nav_msgs::Odometry> ("/cam2_to_init", 1);
  voData2PubPointer = &voData2Pub;
  voData2.header.frame_id = "/camera_init";
  voData2.child_frame_id = "/camera2";

  tf::TransformBroadcaster tfBroadcaster2;
  tfBroadcaster2Pointer = &tfBroadcaster2;
  voDataTrans2.frame_id_ = "/camera_init";
  voDataTrans2.child_frame_id_ = "/camera2";

  ros::spin();

  return 0;
}
