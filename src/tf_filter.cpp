#include <chrono>
#include <cpplogging/cpplogging.h>
#include <cpputils/Timer.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <vector>

using namespace cpplogging;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  // TODO: MAKE THIS PARAMETRIZABLE
  auto log = CreateLogger("tf_filter");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

  ros::init(argc, argv, "tf_filter");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::TransformBroadcaster tfBroadcaster;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  tf::StampedTransform cameraMrov, wMrov;
  ros::Rate rate(30);
  while (ros::ok()) {
    try {
      listener.lookupTransform("bluerov2_camera", "hil", ros::Time(0),
                               cameraMrov);
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "camera";
      static_transformStamped.child_frame_id = "aruco_rov";
      static_transformStamped.transform.translation.x =
          cameraMrov.getOrigin().getX();
      static_transformStamped.transform.translation.y =
          cameraMrov.getOrigin().getY();
      static_transformStamped.transform.translation.z =
          cameraMrov.getOrigin().getZ();
      static_transformStamped.transform.rotation.x =
          cameraMrov.getRotation().x();
      static_transformStamped.transform.rotation.y =
          cameraMrov.getRotation().y();
      static_transformStamped.transform.rotation.z =
          cameraMrov.getRotation().z();
      static_transformStamped.transform.rotation.w =
          cameraMrov.getRotation().w();
      static_transforms.push_back(static_transformStamped);
      break;
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      rate.sleep();
    }
  }
  static_broadcaster.sendTransform(static_transforms);

  std::list<tf::StampedTransform> fwindow;

  tf::Vector3 wTrov, auxT, lastT, lastNaifT;
  tf::Quaternion wRrov, auxR, lastR, lastNaifR;

  cpputils::Timer pubTimer, logTimer;

  geometry_msgs::Pose nmsg;
  pubTimer.Reset();
  double pubRate = 3;
  double pubPeriod = 1 / pubRate * 1000; // ms
  logTimer.Reset();
  bool first = true;
  while (ros::ok()) {
    try {
      listener.lookupTransform("world", "aruco_rov", ros::Time(0), wMrov);
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (pubTimer.Elapsed() < pubPeriod)
      continue;

    auxT = wMrov.getOrigin();
    auxR = wMrov.getRotation();

    pubTimer.Reset();

    if (first) {
      lastR = auxR;
      lastT = auxT;
      lastNaifT = lastT;
      lastNaifR = lastR;
      first = false;
    }

    double delta = 0.15;
    double deltaB = 0.85;
    wTrov[0] = deltaB * lastNaifT[0] + delta * auxT[0];
    wTrov[1] = deltaB * lastNaifT[1] + delta * auxT[1];
    wTrov[2] = deltaB * lastNaifT[2] + delta * auxT[2];

    wRrov.setX(deltaB * lastNaifR.getX() + delta * auxR.getX());
    wRrov.setY(deltaB * lastNaifR.getY() + delta * auxR.getY());
    wRrov.setZ(deltaB * lastNaifR.getZ() + delta * auxR.getZ());
    wRrov.setW(deltaB * lastNaifR.getW() + delta * auxR.getW());

    log->Info("{} {} {} -- {} {} {} ", auxT[0], auxT[1], auxT[2], wTrov[0],
              wTrov[1], wTrov[2]);

    lastNaifT[0] = wTrov[0];
    lastNaifT[1] = wTrov[1];
    lastNaifT[2] = wTrov[2];
    lastNaifR.setX(wRrov.getX());
    lastNaifR.setY(wRrov.getY());
    lastNaifR.setZ(wRrov.getZ());
    lastNaifR.setW(wRrov.getW());

    wMrov.setOrigin(wTrov);
    wMrov.setRotation(wRrov.normalize());

    tfBroadcaster.sendTransform(
        tf::StampedTransform(wMrov, ros::Time::now(), "world", "erov"));

    ros::spinOnce();
  }
  return 0;
}
