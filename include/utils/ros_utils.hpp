#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static Eigen::Isometry3d transform2isometry(const tf::StampedTransform& transform){
  tf::Quaternion q = transform.getRotation();
  tf::Vector3 tl = transform.getOrigin();
  Eigen::Quaterniond Eq(q.w(),q.x(),q.y(),q.z());
  Eigen::Isometry3d isome;
  isome.linear() = Eq.toRotationMatrix();
  Eigen::Vector3d Etl(tl.getX(),tl.getY(),tl.getZ());
  std::cout << "Quaternion: "<< q.w()<< " "  << q.x()<< " "  << q.y() << " " << q.z() << " translation: " << tl.getX() << " " << tl.getY()<< " " << tl.getZ() <<std::endl;
  std::cout << "Quaternion: "<< Eq.w()<< " "  << Eq.x()<< " "  << Eq.y() << " " << Eq.z() << " translation: " << Etl.x() << " " << Etl.y()<< " " << Etl.z() <<std::endl;
  
  isome.translation() = Etl;
  return isome;
}

static Eigen::Isometry3d pose2isometry(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  mat.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return mat;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}

static Eigen::Matrix4f odom2matrix(const nav_msgs::OdometryConstPtr& odom_msg) {
  Eigen::Isometry3d isometry = odom2isometry(odom_msg);
  Eigen::Matrix4d matrix4d = isometry.matrix();
  Eigen::Matrix4f matrix(Eigen::Matrix4f::Identity());
  for(size_t i = 0;i<4;++i){
    for (size_t j = 0; j < 4; j++)
    {
      matrix(i,j) = static_cast<float>(matrix4d(i,j)); 
    }
  }
  return matrix;
}

#endif // ROS_UTILS_HPP
