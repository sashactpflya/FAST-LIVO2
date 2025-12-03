#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace fast_livo::utils
{

template <typename TDest, typename TSrc>
inline TDest toMsg(const TSrc &)
{
  static_assert(sizeof(TSrc) == 0, "Unsupported toMsg conversion.");
}

template <>
inline geometry_msgs::msg::Quaternion toMsg<geometry_msgs::msg::Quaternion, tf2::Quaternion>(const tf2::Quaternion &q)
{
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.w = q.w();
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  return q_msg;
}

template <>
inline geometry_msgs::msg::Transform toMsg<geometry_msgs::msg::Transform, tf2::Transform>(const tf2::Transform &tf)
{
  geometry_msgs::msg::Transform tf_msg;
  tf_msg.translation.x = tf.getOrigin().x();
  tf_msg.translation.y = tf.getOrigin().y();
  tf_msg.translation.z = tf.getOrigin().z();
  tf_msg.rotation.w = tf.getRotation().w();
  tf_msg.rotation.x = tf.getRotation().x();
  tf_msg.rotation.y = tf.getRotation().y();
  tf_msg.rotation.z = tf.getRotation().z();
  return tf_msg;
}

template <>
inline geometry_msgs::msg::TransformStamped toMsg<geometry_msgs::msg::TransformStamped, tf2::Transform>(const tf2::Transform &tf)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.transform.translation.x = tf.getOrigin().x();
  tf_msg.transform.translation.y = tf.getOrigin().y();
  tf_msg.transform.translation.z = tf.getOrigin().z();
  tf_msg.transform.rotation.w = tf.getRotation().w();
  tf_msg.transform.rotation.x = tf.getRotation().x();
  tf_msg.transform.rotation.y = tf.getRotation().y();
  tf_msg.transform.rotation.z = tf.getRotation().z();
  return tf_msg;
}

inline void fromMsg(const geometry_msgs::msg::Transform &tf_msg, tf2::Transform &tf)
{
  tf2::Vector3 origin(tf_msg.translation.x, tf_msg.translation.y, tf_msg.translation.z);
  tf2::Quaternion rotation(tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z, tf_msg.rotation.w);
  tf.setOrigin(origin);
  tf.setRotation(rotation);
}

// Overloads enabling type deduction without specifying TDest explicitly.
inline geometry_msgs::msg::Quaternion toMsg(const tf2::Quaternion &q)
{
  return toMsg<geometry_msgs::msg::Quaternion, tf2::Quaternion>(q);
}

inline geometry_msgs::msg::Transform toMsg(const tf2::Transform &tf)
{
  return toMsg<geometry_msgs::msg::Transform, tf2::Transform>(tf);
}

inline geometry_msgs::msg::TransformStamped toMsg(const tf2::Transform &tf, const rclcpp::Time &stamp,
                                                  const std::string &frame_id, const std::string &child_frame_id)
{
  geometry_msgs::msg::TransformStamped tf_msg = toMsg<geometry_msgs::msg::TransformStamped, tf2::Transform>(tf);
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  return tf_msg;
}

}  // namespace fast_livo::utils
