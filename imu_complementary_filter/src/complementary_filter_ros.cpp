/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

        @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
        All rights reserved.

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are
  met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
        DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
        DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//
#include "imu_complementary_filter/complementary_filter_ros.h"

// ROS
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace imu_tools {

void ReadParam(const std::string &param_name, bool result) {
  if (!result) {
    ROS_FATAL("Param read failure. : %s", param_name.c_str());
  }
}

ComplementaryFilterROS::ComplementaryFilterROS(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), initialized_filter_(false) {
  ROS_INFO("Starting ComplementaryFilterROS");
  initializeParams();

  int queue_size = 5;

  // Register publishers:
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(
      ros::names::resolve("imu") + "/data", queue_size);

  if (publish_debug_topics_) {
    rpy_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        ros::names::resolve("imu") + "/rpy/filtered", queue_size);

    if (filter_.getDoBiasEstimation()) {
      state_publisher_ = nh_.advertise<std_msgs::Bool>(
          ros::names::resolve("imu") + "/steady_state", queue_size);
    }
  }

  /**
  // Register IMU raw data subscriber.
  imu_subscriber_.reset(new ImuSubscriber(
      nh_, ros::names::resolve("imu") + "/data_raw", queue_size));

  // Register magnetic data subscriber.
  if (use_mag_) {
    mag_subscriber_.reset(new MagSubscriber(
        nh_, ros::names::resolve("imu") + "/mag", queue_size));

    sync_.reset(new Synchronizer(SyncPolicy(queue_size), *imu_subscriber_,
                                 *mag_subscriber_));
    sync_->registerCallback(
        boost::bind(&ComplementaryFilterROS::imuMagCallback, this, _1, _2));
  } else {
    // imu_subscriber_->registerCallback(&ComplementaryFilterROS::imuCallback,
    //                                  this);
    // imu_raw_subscriber->registerCallback(
    //    &ComplementaryFilterROS::imuRawCallback, this);
    // imu_raw_subscriber.subscribe(&ComplementaryFilterROS::imuRawCallback,
    // this);
  }
  **/

  ROS_INFO("Before Subscribe");
  imu_raw_subscriber_ =
      nh_.subscribe("/imu/data_raw", queue_size,
                    &ComplementaryFilterROS::imuRawCallback, this);
  ROS_INFO("After Subscribe");
}

ComplementaryFilterROS::~ComplementaryFilterROS() {
  ROS_INFO("Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams() {
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;

  // Frame
  ReadParam("fixed_frame", nh_private_.param<std::string>(
                               "fixed_frame", fixed_frame_, "odom"));
  ReadParam("usg_mag", nh_private_.param<bool>("use_mag", use_mag_, false));

  // TF Configuration.
  ReadParam("publish_tf",
            nh_private_.param<bool>("publish_tf", publish_tf_, false));
  ReadParam("reverse_tf",
            nh_private_.param<bool>("reverse_tf", reverse_tf_, false));
  ReadParam("constant_dt",
            nh_private_.param<double>("constant_dt", constant_dt_, 0.0));

  // Conversion
  ReadParam("acc_conv_ratio",
            nh_private_.param<double>("acc_conv_ratio", acc_conv_ratio, 0.0));
  ReadParam("ang_conv_ratio",
            nh_private_.param<double>("ang_conv_ratio", ang_conv_ratio, 0.0));

  // Offset Setting
  ReadParam("offset_acc_x",
            nh_private_.param<double>("offset_acc_x", offset_acc_x_, 0.0));
  ReadParam("offset_acc_y",
            nh_private_.param<double>("offset_acc_y", offset_acc_y_, 0.0));
  ReadParam("offset_acc_z",
            nh_private_.param<double>("offset_acc_z", offset_acc_z_, 0.0));
  ReadParam("offset_gyro_x",
            nh_private_.param<double>("offset_gyro_x", offset_gyro_x_, 0.0));
  ReadParam("offset_gyro_y",
            nh_private_.param<double>("offset_gyro_y", offset_gyro_y_, 0.0));
  ReadParam("offset_gyro_z",
            nh_private_.param<double>("offset_gyro_z", offset_gyro_z_, 0.0));

  // Gain Setting.
  ReadParam("gain_acc", nh_private_.param<double>("gain_acc", gain_acc, 0.01));
  ReadParam("gain_mag", nh_private_.param<double>("gain_mag", gain_mag, 0.01));
  ReadParam(
      "do_adaptive_gain",
      nh_private_.param<bool>("do_adaptive_gain", do_adaptive_gain, true));

  // Bias Setting.
  ReadParam(
      "do_bias_estimation",
      nh_private_.param<bool>("do_bias_estimation", do_bias_estimation, true));
  ReadParam("bias_alpha",
            nh_private_.param<double>("bias_alpha", bias_alpha, 0.01));

  {
    double orientation_stddev;
    ReadParam("orientation_stddev",
              nh_private_.param<double>("orientation_stddev",
                                        orientation_stddev, 0.0));
    orientation_variance_ = orientation_stddev * orientation_stddev;
  }

  // Debug Related.
  ReadParam("publish_debug_topics",
            nh_private_.param<bool>("publish_debug_topics",
                                    publish_debug_topics_, false));

  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setDoAdaptiveGain(do_adaptive_gain);

  if (!filter_.setGainAcc(gain_acc))
    ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_) {
    if (!filter_.setGainMag(gain_mag))
      ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation) {
    if (!filter_.setBiasAlpha(bias_alpha))
      ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
  }

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0) {
    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    ROS_WARN("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
             constant_dt_);
    constant_dt_ = 0.0;
  }
}

void ComplementaryFilterROS::imuCallback(const ImuMsg::ConstPtr &imu_msg_raw) {
  const geometry_msgs::Vector3 &a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3 &w = imu_msg_raw->angular_velocity;
  const ros::Time &time = imu_msg_raw->header.stamp;

  // Initialize.
  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0) {
    dt = constant_dt_;
  } else {
    dt = (time - time_prev_).toSec();
  }

  time_prev_ = time;

  // Update the filter.
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  // Publish state.
  publish(imu_msg_raw);
}

void ComplementaryFilterROS::imuRawCallback(
    //    const kmsgs::Imu::ConstPtr &imu_msg_raw) {
    const kmsgs::Imu &imu_msg_raw) {

  ROS_ERROR("Inside imuRawCallback");

  boost::shared_ptr<sensor_msgs::Imu> imu_msg =
      boost::make_shared<sensor_msgs::Imu>();

  imu_msg->header = imu_msg_raw.header;
  ros::Time time = imu_msg_raw.header.stamp;

  // Linear Acceleration.
  imu_msg->linear_acceleration.x =
      acc_conv_ratio * (imu_msg_raw.accX + offset_acc_x_);
  imu_msg->linear_acceleration.y =
      acc_conv_ratio * (imu_msg_raw.accY + offset_acc_y_);
  imu_msg->linear_acceleration.z =
      acc_conv_ratio * (imu_msg_raw.accZ + offset_acc_z_);

  // Angular Velocity.
  imu_msg->angular_velocity.x =
      ang_conv_ratio * (imu_msg_raw.omegaX + offset_gyro_x_);
  imu_msg->angular_velocity.y =
      ang_conv_ratio * (imu_msg_raw.omegaY + offset_gyro_y_);
  imu_msg->angular_velocity.z =
      ang_conv_ratio * (imu_msg_raw.omegaZ + offset_gyro_z_);

  // Initialize.
  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0) {
    dt = constant_dt_;
  } else {
    dt = (time - time_prev_).toSec();
  }

  time_prev_ = time;

  filter_.update(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                 imu_msg->linear_acceleration.z, imu_msg->angular_velocity.x,
                 imu_msg->angular_velocity.y, imu_msg->angular_velocity.z, dt);

  publish(imu_msg);
  // publish(ImuMsg::ConstPtr(imu_msg));
}

void ComplementaryFilterROS::imuMagCallback(const ImuMsg::ConstPtr &imu_msg_raw,
                                            const MagMsg::ConstPtr &mag_msg) {

  const geometry_msgs::Vector3 &a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3 &w = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3 &m = mag_msg->magnetic_field;
  const ros::Time &time = imu_msg_raw->header.stamp;

  // Initialize.
  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // Calculate dt.
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;
  // ros::Time t_in, t_out;
  // t_in = ros::Time::now();
  // Update the filter.
  if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z)) {
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
  } else {
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);
  }

  // t_out = ros::Time::now();
  // float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
  // printf("%.6f\n", dt_tot);
  // Publish state.
  publish(imu_msg_raw);
}

tf::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(double q0,
                                                              double q1,
                                                              double q2,
                                                              double q3) const {
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(
    const sensor_msgs::Imu::ConstPtr &imu_msg_raw) {
  // Get the orientation:
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // Create and publish fitlered IMU message.
  boost::shared_ptr<sensor_msgs::Imu> imu_msg =
      boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);
  tf::quaternionTFToMsg(q, imu_msg->orientation);

  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance_;

  // Account for biases.
  if (filter_.getDoBiasEstimation()) {
    imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
    imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
    imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
  }

  imu_publisher_.publish(imu_msg);

  if (publish_debug_topics_) {
    // Create and publish roll, pitch, yaw angles
    geometry_msgs::Vector3Stamped rpy;
    rpy.header = imu_msg_raw->header;

    tf::Matrix3x3 M;
    M.setRotation(q);
    M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    rpy_publisher_.publish(rpy);

    // Publish whether we are in the steady state, when doing bias estimation
    if (filter_.getDoBiasEstimation()) {
      std_msgs::Bool state_msg;
      state_msg.data = filter_.getSteadyState();
      state_publisher_.publish(state_msg);
    }
  }

  if (publish_tf_) {
    // Create and publish the ROS tf.
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);

    if (reverse_tf_) {
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(transform.inverse(), imu_msg_raw->header.stamp,
                               imu_msg_raw->header.frame_id, fixed_frame_));
    } else {
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(transform, imu_msg_raw->header.stamp,
                               fixed_frame_, imu_msg_raw->header.frame_id));
    }
  }
}

} // namespace imu_tools
