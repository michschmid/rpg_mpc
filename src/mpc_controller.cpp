/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "rpg_mpc/mpc_controller.h"

#include <ctime>

namespace rpg_mpc {

template<typename T>
MpcController<T>::MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& topic) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(MpcWrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
                                                  0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
    point_of_interest_(Eigen::Matrix<T, 6, 1>::Zero()),
    online_data_check_(Eigen::Matrix<T, kOdSize, 1>::Zero()),
    obstacle_(Eigen::Matrix<T, 10, 1>::Zero()) {
  pub_predicted_trajectory_ =
      nh_.advertise<nav_msgs::Path>(topic, 1);
  pub_reference_trajectory_ =
      nh_.advertise<nav_msgs::Path>("mpc/trajectory_reference", 1);
  pub_angle_ = nh_.advertise<std_msgs::Float32>("mpc/projection_angle", 1);
  pub_radius_ = nh_.advertise<std_msgs::Float32>("mpc/projection_radius", 1);

  sub_point_of_interest_ = nh_.subscribe("/line_poi", 1,
                                         &MpcController<T>::pointOfInterestCallback, this);
  sub_obstacles_ = nh_.subscribe("/obstacles", 1, &MpcController<T>::obstacleCallback, this);
  sub_autopilot_off_ = nh_.subscribe("autopilot/off", 1,
                                     &MpcController<T>::offCallback, this);

  cost_serv_ = nh_.advertiseService("set_perception_cost", &MpcController<T>::setPerceptionCost, this);

  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  params_.changed_ = true;
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}

template<typename T>
bool MpcController<T>::setPerceptionCost(rpg_mpc::set_perception_cost::Request& request, rpg_mpc::set_perception_cost::Response& response)
{
  params_.Q_(kCostSize-2, kCostSize-2) = request.theta_cost;
  params_.Q_(kCostSize-1, kCostSize-1) = request.radius_cost;
  params_.changed_ = true;
  setNewParams(params_);
  return true;
}

template<typename T>
void MpcController<T>::pointOfInterestCallback(
    const geometry_msgs::PoseArray::ConstPtr& msg) {
  point_of_interest_(0) = msg->poses[0].position.x;
  point_of_interest_(1) = msg->poses[0].position.y;
  point_of_interest_(2) = msg->poses[0].position.z;
  point_of_interest_(3) = msg->poses[1].position.x;
  point_of_interest_(4) = msg->poses[1].position.y;
  point_of_interest_(5) = msg->poses[1].position.z;
  mpc_wrapper_.setPointOfInterest(point_of_interest_);
  // ROS_INFO("======================DEBUG===========================");
  // ROS_INFO("x1: %f, y1: %f, z1: %f, x2: %f, y2: %f, z2: %f ", point_of_interest_(0), point_of_interest_(1), point_of_interest_(2), point_of_interest_(3), point_of_interest_(4), point_of_interest_(5));
  // ROS_INFO("======================DEBUG===========================");
}

template<typename T>
void MpcController<T>::obstacleCallback(
  const vision_node::ObstacleArray::ConstPtr& msg)
{
  // Assumes the messages being ordered with the first obstacle being the closest
  obstacle_(0) = msg->obstacles[0].translation.x;
  obstacle_(1) = msg->obstacles[0].translation.y;
  obstacle_(2) = msg->obstacles[0].translation.z;
  obstacle_(3) = msg->obstacles[0].dimensions.x;
  obstacle_(4) = msg->obstacles[0].dimensions.y;
  obstacle_(5) = msg->obstacles[0].dimensions.z;
  obstacle_(6) = msg->obstacles[0].rotation.w;
  obstacle_(6) = msg->obstacles[0].rotation.x;
  obstacle_(6) = msg->obstacles[0].rotation.y;
  obstacle_(6) = msg->obstacles[0].rotation.z;
  mpc_wrapper_.setObstacle(obstacle_);
}

template<typename T>
void MpcController<T>::offCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  solve_from_scratch_ = true;
}

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::off() {
  quadrotor_common::ControlCommand command;

  command.zero();

  return command;
}

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::run(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const quadrotor_common::Trajectory& reference_trajectory,
    const MpcParams<T>& params) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }

  preparation_thread_.join();

  // Convert everything into Eigen format.
  setStateEstimate(state_estimate);
  setReference(reference_trajectory);

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);
  // ~~~~ DEBUGGING ~~~~
  // TODO: make this an if clause
  // Could not find a way to output acado intermediateState, therefore redo here for debugging and recording 
  // might not coincide with the actual value at initialization!!
  // mpc_wrapper_.getReferenceState(1, reference_state_check_);
  // ROS_INFO("DEBUG: u_ref = %f , v_ref = %f", reference_state_check_[14], reference_state_check_[14]);
  mpc_wrapper_.getOnlineData(online_data_check_);
  // ROS_INFO("DEBUG: test = %f", online_data_check_[0]);
  const double epsilon = 0.1; 
  // States
  float     p_x, p_y, p_z;
  float     q_w, q_x, q_y, q_z;
  float     v_x, v_y, v_z;
  p_x = predicted_states_(kPosX);
  p_y = predicted_states_(kPosY);
  p_z = predicted_states_(kPosZ);
  q_w = predicted_states_(kOriW);
  q_x = predicted_states_(kOriX);
  q_y = predicted_states_(kOriY);
  q_z = predicted_states_(kOriZ);
  v_x = predicted_states_(kVelX);
  v_y = predicted_states_(kVelY);
  v_z = predicted_states_(kVelZ);
  // std::cout << predicted_states_ << "\n";
  // Online data
  float     p_F1_x, p_F1_y, p_F1_z, p_F2_x, p_F2_y, p_F2_z;
  float     t_B_C_x, t_B_C_y, t_B_C_z;
  float     q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
  p_F1_x = online_data_check_(0);
  p_F1_y = online_data_check_(1);
  p_F1_z = online_data_check_(2);
  p_F2_x = online_data_check_(3);
  p_F2_y = online_data_check_(4);
  p_F2_z = online_data_check_(5);
  t_B_C_x = online_data_check_(6);
  t_B_C_y = online_data_check_(7);
  t_B_C_z = online_data_check_(8);
  q_B_C_w = online_data_check_(9);
  q_B_C_x = online_data_check_(10);
  q_B_C_y = online_data_check_(11);
  q_B_C_z = online_data_check_(12);

  // projection calculation
  float intSx1 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F1_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F1_z-p_z));
  float intSy1 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w))*(p_F1_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F1_x-p_x));
  float intSz1 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w))*(p_F1_y-p_y));

  float intSx2 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F2_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F2_z-p_z));
  float intSy2 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w))*(p_F2_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F2_x-p_x));
  float intSz2 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w))*(p_F2_y-p_y));

  // normalized image coordinates
  float u_norm1 = intSx1/(intSz1 + epsilon);
  float v_norm1 = intSy1/(intSz1 + epsilon);
  float u_norm2 = intSx2/(intSz2 + epsilon);
  float v_norm2 = intSy2/(intSz2 + epsilon);

  // TODO: maybe handle case when two points are identical, e.g. origin when initialized
  // ROS_INFO_THROTTLE(0.5, "DEBUG: u1 = %f, v1 = %f, u2 = %f, v2 = %f", u_norm1, v_norm1, u_norm2, v_norm2);

  // Calculate polar representation
  float theta, radius;
  theta = atan(-(u_norm2 - u_norm1) / (v_norm2 - v_norm1));
  radius = (v_norm1 - (v_norm2 - v_norm1) / (u_norm2 - u_norm1) * u_norm1) * sin(atan(-(u_norm2 - u_norm1)/ (v_norm2 - v_norm1)));

  float d = sqrt(((p_x - p_F1_x)*(p_y - p_F2_y) - (p_y - p_F1_y)*(p_x - p_F2_x))*((p_x - p_F1_x)*(p_y - p_F2_y) - (p_y - p_F1_y)*(p_x - p_F2_x)) + ((p_x - p_F1_x)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_x - p_F2_x))*((p_x - p_F1_x)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_x - p_F2_x)) + ((p_y - p_F1_y)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_y - p_F2_y))*((p_y - p_F1_y)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_y - p_F2_y)))/sqrt((p_F1_x - p_F2_x)*(p_F1_x - p_F2_x) + (p_F1_y - p_F2_y)*(p_F1_y - p_F2_y) + (p_F1_z - p_F2_z)*(p_F1_z - p_F2_z));

  // Chance constraint
  double p_o_x = online_data_check_(13);
  double p_o_y = online_data_check_(14);
  double p_o_z = online_data_check_(15);
  double a_o = online_data_check_(16);
  double b_o = online_data_check_(17);
  double c_o = online_data_check_(18);
  const double r_o = 0.4;
  const double so = 0.001;
  const double sb = 0.001;
  float dp_norm = sqrt((p_x - p_o_x)*(p_x - p_o_x) + (p_y - p_o_y)*(p_y - p_o_y) + (p_z - p_o_z)*(p_z - p_o_z));
  float n_o_x = (p_x - p_o_x) / dp_norm;
  float n_o_y = (p_y - p_o_y) / dp_norm;
  float n_o_z = (p_z - p_o_z) / dp_norm;
    // Quaternion to rotation matrix
  float q_W_O_w = online_data_check_(19);
  float q_W_O_x = online_data_check_(20);
  float q_W_O_y = online_data_check_(21);
  float q_W_O_z = online_data_check_(22);
  float R_W_O_11 = 2*q_W_O_w*q_W_O_w + 2*q_W_O_x*q_W_O_x - 1;
  float R_W_O_12 = 2*q_W_O_x*q_W_O_y - 2*q_W_O_w*q_W_O_z;
  float R_W_O_13 = 2*q_W_O_w*q_W_O_y + 2*q_W_O_x*q_W_O_z;
  float R_W_O_21 = 2*q_W_O_w*q_W_O_z + 2*q_W_O_x*q_W_O_y;
  float R_W_O_22 = 2*q_W_O_w*q_W_O_w + 2*q_W_O_y*q_W_O_y - 1;
  float R_W_O_23 = 2*q_W_O_y*q_W_O_z - 2*q_W_O_w*q_W_O_x;
  float R_W_O_31 = 2*q_W_O_x*q_W_O_z - 2*q_W_O_w*q_W_O_y;
  float R_W_O_32 = 2*q_W_O_w*q_W_O_x + 2*q_W_O_y*q_W_O_z;
  float R_W_O_33 = 2*q_W_O_w*q_W_O_w + 2*q_W_O_z*q_W_O_z - 1;
  // std::cout << R_W_O_11 << " " << R_W_O_12 << " " << R_W_O_13 << "\n" 
  //   << R_W_O_21 << " " << R_W_O_22 << " " << R_W_O_23 << "\n"
  //   << R_W_O_31 << " " << R_W_O_32 << " " << R_W_O_33 << "\n";
  // Chance constraint helper (delta = 0.05)
  float cc_leftside = (p_x - p_o_x)*(n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))) + (p_y - p_o_y)*(n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (p_z - p_o_z)*(n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) - 1;
  float cc_rightside = 1.163087*sqrt(2)*sqrt((n_o_x*((sb + so)*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o))*(2*n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))*(2*n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + 2*n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o))*(2*n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + 2*n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)))))/2 + (n_o_y*((sb + so)*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o))*(2*n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + 2*n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))) + (sb + so)*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))*(2*n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + 2*n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o))*(2*n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o)))))/2 + (n_o_z*((sb + so)*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))*(2*n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + 2*n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))) + (sb + so)*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))*(2*n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o))*(2*n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + 2*n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o)))))/2);

  float alpha = predicted_inputs_(kAlpha);
  const float alpha_max = 10.0;
  float alpha_frac = 1 - alpha/alpha_max;
  const float factor = 5;
  float cc = factor*alpha_frac + cc_rightside - cc_leftside;

  ROS_INFO_THROTTLE(0.5, "DEBUG: chance constraint = %f", cc);
  // ROS_INFO_THROTTLE(0.5, "DEBUG: d = %f", d);
  // ROS_INFO_THROTTLE(0.5, "DEBUG: theta = %f, r = %f ", theta, radius);
  // ROS_INFO_THROTTLE(0.5, "DEBUG: u1 = %f, v1 = %f, u2 = %f, v2 = %f", u_norm1, v_norm1, u_norm2, v_norm2);

  // ROS_INFO_THROTTLE(0.5, "DEBUG: Sx1 = %f, Sy1 = %f, Sz1 = %f", intSx1, intSy1, intSz1);
  // ROS_INFO_THROTTLE(0.5, "DEBUG: Sx2 = %f, Sy2 = %f, Sz2 = %f", intSx2, intSy2, intSz2);
  // ROS_INFO_THROTTLE(0.5, "DEBUG: p1 = %f, p2 = %f", intSy1/(intSz1 + epsilon), intSy2/(intSz2 + epsilon));
  // ROS_INFO_THROTTLE(0.1, "DEBUG: p_F_x = %f, p_F_y = %f", p_F_x, p_F_y);

  // ~~~~~~~~~~~~~~~~~~~

  // Publish the predicted trajectory.
  publishPrediction(predicted_states_, predicted_inputs_, call_time);
  // Publish the reference trajectory for visualization.
  publishReference(reference_states_, call_time);
  // Publish the projection for debugging and visualization
  // TODO: This can be done better by retrieving the full state x of the acado variables
  // std_msgs::Float32 msg_angle;
  // msg_angle.data = theta;
  // pub_angle_.publish(msg_angle);
  // std_msgs::Float32 msg_radius;
  // msg_radius.data = radius;
  // pub_radius_.publish(msg_radius);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  return updateControlCommand(predicted_states_.col(0),
                              predicted_inputs_.col(0),
                              call_time);
}

template<typename T>
bool MpcController<T>::setStateEstimate(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  est_state_(kPosX) = state_estimate.position.x();
  est_state_(kPosY) = state_estimate.position.y();
  est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kOriW) = state_estimate.orientation.w();
  est_state_(kOriX) = state_estimate.orientation.x();
  est_state_(kOriY) = state_estimate.orientation.y();
  est_state_(kOriZ) = state_estimate.orientation.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();
  est_state_(kDummy1) = 0;
  est_state_(kDummy2) = 0;
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
  return quaternion_norm_ok;
}

template<typename T>
bool MpcController<T>::setReference(
    const quadrotor_common::Trajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (reference_trajectory.points.size() == 1) {
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory.points.front().heading,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    q_orientation = reference_trajectory.points.front().orientation.template cast<T>() * q_heading;
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory.points.front().position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory.points.front().velocity.template cast<T>(), 
        0.0, 0.0
    ).finished().replicate(1, kSamples + 1);

    acceleration << reference_trajectory.points.front().acceleration.template cast<T>() - gravity;
    // TODO: it was here where the bug hiding for so long, make this clearer as it overwrites initialization
    // not set fields are initialized to a potentially non-zero number
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
        reference_trajectory.points.front().bodyrates.template cast<T>(), 0.0, 0.0
    ).finished().replicate(1, kSamples + 1);
  } else {
    auto iterator(reference_trajectory.points.begin());
    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = reference_trajectory.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
      q_orientation = q_heading * iterator->orientation.template cast<T>();
      reference_states_.col(i) << iterator->position.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          iterator->velocity.template cast<T>(),
          0.0, 0.0;
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          est_state_.segment(kOriW, 4)) < 0.0)
        reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
      acceleration << iterator->acceleration.template cast<T>() - gravity;
      reference_inputs_.col(i) << acceleration.norm(),
          iterator->bodyrates.template cast<T>(), 0.0, 0.0;
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));

  ROS_INFO_THROTTLE(0.5,  "alpha = %f, slack = %f", input_bounded(INPUT::kAlpha), input_bounded(INPUT::kSlack));

  quadrotor_common::ControlCommand command;

  command.timestamp = time;
  command.armed = true;
  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  command.expected_execution_time = time;
  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  command.orientation.w() = state(STATE::kOriW);
  command.orientation.x() = state(STATE::kOriX);
  command.orientation.y() = state(STATE::kOriY);
  command.orientation.z() = state(STATE::kOriZ);
  return command;
}

template<typename T>
bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);

  return true;
}

template<typename T>
bool MpcController<T>::publishReference(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> reference_states,
    ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = reference_states(kPosX, i);
    pose.pose.position.y = reference_states(kPosY, i);
    pose.pose.position.z = reference_states(kPosZ, i);
    pose.pose.orientation.w = reference_states(kOriW, i);
    pose.pose.orientation.x = reference_states(kOriX, i);
    pose.pose.orientation.y = reference_states(kOriY, i);
    pose.pose.orientation.z = reference_states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_reference_trajectory_.publish(path_msg);

  return true;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_, 
      params.max_alpha_, params.max_slack_);
  mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class MpcController<float>;

template
class MpcController<double>;

} // namespace rpg_mpc
