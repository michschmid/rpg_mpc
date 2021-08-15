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


#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = true;

  // System variables
  DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  DifferentialState     dummy_1, dummy_2; // workaround for https://github.com/acado/acado/issues/79
  Control               T, w_x, w_y, w_z;
  Control               alpha, slack;
  DifferentialEquation  f;
  Function              h, hN;
  OnlineData            p_F1_x, p_F1_y, p_F1_z, p_F2_x, p_F2_y, p_F2_z;
  OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
  OnlineData            p_o_x, p_o_y, p_o_z, a_o, b_o, c_o;
  OnlineData            q_W_O_w, q_W_O_x, q_W_O_y, q_W_O_z;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double T_min = 2;         // Minimal thrust [N]
  const double T_max = 20;        // Maximal thrust [N]
  const double alpha_max = 10;    // Maximum value for slack variable alpha [-]

  // Bias to prevent division by zero.
  const double epsilon1 = 0.1;     // Camera projection recover bias [m]
  const double epsilon2 = 0.001;   // Cartesian to polar conversion bias [m]
  const double epsilon3 = 0.001;   // Bias for sqrt


  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * T;
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * T;
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T - g_z;
  f << dot(dummy_1) == 0.00001 * alpha;
  f << dot(dummy_2) == 0.00001 * slack;

  // Optimization variable to trade-off between perception awareness and obstacle avoidance
  IntermediateState alpha_frac = 1 - alpha/alpha_max;

  // Intermediate states to calculate point of interest projection!
  // IMPORTANT: This assumes the camera coordinate system to be oriented as in the paper (optical axis z, y down),
  // therefore check that the provided q_BC is correct
  IntermediateState intSx1 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F1_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F1_z-p_z));
  IntermediateState intSy1 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w))*(p_F1_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F1_x-p_x));
  IntermediateState intSz1 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F1_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w))*(p_F1_y-p_y));

  IntermediateState intSx2 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F2_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F2_z-p_z));
  IntermediateState intSy2 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w))*(p_F2_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F2_x-p_x));
  IntermediateState intSz2 = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F2_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w))*(p_F2_y-p_y));

  // normalized image coordinates
  IntermediateState u_norm1 = intSx1/(intSz1 + epsilon1);
  IntermediateState v_norm1 = intSy1/(intSz1 + epsilon1);
  IntermediateState u_norm2 = intSx2/(intSz2 + epsilon1);
  IntermediateState v_norm2 = intSy2/(intSz2 + epsilon1);

  // Calculate polar representation
  IntermediateState theta, radius;
  theta = alpha_frac * atan(-(u_norm2 - u_norm1) / (v_norm2 - v_norm1 + epsilon2));

  //TODO:
  radius = alpha_frac * (v_norm1 - (v_norm2 - v_norm1) / (u_norm2 - u_norm1  + epsilon2) * u_norm1) * sin(atan(-(u_norm2 - u_norm1) / (v_norm2 - v_norm1  + epsilon2)));
  //radius = 1.0 * (v_norm1 - (v_norm2 - v_norm1) / (u_norm2 - u_norm1  + epsilon2) * u_norm1) * sin(atan(-(u_norm2 - u_norm1) / (v_norm2 - v_norm1  + epsilon2)));

  //TODO:
  // Distance from quadrotors position to powerline
  IntermediateState d_l = sqrt(alpha_frac*((p_x - p_F1_x)*(p_y - p_F2_y) - (p_y - p_F1_y)*(p_x - p_F2_x))*((p_x - p_F1_x)*(p_y - p_F2_y) - (p_y - p_F1_y)*(p_x - p_F2_x)) + ((p_x - p_F1_x)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_x - p_F2_x))*((p_x - p_F1_x)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_x - p_F2_x)) + ((p_y - p_F1_y)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_y - p_F2_y))*((p_y - p_F1_y)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_y - p_F2_y))+epsilon3)/sqrt((p_F1_x - p_F2_x)*(p_F1_x - p_F2_x) + (p_F1_y - p_F2_y)*(p_F1_y - p_F2_y) + (p_F1_z - p_F2_z)*(p_F1_z - p_F2_z) + epsilon3);
  //IntermediateState d_l = alpha_frac*sqrt(((p_x - p_F1_x)*(p_y - p_F2_y) - (p_y - p_F1_y)*(p_x - p_F2_x))*((p_x - p_F1_x)*(p_y - p_F2_y) - (p_y - p_F1_y)*(p_x - p_F2_x)) + ((p_x - p_F1_x)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_x - p_F2_x))*((p_x - p_F1_x)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_x - p_F2_x)) + ((p_y - p_F1_y)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_y - p_F2_y))*((p_y - p_F1_y)*(p_z - p_F2_z) - (p_z - p_F1_z)*(p_y - p_F2_y))+epsilon3)/sqrt((p_F1_x - p_F2_x)*(p_F1_x - p_F2_x) + (p_F1_y - p_F2_y)*(p_F1_y - p_F2_y) + (p_F1_z - p_F2_z)*(p_F1_z - p_F2_z) + epsilon3);

  //TODO:
  // Difference in yaw angle and line bearing
  IntermediateState yaw = atan(2*(q_w*q_z + q_x*q_y)/(1 - 2*(q_y*q_y + q_z*q_z)));
  IntermediateState line_bearing = atan((p_F1_y - p_F2_y)/(p_F1_x - p_F2_x));
  IntermediateState diff_angle = (1-alpha_frac)*(line_bearing - yaw + 3.141592/2.0);

  // Quadrotor radius
  const double r_o = 0.4;
  // Fix covariance
  const double so = 0.001;
  const double sb = 0.001;
  // Logistic cost on distance from quadrotor to obstacle
  // Sqrt around the logistic function to fit in quadratic term 
  // minimizeLSQLinearTerms() would be an alternative
  const double r_o_tune = 1;
  const double lambda_o = 1;
  // IntermediateState d_o_k = sqrt((p_x - p_o_x)*(p_x - p_o_x) + (p_y - p_o_y)*(p_y - p_o_y) + (p_z - p_o_z)*(p_z - p_o_z) + epsilon3);
  IntermediateState d_o_k = sqrt((p_x - p_o_x)*(p_x - p_o_x) + (p_y - p_o_y)*(p_y - p_o_y) + epsilon3);
  IntermediateState d_o_log_sqrt = sqrt(1 / (1 + exp(lambda_o * (d_o_k - r_o_tune))) + epsilon3);

  // Intermediate state for constraints
  IntermediateState dp_norm = sqrt((p_x - p_o_x)*(p_x - p_o_x) + (p_y - p_o_y)*(p_y - p_o_y) + (p_z - p_o_z)*(p_z - p_o_z) + epsilon3);
  IntermediateState n_o_x = (p_x - p_o_x) / dp_norm;
  IntermediateState n_o_y = (p_y - p_o_y) / dp_norm;
  IntermediateState n_o_z = (p_z - p_o_z) / dp_norm;
  // Quaternion to rotation matrix
  IntermediateState R_W_O_11 = 2*q_W_O_w*q_W_O_w + 2*q_W_O_x*q_W_O_x - 1;
  IntermediateState R_W_O_12 = 2*q_W_O_x*q_W_O_y - 2*q_W_O_w*q_W_O_z;
  IntermediateState R_W_O_13 = 2*q_W_O_w*q_W_O_y + 2*q_W_O_x*q_W_O_z;
  IntermediateState R_W_O_21 = 2*q_W_O_w*q_W_O_z + 2*q_W_O_x*q_W_O_y;
  IntermediateState R_W_O_22 = 2*q_W_O_w*q_W_O_w + 2*q_W_O_y*q_W_O_y - 1;
  IntermediateState R_W_O_23 = 2*q_W_O_y*q_W_O_z - 2*q_W_O_w*q_W_O_x;
  IntermediateState R_W_O_31 = 2*q_W_O_x*q_W_O_z - 2*q_W_O_w*q_W_O_y;
  IntermediateState R_W_O_32 = 2*q_W_O_w*q_W_O_x + 2*q_W_O_y*q_W_O_z;
  IntermediateState R_W_O_33 = 2*q_W_O_w*q_W_O_w + 2*q_W_O_z*q_W_O_z - 1;
  // Chance constraint helpers (delta = 0.01)
  IntermediateState cc_leftside = (p_x - p_o_x)*(n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))) + (p_y - p_o_y)*(n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (p_z - p_o_z)*(n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) - 1;
  IntermediateState cc_rightside = 1.644976*sqrt(2)*sqrt((n_o_x*((sb + so)*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o))*(2*n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))*(2*n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + 2*n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o))*(2*n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + 2*n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)))))/2 + (n_o_y*((sb + so)*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o))*(2*n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + 2*n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))) + (sb + so)*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))*(2*n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + 2*n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o))*(2*n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o)))))/2 + (n_o_z*((sb + so)*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))*(2*n_o_x*(R_W_O_11*R_W_O_11/(a_o + r_o) + R_W_O_21*R_W_O_21/(b_o + r_o) + R_W_O_31*R_W_O_31/(c_o + r_o)) + 2*n_o_y*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o))) + (sb + so)*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))*(2*n_o_y*(R_W_O_12*R_W_O_12/(a_o + r_o) + R_W_O_22*R_W_O_22/(b_o + r_o) + R_W_O_32*R_W_O_32/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_12)/(a_o + r_o) + (R_W_O_21*R_W_O_22)/(b_o + r_o) + (R_W_O_31*R_W_O_32)/(c_o + r_o)) + 2*n_o_z*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o))) + (sb + so)*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o))*(2*n_o_z*(R_W_O_13*R_W_O_13/(a_o + r_o) + R_W_O_23*R_W_O_23/(b_o + r_o) + R_W_O_33*R_W_O_33/(c_o + r_o)) + 2*n_o_x*((R_W_O_11*R_W_O_13)/(a_o + r_o) + (R_W_O_21*R_W_O_23)/(b_o + r_o) + (R_W_O_31*R_W_O_33)/(c_o + r_o)) + 2*n_o_y*((R_W_O_12*R_W_O_13)/(a_o + r_o) + (R_W_O_22*R_W_O_23)/(b_o + r_o) + (R_W_O_32*R_W_O_33)/(c_o + r_o)))))/2);
  const double factor = 4;

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << dummy_1 << dummy_2
    << theta << radius << d_l << d_o_log_sqrt << diff_angle
    << T << w_x << w_y << w_z << alpha << slack;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 100;   // x
  Q(1,1) = 100;   // y
  Q(2,2) = 100;   // z
  Q(3,3) = 100;   // qw
  Q(4,4) = 100;   // qx
  Q(5,5) = 100;   // qy
  Q(6,6) = 100;   // qz
  Q(7,7) = 10;    // vx
  Q(8,8) = 10;    // vy
  Q(9,9) = 10;    // vz
  Q(10,10) = 0;   // dummy 1
  Q(11,11) = 0;   // dummy 2
  Q(12,12) = 0;   // Cost on perception
  Q(13,13) = 0;   // Cost on perception
  Q(14,14) = 0;   // Cost on distance to line
  Q(15,15) = 0;   // Cost on distance to obstacle
  Q(16,16) = 10;  // Cost on difference of quad heading and line bearing
  Q(17,17) = 1;   // T, thrust
  Q(18,18) = 1;   // wx, pitch 
  Q(19,19) = 1;   // wy, roll
  Q(20,20) = 1;   // wz, yaw
  Q(21,21) = 1;   // alpha
  Q(22,22) = 1;   // slack
  

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = Q(0,0);   // x
  QN(1,1) = Q(1,1);   // y
  QN(2,2) = Q(2,2);   // z
  QN(3,3) = Q(3,3);   // qw
  QN(4,4) = Q(4,4);   // qx
  QN(5,5) = Q(5,5);   // qy
  QN(6,6) = Q(6,6);   // qz
  QN(7,7) = Q(7,7);   // vx
  QN(8,8) = Q(8,8);   // vy
  QN(9,9) = Q(9,9);   // vz

  // TODO:
  // Linear cost for slack variable alpha to ensure slack variable equal to zero when possible
  // Weight vector for states, actual weights are reset at run time
  DVector Slx(f.getDim());
  Slx.setZero();

  // Weight vector for inputs
  DVector Slu(6);
  Slu.setZero();
  // Just include linear cost for slack variable alpha
  Slu(4) = 15;


  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(0) = 2.0;
  r(3) = 1.0;
  r(10) = g_z;

  DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(3) = r(3);


  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );
  if(!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );
  }else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
    // Add linear cost terms
    ocp.minimizeLSQLinearTerms(Slx, Slu);
  }

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo( T_min <= T <= T_max);
  ocp.subjectTo( 0.0 <= alpha <= alpha_max);
  ocp.subjectTo( 0.0 <= slack);
  // Obstacle chance constraint 
  // ocp.subjectTo(factor*alpha_frac + cc_rightside - cc_leftside - slack <= 0);
  ocp.subjectTo(factor*alpha_frac + cc_rightside - cc_leftside <= 0);

  ocp.setNOD(23);


  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, p_x ==  0.0 );
    ocp.subjectTo( AT_START, p_y ==  0.0 );
    ocp.subjectTo( AT_START, p_z ==  0.0 );
    ocp.subjectTo( AT_START, q_w ==  1.0 );
    ocp.subjectTo( AT_START, q_x ==  0.0 );
    ocp.subjectTo( AT_START, q_y ==  0.0 );
    ocp.subjectTo( AT_START, q_z ==  0.0 );
    ocp.subjectTo( AT_START, v_x ==  0.0 );
    ocp.subjectTo( AT_START, v_y ==  0.0 );
    ocp.subjectTo( AT_START, v_z ==  0.0 );
    ocp.subjectTo( AT_START, w_x ==  0.0 );
    ocp.subjectTo( AT_START, w_y ==  0.0 );
    ocp.subjectTo( AT_START, w_z ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( p_x,"position x" );
    window1.addSubplot( p_y,"position y" );
    window1.addSubplot( p_z,"position z" );
    window1.addSubplot( v_x,"verlocity x" );
    window1.addSubplot( v_y,"verlocity y" );
    window1.addSubplot( v_z,"verlocity z" );

    GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
    window3.addSubplot( w_x,"rotation-acc x" );
    window3.addSubplot( w_y,"rotation-acc y" );
    window3.addSubplot( w_z,"rotation-acc z" ); 
    window3.addSubplot( T,"Thrust" );


    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );
    algorithm << window1;
    algorithm << window3;
    algorithm.solve();

  }else{
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,   N);
    mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,            YES);
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}