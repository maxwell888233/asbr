
#include "ekf_models.hpp"
#include <tf/tf.h>
#include "utilities.h"
#include <eigen3/Eigen/Dense>

#include <cmath>
using namespace Eigen;

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of WMWt are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out] WMWt Covariance matrix of the system.
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_WMWt( double WMWt[6][6], const State& state, double v, double w, double dt ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      WMWt[r][c] = 0.0;

  // TODO fill in the matrix WMWt

  // along the main diagonal
  /*WMWt[0][0] = 10.0;
  WMWt[1][1] = 10.0;
  WMWt[2][2] = 0.01;
  WMWt[3][3] = 0.01;
  WMWt[4][4] = 0.01;
  WMWt[5][5] = 0.01;
  
  // off-diagonal elements
  //WMWt[6][6] = (double) ;
  WMWt[0][4] = .001;
  WMWt[0][5] = .001;
  WMWt[1][5] = .001;

  WMWt[4][0] = .001;
  WMWt[5][0] = .001;
  WMWt[5][1] = .001;
*/
  WMWt[0][0] = 0.0001 * (dt*dt *v*v);
  WMWt[5][5] = 0.0001 * (dt*dt *w*w);
  }

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R( double R[6][6], const State& state ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      R[r][c] = 0.0;

  // TODO fill in the matrix R
  R[0][0] = 0.1;
  R[1][1] = 0.1;
  R[2][2] = 0.01;
  R[3][3] = 0.001;
  R[4][4] = 0.001;
  R[5][5] = 0.001;
}


/**
   TODO
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the 
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
State sys_evaluate_g( const State& state_in, double v, double w, double dt ){

  State state_out;

  // TODO Given state_in and v and w and dt (time increment) determine the prior
  // estimate state_out
  
  // same as in lecture, just in 3D
  
  double roll = state_in.x[State::ROT_R];
  double pitch = state_in.x[State::ROT_P];
  double yaw = state_in.x[State::ROT_Y];
  /*tf::Matrix3x3 R_t; // rotation before step
  R_t.setRPY(roll,pitch,yaw); //should this be yaw, pitch, roll ?? */

  
  Eigen::AngleAxisd R(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd P(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Y(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<double> q = R * P * Y;
  Eigen::Matrix3d R_t = q.matrix();

  Eigen::Matrix3d R_step;
  R_step << cos(dt*w), -1.0*sin(dt*w), 0, sin(dt*w), cos(dt*w), 0, 0,0,1 ;
  
  
  //R_step.setRPY(0,0,dt*w);
  //now compute rotation after step
  Eigen::Matrix3d R_t1 = R_t * R_step;
  
  
  // POSITION
  double x = state_in.x[State::POS_X];
  double y = state_in.x[State::POS_Y];
  double z = state_in.x[State::POS_Z];
  Eigen::Vector3d translation;
  translation << x,y,z;
  Eigen::Vector3d step;
  step << dt*v,0,0;
  //now compute position after step
  Eigen::Vector3d translation_1 = (R_t * step) + translation;

  // Finally, we have the rotation matrix and translation vector after the step. Extract x,y,z,R,P,Y
  //R_t1.getRPY(state_out.x[State::ROT_R], state_out.x[State::ROT_P], state_out.x[State::ROT_Y]);
  Eigen::Vector3d euler_rpy = R_t1.eulerAngles(0,1,2);

  state_out.x[State::POS_X] = translation_1(0);
  state_out.x[State::POS_Y] = translation_1(1);
  state_out.x[State::POS_Z] = translation_1(2);
  state_out.x[State::ROT_R] = euler_rpy(0);
  state_out.x[State::ROT_P] = euler_rpy(1);
  state_out.x[State::ROT_Y] = euler_rpy(2);
  
  return state_out;
}

/**
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see 
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param state       The state of the robot
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_G( double G[6][6], const State& state, double v, double w, double dt ){
  
  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      G[r][c] = 0.0;
  
  // TODO
  // Given state, v and w, compute the system Jacobian G
  
  double R = state.x[State::ROT_R];
  double P = state.x[State::ROT_P];
  double Y = state.x[State::ROT_Y];
  /*
  Eigen::AngleAxisd r(R, Eigen::VEctor3d::UnitX());
  Eigen::AngleAxisd p(P, Eigen::VEctor3d::UnitY());
  Eigen::AngleAxisd y(Y, Eigen::VEctor3d::UnitZ());

  Eigen::Quaternion<double> q = r * p * y;
  Eigen::Matrix3d R_t = q.matrix();*/

  /*
  tf::Matrix3x3 R_t; // rotation before step
  R_t.setRPY(roll,pitch,yaw);
  tf::Matrix3x3 R_step;
  R_step.setRPY(0,0,dt*w);
  //now compute rotation after step
  tf::Matrix3x3 R_t1 = R_t * R_step;
  // now get rot fo
  */


  G[0][0] = 1.0;
  G[0][4] = -dt*v*cos(Y)*sin(P);
  G[0][5] = -dt*v*cos(P)*sin(Y);
  G[1][1] = 1.0;
  G[1][3] = -dt*v*(sin(R)*sin(Y)-cos(R)*cos(Y)*sin(P));
  G[1][4] = dt*v*cos(P)*cos(Y)*sin(R);
  G[1][5] = dt*v*(cos(R)*cos(Y)-sin(P)*sin(R)*sin(Y));
  G[2][2] = 1.0;
  G[2][3] = dt*v*(cos(R)*sin(Y)+cos(Y)*sin(P)*sin(R));
  G[2][4] = -dt*v*cos(P)*cos(R)*cos(Y);
  G[2][5] = dt*v*(cos(Y)*sin(R)+cos(R)*sin(P)*sin(Y));
  G[3][3] = 1.0;
  G[4][4] = 1.0;
  G[5][5] = 1.0;
}

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state The state estimate
   \return      A satellite navigation fix (only the latitute, longitude
                and altitude members are used)
*/
sensor_msgs::NavSatFix meas_evaluate_gps( const State& state ){

  sensor_msgs::NavSatFix nsf;

  // TODO
  // Given prior estimate state, determine the expected GPS measurement nsf


  nsf.latitude = state.x[State::POS_Z] + 35.8597;
  nsf.longitude = state.x[State::POS_Z] - 108.2366;
  nsf.altitude = 0.9937*state.x[State::POS_Z] + 63.9549;
  
  return nsf;
}

/**
   TODO
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the 
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::RPY meas_evaluate_imu( const State& state ){
  sensor_msgs::RPY rpy;

  // TODO
  // Given the prior estimate state, determine the expected RPY measurement rpy  

  rpy.roll = state.x[State::ROT_R];
  rpy.pitch = state.x[State::ROT_P];
  rpy.yaw = state.x[State::ROT_Y];
  
  return rpy;
}

/** 
    TODO
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps( double Hgps[3][3], const State& state ){

  // TODO
  // Fill the Jacobian matrix Hgps of the GPS observations

  Hgps[0][0] = 1.0;
  Hgps[0][1] = 0.0;
  Hgps[0][2] = 0.0;

  Hgps[0][0] = 0.0;
  Hgps[1][1] = -1.0;
  Hgps[0][0] = 0.0;

  Hgps[0][0] = 0.0;
  Hgps[0][0] = 0.0;
  Hgps[2][2] = 1.0 * 0.9937;

}

/** 
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu( double Himu[3][3], const State& state ){

  // TODO
  // Fill the Jacobian matrix Himu of the IMU observations
  
  // 3x3 identity matrix
  for( int r=0; r<3; r++ )
    for( int c=0; c<3; c++ )
      Himu[r][c] = 0.0;

  Himu[0][0] = 1.0;
  Himu[1][1] = 1.0;
  Himu[2][2] = 1.0;
}

