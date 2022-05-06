/**
   EN.601.463/663
   Assignment #1 

   Cartesian trajectory generation
   
 */
#include "assignment1.hpp"
#include <Eigen/Dense>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace Eigen;

// Compute the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematics( double q1, double q2, double q3, double E[4][4] ){

  // TODO
  // Fill the values of the forward kinematics (homogeneous matrix E)
    double l1 = 0.089159;
    double l2 = 0.135850;
    double l3 = 0.1197;
    double l4 = 0.425;
    double l5 = 0.212543;
    double l6 = 0.092989;
    double l7 = 0.485779;
	
	for(int i=0; i<4; i++) {
		for(int j=0; j<4; j++) {
			E[i][j] = 0.0;
		}
	}
    E[0][0] = sin(q1);
    E[0][1] = cos(q1)*cos(q3)*sin(q2+M_PI/2.0)+cos(q1)*cos(q2+M_PI/2.0)*sin(q3);
    E[0][2] = cos(q1)*sin(q3)*sin(q2+M_PI/2.0)-cos(q1)*cos(q3)*cos(q2+M_PI/2.0);
    E[0][3] = l5*(cos(q1)*sin(q3)*sin(q2+M_PI/2.0)-cos(q1)*cos(q3)*cos(q2+M_PI/2.0))+l7*(cos(q1)*cos(q3)*sin(q2+M_PI/2.0)+cos(q1)*cos(q2+M_PI/2.0)*sin(q3))-l2*sin(q1)+l3*sin(q1)-l6*sin(q1)+l4*cos(q1)*sin(q2+M_PI/2.0);
    E[1][0] = -cos(q1);
    E[1][1] = cos(q3)*sin(q1)*sin(q2+M_PI/2.0)+cos(q2+M_PI/2.0)*sin(q1)*sin(q3);
    E[1][2] = sin(q1)*sin(q3)*sin(q2+M_PI/2.0)-cos(q3)*cos(q2+M_PI/2.0)*sin(q1);
    E[1][3] = l5*(sin(q1)*sin(q3)*sin(q2+M_PI/2.0)-cos(q3)*cos(q2+M_PI/2.0)*sin(q1))+l7*(cos(q3)*sin(q1)*sin(q2+M_PI/2.0)+cos(q2+M_PI/2.0)*sin(q1)*sin(q3))+l2*cos(q1)-l3*cos(q1)+l6*cos(q1)+l4*sin(q1)*sin(q2+M_PI/2.0);
    E[2][1] = cos(q3)*cos(q2+M_PI/2.0)-sin(q3)*sin(q2+M_PI/2.0);
    E[2][2] = cos(q3)*sin(q2+M_PI/2.0)+cos(q2+M_PI/2.0)*sin(q3);
    E[2][3] = l1+l5*(cos(q3)*sin(q2+M_PI/2.0)+cos(q2+M_PI/2.0)*sin(q3))+l7*(cos(q3)*cos(q2+M_PI/2.0)-sin(q3)*sin(q2+M_PI/2.0))+l4*cos(q2+M_PI/2.0);
    E[3][3] = 1.0;
	
}

// Compute the inverse of the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematicsInverse( double q1, double q2, double q3, double E[4][4] ){

  // TODO
  // Fill the values of the inverse of the forward kinematics (homogeneous matrix E)
    double l1 = 0.089159;
    double l2 = 0.135850;
    double l3 = 0.1197;
    double l4 = 0.425;
    double l5 = 0.212543;
    double l6 = 0.092989;
    double l7 = 0.485779;

	for(int i=0; i<4; i++) {
		for(int j=0; j<4; j++) {
			E[i][j] = 0.0;
		}
	}

	E[0][0] = sin(q1);
    E[0][1] = -cos(q1);
    E[0][3] = l2-l3+l6;
    E[1][0] = cos(q2+q3)*cos(q1);
    E[1][1] = cos(q2+q3)*sin(q1);
    E[1][2] = -sin(q2+q3);
    E[1][3] = -l7+l1*sin(q2+q3)-l4*cos(q3);
    E[2][0] = sin(q2+q3)*cos(q1);
    E[2][1] = sin(q2+q3)*sin(q1);
    E[2][2] = cos(q2+q3);
    E[2][3] = -l5-l1*cos(q2+q3)-l4*sin(q3);
    E[3][3] = 1.0;

}

// Compute the Adjoint transformation inverse matrix
// input E: the rotation/translation between the base and the hand frame
//          (as computed by the forward kinematics)
// output Ad: the 6x6 adjoint transformation inverse matrix
void AdjointTransformationInverse( double E[4][4], double Ad[6][6] ){

  // TODO
  // Compute the Adjoint Transformation Inverse A^-1
	Eigen::Matrix3d rot;
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++) {
			rot(i,j) =  E[i][j];
		}
	}
	Eigen::Matrix3d t_skew;
    t_skew << 0.0, -E[2][3], E[1][3],
			 E[2][3], 0.0, -E[0][3],
			-E[1][3], E[0][3], 0.0 ;
	
	Eigen::Matrix3d zerosfiller;
	zerosfiller << 0,0,0, 0,0,0, 0,0,0;
	
	Eigen::MatrixXd Ad_inv(6,6);
	
	Ad_inv.topLeftCorner(3,3) = rot;
	Ad_inv.topRightCorner(3,3) = t_skew*rot;
	Ad_inv.bottomLeftCorner(3,3) = zerosfiller;
	Ad_inv.bottomRightCorner(3,3) = rot;
	Ad_inv = Ad_inv.inverse();
	
	// finally, assign element-wise
	for(int i=0; i<6; i++) {
		for(int j=0; j<6; j++) {
			Ad[i][j] = Ad_inv(i,j);
		}
	}


}

// Compute and return the Jacobian of the robot given the current joint 
// positions
// input: the joints angles
// output: the 6x3 Jacobian (position only)
void Jacobian( double q1, double q2, double q3, double J[6][3] ){
  
  // TODO
  // Fill the values of the Jacobian matrix J

	double l1 = 0.089159;
	double l2 = 0.135850;
	double l3 = 0.1197;
	double l4 = 0.425;
	double l5 = 0.212543;
	double l6 = 0.092989;
	double l7 = 0.485779;

	for(int i=0; i < 6; i++) {
		for(int j=0; j < 3; j++) {
            J[i][j] = 0.0;
        }
    }
		
    J[0][1] = -l1*cos(q1);
    J[0][2] = -cos(q1)*(l1-l4*sin(q2));
    J[1][1] = -l1*sin(q1);
    J[1][2] = -sin(q1)*(l1-l4*sin(q2));
    J[2][2] = l4*cos(q2);
    J[3][1] = -sin(q1);
    J[3][2] = -sin(q1);
    J[4][1] = cos(q1);
    J[4][2] = cos(q1);
    J[5][0] = 1.0;
	
}
