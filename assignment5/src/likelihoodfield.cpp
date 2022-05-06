/**
   likelihood_field
   Implement the likelihood field model for a laser range finder. The function
   returns the probability P( z | s, m ), that is the probability that a 
   measurement z was caused by a robot in a state s and in a map m.
   \input Scan z: A vector of laser beams. Each beam has two members: range and 
                  bearing.
   \input State s: The state of the robot with respect to the world frame.
                   The x coordinate is given by s[0], the y coordinate by s[1]
		   and the angle theta by s[2].
   \input Map* m:  The map of the environment the robot.
   \input Pose laser_pose: The position and orientation of the laser with 
                           respect to the robot's coordinate frame.
   \input sigma_hit: The standard variation of errors
   \input z_max: The laser maximum range
   \input w_hit: The coefficient of measurements
   \input w_rand: The coefficient of random errors
*/

#include <math.h>       /* pow */
# define M_PI           3.14159265358979323846  /* pi */

double likelihood_field( Scan& z,
			 State s, 
			 Map* m,
			 Pose laser_pose, 
			 double sigma_hit, 
			 double z_max, 
			 double w_hit, 
			 double w_rand ){

   double p=0;

   double x_lf[2]; // beam in laser scanner frame
   double x_rf[2]; // beam in robot frame
   double x[2]; // beam in world frame

   double d = 0;
   double q_hit;
   double q = 1;


  for(int i=0; i<z.size(); i++) {
     if(z[i].range > 0 &&  z[i].range < z_max) {
  /*      x_lf[0] = z[i].range * cos(z[i].bearing);
        x_lf[1] = z[i].range * sin(z[i].bearing);
        
        x_rf[0] = laser_pose[0] + x_lf[0]*cos(laser_pose[2]) - x_lf[1]*sin(laser_pose[2]);
        x_rf[1] = laser_pose[1] + x_lf[0]*cos(laser_pose[2]) + x_lf[1]*sin(laser_pose[2]);

        x[0] = s[0] + x_rf[0]*cos(s[2]) - x_rf[1]*sin(s[2]);
        x[1] = s[1] + x_rf[0]*cos(s[2]) + x_rf[1]*sin(s[2]);
*/
	x[0] = s[0] + laser_pose[0]*cos(s[2]) - laser_pose[1]*sin(s[2])  + z[i].range*cos(z[i].bearing)*cos(s[2] + laser_pose[2]);
	x[1] = s[1] + laser_pose[1]*cos(s[2]) + laser_pose[0]*sin(s[2]) + z[i].range*sin(z[i].bearing)*sin(s[2] + laser_pose[2]);

        d = GetDistanceFromMap(m, x[0], x[1]);

        q_hit = exp(-pow(d,2)/(2*pow(sigma_hit,2))) / (sigma_hit*sqrt(2*M_PI));
        q = q * (  w_hit*q_hit +  w_rand/z_max );

     }
  }

  return q;

}

