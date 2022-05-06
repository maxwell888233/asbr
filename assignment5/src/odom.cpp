#include <math.h>
/**
   TODO
   Implement a sample motion odometry
   \input sample: The pose of a sample. sample[0] is the x coordinate
   sample[1] is the y coordinate and sample[2] is the angle.
   \input delta: The pose increment to be adde to the pose
   \input alpha1: 
   \input alpha2: 
   \input alpha3: 
   \input alpha4: 
 */
void odometry_model( double sample[3],
		     double delta[3],
		     double alpha1,
		     double alpha2,
		     double alpha3,
		     double alpha4 )
{
  //double x = sample[0] + delta[0];
  //double y = sample[1] + delta[1];
  
  

  double x = delta[0];
  double  y = delta[1];
  double d = sqrt(x*x + y*y);

  double a = atan2(delta[1], delta[0]) - sample[2];
  double b = delta[2] - a;
/*
  if(a > M_PI) {
      a = M_PI;
  }
  else if(a < - M_PI) {
    a = -M_PI;
  }

  if(b > M_PI) {
      b = M_PI;
  }
  else if(b < - M_PI) {
    b = -M_PI;
  }
*/


  double gauss_a1 = pf_ran_gaussian(alpha1), gauss_a2 = pf_ran_gaussian(alpha2);
  double gauss_d1 = pf_ran_gaussian(alpha3), gauss_d2 = pf_ran_gaussian(alpha4);
/*
  if(gauss_a1 > M_PI) {
      gauss_a1 = M_PI;
  }
  else if(gauss_a1 < - M_PI) {
    gauss_a1 = -M_PI;
  }


  if(gauss_a2 > M_PI) {
      gauss_a2 = M_PI;
  }
  else if(gauss_a2 < - M_PI) {
    gauss_a2 = -M_PI;
  }


  if(gauss_b1 > M_PI) {
    gauss_b1 = M_PI;
  }
  else if(gauss_b1 < - M_PI) {
    gauss_b1 = -M_PI;
  }
  if(gauss_b2 > M_PI) {
      gauss_b2 = M_PI;
  }
  else if(gauss_b2 < - M_PI) {
    gauss_b2 = -M_PI;
  }
*/

  double a_prime = a + a*gauss_a1 + d*gauss_a2;
  double b_prime = b + b*gauss_a1 + d*gauss_a2;
  //double b_prime = b + b*pf_ran_gaussian(alpha1) + d*pf_ran_gaussian(alpha2);
  double d_prime = d + d*gauss_d1 + (a+b)*gauss_d2;

  sample[0] = sample[0] + d_prime * cos(sample[2] + a_prime);
  sample[1] = sample[1] + d_prime * sin(sample[2] + a_prime);
  sample[2] = sample[2] + a_prime + b_prime;
/*
  while(sample[0] > M_PI) {
    sample[0] = sample[0] - 2*M_PI;
  }
  while(sample[0] < -M_PI) {
    sample[0] = sample[0] + 2*M_PI;
  }

  while(sample[1] > M_PI) {
    sample[1] = sample[1] - 2*M_PI;
  }
  while(sample[1] < -M_PI) {
    sample[1] = sample[1] + 2*M_PI;
  }
*/

}
