#include "particle_filter.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>


// Take the recipe for this formula from these sources:
// https://www.mathworks.com/help/stats/normpdf.html
// https://stackoverflow.com/questions/29397296/c11-normal-distribution-and-matlab-normpdf
const double ONE_OVER_SQRT_2PI = 1.0/sqrt(2*M_PI);
double normpdf(double x, double mu, double sigma)
{
  return (ONE_OVER_SQRT_2PI) * exp(-0.5*pow(x-mu, 2)/pow(sigma, 2) );
}

Vector3f normpdf3f (Vector3f x, Vector3f mu, Vector3f sigma)
{
  Vector3f normVec;
  for (int i = 0; i < 3; i++)
    {
      normVec(i) = normpdf(x(i), mu(i), sigma(i));
    }
  return normVec;
}

//Blank constructor
ParticleFilter::ParticleFilter()
{
  
}

//Constructor/initialization
ParticleFilter::ParticleFilter(Matrix3f A, Matrix3f B, )
{
}

ParticleFilter::~ParticleFilter()
{
  
}

void ParticleFilter::particleUpdate(Vector3f input) {
  Vector3f error;
  std::vector<Vector3f>::iterator ptr;
  std::vector<Vector3f>::iterator weight_ptr = weights.begin();
  for (ptr = particles.begin(); ptr < particles.end(); ptr++)
    {
      // Random disturbance
      error = randomError();
      // Propogate each particle through motion model
      *ptr = (motionA * (*ptr) + motionB * input) + error;
      //
      *weight_ptr = normpdf3f(y, measurementC * (*ptr), stddev);
      weight_ptr++;
    }
  for (weight_ptr = weights.begin(); ptr < weights.end(); weight_ptr++)
    {
      cumWeight = cumWeight + (*weight_ptr);
    }
}

Vector3f ParticleFilter::randomError() {
  
}
