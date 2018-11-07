#include "particle_filter.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>

/*
 * Standard mathematical functions to be used within this file
 * normpdf, normpdf3d
 */


// Take the recipe for this formula from these sources:
// https://www.mathworks.com/help/stats/normpdf.html
// https://stackoverflow.com/questions/29397296/c11-normal-distribution-and-matlab-normpdf
const double ONE_OVER_SQRT_2PI = 1.0/sqrt(2*M_PI);
double normpdf(double x, double mu, double sigma)
{
  return (ONE_OVER_SQRT_2PI) * exp(-0.5*pow(x-mu, 2)/pow(sigma, 2) );
}

Vector3f normpdf3d (Vector3d x, Vector3d mu, Vector3d sigma)
{
  Vector3d normVec;
  for (int i = 0; i < 3; i++)
    {
      normVec(i) = normpdf(x(i), mu(i), sigma(i));
    }
  return normVec;
}


ParticleFilter::double sample(double stddev)
{
  // Further optimization - set up vector of nd from R in initialization
  normal_distribution<> nd(stddev,1.0);
  return nd(rng);
}

//Blank constructor
ParticleFilter::ParticleFilter() : ParticleFilter(Matrix3d::Identity(), Matrix3d::Identity(), Matrix3d::Identity(), 0)
{
}

//Constructor/initialization
ParticleFilter::ParticleFilter(Matrix3d A, Matrix3d B, Matrix3d C,
                               int numParticles)
    : motionA(A), motionB(B), measurementC(C) {
  for (int i = 0; i < numParticles; i++) {
    // can i do that
    // im pretty sure i cant do that
    particles.insert(Vector3d::Zero());
  }
}

ParticleFilter::~ParticleFilter()
{
  
}

void ParticleFilter::measurementUpdate(Vector3d measurement)
{
  measurementY = measurement;
}

void ParticleFilter::particleUpdate(Vector3d input) {

  // Set up
  std::vector<Vector3d> tempParticles = particles;
  std::vector<Vector3d> weights;
  std::vector<Vector3d> cumWeight;
  weights.reserve(tempParticles.size());
  cumWeights.reserve(tempParticles.size());

  Vector3d error;

  for (unsigned i : indices(tempParticles))
    {
      // Random disturbance
      error = R.unaryExpr(ptr_fun(sample));
      // Propogate each particle through motion model
      tempParticles[i] = (motionA * (*ptr) + motionB * input) + error;
      // Calculate weighting
      weights.push_back(normpdf3d(measurementY, measurementC * (*ptr), stddev));
      // Calculate cumulative sum
      cumWeights.push_back( i > 0 ? cumWeights[i - 1] + weight[i] : weight[i]);
    }

  // Resample
    for (std::vector<Vector3d>::iterator ptr = particles.begin();
         ptr < particles.end(); ptr++) {
      seed = cumWeights.back() * dis(rng);
      for (unsigned i : indices(weights))
        {
          if ( weights[i] > seed )
            {
              ptr = tempParticles[i];
              break;
            }
        }
    }
}

void ParticleFilter::calculateStats()
{
  for (std::vector<Vector3d>::iterator ptr = particles.begin();
       ptr < particles.end(); ptr)
    {
      particleMean = particleMean + (*ptr);
    }
    particleMean = particleMean.unaryExpr(
        [](double elem) { return elem / double(particles.size()); });
}


// Accessors

Matrix3d getMotionA() { return motionA; }
Matrix3d getMotionB() { return motionB; }
Vector3d getMean() {return particleMean; }
