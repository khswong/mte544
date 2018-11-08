#include "particle_filter.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

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

Eigen::Vector3d normpdf3d (Eigen::Vector3d x, Eigen::Vector3d mu, Eigen::Vector3d sigma)
{
  Eigen::Vector3d normVec;
  for (int i = 0; i < 3; i++)
    {
      normVec(i) = normpdf(x(i), mu(i), sigma(i));
    }
  return normVec;
}


double ParticleFilter::sample(double stddev)
{
  // Further optimization - set up vector of nd from R in initialization
  std::normal_distribution<> nd(stddev,1.0);
  return nd(rng);
}

//Blank constructor
ParticleFilter::ParticleFilter() : ParticleFilter(0)
{
}

ParticleFilter::ParticleFilter(int numParticles)
    : ParticleFilter(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
                     Eigen::Matrix3d::Identity(), numParticles) {}
//Constructor/initialization
ParticleFilter::ParticleFilter(Eigen::Matrix3d A, Eigen::Matrix3d B, Eigen::Matrix3d C,
                               int numParticles)
    : motionA(A), motionB(B), measurementC(C) {
  // Initialize random number generators
  dis = std::uniform_real_distribution<double>(0.0, 1.0);
  ROS_INFO("CONSTRUCOTR");
  for (int i = 0; i < numParticles; i++) {
    // can i do that
    // im pretty sure i cant do that
    Eigen::Vector3d particle = Eigen::Vector3d::Zero();
    particles.push_back(particle);
  }
}

void ParticleFilter::measurementUpdate(Eigen::Vector3d measurement)
{
  measurementY = measurement;
}

void ParticleFilter::particleUpdate(Eigen::Vector3d input) {

  // Set up
  std::vector<Eigen::Vector3d> tempParticles = particles;
  std::vector<Eigen::Vector3d> weights;
  std::vector<Eigen::Vector3d> cumWeights;
  weights.reserve(tempParticles.size());
  cumWeights.reserve(tempParticles.size());

  std::vector<Eigen::Vector3d>::iterator particles_iter;
  std::vector<Eigen::Vector3d>::iterator tempParticles_iter;
  std::vector<Eigen::Vector3d>::iterator weights_iter;

  particles_iter = particles.begin();
  tempParticles_iter = tempParticles.begin();

  ROS_INFO("ParticleFilter update");

  while(particles_iter != particles.end() || tempParticles_iter != tempParticles.end())
    {
      // Random disturbance
      Eigen::Vector3d error;
      error << sample(R(0)), sample(R(1)), sample(R(2));

      // Propogate each particle through motion model
      *tempParticles_iter = (motionA * (*particles_iter) + motionB * input) + error;
      // Calculate weighting
      weights.push_back(normpdf3d(measurementY, measurementC * (*tempParticles_iter), R));
      // Calculate cumulative sum
      cumWeights.push_back( cumWeights.size() > 0 ? cumWeights.back() + weights.back() : weights.back());

      //Iterate
      if (particles_iter != particles.end())
        {
          particles_iter++;
        }
      if (tempParticles_iter != tempParticles.end())
        {
          tempParticles_iter++;
        }
    }

  // Resample
  for (particles_iter = particles.begin();
         particles_iter < particles.end(); particles_iter++) {
    Eigen::Vector3d seed = cumWeights.back() * dis(rng);
    for (tempParticles_iter = tempParticles.begin(), weights_iter = weights.begin(); weights_iter < weights.end() &&
           tempParticles_iter < tempParticles.end();
         weights_iter++, tempParticles_iter++) {
      // Am I cheating? At least a little bit.
      // (Comparing the individual weights is too hard so I'm not gonna do it)
      if ( (*weights_iter).norm() > seed.norm()) {
        *particles_iter = *tempParticles_iter;
        ROS_INFO("Resample: %f %f %f", (*particles_iter)(0), (*particles_iter)(1), (*particles_iter)(2));
        break;
      }
        }
    }
}

void ParticleFilter::calculateStats()
{
  particleMean = Eigen::Vector3d::Zero();
  for (std::vector<Eigen::Vector3d>::iterator ptr = particles.begin();
       ptr < particles.end(); ptr)
    {
      particleMean = particleMean + (*ptr);
    }
    particleMean = particleMean/ double(particles.size());
    ROS_INFO("Mean: %f %f %f", particleMean(0), particleMean(1), particleMean(2));
}


// Mutators
void ParticleFilter::setMotionA(Eigen::Matrix3d A) {motionA = A;}
void ParticleFilter::setMotionB(Eigen::Matrix3d B) {motionB = B;}
void ParticleFilter::setR(Eigen::Vector3d r) {R = r;}

// Accessors

Eigen::Matrix3d ParticleFilter::getMotionA() { return motionA; }
Eigen::Matrix3d ParticleFilter::getMotionB() { return motionB; }
Eigen::Vector3d ParticleFilter::getMean() {return particleMean; }
