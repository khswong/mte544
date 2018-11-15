#include "particle_filter.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <random>

/*
 * Standard mathematical functions to be used within this file
 * normpdf, normpdf3d
 */

// Take the recipe for this formula from these sources:
// https://www.mathworks.com/help/stats/normpdf.html
// https://stackoverflow.com/questions/29397296/c11-normal-distribution-and-matlab-normpdf
const double ONE_OVER_SQRT_2PI = 1.0 / sqrt(2 * M_PI);
double normpdf(double x, double mu, double sigma) {
  return (ONE_OVER_SQRT_2PI)*exp(-0.5 * pow(x - mu, 2) / pow(sigma, 2));
}

Eigen::Vector3d normpdf3d(Eigen::Vector3d x, Eigen::Vector3d mu,
                          Eigen::Vector3d sigma) {
  Eigen::Vector3d normVec;
  for (int i = 0; i < 3; i++) {
    normVec(i) = normpdf(x(i), mu(i), sigma(i));
  }
  return normVec;
}

// I'm passing in 3d vectors but im only doing a bivariate multinorm pdf
double multinormpdf(Eigen::Vector3d x, Eigen::Vector3d mu,
                    Eigen::Vector3d sigma) {
  return exp(-0.5 * (pow((x(0) - mu(0)), 2) / pow(sigma(0), 2) +
                     pow((x(1) - mu(1)), 2) / pow(sigma(1), 2))) /
         (2 * M_PI * sigma(0) * sigma(1));
}

double ParticleFilter::sample(double stddev) {
  // Further optimization - set up vector of nd from R in initialization
  std::normal_distribution<> nd(0, stddev);
  return nd(rng);
}

// Blank constructor
ParticleFilter::ParticleFilter() : ParticleFilter(0) {}

ParticleFilter::ParticleFilter(int numParticles)
    : ParticleFilter(Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
                     Eigen::Matrix3d::Identity(), numParticles) {}

// Constructor/initialization
ParticleFilter::ParticleFilter(Eigen::Matrix3d A, Eigen::Matrix3d B,
                               Eigen::Matrix3d C, int numParticles)
    : motionA(A), motionB(B), measurementC(C) {
  // Initialize random number generators
  dis = std::uniform_real_distribution<double>(0.0, 1.0);
  ROS_INFO("CONSTRUCOTR");
  for (int i = 0; i < numParticles; i++) {
    // Random disturbance
    Eigen::Vector3d initialSpread;
    initialSpread << 0.5, 0.5, 0.5;
    Eigen::Vector3d disturbance;
    disturbance << sample(initialSpread(0)), sample(initialSpread(1)),
        sample(initialSpread(2));
    Eigen::Vector3d particle = Eigen::Vector3d::Zero() + disturbance;
    particles.push_back(particle);
  }
}

void ParticleFilter::measurementUpdate(Eigen::Vector3d measurement) {
  measurementY = measurement; // + error;
}

void ParticleFilter::particleInit(Eigen::Vector3d pose) {
  for (std::vector<Eigen::Vector3d>::iterator particle_itr = particles.begin();
       particle_itr < particles.end(); particle_itr++) {
    // Random dxisturbance
    Eigen::Vector3d initialSpread;
    initialSpread << 0.5, 0.5, 0.5;
    Eigen::Vector3d disturbance;
    disturbance << sample(initialSpread(0)), sample(initialSpread(1)),
        sample(initialSpread(2));
    (*particle_itr) = pose + disturbance;
  }
}

void ParticleFilter::particleUpdate(Eigen::Vector3d input) {

  // Set up
  std::vector<Eigen::Vector3d> tempParticles = particles;
  std::vector<Eigen::Vector3d> weights;
  std::vector<Eigen::Vector3d> cumWeights;

  std::vector<double> weights_d;
  std::vector<double> cumWeights_d;

  weights.reserve(tempParticles.size());
  cumWeights.reserve(tempParticles.size());

  std::vector<Eigen::Vector3d>::iterator particles_iter;
  std::vector<Eigen::Vector3d>::iterator tempParticles_iter;
  std::vector<Eigen::Vector3d>::iterator weights_iter;
  std::vector<double>::iterator weightsd_iter;

  particles_iter = particles.begin();
  tempParticles_iter = tempParticles.begin();

  //  ROS_DEBUG("ParticleFilter update");

  while (particles_iter != particles.end() ||
         tempParticles_iter != tempParticles.end()) {
    // Random disturbance
    Eigen::Vector3d error;
    error << sample(R(0)), sample(R(1)), sample(R(2));

    // Propogate each particle through motion model
    *tempParticles_iter =
        (motionA * (*particles_iter) + motionB * input) + error;
    
    // Calculate weighting
    weights.push_back(
        normpdf3d(measurementY, measurementC * (*tempParticles_iter), R));
    // Calculate cumulative sum
    cumWeights.push_back(cumWeights.size() ? cumWeights.back() + weights.back()
                                           : weights.back());

    // Calculate weighting
    weights_d.push_back(
        multinormpdf(measurementY, measurementC * (*tempParticles_iter), R));
    // Calculate cumulative sum
    cumWeights_d.push_back(cumWeights_d.size()
                               ? cumWeights_d.back() + weights_d.back()
                               : weights_d.back());

    // Saturator
    // Saturtor should go AFTER the probability stuff.
    (*tempParticles_iter)(2) =
      fmod((*tempParticles_iter)(2), 2 * M_PI);

    // Iterate
    if (particles_iter != particles.end()) {
      particles_iter++;
    }
    if (tempParticles_iter != tempParticles.end()) {
      tempParticles_iter++;
    }
  }

  Eigen::Vector3d seed;
  Eigen::Vector3i set;
  // Resample
  for (particles_iter = particles.begin(); particles_iter < particles.end();
       particles_iter++) {
    seed = cumWeights.back() * dis(rng);
    set = Eigen::Vector3i::Zero();
#if 0
    for (tempParticles_iter = tempParticles.begin(),
        weights_iter = weights.begin();
         weights_iter < weights.end() ||
         tempParticles_iter < tempParticles.end();
          weights_iter++, tempParticles_iter++) {
     if ((*weights_iter)(0) > seed(0) && !set(0)) {
        (*particles_iter)(0) = (*tempParticles_iter)(0);
        set(0) = 1;
      }
      if ((*weights_iter)(1) > seed(1) && !set(1)) {
        (*particles_iter)(1) = (*tempParticles_iter)(1);
        set(1) = 1;
      }
      if ((*weights_iter)(2) > seed(2) && !set(2)) {
        (*particles_iter)(2) = (*tempParticles_iter)(2);
        set(2) = 1;
      }
      if (set(0) && set(1) && set(2)) {
        break;
      }
    }
#else
    double seed_d = cumWeights_d.back() * dis(rng);
    for (tempParticles_iter = tempParticles.begin(),
        weightsd_iter = weights_d.begin();
         weightsd_iter < weights_d.end() ||
         tempParticles_iter < tempParticles.end();
         weightsd_iter++, tempParticles_iter++) {
      if ((*weightsd_iter) > seed_d) {
        (*particles_iter) = (*tempParticles_iter);
        break;
      }
    }
#endif

    // ROS_INFO("Weight: %f %f %f norm: %f", (*weights_iter)(0),
    //         (*weights_iter)(1), (*weights_iter)(2), (*weights_iter).norm());
    // ROS_INFO("Seed: %f %f %f norm: %f", seed(0), seed(1), seed(2),
    // seed.norm());
    // ROS_INFO("Resample: %f %f %f", (*particles_iter)(0),
    // (*particles_iter)(1),
    //         (*particles_iter)(2));
  }
}

void ParticleFilter::calculateStats() {
  particleMean = Eigen::Vector3d::Zero();
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  for (std::vector<Eigen::Vector3d>::iterator ptr = particles.begin();
       ptr < particles.end(); ptr++) {
    particleMean = particleMean + (*ptr);
    x.push_back((*ptr)(0));
    y.push_back((*ptr)(1));
    theta.push_back((*ptr)(2));
  }
  std::sort(x.begin(), x.end(), [](double a, double b) { return a > b; });
  std::sort(y.begin(), y.end(), [](double a, double b) { return a > b; });
  std::sort(theta.begin(), theta.end(),
            [](double a, double b) { return a > b; });

  particleMedian(0) = *(x.begin() + (x.size() / 2));
  particleMedian(1) = *(y.begin() + (y.size() / 2));
  particleMedian(2) = *(theta.begin() + (theta.size() / 2));
  particleMean = particleMean * (1.0 / double(particles.size()));

  particleMedian(2) = fmod(particleMedian(2) + M_PI, 2 * M_PI) - M_PI;
  particleMean(2) = fmod(particleMean(2) + M_PI, 2 * M_PI) - M_PI;
  ROS_INFO("Mean pose     X: %f Y: %f Yaw: %f", particleMean(0),
           particleMean(1), particleMean(2));
  ROS_INFO("Median pose     X: %f Y: %f Yaw: %f", particleMedian(0),
           particleMedian(1), particleMedian(2));
}

// Mutators
void ParticleFilter::setMotionA(Eigen::Matrix3d A) { motionA = A; }
void ParticleFilter::setMotionB(Eigen::Matrix3d B) { motionB = B; }
void ParticleFilter::setR(Eigen::Vector3d r) { R = r; }

// Accessors

Eigen::Matrix3d ParticleFilter::getMotionA() { return motionA; }
Eigen::Matrix3d ParticleFilter::getMotionB() { return motionB; }
Eigen::Vector3d ParticleFilter::getMean() { return particleMean; }
Eigen::Vector3d ParticleFilter::getMedian() { return particleMedian; }
std::vector<Eigen::Vector3d> ParticleFilter::getParticles() {
  return particles;
}
