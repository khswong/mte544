#include <ros/ros.h>

#include <vector>
#include <map>
#include <random>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class ParticleFilter {

  // RNG
  std::random_device rd;
  std::uniform_real_distribution<double> dis;
  std::mt19937 rng;
  // particles
  std::vector<Eigen::Vector3d> particles;

  // Motion model
  Eigen::Matrix3d motionA;
  Eigen::Matrix3d motionB;
  Eigen::Vector3d R = Eigen::Vector3d::Zero();

  //Measurement mode
  Eigen::Vector3d measurementY;
  Eigen::Matrix3d measurementC;

  Eigen::Vector3d particleMean;
  Eigen::Vector3d particleVariance;

  double sample(double stddev);

 public:
  // Constructors
  ParticleFilter();
  ParticleFilter(int numParticles);
  ParticleFilter(Eigen::Matrix3d A, Eigen::Matrix3d B, Eigen::Matrix3d C, int numParticles);

  // Mutators
  void setMotionA(Eigen::Matrix3d A);
  void setMotionB(Eigen::Matrix3d B);
  void setR(Eigen::Vector3d r);

  // Accessors
  Eigen::Matrix3d getMotionA();
  Eigen::Matrix3d getMotionB();
  Eigen::Vector3d getMean();
  std::vector<Eigen::Vector3d> getParticles();
  // State updators
  void particleUpdate(Eigen::Vector3d input);
  void measurementUpdate(Eigen::Vector3d measurement);
  void calculateStats();
};
