#include <ros/ros.h>

#include <vector>
#include <map>
#include <random>
#include <eigen3/Eigen/Dense>

class ParticleFilter {

  // RNG
  std::random_device rd;
  std::mt19937 rng;
  std::uniform_real_distribution<> dis(0.0, 1.0);
  // particles
  std::vector<Vector3d> particles;

  // Motion model
  Matrix3d motionA;
  Matrix3d motionB;

  //Measurement model
  Vector3d measurementY;
  Matrix3d measurementC;

  double sample(double stddev);

 public:
  Matrix3d getMotionA();
  Matrix3d getMotionB();
  void particleUpdate(Vector3d input);
  Vector3d randomError();

}
