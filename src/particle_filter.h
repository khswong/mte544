#include <ros/ros.h>

#include <vector>
#include <map>
#include <random>
#include <eigen3/Eigen/Dense>

class ParticleFilter {
  std::vector<Vector3f> particles;
  std::vector<Vector3f> weights;
  Vector3f cumWeight; 

  // Motion model
  Matrix3f motionA;
  Matrix3f motionB;

  //Measurement model
  Matrix3f measurementC;

 public:
  Matrix3f getMotionA();
  Matrix3f getMotionB();
  void particleUpdate(Vector3f input);
  Vector3f randomError();

}
