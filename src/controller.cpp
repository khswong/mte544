/*
waypoints come in as array of (x,y)
[(x,y),(x,y)..]
*/
#include <cmath>
#include <stdio.h>

struct Point {
  double x, y;
};

int main() {

  Point point_arr[4] = {
      {0, 0}, {5, 0}, {10, 5}, {10, 10},
  };

  double rad2deg = 360 / (2 * M_PI);
  double deg2rad = 2 * M_PI / 360;

  double tolerance = 0.25;
  double angle_tolerance = 5 * deg2rad;
  double speed = 0.1;
  double angle_gain = 0.5;
  double curr_x, curr_y, theta;
  curr_x = 0;
  curr_y = 0;

  double dx, dy, dist_to_pt, angle_to_pt;
  int arr_len = sizeof(point_arr) / sizeof(*point_arr);
  printf("arr_length: %d", arr_len);
  for (int i = 0; i < arr_len; i++) {
    Point point = point_arr[i];

    ////////////////////// GET ACTUAL POSE FROM IPS
    dx = point.x - curr_x;
    dy = point.y - curr_y;
    dist_to_pt = sqrt(pow(dx, 2) + pow(dy, 2));
    printf("\n\nNext Point: x: %f, y: %f: \n", point.x, point.y);

    while (std::abs(dist_to_pt) > tolerance) {
      angle_to_pt = atan2(dy, dx);

      // Rotate towards point

      ///////////////// GET ACTUAL ANGLE FROM IPS
      double angle_diff =
          std::fmod((angle_to_pt - theta + M_PI), (2 * M_PI)) - M_PI;

      while (std::abs(angle_diff) > angle_tolerance) {
        // P controller on angle
        theta += angle_diff * angle_gain;

        ////////////////GET ACTUAL ANGLE FROM IPS
        angle_diff = std::fmod((angle_to_pt - theta + M_PI), (2 * M_PI)) - M_PI;
        printf(
            "ANGLE: x:%.3f, y: %.3f, theta(deg):%.3f, angle_diff(deg): %.3f \n",
            curr_x, curr_y, theta * rad2deg, angle_diff * rad2deg);
      }

      // move towards point
      curr_x += speed * cos(theta);
      curr_y += speed * sin(theta);

      ////////////////////// GET ACTUAL POSE FROM IPS
      dx = point.x - curr_x;
      dy = point.y - curr_y;
      dist_to_pt = sqrt(pow(dx, 2) + pow(dy, 2));
      printf("SPEED: x:%.3f, y: %.3f, theta(deg):%.3f, dist: %.3f\n", curr_x,
             curr_y, theta * rad2deg, dist_to_pt);
    }
  }
}
