#ifndef __FLOCK_HPP__
#define __FLOCK_HPP__

#include <vector>
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <math.h>
#include <thread>
#include <memory>

#include "Boid.hpp"

namespace murmurations {
class Flock {
public:
  Flock(int);

  std::vector<Boid> boids;

  void print();
  void flock();
  Eigen::Vector2d center();
};
} // namespace murmurations

#endif
