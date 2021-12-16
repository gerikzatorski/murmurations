#ifndef __BOID_HPP__
#define __BOID_HPP__

#include "Helpers.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <thread> // TODO: only for printing thread_id atm
#include <memory>

namespace murmurations {

class Boid {

public:
  int id;
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  Eigen::Vector2d acceleration;
  double maxSpeed;
  double mass;
  double radius;

  Boid(int, Eigen::Vector2d, Eigen::Vector2d);

  void flock(std::vector<Boid> &);
  void basic(std::vector<Boid> &);
  void projection(std::vector<Boid> &);

  void update();
  void applyForce(Eigen::Vector2d);
  double euclideanDistance(Boid &) const;
  void print();
};
} // namespace murmurations

#endif
