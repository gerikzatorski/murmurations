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

// class Flock; // declare for Boid's pointer
  
class Boid {

public:
  Boid(int, std::function<void(Boid &, std::vector<Boid> &)>,
       Eigen::Vector2d, Eigen::Vector2d);

  void setStrategy(std::function<void(Boid &, std::vector<Boid> &)>);
  void flock(std::vector<Boid> &);
  friend void basic(Boid &, std::vector<Boid> &);
  friend void projection(Boid &, std::vector<Boid> &);

  void update();
  void applyForce(Eigen::Vector2d);
  double euclideanDistance(Boid &) const;
  void print();

  int id;
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  Eigen::Vector2d acceleration;
  double maxSpeed;
  double mass;
  double radius;

private:
  std::function<void(Boid &, std::vector<Boid> &)> _strategy;
  // std::unique_ptr<Flock> _flock;
};
} // namespace murmurations

#endif
