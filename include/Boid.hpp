#ifndef __BOID_HPP__
#define __BOID_HPP__

#include "Helpers.hpp"
#include <vector>
#include <Eigen/Dense>

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
  void print() const ;
};

} // namespace murmurations

#endif
