#ifndef __BOID_HPP__
#define __BOID_HPP__

#include "Helpers.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <thread> // TODO: only for printing thread_id atm

namespace murmurations {

class Boid {

public:
  Boid(int id, std::function<void(Boid &, std::vector<Boid> &)> strategy,
       Eigen::Vector2d position, Eigen::Vector2d velocity)
      : _id(id), _strategy(strategy), _position(position), _velocity(velocity),
        _acceleration(Eigen::Vector2d(0, 0)), _maxSpeed(2.0), _mass(50.0),
        _radius(3.0) {}

  ~Boid() = default;

  int id() const;
  double radius() const;
  double maxSpeed() const;
  Eigen::Vector2d position() const;
  Eigen::Vector2d velocity() const;
  Eigen::Vector2d acceleration() const;

  void setStrategy(std::function<void(Boid &, std::vector<Boid> &)>);
  void flock(std::vector<Boid> &);
  friend void basic(Boid &, std::vector<Boid> &);
  friend void projection(Boid &, std::vector<Boid> &);

  void update();
  void applyForce(Eigen::Vector2d);
  double euclideanDistance(Boid &) const;
  void print();

private:
  int _id;
  Eigen::Vector2d _position;
  Eigen::Vector2d _velocity;
  Eigen::Vector2d _acceleration;
  double _maxSpeed;
  double _mass;
  double _radius;
  std::function<void(Boid &, std::vector<Boid> &)> _strategy;
};
} // namespace murmurations

#endif
