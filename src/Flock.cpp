#include "Flock.hpp"

murmurations::Flock::Flock(unsigned int count) {
  for (unsigned int i = 0; i < count; i++) {
    Eigen::Vector2d position = 100 * Eigen::Vector2d::Random(2);
    Eigen::Vector2d velocity = Eigen::Vector2d::Random(2);

    Boid b = Boid(i, position, velocity);
    boids.push_back(b);
  }
}

void murmurations::Flock::print() const {
  for (auto &b : boids)
    b.print();
}

void murmurations::Flock::flock() {
  for (auto &b : boids)
    b.flock(boids);

  for (auto &b : boids)
    b.update();
}

Eigen::Vector2d murmurations::Flock::center() const {
  Eigen::Vector2d res = Eigen::Vector2d::Constant(0);
  for (auto &b : boids)
    res += b.position;
  return res / boids.size();
}
