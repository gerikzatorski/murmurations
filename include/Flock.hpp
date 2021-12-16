#ifndef __FLOCK_HPP__
#define __FLOCK_HPP__

#include "Boid.hpp"

namespace murmurations {

class Flock {
public:
  std::vector<Boid> boids{};

  Flock(unsigned int count);

  void print() const;
  void flock();
  Eigen::Vector2d center() const;
};

} // namespace murmurations

#endif
