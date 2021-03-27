#include "Flock.hpp"

murmurations::Flock::Flock() {}

void murmurations::Flock::print() {
  for (auto &b : boids)
    b.print();
}

void murmurations::Flock::flock() {
  
  // Threaded approach
  // std::vector<std::thread> threads;
  // for (auto& b : boids)
  //	threads.push_back(std::thread ([&] { b.flock(boids); }));
  // for (auto& th : threads)
  //	th.join();

  // Non-threaded approach
  for (auto &b : boids)
    b.flock(boids);

  for (auto &b : boids)
    b.update();
}

Eigen::Vector2d murmurations::Flock::center() {
  Eigen::Vector2d res = Eigen::Vector2d::Constant(0);
  for (auto &b : boids)
    res += b.position;
  return res / boids.size();
}
