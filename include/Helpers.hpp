#ifndef __HELPERS_HPP__
#define __HELPERS_HPP__

#include <Eigen/Dense>
#include <vector>

namespace murmurations {

class Neighbor {
public:
  Neighbor(int, double);

  friend bool operator<(const Neighbor &lhs, const Neighbor &rhs);
  friend bool operator>(const Neighbor &lhs, const Neighbor &rhs);

  int id;
  double distance;
};

class Event {
public:
  Event(double, int, bool);

  friend bool operator<(const Event &lhs, const Event &rhs);
  friend bool operator>(const Event &lhs, const Event &rhs);

  int id;
  double theta;
  bool frontEdge;
};

} // namespace murmurations

#endif
