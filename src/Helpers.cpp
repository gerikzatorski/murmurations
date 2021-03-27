#include "Helpers.hpp"
/* -------------------- NEIGHBOR --------------------*/

murmurations::Neighbor::Neighbor(int id, double distance) : id(id), distance(distance)
{}

bool murmurations::operator<(const murmurations::Neighbor &lhs,
                             const murmurations::Neighbor &rhs) {
  return lhs.distance < rhs.distance;
}

bool murmurations::operator>(const murmurations::Neighbor &lhs,
                             const murmurations::Neighbor &rhs) {
  return lhs.distance > rhs.distance;
}

/* -------------------- EVENT --------------------*/

murmurations::Event::Event(double theta, int id, bool frontEdge)
      : theta(theta), frontEdge(frontEdge), id(id)
{}

bool murmurations::operator<(const murmurations::Event &lhs,
                             const murmurations::Event &rhs) {
  return lhs.theta < rhs.theta;
}

bool murmurations::operator>(const murmurations::Event &lhs,
                             const murmurations::Event &rhs) {
  return lhs.theta > rhs.theta;
}
