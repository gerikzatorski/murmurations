#include "Boid.hpp"

murmurations::Boid::Boid(int id, Eigen::Vector2d position, Eigen::Vector2d velocity)
      : id(id), position(position), velocity(velocity),
        acceleration(Eigen::Vector2d(0, 0)), maxSpeed(2.0), mass(50.0),
        radius(3.0)
{}

void murmurations::Boid::flock(std::vector<murmurations::Boid> &boids) {
  projection(boids);
  return;
}

void murmurations::Boid::basic(std::vector<murmurations::Boid> &boids) {
  // find local flockmates
  std::vector<murmurations::Boid> localFlockmates{};
  double localRadius = 100.0;
  for (auto &b : boids) {
    if (this->id != b.id && this->euclideanDistance(b) < localRadius)
      localFlockmates.push_back(b);
  }

  Eigen::Vector2d separation = Eigen::Vector2d::Constant(0);
  Eigen::Vector2d alignment = Eigen::Vector2d::Constant(0);
  Eigen::Vector2d cohesion = Eigen::Vector2d::Constant(0);

  for (auto &b : localFlockmates) {
    Eigen::Vector2d dv = this->position - b.position;
    dv.normalize();
    separation += dv;
    alignment += b.velocity;
    cohesion += b.position;
  }

  if (!localFlockmates.empty()) {
    cohesion /= localFlockmates.size();
    // cohesion is currently the center of the local flockmates
    cohesion -= this->position;
  }

  separation.normalize();
  alignment.normalize();
  cohesion.normalize();

  // printf(" -%i- %6.2f, %6.2f | %6.2f, %6.2f | %6.2f, %6.2f\n", this->id,
  // separation.x(), separation.y(), alignment.x(), alignment.y(), cohesion.x(),
  // cohesion.y());

  // TODO: represent these as flock level variables for the basic strategy
  double separation_factor = 0.35;
  double alignment_factor = 0.20;
  double cohesion_factor = 0.45;

  Eigen::Vector2d desiredVelocity = separation_factor * separation +
                                    alignment_factor * alignment +
                                    cohesion_factor * cohesion;

  // Reynolds: steering force = desired velocity - current velocity
  this->applyForce(desiredVelocity.normalized() * this->maxSpeed -
                     this->velocity);
}

void murmurations::Boid::projection(std::vector<murmurations::Boid> &boids) {
  // std::cout << "Flocking for " << id << " on thread " <<
  // std::this_thread::getid() << std::endl;

  bool overlapping = false;

  std::priority_queue<murmurations::Event, std::vector<murmurations::Event>>
      pq; // sorts based on event theta (light/dark domains)
  std::priority_queue<murmurations::Neighbor,
                      std::vector<murmurations::Neighbor>>
      inSight; // sorts based on neighbor distance

  // generate light/dark domain priority queue
  for (auto &other : boids) {
    // skip self when comparing
    // TODO: comparing IDs is cheap
    if (this->id == other.id)
      continue;

    Eigen::Vector2d diff = other.position - this->position;
    double dist = sqrt(diff.dot(diff));
    double reltheta = atan2(diff.y(), diff.x());
    double dtheta = asin(other.radius / dist);

    // printf("[%i] %f,%f,%f reltheta = %f, dtheta = %f\n", b.getID(), d, dx,
    // dy, reltheta, dtheta);

    // no pushing events related to boids that overlap center
    if (dist < other.radius) {
      overlapping = true;
      continue;
    }

    pq.push(murmurations::Event(reltheta + dtheta, other.id, true));
    pq.push(murmurations::Event(reltheta - dtheta, other.id, false));
  }

  std::set<int> status; // ids of boids blocking vision at any given point
  std::set<double> boundaries;

  // unpack light/dark domain priority queue
  while (!pq.empty()) {
    murmurations::Event e = pq.top();

    if (!e.frontEdge) {
      // handle case where other boid straddles beginning theta
      if (!status.empty()) {
        boundaries.insert(e.theta);
        status.erase(e.id);
      }
    } else {
      if (status.empty()) {
        boundaries.insert(e.theta);
        status.insert(e.id);
      }
    }

    // neighbors in line of sight
    int piercedBoid = 0;
    double minDist = 1000.0 * 1000.0;
    if (!status.empty()) {
      // TODO: comparing IDs is cheap
      for (auto &id : status) {
        double dist = this->euclideanDistance(boids[id]);
        if (dist < minDist) {
          piercedBoid = id;
          minDist = dist;
        }
      }
      murmurations::Neighbor n = murmurations::Neighbor(piercedBoid, minDist);
      inSight.push(n);
    }
    pq.pop();
  }

  Eigen::Vector2d boundaryAvg = Eigen::Vector2d::Constant(0);
  Eigen::Vector2d neighborSum = Eigen::Vector2d::Constant(0);

  // TODO: be smarter here
  int i = 0;
  for (auto &theta : boundaries) {
    boundaryAvg += Eigen::Vector2d(cos(theta), sin(theta));
    i++;
  }
  boundaryAvg /= i;

  // get only the nearest 4 neighbors
  for (int i = 0; i <= 4 && !inSight.empty(); i++) {
    murmurations::Neighbor currNeighbor = inSight.top();
    murmurations::Boid b = boids[currNeighbor.id];
    neighborSum += b.velocity;
    inSight.pop();
  }

  // TODO: represent these as flock level variables for the projection strategy
  double phi_p = 0.15;
  double phi_a = 0.60;
  double phi_n = 0.25;

  Eigen::Vector2d desiredVelocity = phi_p * boundaryAvg +
                                    phi_a * neighborSum.normalized() +
                                    phi_n * Eigen::Vector2d::Random();

  // Reynolds: steering force = desired velocity - current velocity
  this->applyForce(desiredVelocity.normalized() * this->maxSpeed -
                     this->velocity);
}

void murmurations::Boid::update() {
  velocity += acceleration;

  if (sqrt(velocity.dot(velocity)) > maxSpeed) {
    velocity.normalize();
    velocity *= maxSpeed;
  }

  position += velocity;
  acceleration = Eigen::Vector2d::Constant(0); // reset acceleration
}

void murmurations::Boid::applyForce(Eigen::Vector2d force) {
  acceleration += (force / mass);
}

double murmurations::Boid::euclideanDistance(Boid &other) const {
  Eigen::Vector2d diff = other.position - position;
  return sqrt(diff.dot(diff));
}

void murmurations::Boid::print() {
  printf("%4i: %6.1f %6.1f | %6.2f %6.2f | %6.2f %6.2f\n", id, position.x(),
         position.y(), velocity.x(), velocity.y(), acceleration.x(),
         acceleration.y());
}
