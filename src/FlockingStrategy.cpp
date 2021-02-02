#include "FlockingStrategy.hpp"


void murmurations::BasicFlocking::flock(std::vector<Boid>& boids)
{
	for (auto& current : boids)
	{
		Eigen::Vector2d separation = Eigen::Vector2d::Constant(0.0);
		Eigen::Vector2d alignment = Eigen::Vector2d::Constant(0.0);
		Eigen::Vector2d cohesion = Eigen::Vector2d::Constant(0.0);

		// TODO: implement basic strategy here
		//current.applyForce(separation);
		//current.applyForce(alignment);
		//current.applyForce(cohesion);
	}
}

void murmurations::ProjectionFlocking::flock(std::vector<Boid>& boids)
{
	// TODO: break this function down
	for (auto& current : boids)
	{
			//printf("Flocking for %i\n", current.id());

			bool overlapping = false;

			std::priority_queue<murmurations::Event, std::vector<murmurations::Event>> pq; // sorts based on event theta (light/dark domains)
			std::priority_queue<murmurations::Neighbor, std::vector<murmurations::Neighbor>> inSight; // sorts based on neighbor distance

			for (auto& other : boids)
			{
				// skip self when comparing
				// TODO: comparing IDs is cheap
				if (current.id() == other.id())
					continue;

				double d = current.euclideanDistance(other);
				double dx = other.position().x() - current.position().x();
				double dy = other.position().y() - current.position().y();

				double reltheta = atan2(dy, dx);
				double dtheta = asin(other.radius() / d);

				//printf("[%i] %f,%f,%f reltheta = %f, dtheta = %f\n", b.getID(), d, dx, dy, reltheta, dtheta);

				// no pushing events related to boids that overlap center
				if (d < other.radius()) {
					overlapping = true;
					continue;
				}

				pq.push(murmurations::Event(reltheta + dtheta, other.id(), true));
				pq.push(murmurations::Event(reltheta - dtheta, other.id(), false));
			}

			std::set<int> status; // ids of boids blocking vision at any given point
			std::set<double> boundaries;

			while (!pq.empty())
			{
				murmurations::Event e = pq.top();

				if (!e.isFront())
				{
					// handle case where other boid straddles beginning theta
					if (!status.empty())
					{
						boundaries.insert(e.getTheta());
						status.erase(e.getID());
					}
				}
				else
				{
					if (status.empty())
					{
						boundaries.insert(e.getTheta());
						status.insert(e.getID());
					}
				}

				// TODO: neighbors in line of sight
				int piercedBoid = 0;
				double minDist = 1000.0 * 1000.0;
				if (!status.empty())
				{
					// TODO: should not use int ID here
					for (auto& id : status) {
						double dist = current.euclideanDistance(boids[id]);
						if (dist < minDist)
						{
							piercedBoid = id;
							minDist = dist;
						}
					}
					murmurations::Neighbor n = murmurations::Neighbor(piercedBoid, minDist);
					inSight.push(n);
				}
				pq.pop();
			}

			std::vector<Eigen::Vector2d> vectorBoundaries;
			std::vector<Eigen::Vector2d> vectorNeighbors;

			for (auto& theta : boundaries)
				vectorBoundaries.push_back(Eigen::Vector2d(cos(theta), sin(theta)));

			// get only the nearest 4 neighbors
			for (int i = 0; i <= 4 && !inSight.empty(); i++)
			{
				murmurations::Neighbor currNeighbor = inSight.top();
				int nid = currNeighbor.id();
				murmurations::Boid b = boids[nid];
				vectorNeighbors.push_back(b.velocity());
			}

			// TODO: this is a flock level variable for the projection strategy
			// represent that in code
			double phi_p = 0.20;
			double phi_a = 0.45;
			double phi_n = 0.35;

			Eigen::Vector2d desiredVelocity = phi_p * murmurations::AverageDirection(vectorBoundaries) +
				phi_a * murmurations::AverageDirection(vectorNeighbors) +
				phi_n * Eigen::Vector2d::Random();

			// Reynolds: steering force = desired velocity - current velocity
			desiredVelocity.normalize();
			desiredVelocity *= current.maxSpeed();
			current.applyForce(desiredVelocity - current.velocity());
	}
}