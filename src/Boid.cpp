#include "Boid.hpp"

int murmurations::Boid::id() const
{
	return _id;
}
double murmurations::Boid::radius() const
{
	return _radius;
}
double murmurations::Boid::maxSpeed() const
{
	return _maxSpeed;
}
Eigen::Vector2d murmurations::Boid::position() const
{
	return _position;
}
Eigen::Vector2d murmurations::Boid::velocity() const
{
	return _velocity;
}
Eigen::Vector2d murmurations::Boid::acceleration() const
{
	return _acceleration;
}

void murmurations::Boid::flock(std::vector<Boid> boids)
{
	//std::cout << "Flocking for " << id() << " on thread " << std::this_thread::get_id() << std::endl;
	
	bool overlapping = false;
	
	std::priority_queue<murmurations::Event, std::vector<murmurations::Event>> pq; // sorts based on event theta (light/dark domains)
	std::priority_queue<murmurations::Neighbor, std::vector<murmurations::Neighbor>> inSight; // sorts based on neighbor distance

	// generate light/dark domain priority queue
	for (auto& other : boids)
	{
		// skip self when comparing
		// TODO: comparing IDs is cheap
		if (id() == other.id())
			continue;

		Eigen::Vector2d diff = other.position() - _position;
		double dist = sqrt(diff.dot(diff));
		double reltheta = atan2(diff.y(), diff.x());
		double dtheta = asin(other.radius() / dist);

		//printf("[%i] %f,%f,%f reltheta = %f, dtheta = %f\n", b.getID(), d, dx, dy, reltheta, dtheta);

		// no pushing events related to boids that overlap center
		if (dist < other.radius()) {
			overlapping = true;
			continue;
		}

		pq.push(murmurations::Event(reltheta + dtheta, other.id(), true));
		pq.push(murmurations::Event(reltheta - dtheta, other.id(), false));
	}

	std::set<int> status; // ids of boids blocking vision at any given point
	std::set<double> boundaries;

	// unpack light/dark domain priority queue
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

		// neighbors in line of sight
		int piercedBoid = 0;
		double minDist = 1000.0 * 1000.0;
		if (!status.empty())
		{
			// TODO: comparing IDs is cheap
			for (auto& id : status)
			{
				double dist = euclideanDistance(boids[id]);
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

	Eigen::Vector2d boundaryAvg = Eigen::Vector2d::Constant(0);
	Eigen::Vector2d neighborSum = Eigen::Vector2d::Constant(0);

	// TODO: be smarter here
	int i = 0;
	for (auto& theta : boundaries)
	{
		boundaryAvg += Eigen::Vector2d(cos(theta), sin(theta));
		i++;
	}
	boundaryAvg /= i;

	// get only the nearest 4 neighbors
	for (int i = 0; i <= 4 && !inSight.empty(); i++)
	{
		murmurations::Neighbor currNeighbor = inSight.top();
		murmurations::Boid b = boids[currNeighbor.id()];
		neighborSum += b.velocity();
		inSight.pop();
	}

	// TODO: this is a flock level variable for the projection strategy
	// represent that in code
	double phi_p = 0.10;
	double phi_a = 0.25;
	double phi_n = 0.65;

	Eigen::Vector2d desiredVelocity = phi_p * boundaryAvg +
		                              phi_a * neighborSum.normalized() +
		                              phi_n * Eigen::Vector2d::Random();

	// Reynolds: steering force = desired velocity - current velocity
	desiredVelocity.normalize();
	desiredVelocity *= maxSpeed();
	applyForce(desiredVelocity - velocity());
}
void murmurations::Boid::update()
{
	_velocity += _acceleration;

	if (sqrt(_velocity.dot(_velocity)) > _maxSpeed)
	{
		_velocity.normalize();
		_velocity *= _maxSpeed;
	}
	
	_position += _velocity;
	_acceleration = Eigen::Vector2d::Constant(0); // reset acceleration
}

void murmurations::Boid::applyForce(Eigen::Vector2d force)
{
	_acceleration += (force/_mass);
}
double murmurations::Boid::euclideanDistance(Boid& other) const
{
	Eigen::Vector2d diff = other.position() - _position;
	return sqrt(diff.dot(diff));
}
void murmurations::Boid::print()
{
	printf("%4i: %6.1f %6.1f | %6.2f %6.2f | %6.2f %6.2f | %6.2f\n", _id, _position.x(), _position.y(), _velocity.x(), _velocity.y(), _acceleration.x(), _acceleration.y(), sqrt(_velocity.dot(_velocity)));
}
