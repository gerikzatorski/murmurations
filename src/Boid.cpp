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

void murmurations::Boid::update()
{
	_velocity += _acceleration;
	_velocity.normalize();
	_velocity *= _maxSpeed;
	_position += _velocity;
	_acceleration = Eigen::Vector2d::Constant(0); // reset acceleration
}
void murmurations::Boid::applyForce(Eigen::Vector2d force)
{
	_acceleration += (force/_mass);
}
double murmurations::Boid::euclideanDistance(Boid other) const
{
	return sqrt(pow((this->position().x() - other.position().x()), 2) + pow((this->position().y() - other.position().y()), 2));
}
void murmurations::Boid::print()
{
	printf("%4i: %6.1f %6.1f | %6.2f %6.2f | %6.2f %6.2f\n", _id, _position.x(), _position.y(), _velocity.x(), _velocity.y(), _acceleration.x(), _acceleration.y());
}
