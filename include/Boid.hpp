#ifndef __BOID_HPP__
#define __BOID_HPP__

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <queue>
#include <set>
#include <vector>

#include "Helpers.hpp"

namespace murmurations {

	class Boid {
	public:
		Boid(int id, Eigen::Vector2d position, Eigen::Vector2d velocity)
			: _id(id)
			, _position(position)
			, _velocity(velocity)
			, _acceleration(Eigen::Vector2d(0, 0))
			, _maxSpeed(3)
			, _mass(2)
			, _radius(4) 
		{}

		int id() const;
		double radius() const;
		double maxSpeed() const;
		Eigen::Vector2d position() const;
		Eigen::Vector2d velocity() const;
		Eigen::Vector2d acceleration() const;

		void update();
		void applyForce(Eigen::Vector2d);
		double euclideanDistance(Boid) const;
		void print();

	private:
		int _id;
		Eigen::Vector2d _position;
		Eigen::Vector2d _velocity;
		Eigen::Vector2d _acceleration;
		double _maxSpeed;
		double _mass;
		double _radius;
	};

}

#endif