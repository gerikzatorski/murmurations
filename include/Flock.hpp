#ifndef __FLOCK_HPP__
#define __FLOCK_HPP__

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>
#include <Windows.h>
#include <thread>

#include "Boid.hpp"

namespace murmurations {
	class Flock {
	public:
		Flock(int);
		
		std::vector<Boid> boids;

		void print();
		void flock();
		Eigen::Vector2d center();
	};
}

#endif