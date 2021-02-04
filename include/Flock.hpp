#ifndef __FLOCK_HPP__
#define __FLOCK_HPP__

#include <random>
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
		void print();
		void flock();
		std::vector<Boid> boids;
	};
}

#endif