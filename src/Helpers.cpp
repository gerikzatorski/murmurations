#include "Helpers.hpp"
/* -------------------- NEIGHBOR --------------------*/

double murmurations::Neighbor::distance() const
{
	return _distance;
}

int murmurations::Neighbor::id()
{
	return _id;
}
bool murmurations::operator < (const murmurations::Neighbor& lhs, const murmurations::Neighbor& rhs)
{
	return lhs.distance() < rhs.distance();
}

bool murmurations::operator > (const murmurations::Neighbor& lhs, const murmurations::Neighbor& rhs)
{
	return lhs.distance() > rhs.distance();
}

/* -------------------- EVENT --------------------*/

int murmurations::Event::getID()
{
	return _id;
}

double murmurations::Event::getTheta() const
{
	return _theta;
}

bool murmurations::Event::isFront() const
{
	return _frontEdge;
}

bool murmurations::operator < (const murmurations::Event& lhs, const murmurations::Event& rhs)
{
	return lhs.getTheta() < rhs.getTheta();
}

bool murmurations::operator > (const murmurations::Event& lhs, const murmurations::Event& rhs)
{
	return lhs.getTheta() > rhs.getTheta();
}
