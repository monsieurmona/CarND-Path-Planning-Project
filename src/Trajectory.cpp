#include "Trajectory.hpp"

void Trajectory::straight(const Coordinate2D & position)
{
   m_pathPoints.simpleTrajectory(position, 0.5, 0);
}
