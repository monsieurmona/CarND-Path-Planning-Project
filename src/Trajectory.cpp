#include "Trajectory.hpp"

void Trajectory::calculate(const CarState & carState, const CarState & targetCarState, const Track &track)
{
   const size_t nPreviousPathPoints = m_pathPoints.getLength();
   double angleInRad = 0.0;
   Coordinate2D lastPosition;
   Coordinate2D lastButOnePosition;

   const Coordinate2D egoCoordinate = carState.m_carPositionXY;
   const double egoYawInRad = carState.m_yawInRad;

   if (nPreviousPathPoints < 2)
   {
      lastPosition = carState.m_carPositionXY;
      angleInRad = carState.m_yawInRad;
      lastButOnePosition = lastPosition.move(-1.0, angleInRad);
   }
   else
   {
      lastPosition = m_pathPoints.getLast();
      lastButOnePosition = m_pathPoints[nPreviousPathPoints - 2];
   }

   m_wayPoints.push_back(lastButOnePosition);
   m_wayPoints.push_back(lastPosition);

   m_pathPoints.simpleTrajectory(lastPosition, 0.5, angleInRad);
}
