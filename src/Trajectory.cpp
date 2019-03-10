#include "Trajectory.hpp"
#include "Spline/spline.h"

Trajectory::Trajectory() : m_splinePtr(new NONAME::tk::spline())
{

}

Trajectory::~Trajectory() = default;

void Trajectory::calculate(const CarState & carState, const CarState & targetCarState, const Track &track, const double updateInterval)
{
   NONAME::tk::spline & spline = *m_splinePtr;
   const size_t nPreviousPathPoints = m_pathPoints.getLength();

   double angleInRad = 0.0;
   Coordinate2D lastPosition;
   Coordinate2D lastButOnePosition;

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

   const Coordinate2D lastPositionSD = track.convertXYtoSD(lastPosition, angleInRad);

   // interpolate two S coordinates
   const double sDistance = targetCarState.m_carPositionSD.getS() + carState.m_carPositionSD.getS() - lastPositionSD.getS();
   const double sSection = sDistance / 3.0;
   const double s1 = lastPositionSD.getS() + sSection;
   const double s2 = s1 + sSection;
   const double d = targetCarState.m_carPositionSD.getD();

   const Coordinate2D nextWp1XY = track.convertToSDtoXY(Coordinate2D(s1, d));
   const Coordinate2D nextWp2XY = track.convertToSDtoXY(Coordinate2D(s2, d));

   m_wayPoints.push_back(nextWp1XY);
   m_wayPoints.push_back(nextWp2XY);
   m_wayPoints.push_back(targetCarState.m_carPositionXY + carState.m_carPositionXY);

   // convert to ego coordinates
   m_wayPoints.transformToCoordinateSystem(carState.m_carPositionXY, carState.m_yawInRad);

   spline.set_points(m_wayPoints.getX(), m_wayPoints.getY());

   // calculate remaining path points for the desired speed
   const double targetSpeedInMps = targetCarState.getSpeedInMetersPerSecond();

   // horizon
   const double horizonX = 30; // meter
   const Coordinate2D horizon(horizonX, spline(horizonX));
   const double horizonDistance = horizon.distance();
   const size_t remainingPoints = m_pathPoints.getMax() - m_pathPoints.getLength();
   double xAddOn = 0.0;

   // fill up the remaining points
   for (size_t i = 0; i < remainingPoints; ++i)
   {
      const double N = horizonDistance / (updateInterval * targetSpeedInMps);
      const double x = xAddOn + horizonX / N;
      const double y = spline(x);

      xAddOn = x;

      Coordinate2D pathPoint(x,y);
      pathPoint.inverseTransformToCoordinateSystem(carState.m_carPositionXY, carState.m_yawInRad);
      m_pathPoints.push_back(pathPoint);
   }
}
