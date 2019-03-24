/** Author: Mario Lüder
 * Date: 2019-03-24
 */

#include <iostream>
#include <sstream>

#include "Trajectory.hpp"
#include "Spline/spline.h"

Trajectory::Trajectory()
{

}

Trajectory::~Trajectory() = default;

const Trajectory::PathPoints & Trajectory::calculateLanePath(const CarState & carState, const CarState & targetCarState, const TargetSpeed & targetSpeed, const Track &track, const double updateInterval)
{   
   m_waySplinePtr.reset(new NONAME::tk::spline());
   size_t nPreviousPathPoints = m_pathPoints.getLength();

   if (nPreviousPathPoints > 1)
   {
      const double predictedDistance = carState.m_carPositionXY.distance(m_pathPoints.getLast());
      const double halfDistanceToTargetSpeed = abs(targetSpeed.m_distance / 2.0);

      if (predictedDistance > halfDistanceToTargetSpeed)
      {
         const size_t pathPointsLimit = static_cast<size_t>((nPreviousPathPoints / predictedDistance) * halfDistanceToTargetSpeed);

         if (pathPointsLimit > 2)
         {
            m_pathPoints.limit(pathPointsLimit);
         }
         else
         {
            m_pathPoints.limit(2);
         }
         nPreviousPathPoints = m_pathPoints.getLength();
      }
   }

   // calculate remaining path points for the desired speed
   const size_t remainingPoints = m_pathPoints.getMax() - m_pathPoints.getLength();

   // if the car is not moving
   if (0.0001 > abs(carState.getSpeedInMetersPerSecond()) && 0.0001 > abs(targetCarState.getSpeedInMetersPerSecond()))
   {
      for (size_t i = 0; i < remainingPoints; ++i)
      {
         m_pathPoints.push_back(carState.m_carPositionXY);
      }

      return m_pathPoints;
   }

   NONAME::tk::spline & waySpline = *m_waySplinePtr;

   m_wayPoints.clear();

   double angleInRad = 0.0;
   Coordinate2D lastPosition;
   Coordinate2D lastButOnePosition;

   // get start points for the path
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
      angleInRad = lastButOnePosition.headingTo(lastPosition);
   }

   m_wayPoints.push_back(lastButOnePosition);
   m_wayPoints.push_back(lastPosition);

   double speedInMps0;
   double speedInMps1;

   // get start points for speed
   if (nPreviousPathPoints == 0)
   {
      speedInMps0 = carState.getSpeedInMetersPerSecond();
      speedInMps1 = carState.getSpeedInMetersPerSecond();
   }
   else if (nPreviousPathPoints == 1)
   {
      speedInMps0 = carState.getSpeedInMetersPerSecond();
      speedInMps1 = getSpeed(m_pathPoints.getLast(), carState.m_carPositionXY, track, updateInterval);
   }
   else
   {
      speedInMps0 = getSpeed(m_pathPoints[nPreviousPathPoints - 3], m_pathPoints[nPreviousPathPoints - 2], track, updateInterval);;
      speedInMps1 = getSpeed(m_pathPoints[nPreviousPathPoints - 2], m_pathPoints.getLast(), track, updateInterval);
   }

   // const double lastAcceleration = (speedInMps1 - speedInMps0) / updateInterval;

   const Coordinate2D lastPositionSD = track.convertXYtoSD(lastPosition, angleInRad);

   // interpolate two S coordinates
   const double sDistance = track.sDistance(
            lastPositionSD.getS(),
            targetCarState.m_carPositionSD.getS());

   const double sSection = sDistance / 3.0;
   const double s1 = track.sNormalize(lastPositionSD.getS() + sSection);
   const double s2 = track.sNormalize(s1 + sSection);
   const double d = targetCarState.m_carPositionSD.getD();

   const Coordinate2D nextWp3XY = track.convertToSDtoXY(Coordinate2D(s1, d));
   const Coordinate2D nextWp4XY = track.convertToSDtoXY(Coordinate2D(s2, d));

   // track anchor points
   m_wayPoints.push_back(nextWp3XY);
   m_wayPoints.push_back(nextWp4XY);
   const Coordinate2D & nextWp5XY = targetCarState.m_carPositionXY;
   m_wayPoints.push_back(nextWp5XY);

   // convert to ego coordinates
   m_wayPoints.transformToCoordinateSystem(carState.m_carPositionXY, carState.m_yawInRad);
   waySpline.set_points(m_wayPoints.getX(), m_wayPoints.getY());

   // speed anchor points
   const double targetSpeedInMps = targetSpeed.m_speedInMps;
   const double speedOffsetInMps = targetSpeedInMps - speedInMps1;


   /* acceleration equations, just for checking/debugging */
   // v = s / t
   // t = s / v

   // a = v / t
   // t = v / a

   // a = 2 * s / t^2
   // a = 2 * s / (v / a)^2
   // a = 2 * s / (v^2 / a^2)
   // v^2 / a = 2 * s
   // v^2 = 2 * s * a
   // 2 * s * a = v^2
   // a = v^2 / (2 * s)

   // a = 2 * s/t^2 - 2 * v0/t
   // a + 2 * v0 / t = 2 * s / t^2
   // ((a + 2 * v0 / t) *  t^2) / 2 = s
   // s = (a * t^2 + 2 * v0 * t) / 2

   Coordinate2D lastPathPoint = m_wayPoints[1];
   const Coordinate2D lastButOnePathPoint = m_wayPoints[0];

   double vDistance = track.sDistance(
            lastPositionSD.getS(),
            carState.m_carPositionSD.getS() + targetSpeed.m_distance);

   if (vDistance < 10.0)
   {
      vDistance = 10.0;
   }

   double heading = lastButOnePathPoint.headingTo(lastPathPoint);
   const double accelerationSign = copysign(1.0, speedOffsetInMps);
   double a = pow(speedOffsetInMps, 2) / (2 * vDistance) * accelerationSign;

   if (abs(a) < 0.3)
   {
      a = 0.3 * accelerationSign;
   }

   const double x0 = lastPathPoint.getX();
   std::cout << "v0:" << speedInMps0 << " v1:" << speedInMps1 << " v2:" << targetSpeedInMps << std::endl;
   std::cout << "remaining Points:" << remainingPoints << " x0:" << x0 << " vLastInMps:" << speedInMps1 << " vTarget:" << targetSpeedInMps << " vDiff:" << speedOffsetInMps << " a:" << a << std::endl;
   std::cout << " vDistance:" << vDistance << std::endl;
   std::stringstream speedLog;
   std::stringstream ratioLog;
   std::stringstream desiredSpeedLog;
   std::stringstream headingLog;

   speedLog << " " << getSpeed(lastButOnePathPoint, lastPathPoint, track, updateInterval);


   for (size_t i = 0; i < remainingPoints; ++i)
   {
      // guess next X position bases on last position, heading and speed
      const double xRatio = sin(heading + M_PI_2);
      const double t = (i + 1) * updateInterval;
      const double v = speedInMps1 + a * t;
      const double x = lastPathPoint.getX() + xRatio * (v * updateInterval);
      const double y = waySpline(x);

      Coordinate2D pathPoint(x,y);

      speedLog << " " << getSpeed(lastPathPoint, pathPoint, track, updateInterval);
      ratioLog << " " << xRatio;
      desiredSpeedLog << " " << v;
      headingLog << " " << heading;

      heading = lastPathPoint.headingTo(pathPoint);
      lastPathPoint = pathPoint;

      pathPoint.inverseTransformToCoordinateSystem(carState.m_carPositionXY, carState.m_yawInRad);
      m_pathPoints.push_back(pathPoint);
   }

   std::cout << "Speed:" << speedLog.str() << std::endl;
   std::cout << "Desired Speed:" << desiredSpeedLog.str() << std::endl;
   std::cout << "Ratio:" << ratioLog.str() << std::endl;
   std::cout << "Heading:" << headingLog.str() << std::endl;
   std::cout << std::endl;

   return m_pathPoints;
}

const Trajectory::PathPoints & Trajectory::calculateLinearPath(const CarState & carState, const CarState & targetCarState, const Track & track, const double updateInterval)
{
   m_pathPoints.clear();
   m_waySplinePtr.reset(nullptr);
   m_speedSplinePtr.reset(nullptr);

   if (0.0001 > abs(carState.getSpeedInMetersPerSecond()) && 0.0001 > abs(targetCarState.getSpeedInMetersPerSecond()))
   {
      for (size_t i = 0; i < m_pathPoints.getMax(); ++i)
      {
         m_pathPoints.push_back(carState.m_carPositionXY);
      }

      return m_pathPoints;
   }

   const double speedInMps = carState.getSpeedInMetersPerSecond();
   const double intervallInM = speedInMps * updateInterval;

   for (size_t i = 0; i < m_pathPoints.getMax(); ++i)
   {
      const double x = (i + 1) * intervallInM;
      const double y = 0;
      Coordinate2D pathPoint(x,y);
      pathPoint.inverseTransformToCoordinateSystem(carState.m_carPositionXY, carState.m_yawInRad);
      m_pathPoints.push_back(pathPoint);
   }

   return m_pathPoints;
}

double Trajectory::getSpeed(const Coordinate2D & xyPos0,  const Coordinate2D & xyPos1, const Track &track, const double updateInterval)
{
   (void)track;
   // const double heading = xyPos0.heading(xyPos1);
   // const Coordinate2D sdPos0 = track.convertXYtoSD(xyPos0, heading);
   // const Coordinate2D sdPos1 = track.convertXYtoSD(xyPos1, heading);

   // const bool backward = sdPos1.getS() < sdPos0.getS();
   // const double_t directionSign = 1.0 - 2.0 * backward;
   //const double speedInMps = directionSign * xyPos0.distance(xyPos1) / updateInterval;
   const double speedInMps = xyPos0.distance(xyPos1) / updateInterval;
   return speedInMps;
}
