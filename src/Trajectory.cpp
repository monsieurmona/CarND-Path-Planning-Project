#include <iostream>
#include <sstream>

#include "Trajectory.hpp"
#include "Spline/spline.h"

Trajectory::Trajectory()
{

}

Trajectory::~Trajectory() = default;

void Trajectory::calculate(const CarState & carState, const CarState & targetCarState, const Track &track, const double updateInterval)
{
   m_waySplinePtr.reset(new NONAME::tk::spline());
   m_speedSplinePtr.reset(new NONAME::tk::spline());
   NONAME::tk::spline & waySpline = *m_waySplinePtr;
   NONAME::tk::spline & speedSpline = *m_speedSplinePtr;
   m_pathPoints.limit(m_pathPoints.getMax() - m_pathPoints.getMax() / 3);
   const size_t nPreviousPathPoints = m_pathPoints.getLength();
   m_wayPoints.clear();
   m_speedPoints.clear();

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

   const Coordinate2D lastPositionSD = track.convertXYtoSD(lastPosition, angleInRad);

   // interpolate two S coordinates
   const double sDistance = targetCarState.m_carPositionSD.getS() - lastPositionSD.getS();
   const double sSection = sDistance / 3.0;
   const double s1 = lastPositionSD.getS() + sSection;
   const double s2 = s1 + sSection;
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

   // speed anchor points
   const double targetSpeedInMps = targetCarState.getSpeedInMetersPerSecond();
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

   constexpr double minAcceleration = 0.5;
   //const double maxAcceleration = std::pow(speedOffsetInMps,2) / (2 * sDistance) * ((speedOffsetInMps < 0.0) ? -1.0 : 1.0);
   const double maxAcceleration = std::pow(speedOffsetInMps,2) / (2 * sSection);
   const double undirectedAcceleration = (minAcceleration > abs(maxAcceleration)) ? minAcceleration : abs(maxAcceleration);
   const double maxSpeedChangeDuration = speedOffsetInMps / undirectedAcceleration;
   const double speedChangeDuration = (maxSpeedChangeDuration < updateInterval) ? updateInterval : maxSpeedChangeDuration;
   const double a = undirectedAcceleration * ((speedOffsetInMps < 0.0) ? -1.0 : 1.0);

   // const double maxSpeedChangeDuration = abs(speedOffsetInMps) / minAcceleration;
   // const double minSpeedChangeDuration = sSection / abs(speedInMps1 + speedOffsetInMps * 0.5);
   // const double speedChangeDuration = (minSpeedChangeDuration > maxSpeedChangeDuration) ? maxSpeedChangeDuration : minSpeedChangeDuration;

   const double t0 = - (updateInterval /* / 2.0 */);
   const double t1 = t0 + updateInterval;
   const double t2 = t1 + speedChangeDuration;
   const double t3 = t2 + updateInterval;
   const double t4 = t3 + updateInterval;

   std::cout << "t0:" << t0 << " t1:" << t1 << " t2:" << t2 << " t3:" << t3 << " t4:" << t4 << std::endl;
   std::cout << "v0:" << speedInMps0 << " v1:" << speedInMps1 << " v2:" << targetSpeedInMps << std::endl;

   m_speedPoints.push_back(Coordinate2D(t0, speedInMps0      ));
   m_speedPoints.push_back(Coordinate2D(t1, speedInMps1      ));
   m_speedPoints.push_back(Coordinate2D(t2, targetSpeedInMps ));
   m_speedPoints.push_back(Coordinate2D(t3, targetSpeedInMps ));
   m_speedPoints.push_back(Coordinate2D(t4, targetSpeedInMps ));

   Coordinate2D lastPathPoint = m_wayPoints[1];
   const Coordinate2D lastButOnePathPoint = m_wayPoints[0];
   const double x0 = lastPathPoint.getX();

   waySpline.set_points(m_wayPoints.getX(), m_wayPoints.getY());
   speedSpline.set_points(m_speedPoints.getX(), m_speedPoints.getY());

   // calculate remaining path points for the desired speed
   const size_t remainingPoints = m_pathPoints.getMax() - m_pathPoints.getLength();

   double heading = lastButOnePathPoint.headingTo(lastPathPoint);


   std::cout << "remaining Points:" << remainingPoints << " x0:" << x0 << " vLastInMps:" << speedInMps1 << " vTarget:" << targetSpeedInMps << " vDiff:" << speedOffsetInMps << " a:" << a << std::endl;
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
      const double v = speedSpline(t);
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

   // std::cout << std::endl;
   std::cout << "Speed:" << speedLog.str() << std::endl;
   std::cout << "Desired Speed:" << desiredSpeedLog.str() << std::endl;
   std::cout << "Ratio:" << ratioLog.str() << std::endl;
   std::cout << "Heading:" << headingLog.str() << std::endl;
   std::cout << std::endl;
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
