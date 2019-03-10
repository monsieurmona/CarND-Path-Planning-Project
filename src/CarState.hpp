#ifndef CARSTATE_HPP
#define CARSTATE_HPP

#include "Coordinate2D.hpp"
#include "Track.hpp"
#include "Lane.hpp"

class CarState
{
public:
   CarState(const double x, const double y, const double s, const double d, const double yawInRad, const double speedInMps)
      : m_carPositionXY(x, y)
      , m_carPositionSD(s, d)
      , m_yawInRad(yawInRad)
      , m_speedInMps(speedInMps)
   {}

   CarState(const double laneWidth,
            const int laneIndex,
            const double distance,
            const double desiredSpeedMps,
            const Track & track)
      : m_yawInRad(0.0)
      , m_speedInMps(desiredSpeedMps)
   {
      const double s = distance;
      const double d = Lane(laneWidth).getD(laneIndex);
      m_carPositionSD.set(s, d);
      m_carPositionXY = track.convertToSDtoXY(m_carPositionSD);
   }

   Coordinate2D m_carPositionXY;
   Coordinate2D m_carPositionSD;
   double m_yawInRad;
   double m_speedInMps;
};

#endif // CARSTATE_HPP
