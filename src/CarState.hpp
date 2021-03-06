/** Author: Mario Lüder
 * Date: 2019-03-24
 */

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

   CarState(const Coordinate2D & posXY, const Coordinate2D & posSD, const double yawInRad, const double speedInMps)
      : m_carPositionXY(posXY)
      , m_carPositionSD(posSD)
      , m_yawInRad(yawInRad)
      , m_speedInMps(speedInMps)
   {}

   CarState(const int laneIndex,
            const double s,
            const double desiredSpeedMps,
            const Track & track)
      : m_yawInRad(0.0)
      , m_speedInMps(desiredSpeedMps)
   {
      const double d = track.getLane().getD(laneIndex);
      const double normalizedS = track.sNormalize(s);
      m_carPositionSD.set(normalizedS, d);
      m_carPositionXY = track.convertToSDtoXY(m_carPositionSD);
   }

   double calcSdYaw(const Track & track) const
   {
      const Coordinate2D delta(cos(m_yawInRad), sin(m_yawInRad));
      const Coordinate2D nextPosXY = m_carPositionXY + delta;
      const Coordinate2D nextPosSD = track.convertXYtoSD(nextPosXY, m_yawInRad);
      const Coordinate2D diff = nextPosSD - m_carPositionSD;
      const double sdYaw = atan2(diff.getD(), diff.getS());
      return sdYaw;
   }


   double getSpeedInMetersPerSecond() const { return m_speedInMps; }
   double getSpeedInMilesPerHour() const { return m_speedInMps * 2.23694; }

   static double convertMetersPerSecondToMilesPerHour(const double mps) { return mps * 2.23694; }
   static double convertMilesPerHourToMetersPerSecond(const double mph) { return mph * 0.44704; }

   Coordinate2D m_carPositionXY;
   Coordinate2D m_carPositionSD;
   double m_yawInRad;

private:
   double m_speedInMps;
};

#endif // CARSTATE_HPP
