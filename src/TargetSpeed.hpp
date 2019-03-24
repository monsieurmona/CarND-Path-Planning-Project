/** Author: Mario Lüder
 * Date: 2019-03-24
 */

#ifndef TARGETSPEED_HPP
#define TARGETSPEED_HPP

class TargetSpeed
{
public:
   TargetSpeed(const double speedInMps, const double distance)
      : m_speedInMps{speedInMps}, m_distance{distance} {}
   double m_speedInMps;
   double m_distance;
};

#endif // TARGETSPEED_HPP
