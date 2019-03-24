/** Author: Mario Lüder
 * Date: 2019-03-24
 */
#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <limits>
#include <vector>

#include "Track.hpp"
#include "Vehicle.hpp"

class Environment
{
public:
   Environment(size_t nLanes, const Lane & lane,
               const std::vector<double> &maps_s,
               const std::vector<double> &maps_x,
               const std::vector<double> &maps_y)
      : m_nLanes{nLanes}
      , m_lane{lane}
      , m_currentLaneIdx{0}
      , m_track(lane, maps_s, maps_x, maps_y)
   {}

   struct DistanceSpeed
   {
      DistanceSpeed(const double distanceInM, const double speedInMps)
         : m_distanceInM{distanceInM}, m_speedInMps{speedInMps} {}
      double m_distanceInM;
      double m_speedInMps;
   };

   struct ClosestCarsInLane
   {
      DistanceSpeed m_front = {std::numeric_limits<double>::max(), 0};
      DistanceSpeed m_back = {std::numeric_limits<double>::max(), 0};
   };

   void setEnvironment(const std::vector<std::vector<double>> & sensorFusion, const CarState & egoCarState);

   inline const std::vector<double_t> & getLanesSpeedInMps() const { return m_lanesSpeedInMps; }
   inline size_t getCurrentLaneIdx() const { return m_currentLaneIdx; }
   inline size_t getLaneCount() const { return m_nLanes; }
   inline double getLaneOffset(const size_t laneIdx) const { return m_lane.getD(static_cast<int>(laneIdx)); }
   inline const Track & getTrack() const { return m_track; }
   inline double getDistanceToNextCar( const size_t laneIdx ) const { return m_nextCarDistance[laneIdx]; }

   ClosestCarsInLane getClosestCarsInLane(const int laneIdx) const { return m_closestCarInLanes[static_cast<size_t>(laneIdx)]; }

   double predictLaneSpeed(const CarState & egoCarState, const size_t laneIdx, const double t) const;
   const double m_safefyDistanceFactor = 0.7;
   const double m_maxSpeedInMps = CarState::convertMilesPerHourToMetersPerSecond(49.0);

private:
   void calculateLaneSpeed(const CarState & egoCarState);
   ClosestCarsInLane getClosestCarsInLane(const CarState &egoCarPosition, const int laneIdx) const;

   inline void addVehicle(const int id, const CarState & carState) { m_vehicles.emplace_back(Vehicle(id, carState)); }

   size_t m_nLanes;
   Lane m_lane;

   std::vector<Vehicle> m_vehicles;
   std::vector<double_t> m_lanesSpeedInMps;
   std::vector<double_t> m_nextCarDistance;
   size_t m_currentLaneIdx;
   Track m_track;
   std::vector<ClosestCarsInLane> m_closestCarInLanes;
};

#endif // ENVIRONMENT_HPP
