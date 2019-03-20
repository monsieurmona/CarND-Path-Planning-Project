#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

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

   void setEnvironment(const std::vector<std::vector<double>> & sensorFusion, const CarState & egoCarState);
   void predict(const double_t horizon, const double updateInterval);

   inline const std::vector<double_t> & getLanesSpeedInMps() const { return m_lanesSpeedInMps; }
   inline size_t getCurrentLaneIdx() const { return m_currentLaneIdx; }
   inline size_t getLaneCount() const { return m_nLanes; }
   inline double getLaneOffset(const size_t laneIdx) const { return m_lane.getD(laneIdx); }
   inline const Track & getTrack() const { return m_track; }

private:
   void calculateLaneSpeed(const CarState & egoCarState);
   inline void addVehicle(const int id, const CarState & carState) { m_vehicles.emplace_back(Vehicle(id, carState)); }
   size_t m_nLanes;
   Lane m_lane;

   std::vector<Vehicle> m_vehicles;
   std::vector<double_t> m_lanesSpeedInMps;
   size_t m_currentLaneIdx;
   Track m_track;
};

#endif // ENVIRONMENT_HPP
