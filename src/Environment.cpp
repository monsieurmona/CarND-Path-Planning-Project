#include <cmath>

#include "Environment.hpp"

void Environment::setEnvironment(const std::vector<std::vector<double>> & sensorFusion, const CarState & egoCarState)
{
   const size_t nCars = sensorFusion.size();

   // as we do simple prediction for now, erase previous states
   m_vehicles.clear();

   for (size_t carIdx = 0; carIdx < nCars; ++carIdx)
   {
      const std::vector<double> & carData = sensorFusion[carIdx];

      const int id = static_cast<int>(carData[0]);
      const double x = carData[1];
      const double y = carData[2];
      const double vx = carData[3];
      const double vy = carData[4];
      const double s = carData[5];
      const double d = carData[6];

      const double v = sqrt(vx * vx + vy * vy);
      const double yaw = atan2(vy, vx);

      addVehicle(id, CarState(x, y, s, d, yaw, v));
   }

   calculateLaneSpeed(egoCarState);

   const int laneIdx = m_lane.getLaneIdx(egoCarState.m_carPositionSD.getD());

   if (laneIdx >= 0 || laneIdx < static_cast<int>(m_nLanes))
   {
      m_currentLaneIdx = static_cast<size_t>(laneIdx);
   }
}

void Environment::predict(const double_t horizon, const double updateInterval)
{
   for (Vehicle & vehicle : m_vehicles)
   {
      vehicle.predict(m_track, horizon, updateInterval);
   }
}

void Environment::calculateLaneSpeed(const CarState & egoCarState)
{
   const double maxSpeedInMps = CarState::convertMilesPerHourToMetersPerSecond(49.0);

   assert(m_nLanes > 0);
   m_lanesSpeedInMps.resize(static_cast<size_t>(m_nLanes), std::numeric_limits<double>::max());

   for (const Vehicle & vehicle : m_vehicles)
   {
      const Coordinate2D & vehicleSdCoordinates = vehicle.getCarState().m_carPositionSD;
      const double distance = m_track.sDistance(egoCarState.m_carPositionSD.getS(), vehicleSdCoordinates.getS());

      if (distance < 0)
      {
         // if a car is behind, we don't want to count it
         continue;
      }

      const int laneIdx = m_lane.getLaneIdx(vehicleSdCoordinates.getD());

      if (laneIdx < 0 || laneIdx >= static_cast<int>(m_nLanes))
      {
         // car shall be on ego vehicle side of the road
         continue;
      }

      double & laneSpeedInMps = m_lanesSpeedInMps[static_cast<size_t>(laneIdx)];
      // const double vehicleSpeedInMps = vehicle.getCarState().getSpeedInMetersPerSecond();

      const double safetyDistance = 5;
      const double maxPossibleVehicleSpeedInMps = CarState::convertMilesPerHourToMetersPerSecond(
               distance - safetyDistance);
      const double vehicleSpeedInMps = maxPossibleVehicleSpeedInMps > maxSpeedInMps ? maxSpeedInMps : maxPossibleVehicleSpeedInMps;

      if (laneSpeedInMps > vehicleSpeedInMps)
      {
         laneSpeedInMps = vehicleSpeedInMps;
      }
   }
}
