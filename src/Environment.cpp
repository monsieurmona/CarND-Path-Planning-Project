#include <cmath>

#include "Environment.hpp"

void Environment::setEnvironment(const std::vector<std::vector<double>> & sensorFusion, const Track & track, const CarState & egoCarState)
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

   calculateLaneSpeed(track, egoCarState);
}

void Environment::predict(const Track & track, const double_t horizon, const double updateInterval)
{
   for (Vehicle & vehicle : m_vehicles)
   {
      vehicle.predict(track, horizon, updateInterval);
   }
}

void Environment::calculateLaneSpeed(const Track & track, const CarState & egoCarState)
{
   assert(m_nLanes > 0);
   m_lanesSpeedInMps.resize(static_cast<size_t>(m_nLanes), std::numeric_limits<double>::max());

   for (const Vehicle & vehicle : m_vehicles)
   {
      const Coordinate2D & vehicleSdCoordinates = vehicle.getCarState().m_carPositionSD;
      const double distance = track.sDistance(egoCarState.m_carPositionSD.getS(), vehicleSdCoordinates.getS());
      if (distance < 0)
      {
         // if a car is behind, we don't want to count it
         continue;
      }

      int laneIdx = m_lane.getLaneIdx(vehicleSdCoordinates.getD());

      if (laneIdx < 0 || laneIdx >= m_nLanes)
      {
         // car shall be on ego vehicle side
         continue;
      }

      double & laneSpeedInMps = m_lanesSpeedInMps[static_cast<size_t>(laneIdx)];
      const double vehicleSpeedInMps = vehicle.getCarState().getSpeedInMetersPerSecond();

      if (laneSpeedInMps > vehicleSpeedInMps)
      {
         laneSpeedInMps = vehicleSpeedInMps;
      }
   }
}
