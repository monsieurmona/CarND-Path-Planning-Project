#include <cmath>

#include "Environment.hpp"

void Environment::setEnvironment(const std::vector<std::vector<double>> & sensorFusion)
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
}

void Environment::predict(const Track & track, const double_t horizon, const double updateInterval)
{
   for (Vehicle & vehicle : m_vehicles)
   {
      vehicle.predict(track, horizon, updateInterval);
   }
}
