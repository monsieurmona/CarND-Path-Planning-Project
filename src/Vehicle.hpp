#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <vector>
#include <iostream>

#include "CarState.hpp"
#include "Predictions.hpp"

class Vehicle
{
public:
   Vehicle(const int id, const CarState & state) : m_id(id), m_state(state) {}

   void predict(const Track & track, const double_t horizon, const double updateInterval)
   {
      std::cout << "Car id:" << m_id << std::endl;
      m_predictions.predict(track, m_state, horizon, updateInterval);
   }

private:
   int m_id;
   CarState m_state;
   Predictions m_predictions;
};

#endif // VEHICLE_HPP
