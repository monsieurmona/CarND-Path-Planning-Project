#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include "Trajectory.hpp"

class Prediction
{
public:
   void calculate(const CarState & carState, const CarState & targetCarState, const Track & track, const double updateInterval)
   {
      (void)m_trajetory.calculateLinearPath(carState, targetCarState, track, updateInterval);
   }
private:
   // double m_probability;
   Trajectory m_trajetory;
};

#endif // PREDICTION_HPP
