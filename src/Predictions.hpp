#ifndef PREDICTIONS_HPP
#define PREDICTIONS_HPP

#include <vector>

#include "Prediction.hpp"

class Predictions
{
public:
   void predict(const Track & track, const CarState & state, const double horizon, const double updateInterval);

   CarState getLinearPredictedCarState(const Track & track, const CarState & state, const double horizon);

private:
   std::vector<std::unique_ptr<Prediction>> m_predictions;
};

#endif // PREDICTIONS_HPP
