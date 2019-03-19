#include <iostream>

#include "Predictions.hpp"

void Predictions::predict(const Track & track, const CarState & state, const double horizon, const double updateInterval)
{
   //const double sdYaw = calcSdYaw(track, state);
   //std::cout << " sdYaw:" << sdYaw << std::endl;
   CarState predictedCarState = getLinearPredictedCarState(track, state, horizon);
   std::unique_ptr<Prediction> predictionPtr = std::unique_ptr<Prediction>(new Prediction());
   predictionPtr->calculate(state, predictedCarState, track, updateInterval);

   m_predictions.clear();
   m_predictions.push_back(std::move(predictionPtr));
}

CarState Predictions::getLinearPredictedCarState(const Track & track, const CarState & state, const double horizon)
{
   const Coordinate2D delta(cos(state.m_yawInRad), sin(state.m_yawInRad));
   const Coordinate2D nextPosXY = state.m_carPositionXY + (delta * horizon);
   const Coordinate2D nextPosSD = track.convertXYtoSD(nextPosXY, state.m_yawInRad);
   CarState newState(nextPosXY, nextPosSD, state.m_yawInRad, state.getSpeedInMetersPerSecond());
   return newState;
}
