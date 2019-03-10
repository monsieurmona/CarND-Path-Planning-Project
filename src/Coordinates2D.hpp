#ifndef COORDINATES2D_HPP
#define COORDINATES2D_HPP

#include <array>
#include <limits>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "Coordinate2D.hpp"

template<size_t N>
class Coordinates2D
{
public:
   // I would have liked to use std::array instead, as the size is fixes
   // but unfortunately the spline library takes vectors.
   // Copying the data would result in more overhead.
   //
   // Still, I avoid reallocation
   using Axis = std::vector<double>;

   Coordinates2D()
   {
      m_x.resize(N);
      m_y.resize(N);
   }

   void simpleTrajectory(Coordinate2D position, const double interval, const double rotationInRad)
   {

      for (size_t i = m_length; i < N; ++i)
      {
         const Coordinate2D nextCoordinate = position.move(interval, rotationInRad + (i + 1) * (M_PI / (N * 2)));
         m_x[i] = nextCoordinate.getX();
         m_y[i] = nextCoordinate.getY();
         position = nextCoordinate;
      }

      m_length = N;
   }

   void insert(const std::vector<double> & xCoordinates, const std::vector<double> & yCoordinates)
   {
      m_length = 0;

      size_t maxLength = xCoordinates.size() < N ? xCoordinates.size() : N;
      maxLength = yCoordinates.size() < maxLength ? yCoordinates.size() : maxLength;

      std::copy(m_x, xCoordinates.begin(), xCoordinates.begin() + maxLength);
      std::copy(m_y, yCoordinates.begin(), yCoordinates.begin() + maxLength);

      m_length = maxLength;
   }

   size_t getLength() const { return m_length; }

   Coordinate2D getLast() const
   {
      if (m_length > 0)
      {
         const size_t lastIdx = m_length - 1;
         return Coordinate2D(m_x[lastIdx], m_y[lastIdx]);
      }
      else
      {
         return Coordinate2D(0,0);
      }
   }

   Coordinate2D operator[](const size_t i) const
   {
      return Coordinate2D(m_x[i], m_y[i]);
   }

   Coordinate2D at(const size_t i) const
   {
      return Coordinate2D(m_x[i], m_y[i]);
   }

   void set(const size_t index, const Coordinate2D & coordinate)
   {
      m_x[index] = coordinate.getX();
      m_y[index] = coordinate.getY();
   }

   void push_back(const Coordinate2D & coordinate2D)
   {
      assert(m_length < N);
      m_x[m_length] = coordinate2D.getX();
      m_y[m_length] = coordinate2D.getY();
      m_length++;
   }

   double getLastAngleInRad()
   {
      double angle = std::numeric_limits<double>::max();

      if (m_length > 1)
      {
         const size_t lastIdx = m_length - 1;
         const size_t lastButOneIdx = m_length - 2;
         const double x1 = m_x[lastIdx];
         const double x2 = m_x[lastButOneIdx];
         const double y1 = m_y[lastIdx];
         const double y2 = m_y[lastButOneIdx];
         angle = atan2(y1 - y2, x1 - x2);
      }

      return angle;
   }

   void transformToCoordinateSystem(const Coordinate2D  & referenceCoordinates, const double referenceYaw)
   {
      for (size_t i = 0; i < m_length; ++i)
      {
         Coordinate2D point = at(i);
         point.transformToCoordinateSystem(referenceCoordinates, referenceYaw);
         set(i, point);
      }
   }

   const Axis & getX() const {return m_x;}
   const Axis & getY() const {return m_y;}

   const Axis & getS() const {return m_x;}
   const Axis & getD() const {return m_y;}
private:
   Axis m_x; // respectivly s
   Axis m_y; // respectivly d

   size_t m_length = 0;
};

#endif // COORDINATES2D_HPP
