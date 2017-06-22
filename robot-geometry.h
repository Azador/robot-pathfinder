/*
 *
 */

#include <vector>
#include <map>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#if __cplusplus >= 201700L
#  include <optional>
#else
namespace std
{
  template<class T>
  class optional
  {
    public:
      optional () : _d (0) {};
      ~optional () { reset (); };

      void reset () { if (_d != 0) { delete _d; _d = 0;} };
      T & emplace () { if (_d == 0) new T; return *_d; };

      T * operator-> () { return _d; };
      const T * operator-> () const { return _d; };
      T & operator* () { return *_d; };
      const T & operator* () const { return *_d; };
      bool has_value () const { return _d != 0; };

      const T & operator= (const T & v)
      {
        emplace ();
        *_d = v;
        return *_d;
      }

    private:
      T * _d;
  };
}
#endif

namespace Pathfinder
{
  class Position : public Eigen::Vector2d
  {
    public:
      Position ();
      Position (const Eigen::Vector2d & v);
      Position (double x, double y);

      double & x ()             { return operator[] (0); }
      const double & x () const { return operator[] (0); }

      double & y ()             { return operator[] (1); }
      const double & y () const { return operator[] (1); }

      double distance (const Position & other) const;
  };

  class Line
  {
    public:
      Line (const Position & p1, const Position & p2);
      virtual ~Line ();

      virtual Position perpend (const Position & pos, double *t) const;
      virtual double distance (const Position & pos) const;
      virtual double distance (const Position & pos, double *t) const;

      const Position & getPosition1 () const;
      const Position & getPosition2 () const;
      Position getDirection () const;

    private:
      Position _p1;
      Position _p2;
  };

  class LineSegment : public Line
  {
    public:
      LineSegment (const Position & p1, const Position & p2);
      virtual ~LineSegment ();

      virtual Position perpend (const Position & pos, double *t) const;
  };

  template<uint32_t degree>
  class PolynomCurve
  {
    public:
      PolynomCurve ();

      Position get (double t) const;
      std::optional<double> adjust (const std::vector<Position,Eigen::aligned_allocator<Position>> & positions);
      Position projectOnCurve (const Position & pos, double t_min = -1.0, double t_max = 1.0) const;

      static void test ();

    private:
      Eigen::Matrix<Position, degree+1, 1> _coeff;
  };

  class Transformation : public Eigen::Affine2d
  {
    public:
      Transformation ();
      Transformation (const Eigen::Affine2d & trafo);
      Transformation (const Position & translation, double rotation, double scale);

      void set (const Position & translation, double rotation, double scale);
      Position getTranslation () const;
      double getRotation () const;
      double getScale () const;

      Position transformPosition (const Position & pos) const;
      Position rotatePosition (const Position & pos) const;
  };



  //====================================================================
  //
  // Implementation of template classes
  //

  template<uint32_t degree>
  PolynomCurve<degree>::PolynomCurve ()
  : _coeff ()
  {
  }

  template<uint32_t degree>
  Position PolynomCurve<degree>::get (double t) const
  {
    Position result (0, 0);
    for (uint32_t i=0; i <= degree; ++i)
      result += _coeff[i] * std::pow (t, i);

    return result;
  }

  template<uint32_t degree>
  std::optional<double> PolynomCurve<degree>::adjust
  (const std::vector<Position,Eigen::aligned_allocator<Position>> & positions)
  {
    std::optional<double> residual;
    if (positions.size () <= degree)
      return residual;

    Eigen::Matrix<double, Eigen::Dynamic, degree+1> a;
    a.resize (positions.size (), degree+1);
    Eigen::VectorXd vx (positions.size ());
    Eigen::VectorXd vy (positions.size ());

    std::vector<double> t (positions.size ());
    t[0] = 0;
    for (uint32_t i=1; i < positions.size (); ++i)
      t[i] = t[i-1] + positions[i].distance (positions[i-1]);

    if (t.back() <= 0.0)
      return residual;

    for (double& ti: t)
      ti = 2.0 * ti / t.back () - 1.0;

    for (uint32_t i=0; i < t.size (); ++i)
      std::cerr << "t[" << i << "] = " << t[i] << std::endl;

    for (uint32_t i=0; i < positions.size (); ++i)
      {
        double ti = t[i];

        for (uint32_t j=0; j <= degree; ++j)
          a (i, j) = ::pow (ti, j);

        vx[i] = positions[i].x ();
        vy[i] = positions[i].y ();
      }

    Eigen::Matrix<double, degree+1, degree+1> n = a.transpose () * a;
    Eigen::Matrix<double, degree+1, 1> lx = a.transpose () * vx;
    Eigen::Matrix<double, degree+1, 1> ly = a.transpose () * vy;

    Eigen::LDLT<Eigen::Matrix<double, degree+1, degree+1>> cholesky (n);
    Eigen::Matrix<double, degree+1, 1> xx = cholesky.solve (lx);
    Eigen::Matrix<double, degree+1, 1> xy = cholesky.solve (ly);

    for (uint32_t i=0; i <= degree; ++i)
      _coeff[i] = Position (xx[i], xy[i]);

    residual = 0;

    for (uint32_t i=0; i < positions.size (); ++i)
      {
        Position p = get (t[i]);
        *residual += fabs (p.x () - positions[i].x ()) + fabs (p.y () - positions[i].y ());
      }

    *residual /= positions.size () * 2;

    return residual;
  }

  template<uint32_t degree>
  Position PolynomCurve<degree>::projectOnCurve (const Position & pos, double t_min, double t_max) const
  {
    // First, try to find a good starting point for the adjustment.
    // Probe n positions from t_min to t_max (n==32):
    double step = (t_max - t_min) / 32;
    double best_t = t_min;
    double best_dist = get (t_min).distance (pos);

    for (uint32_t i=1; i <= 32; ++i)
      {
        double t = t_min + i * step;
        double dist = get (t).distance (pos);

        if (dist < best_dist)
          {
            best_dist = dist;
            best_t = t;
          }
      }

    // Now try to iteratively minimize the error
    uint32_t iter = 0;
    double x;
    do
      {
        Position deriv (0, 0);
        for (uint32_t i=1; i <= degree; ++i)
          deriv += _coeff[i] * pow (best_t, i-1) * i;
        Position dist = get (best_t) - pos;

        double n = deriv.norm ();
        double l = deriv.transpose () * dist;
        x = l/n;
        best_t += x;
        if (best_t > t_max)
          {
            best_t = t_max;
            break;
          }
        else if (best_t < t_min)
          {
            best_t = t_min;
            break;
          }

        ++iter;
      }
    while (x > 1e-12 && iter < 20);

    return get (best_t);
  }

  template<uint32_t degree>
  void PolynomCurve<degree>::test ()
  {
    if (degree > 3)
      return;

    std::vector<Position,Eigen::aligned_allocator<Position>> pos;
    pos.push_back (Position (0, 0));
    pos.push_back (Position (1, 0.5));
    pos.push_back (Position (2, 0.75));
    pos.push_back (Position (3, 1.5));

    PolynomCurve p;
    std::optional<double> residual = p.adjust (pos);
    if (!residual.has_value ())
      {
        std::cerr << "Adjust of PolynomCurve degree " << degree << " failed" << std::endl;
        return;
      }

    std::cerr << "PolynomCurve degree " << degree << ":" << std::endl;
    for (uint32_t i=0; i <= degree; ++i)
      {
        if (i > 0)
          std::cerr << ", ";
        std::cerr << "(" << p._coeff[i].x () << ", " << p._coeff[i].y () << ")";
      }
    std::cerr << std::endl;
    for (double t=-1.0; t <= 1.0; t += 0.125)
      {
        Position pt = p.get (t);
        std::cerr << t << ": " << pt.x () << ", " << pt.y () << std::endl;
      }
  }
}
