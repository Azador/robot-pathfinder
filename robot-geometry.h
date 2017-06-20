/*
 *
 */

#include <vector>
#include <cstdint>
#include <cmath>
#include <Eigen/Dense>

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
      bool adjust (const std::vector<Position> & positions);

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
  bool PolynomCurve<degree>::adjust (const std::vector<Position> & positions)
  {
    Eigen::MatrixXd a;
    return false;
  }
}
