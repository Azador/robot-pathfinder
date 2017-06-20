/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-geometry.h"

namespace Pathfinder
{
  Position::Position ()
  : Eigen::Vector2d ()
  {
  }

  Position::Position (const Eigen::Vector2d & v)
  : Eigen::Vector2d (v)
  {
  }

  Position::Position (double x, double y)
  : Eigen::Vector2d (x, y)
  {
  }

  double Position::distance (const Position & other) const
  {
    Eigen::Vector2d diff = other - *this;
    return diff.norm ();
  }


  Line::Line (const Position & p1, const Position & p2)
  : _p1 (p1),
    _p2 (p2)
  {
  }

  Line::~Line ()
  {
  }

  Position Line::perpend (const Position & pos, double *t) const
  {
    Eigen::Vector2d dir = _p2 - _p1;
    Eigen::Vector2d r = pos - _p1;

    *t = (r.dot (dir)) / dir.squaredNorm ();

    return Position (_p1 + dir * *t);
  }

  double Line::distance (const Position & pos) const
  {
    double t;
    Position p = perpend (pos, &t);
    return p.distance (pos);
  }

  double Line::distance (const Position & pos, double *t) const
  {
    Position p = perpend (pos, t);
    return p.distance (pos);
  }

  const Position & Line::getPosition1 () const
  {
    return _p1;
  }

  const Position & Line::getPosition2 () const
  {
    return _p2;
  }

  Position Line::getDirection () const
  {
    return Position (_p2 - _p1);
  }

  LineSegment::LineSegment (const Position & p1, const Position & p2)
  : Line (p1, p2)
  {
  }

  LineSegment::~LineSegment ()
  {
  }

  Position LineSegment::perpend (const Position & pos, double *t) const
  {
    Position result = Line::perpend (pos, t);

    if (*t < 0)
      {
        *t = 0;
        result = getPosition1 ();
        return result;
      }

    if (*t > 1)
      {
        *t = 1;
        result = getPosition2 ();
        return result;
      }

    return result;
  }

  Transformation::Transformation ()
  : Eigen::Affine2d ()
  {
  }

  Transformation::Transformation (const Eigen::Affine2d & trafo)
  : Eigen::Affine2d (trafo)
  {
  }

  Transformation::Transformation (const Position & translation, double rotation, double scale)
  : Eigen::Affine2d ()
  {
    set (translation, rotation, scale);
  }

  void Transformation::set (const Position & translation, double rotation, double scale)
  {
    *this = Eigen::Scaling (scale) * Eigen::Translation2d (translation) * Eigen::Rotation2Dd (rotation);
  }

  Position Transformation::getTranslation () const
  {
    Position result;
    result.x () = (*this)(0, 2);
    result.y () = (*this)(1, 2);
    return result;
  }

  double Transformation::getRotation () const
  {
    Eigen::Matrix2d rotation;
    computeScalingRotation<Eigen::Matrix2d, Eigen::Matrix2d>
      (nullptr /*scaling*/, &rotation);

    Eigen::Rotation2Dd rot2 (0);
    rot2.fromRotationMatrix (rotation);
    return rot2.angle ();
  }

  double Transformation::getScale () const
  {
    Eigen::Matrix2d scaling;
    computeScalingRotation<Eigen::Matrix2d, Eigen::Matrix2d>
      (&scaling, nullptr /*rotation*/);
    return (scaling.diagonal ().operator() (0) + scaling.diagonal ().operator() (1)) / 2.0;
  }

  Position Transformation::transformPosition (const Position & pos) const
  {
    Position result;
    result.x () = ((*this)(0, 0) * pos.x () + (*this)(0, 1) * pos.y () + (*this)(0, 2)) / (*this)(2, 2);
    result.y () = ((*this)(1, 0) * pos.x () + (*this)(1, 1) * pos.y () + (*this)(1, 2)) / (*this)(2, 2);
    return result;
  }

  Position Transformation::rotatePosition (const Position & pos) const
  {
    Position result;
    result.x () = (*this)(0, 0) * pos.x () + (*this)(0, 1) * pos.y ();
    result.y () = (*this)(1, 0) * pos.x () + (*this)(1, 1) * pos.y ();
    return result;
  }
}
