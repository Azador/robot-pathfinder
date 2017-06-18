/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-geometry.h"

namespace Pathfinder
{
  Position::Position ()
  : Vector ()
  {
  }

  Position::Position (const Vector<2> & v)
  : Vector (v)
  {
  }

  Position::Position (double x, double y)
  : Vector ({x, y})
  {
  }

  double Position::distance (const Position & other) const
  {
    Vector diff = other - *this;
    return sqrt (diff*diff);
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
    Vector<2> dir = _p2 - _p1;
    Vector<2> r = pos - _p1;

    *t = (r*dir) / dir.length2 ();

    return _p1 + dir * *t;
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
    return _p2 - _p1;
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
  : Matrix ()
  {
  }

  Transformation::Transformation (const Position & translation, double rotation, double scale)
  : Matrix ()
  {
    set (translation, rotation, scale);
  }

  void Transformation::set (const Position & translation, double rotation, double scale)
  {
    (*this)[0][0] =  cos (rotation);
    (*this)[0][1] = -sin (rotation);
    (*this)[1][0] =  sin (rotation);
    (*this)[1][1] =  cos (rotation);
    (*this)[0][2] = translation.x ();
    (*this)[1][2] = translation.y ();
    (*this)[2][0] = (*this)[2][1] = 0.0;
    (*this)[2][2] = 1.0/scale;
  }

  Position Transformation::getTranslation () const
  {
    Position result;
    result.x () = (*this)[0][2];
    result.y () = (*this)[1][2];
    return result;
  }

  double Transformation::getRotation () const
  {
    return atan2 ((*this)[1][0], (*this)[0][0]);
  }

  double Transformation::getScale () const
  {
    return 1.0 / (*this)[2][2];
  }

  Position Transformation::transformPosition (const Position & pos) const
  {
    Position result;
    result.x () = ((*this)[0][0] * pos.x () + (*this)[0][1] * pos.y () + (*this)[0][2]) / (*this)[2][2];
    result.y () = ((*this)[1][0] * pos.x () + (*this)[1][1] * pos.y () + (*this)[1][2]) / (*this)[2][2];
    return result;
  }

  Position Transformation::rotatePosition (const Position & pos) const
  {
    Position result;
    result.x () = (*this)[0][0] * pos.x () + (*this)[0][1] * pos.y ();
    result.y () = (*this)[1][0] * pos.x () + (*this)[1][1] * pos.y ();
    return result;
  }
}
