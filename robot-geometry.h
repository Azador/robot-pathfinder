/*
 *
 */

#include <vector>
#include <cstdint>

namespace Pathfinder
{
  class Position
  {
    public:
      Position ();
      Position (double x, double y);

      double distance (const Position & other) const;
      bool operator== (const Position & other) const;

      Position operator+ (const Position & other) const;
      Position operator- (const Position & other) const;
      Position operator* (double factor) const;
      Position & operator+= (const Position & other);
      Position & operator-= (const Position & other);
      Position & operator*= (double factor);

      double operator* (const Position & other) const;

      double length2 () const;

    private:
      double _x;
      double _y;
  };

  class Line
  {
    public:
      Line (const Position & p1, const Position & p2);
      virtual ~Line ();

      virtual Position perpend (const Position & pos, double *t) const;
      virtual double distance (const Position & pos) const;

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
}
