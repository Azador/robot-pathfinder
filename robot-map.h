/*
 *
 */

#include <vector>
#include <cstdint>

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

      T * operator-> () { return _d; };
      const T * operator-> () const { return _d; };
      T & operator* () { return *_d; };
      const T & operator* () const { return *_d; };
      bool has_value () const { return _d != 0; };

      const T & operator= (const T & v)
      {
        if (_d == 0) _d = new T;
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

  class MapObject
  {
    public:
      MapObject ();

      const std::vector<Position>& getPolygon () const;
      bool isClosed () const;
      bool isEmpty () const;
      bool join (const MapObject & other, double max_dist);
      std::optional<std::pair<double, uint32_t> > findClosestPosition (const Position & pos) const;

    private:
      std::vector<Position> _poly;
  };

  class Map
  {
    public:
      Map ();

    private:
      std::vector<MapObject> _objects;
  };
}
