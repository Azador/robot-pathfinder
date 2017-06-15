/*
 *
 */

#include <vector>
#include <cstdint>

#include "robot-geometry.h"

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
