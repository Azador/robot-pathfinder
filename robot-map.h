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
  class MapObject
  {
    public:
      MapObject (double min_point_distance);

      const std::vector<Position>& getPolygon () const;
      bool isClosed () const;
      bool isEmpty () const;
      bool join (const MapObject & other, double max_dist);
      bool addPoint (const Position & point, double max_dist);
      void smooth (double max_deviation, uint32_t filter_size);
      void makeEquidistant (double max_dist, uint32_t min_points, double max_deviation);
      void convexHull ();

      struct FindResult
      {
          double distance;
          uint32_t point_index;
          double   fraction_to_next_point;
      };
      std::optional<FindResult> findClosestPosition (const Position & pos) const;

    private:
      double _min_point_distance;
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
