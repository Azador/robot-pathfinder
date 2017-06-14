/*
 *
 */

#include <vector>

namespace Pathfinder
{
  class Position
  {
    public:
      Position (double x, double y);

      bool operator== (const Position & other) const;

    private:
      double _x;
      double _y;
  };

  class MapObject
  {
    public:
      MapObject ();

      const std::vector<Position>& getPolygon () const;
      bool isClosed () const;

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
