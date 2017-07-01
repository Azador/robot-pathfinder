/*
 *
 */

#include "robot-mapwidget.h"

namespace Pathfinder
{
  MapScene::MapScene (QObject * parent, Map * map)
  : QGraphicsScene (parent),
    _map (map)
  {
    updateScene ();
  }

  void MapScene::updateScene ()
  {
    const std::vector<MapObject> & objects = _map->getObjects ();

    for (uint32_t i=0; i < objects.size (); ++i)
      {
	const std::vector<Position,Eigen::aligned_allocator<Position>>& poly = objects[i].getPolygon ();

	for (uint32_t j=1; j < poly.size (); ++j)
	  addLine (poly[j-1].x (), -poly[j-1].y (), poly[j].x (), -poly[j].y (), QPen (Qt::black));
      }
  }
}
