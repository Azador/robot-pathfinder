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
  }
}
