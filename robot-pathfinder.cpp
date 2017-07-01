/*
 *
 */

#include <QApplication>
#include <QCommandLineParser>

#include "robot-pathfinder.h"

namespace Pathfinder
{
  void testModule ()
  {
    PolynomCurve<2>::test ();
  }

  MainWindow::MainWindow ()
  : QMainWindow (),
    _map_view (0),
    _map_scene (0),
    _map ()
  {
    MapObject map_obj (0.2);
    for (uint32_t i=0; i <= 20; ++i)
      {
	Position p (cos (i*3.1415/10.0)*14.7, sin (i*3.1415/10.0)*14.7);
	map_obj.appendPoint (p);
      }

    _map.addObject (map_obj);

    map_obj.clear ();
    map_obj.appendPoint (Position (-6,  5));
    map_obj.appendPoint (Position (-4,  5));

    _map.addObject (map_obj);

    map_obj.clear ();
    map_obj.appendPoint (Position (6,  5));
    map_obj.appendPoint (Position (4,  5));

    _map.addObject (map_obj);

    map_obj.clear ();
    map_obj.appendPoint (Position (0,  3));
    map_obj.appendPoint (Position (0,  -1));

    _map.addObject (map_obj);

    map_obj.clear ();
    map_obj.appendPoint (Position (-6,  -4));
    map_obj.appendPoint (Position (-4,  -6));
    map_obj.appendPoint (Position (4,  -6));
    map_obj.appendPoint (Position (6,  -4));

    _map.addObject (map_obj);


    _map_scene = new MapScene (this, &_map);
    //_map_scene->addText ("Hello, world!");
    //_map_scene->addLine (5,5,10,10);
    _map_view = new QGraphicsView (_map_scene, this);
    setCentralWidget (_map_view);
    _map_view->show ();
  }
}

void qInitResources_application ()
{
}

int main (int argc, char** argv)
{
  //std::cerr << "Qt Version: " << QT_VERSION_STR << std::endl;
  Q_INIT_RESOURCE (application);

  QApplication app (argc, argv);
  //QCoreApplication::setOrganizationName ("QtProject");
  //QCoreApplication::setApplicationName ("Application Example");
  QCoreApplication::setApplicationVersion (QT_VERSION_STR);

  QCommandLineParser parser;
  //parser.setApplicationDescription (QCoreApplication::applicationName ());
  parser.addHelpOption ();
  parser.addVersionOption ();
  parser.addPositionalArgument ("file", "The file to open.");
  parser.process (app);

  Pathfinder::MainWindow main_win;
//  if (!parser.positionalArguments ().isEmpty ())
//      mainWin.loadFile (parser.positionalArguments ().first ());
  main_win.show ();
  return app.exec ();
  //Pathfinder::testModule ();
}
