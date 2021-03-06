/*
 *
 */

#include <QtWidgets>

#include "robot-mapwidget.h"

namespace Pathfinder
{
  class MainWindow : public QMainWindow
  {
      Q_OBJECT;

  public:
      MainWindow ();

      //void loadFile(const QString &fileName);

  protected:
//      void closeEvent (QCloseEvent *event) Q_DECL_OVERRIDE;

//  private slots:
//      void newFile ();
//      void open ();
//      bool save ();
//      bool saveAs ();
//      void about ();
//      void documentWasModified ();
//  #ifndef QT_NO_SESSIONMANAGER
//      void commitData (QSessionManager &);
//  #endif

  private:
//      void createActions ();
//      void createStatusBar ();
//      void readSettings ();
//      void writeSettings ();
//      bool maybeSave ();
//      bool saveFile (const QString &fileName);
//      void setCurrentFile (const QString &fileName);
//      QString strippedName (const QString &fullFileName);
//
//      QPlainTextEdit * textEdit;
//      QString curFile;

      QGraphicsView * _map_view;
      MapScene * _map_scene;
      Map _map;
  };
}
