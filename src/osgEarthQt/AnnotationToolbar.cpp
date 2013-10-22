/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthQt/AnnotationToolbar>

#include <osgEarthQt/AnnotationDialogs>
#include <osgEarthQt/Common>
#include <osgEarthQt/DataManager>

#include <QAction>
#include <QToolBar>

using namespace osgEarth;
using namespace osgEarth::QtGui;


//---------------------------------------------------------------------------

AnnotationToolbar::AnnotationToolbar(osg::Group* root, osgEarth::MapNode* mapNode, DataManager* dm, QWidget* parent)
: QToolBar(tr("_annotation_toolbar"), parent), _root(root), _mapNode(mapNode), _manager(dm)
{
  initialize();
}

void
AnnotationToolbar::initialize()
{
  //setObjectName(tr("ANNOTATION_TOOLBAR"));
	setIconSize(QSize(24, 24));

  createDefaultActions();
}

void
AnnotationToolbar::createDefaultActions()
{
  QObject* actionParent = parent();
  if (!actionParent)
    actionParent = this;

  _addMarker = new QAction(QIcon(":/images/add_marker_bg.png"), tr(""), actionParent);
  _addMarker->setToolTip(tr("Add a marker"));
  connect(_addMarker, SIGNAL(triggered()), this, SLOT(addMarkerAnnotation()));
  addAction(_addMarker);

  _addPath = new QAction(QIcon(":/images/draw_line_bg.png"), tr(""), actionParent);
  _addPath->setToolTip(tr("Draw a path"));
  connect(_addPath, SIGNAL(triggered()), this, SLOT(addPathAnnotation()));
  addAction(_addPath);

  _addPoly = new QAction(QIcon(":/images/draw_poly_bg.png"), tr(""), actionParent);
  _addPoly->setToolTip(tr("Draw a polygon"));
  connect(_addPoly, SIGNAL(triggered()), this, SLOT(addPolyAnnotation()));
  addAction(_addPoly);

  _addEllipse = new QAction(QIcon(":/images/draw_circle_bg.png"), tr(""), actionParent);
  _addEllipse->setToolTip(tr("Draw an ellipse"));
  connect(_addEllipse, SIGNAL(triggered()), this, SLOT(addEllipseAnnotation()));
  addAction(_addEllipse);
}

void AnnotationToolbar::setActiveView(osgViewer::View* view)
{
  removeViews();
  addView(view);
}

void AnnotationToolbar::setActiveViews(const ViewVector& views)
{
  removeViews();

  for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
    addView(*it);
}

void AnnotationToolbar::addView(osgViewer::View* view)
{
  _views.push_back(view);
}

void AnnotationToolbar::removeViews()
{
  _views.clear();
}

void
AnnotationToolbar::addMarkerAnnotation()
{
  _activeDialog = new osgEarth::QtGui::AddMarkerDialog(_root, _mapNode, _views);

  this->setEnabled(false);

  connect(_activeDialog, SIGNAL(finished(int)), this, SLOT(onAddFinished(int)));

  _activeDialog->setWindowTitle(tr("New marker"));
  _activeDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
  _activeDialog->setAttribute(Qt::WA_DeleteOnClose);
  _activeDialog->show();
}

void
AnnotationToolbar::addPathAnnotation()
{
  _activeDialog = new osgEarth::QtGui::AddPathDialog(_root, _mapNode, _views);

  this->setEnabled(false);

  connect(_activeDialog, SIGNAL(finished(int)), this, SLOT(onAddFinished(int)));

  _activeDialog->setWindowTitle(tr("New path"));
  _activeDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
  _activeDialog->setAttribute(Qt::WA_DeleteOnClose);
  _activeDialog->show();
}

void
AnnotationToolbar::addPolyAnnotation()
{
  _activeDialog = new osgEarth::QtGui::AddPolygonDialog(_root, _mapNode, _views);

  this->setEnabled(false);

  connect(_activeDialog, SIGNAL(finished(int)), this, SLOT(onAddFinished(int)));

  _activeDialog->setWindowTitle(tr("New polygon"));
  _activeDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
  _activeDialog->setAttribute(Qt::WA_DeleteOnClose);
  _activeDialog->show();
}

void
AnnotationToolbar::addEllipseAnnotation()
{
  _activeDialog = new osgEarth::QtGui::AddEllipseDialog(_root, _mapNode, _views);

  this->setEnabled(false);

  connect(_activeDialog, SIGNAL(finished(int)), this, SLOT(onAddFinished(int)));

  _activeDialog->setWindowTitle(tr("New ellipse"));
  _activeDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
  _activeDialog->setAttribute(Qt::WA_DeleteOnClose);
  _activeDialog->show();
}

void AnnotationToolbar::onAddFinished(int result)
{
  this->setEnabled(true);

  if (result == QDialog::Accepted)
  {
    if (_root.valid())
    {
      osgEarth::Annotation::AnnotationNode* annotation = _activeDialog->getAnnotation();
      if (annotation)
      {
        if (_manager.valid())
          _manager->addAnnotation(annotation, _root);
        else
          _root->addChild(annotation);
      }
    }
  }
}
