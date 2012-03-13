/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthQt/TerrainProfileWidget>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>
#include <osgEarthQt/TerrainProfileGraph>

#include <osgEarth/Map>
#include <osgEarthUtil/TerrainProfile>

#include <QAction>
#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolBar>
#include <QVBoxLayout>

using namespace osgEarth;
using namespace osgEarth::QtGui;


//---------------------------------------------------------------------------

TerrainProfileWidget::TerrainProfileWidget(osg::Group* root, osgEarth::MapNode* mapNode)
: _root(root), _mapNode(mapNode)
{
  _calculator = new osgEarth::Util::TerrainProfileCalculator(mapNode);
  _guiHandler = new TerrainProfileMouseHandler(_mapNode.get(), _root.get(), this);

  initialize();

  _calculator->addChangedCallback(this);
}

TerrainProfileWidget::~TerrainProfileWidget()
{
}

void TerrainProfileWidget::initialize()
{
  QVBoxLayout* vStack = new QVBoxLayout();
  vStack->setSpacing(0);
  vStack->setContentsMargins(0, 0, 0, 0);
  setLayout(vStack);

  // create actions
  _captureAction = new QAction(QIcon(":/images/crosshair.png"), tr(""), this);
  _captureAction->setToolTip(tr("Capture map clicks"));
  _captureAction->setCheckable(true);
  connect(_captureAction, SIGNAL(toggled(bool)), this, SLOT(onCaptureToggled(bool)));

  _undoZoomAction = new QAction(QIcon(":/images/undo.png"), tr(""), this);
  _undoZoomAction->setToolTip(tr("Undo zoom"));
  connect(_undoZoomAction, SIGNAL(triggered()), this, SLOT(onUndoZoom()));
  _undoZoomAction->setEnabled(false);

  // create toolbar
  QToolBar *buttonToolbar = new QToolBar(tr("Action Toolbar"));
  buttonToolbar->setIconSize(QSize(24, 24));
  buttonToolbar->addAction(_captureAction);
  buttonToolbar->addSeparator();
  buttonToolbar->addAction(_undoZoomAction);
  vStack->addWidget(buttonToolbar);

  // create graph widget
  _graph = new TerrainProfileGraph(_calculator);
  vStack->addWidget(_graph);
}

void TerrainProfileWidget::setActiveView(osgViewer::View* view)
{
  removeViews();
  addView(view);
}

void TerrainProfileWidget::setActiveViews(const ViewVector& views)
{
  removeViews();

  for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
    addView(*it);
}

void TerrainProfileWidget::addView(osgViewer::View* view)
{
  view->addEventHandler(_guiHandler.get());
  _views.push_back(view);
}

void TerrainProfileWidget::removeViews()
{
  for (ViewVector::iterator it = _views.begin(); it != _views.end(); ++it)
    (*it)->removeEventHandler(_guiHandler.get());

  _views.clear();
}

void TerrainProfileWidget::hideEvent(QHideEvent* e)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(false);
}

void TerrainProfileWidget::showEvent(QShowEvent* e)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(_captureAction->isChecked());
}

void TerrainProfileWidget::setStartEnd(const GeoPoint& start, const GeoPoint& end)
{
  _profileStack.clear();
  _calculator->setStartEnd(start, end);
}

void TerrainProfileWidget::onChanged(const osgEarth::Util::TerrainProfileCalculator* sender)
{
  if (_profileStack.size() == 0 || (_profileStack.back().start != _calculator->getStart() || _profileStack.back().end != _calculator->getEnd()))
    _profileStack.push_back(StartEndPair(_calculator->getStart(), _calculator->getEnd()));

  _undoZoomAction->setEnabled(_profileStack.size() > 1);
}

void TerrainProfileWidget::onCaptureToggled(bool checked)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(checked);
}

void TerrainProfileWidget::onUndoZoom()
{
  _profileStack.pop_back();
  _calculator->setStartEnd(_profileStack.back().start, _profileStack.back().end);
}