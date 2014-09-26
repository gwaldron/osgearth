/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthUtil/TerrainProfile>

#include <QAction>
#include <QFrame>
#include <QGLWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolBar>
#include <QVBoxLayout>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::QtGui;


//---------------------------------------------------------------------------


namespace
{
    // Shim to connect the profile calc to the widget
    struct ProfileChangedShim : public TerrainProfileCalculator::ChangedCallback
    {
        ProfileChangedShim(TerrainProfileWidget* widget) : _widget(widget) { }
        void onChanged(const TerrainProfileCalculator* ) { _widget->notifyTerrainProfileChanged(); }
        TerrainProfileWidget* _widget;
    };

    // Shim to connect the position detector to the widget
    struct UpdatePositionShim : public TerrainProfilePositionCallback
    {
        UpdatePositionShim(TerrainProfileWidget* widget) : _widget(widget) { }
        void updatePosition(double lat, double lon, const std::string& text) {
            _widget->updatePosition(lat, lon, text);
        }
        TerrainProfileWidget* _widget;
    };
}


//---------------------------------------------------------------------------

TerrainProfileWidget::TerrainProfileWidget(osg::Group* root, osgEarth::MapNode* mapNode)
: _root(root), _mapNode(mapNode)
{
  _calculator = new osgEarth::Util::TerrainProfileCalculator(mapNode);
  _guiHandler = new TerrainProfileMouseHandler(_mapNode.get(), _root.get(), this);

  initialize();

  // listen for profile changes and marshall to the UI thread to update the graph.
  _calculator->addChangedCallback( new ProfileChangedShim(this) );
  connect( this, SIGNAL(onNotifyTerrainProfileChanged()), this, SLOT(onTerrainProfileChanged()), Qt::QueuedConnection );
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

  _undoZoomAction = new QAction(QIcon(":/images/undo.png"), tr(""), this);
  _undoZoomAction->setToolTip(tr("Undo zoom"));
  _undoZoomAction->setEnabled(false);

  _clearProfilesAction = new QAction(QIcon(":/images/close.png"), tr(""), this);
  _clearProfilesAction->setToolTip(tr("Clear profiles"));

  _copyClipboardAction = new QAction(QIcon(":/images/copy.png"), tr(""), this);
  _copyClipboardAction->setToolTip(tr("Copy"));

  // create toolbar
  QToolBar *buttonToolbar = new QToolBar(tr("Action Toolbar"));
  buttonToolbar->setIconSize(QSize(24, 24));
  buttonToolbar->addAction(_captureAction);
  buttonToolbar->addAction(_clearProfilesAction);
  buttonToolbar->addAction(_copyClipboardAction);
  buttonToolbar->addSeparator();
  buttonToolbar->addAction(_undoZoomAction);
  vStack->addWidget(buttonToolbar);

  // create graph widget
  _graph = new TerrainProfileGraph(_calculator, new UpdatePositionShim(this));
  vStack->addWidget(_graph);

  // Connect the action signals/slots
  connect(_captureAction, SIGNAL(toggled(bool)), this, SLOT(onCaptureToggled(bool)));
  connect(_undoZoomAction, SIGNAL(triggered()), this, SLOT(onUndoZoom()));
  connect(_clearProfilesAction, SIGNAL(triggered()), this, SLOT(onClearProfiles()));
  connect(_copyClipboardAction, SIGNAL(triggered()), _graph, SLOT(onCopyToClipboard()));

  // define a style for the line
  osgEarth::Symbology::LineSymbol* ls = _lineStyle.getOrCreateSymbol<osgEarth::Symbology::LineSymbol>();
  ls->stroke()->color() = osgEarth::Symbology::Color::White;
  ls->stroke()->width() = 2.0f;
  ls->tessellation() = 500;
  _lineStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()->clamping() = osgEarth::Symbology::AltitudeSymbol::CLAMP_TO_TERRAIN;

  // load marker image
  QImage image(":/images/marker.png"); 
  QImage glImage = QGLWidget::convertToGLFormat(image); 

  unsigned char* data = new unsigned char[glImage.byteCount()];
	for(int i=0; i<glImage.byteCount(); i++)
	{
		data[i] = glImage.bits()[i];
	}

  _markerImage = new osg::Image(); 
  _markerImage->setImage(glImage.width(), 
                         glImage.height(), 
                         1, 
                         4, 
                         GL_RGBA, 
                         GL_UNSIGNED_BYTE, 
                         data, 
                         osg::Image::USE_NEW_DELETE, 
                         1); 

  // setup placemark style
  _placeStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
}

void TerrainProfileWidget::setCoordinateFormatter(osgEarth::Util::Formatter* coordinateFormatter)
{
  if(_graph)
  {
    _graph->setCoordinateFormatter(coordinateFormatter);
  }
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

void TerrainProfileWidget::refreshViews()
{
    // to support ON_DEMAND rendering.
    for (ViewVector::iterator it = _views.begin(); it != _views.end(); ++it)
        it->get()->requestRedraw();
}

void TerrainProfileWidget::hideEvent(QHideEvent* e)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(false);

  if (_lineNode.valid())
    _root->removeChild(_lineNode.get());

  if (_markerNode.valid())
    _root->removeChild(_markerNode.get());
  
  refreshViews();
}

void TerrainProfileWidget::showEvent(QShowEvent* e)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(_captureAction->isChecked());

  if (_lineNode.valid())
    _root->addChild(_lineNode.get());

  if (_markerNode.valid())
    _root->addChild(_markerNode);
  
  refreshViews();
}

void TerrainProfileWidget::setStartEnd(const GeoPoint& start, const GeoPoint& end)
{
  if (_markerNode.valid())
  {
    _root->removeChild(_markerNode);
    _markerNode = 0L;
  }

  _profileStack.clear();
  _calculator->setStartEnd(start, end);
}

void TerrainProfileWidget::onTerrainProfileChanged()
{
  if (_profileStack.size() == 0 || (_profileStack.back().start != _calculator->getStart() || _profileStack.back().end != _calculator->getEnd()))
    _profileStack.push_back(StartEndPair(_calculator->getStart(), _calculator->getEnd()));

  _undoZoomAction->setEnabled(_profileStack.size() > 1);

  drawProfileLine();
}

void TerrainProfileWidget::updatePosition(double lat, double lon, const std::string& text)
{
  if (!_markerNode.valid())
  {
    _markerNode = new osgEarth::Annotation::PlaceNode(
        _mapNode.get(),
        GeoPoint(_mapNode->getMapSRS(), lon, lat, 0, osgEarth::ALTMODE_RELATIVE),
        _markerImage,
        text,
        _placeStyle);

    _markerNode->setDynamic(true);

    _root->addChild(_markerNode);
  }
  else
  {
    _markerNode->setPosition(GeoPoint(_mapNode->getMapSRS(), lon, lat, 0, osgEarth::ALTMODE_RELATIVE));
    _markerNode->setText(text);
  }

  refreshViews();
}

void TerrainProfileWidget::onCaptureToggled(bool checked)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(checked);
}

void TerrainProfileWidget::onClearProfiles()
{
  if (_lineNode.valid())
  {
    _root->removeChild(_lineNode.get());
    _lineNode = 0;
  }
  if (_markerNode.valid())
  {
    _root->removeChild(_markerNode.get());
    _markerNode = 0;
  }
  _profileStack.clear();
  _undoZoomAction->setEnabled(false);
  _calculator->setStartEnd(GeoPoint::INVALID, GeoPoint::INVALID);
  _graph->clear();
  refreshViews();
}

void TerrainProfileWidget::onUndoZoom()
{
  _profileStack.pop_back();
  _calculator->setStartEnd(_profileStack.back().start, _profileStack.back().end);
}

void TerrainProfileWidget::drawProfileLine()
{
  osgEarth::Symbology::LineString* line = new osgEarth::Symbology::LineString();
  line->push_back( _calculator->getStart().vec3d() );
  line->push_back( _calculator->getEnd().vec3d() );

  osgEarth::Features::Feature* feature = new osgEarth::Features::Feature(line, _mapNode->getMapSRS());
  feature->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;
  feature->style() = _lineStyle;

  if (!_lineNode.valid())
  {
    _lineNode = new osgEarth::Annotation::FeatureNode( _mapNode, feature );
    _lineNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _root->addChild( _lineNode.get() );
  }
  else
  {
    _lineNode->setFeature(feature);
  }

  refreshViews();
}
