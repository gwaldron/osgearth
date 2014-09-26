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

#include <osgEarthQt/AnnotationDialogs>
#include <osgEarthQt/Common>

#include <osgEarth/Draggers>
#include <osgEarth/Pickers>
#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/AnnotationEditing>
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthSymbology/Geometry>

#include <QCheckBox>
#include <QColor>
#include <QColorDialog>
#include <QDialog>
#include <QGLWidget>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

using namespace osgEarth;
using namespace osgEarth::QtGui;


#define ANNOTATION_PATH_WIDTH 12.0f


//---------------------------------------------------------------------------

BaseAnnotationDialog::BaseAnnotationDialog(osgEarth::MapNode* mapNode, const ViewVector& views, QWidget* parent, Qt::WindowFlags f)
: QDialog(parent, f), _mapNode(mapNode), _views(views.size())
{
  initDefaultUi();

  std::copy(views.begin(), views.end(), _views.begin());
}

void BaseAnnotationDialog::initDefaultUi()
{
  // main layout
  QVBoxLayout* vLayout = new QVBoxLayout;
  setLayout(vLayout);

  // name layout and widgets
  QHBoxLayout* hb1 = new QHBoxLayout;
  hb1->addWidget(new QLabel(tr("Name")));
  
  _nameEdit = new QLineEdit;
  hb1->addWidget(_nameEdit);

  vLayout->addLayout(hb1);

  // description layout and widgets
  QVBoxLayout* vb1 = new QVBoxLayout;
  vb1->addWidget(new QLabel(tr("Description")));

  _descriptionEdit = new QLineEdit();
  vb1->addWidget(_descriptionEdit);

  vLayout->addLayout(vb1);

  // empty layout for custom content
  _customLayout = new QVBoxLayout;
  vLayout->addLayout(_customLayout);

  // ok/cancel buttons
  vLayout->addItem(new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding));

  QHBoxLayout* hb2 = new QHBoxLayout;

  hb2->addStretch();

  _okButton = new QPushButton(tr("OK"));
  hb2->addWidget(_okButton);

  QPushButton* cancelButton = new QPushButton(tr("Cancel"));
  hb2->addWidget(cancelButton);

  vLayout->addLayout(hb2);

  // wire up ui events
  connect(_okButton, SIGNAL(clicked()), this, SLOT(accept()));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));
}

void BaseAnnotationDialog::updateButtonColorStyle(QPushButton* button, const QColor& color)
{
  if (button)
  {
    if (color.alpha() == 0)
    {
      button->setStyleSheet("QPushButton { color: black; }");
    }
    else
    {
      int invR = 255 - color.red();
      int invG = 255 - color.green();
      int invB = 255 - color.blue();
      QColor invColor(invR, invG, invB);

      button->setStyleSheet("QPushButton { color: " + invColor.name() + "; background-color: " + color.name() + " }");
    }
  }
}

//---------------------------------------------------------------------------

AddMarkerDialog::AddMarkerDialog(osg::Group* root, osgEarth::MapNode* mapNode, const ViewVector& views, osgEarth::Annotation::PlaceNode* marker, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(mapNode, views, parent, f), _root(root), _altName(""), _editing(marker ? true : false), _inPosition(marker ? marker->getPosition() : osgEarth::GeoPoint()), _inNameSet(false)
{
  initialize();

  if (marker)
  {
    osgEarth::Annotation::AnnotationData* data = marker->getAnnotationData();
    
    //Get marker text/name
    _inName = marker->getText().length() > 0 ? marker->getText() : (data ? data->getName() : "");
    _nameEdit->setText(tr(_inName.c_str()));
    if (marker->getText().length() > 0 && data && data->getName().length() > 0 && marker->getText() != data->getName())
      _altName = data->getName();

    //Get marker description
    _inDescription = data ? data->getDescription() : "";
    _descriptionEdit->setText(tr(_inDescription.c_str()));

    //Set marker checkbox
    _inNameSet = marker->getText().length() > 0;
    _nameCheckbox->setChecked(_inNameSet);

    _placeNode = marker;
  }

  if (_mapNode.valid() && _views.size() > 0)
  {
    _guiHandler = new AddPointsMouseHandler(this, _mapNode.get(), 0L, false);
    for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
      (*it)->addEventHandler(_guiHandler.get());
  }
}

void AddMarkerDialog::initialize()
{
  _okButton->setEnabled(_editing);

  _nameEdit->setText(tr("New Marker"));

  // add name display checkbox to the dialog
  _nameCheckbox = new QCheckBox(tr("Display name"));
  _nameCheckbox->setCheckState(Qt::Checked);
  _customLayout->addWidget(_nameCheckbox);

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

  // wire up UI events
  connect(_nameCheckbox, SIGNAL(stateChanged(int)), this, SLOT(onNameCheckStateChanged(int)));
  connect(_nameEdit, SIGNAL(textChanged(const QString&)), this, SLOT(onNameTextChanged(const QString&)));
}

void AddMarkerDialog::clearDisplay()
{
  if (!_editing && _root.valid())
    _root->removeChild(_placeNode);

  if (_guiHandler.valid())
    for (ViewVector::const_iterator it = _views.begin(); it != _views.end(); ++it)
      (*it)->removeEventHandler(_guiHandler.get());
}

void AddMarkerDialog::resetValues()
{
  _nameEdit->setText(_inNameSet ? tr(_inName.c_str()) : "");
  _descriptionEdit->setText(tr(_inDescription.c_str()));

  if (_editing)
  {
    _placeNode->setPosition(_inPosition);
  }
  else
  {
    if (_root.valid())
      _root->removeChild(_placeNode);
  }
}

void AddMarkerDialog::mapMouseClick(const osgEarth::GeoPoint& point, int button)
{
  if (button == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
  {
    if (!_placeNode.valid() && _root.valid())
    {
      _placeNode = new osgEarth::Annotation::PlaceNode(_mapNode, point, _markerImage, _nameCheckbox->checkState() == Qt::Checked ? getName() : "", _placeStyle);

      if (_root.valid())
        _root->addChild(_placeNode);
    }
    else
    {
      _placeNode->setPosition(point);
    }

    _okButton->setEnabled(true);
  }
}

void AddMarkerDialog::accept()
{
  clearDisplay();

  if (_placeNode.valid())
  {
    osgEarth::Annotation::AnnotationData* annoData = new osgEarth::Annotation::AnnotationData();
    annoData->setName(_altName.length() > 0 ? _altName : getName());
    annoData->setDescription(getDescription());
    annoData->setViewpoint(osgEarth::Viewpoint(_placeNode->getPosition().vec3d(), 0.0, -90.0, 1e5, _placeNode->getPosition().getSRS()));
    _placeNode->setAnnotationData(annoData);
  }

  QDialog::accept();
}

void AddMarkerDialog::reject()
{
  resetValues();
  clearDisplay();
  QDialog::reject();
}

void AddMarkerDialog::closeEvent(QCloseEvent* event)
{
  clearDisplay();
  QDialog::closeEvent(event);
}

void AddMarkerDialog::onNameCheckStateChanged(int state)
{
  bool checked = state == Qt::Checked;
  if (_placeNode.valid())
    _placeNode->setText(checked ? getName() : "");
}

void AddMarkerDialog::onNameTextChanged(const QString& text)
{
  if (_placeNode.valid() && _nameCheckbox->checkState() == Qt::Checked)
    _placeNode->setText(getName());
}

//---------------------------------------------------------------------------

AddPathDialog::AddPathDialog(osg::Group* root, osgEarth::MapNode* mapNode, const ViewVector& views, osgEarth::Annotation::FeatureNode* path, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(mapNode, views, parent, f), _root(root), _draggers(0L), _pathColor(Color::White), _editing(path ? true : false)
{
  initialize();

  //If editing an existing path
  if (path)
  {
    osgEarth::Annotation::AnnotationData* data = path->getAnnotationData();
    
    //Get path name
    _inName = data ? data->getName() : "";
    _nameEdit->setText(tr(_inName.c_str()));

    //Get path description
    _inDescription = data ? data->getDescription() : "";
    _descriptionEdit->setText(tr(_inDescription.c_str()));

    _pathNode = path;

    const osgEarth::Features::Feature* feat = path->getFeature();
    if (feat)
    {
      _inFeature = const_cast<osgEarth::Features::Feature*>(feat);

      //Get path color
      const osgEarth::Symbology::LineSymbol* lineSymbol = feat->style()->get<LineSymbol>();
      if (lineSymbol)
      {
        _pathColor = lineSymbol->stroke()->color();
        updateButtonColorStyle(_lineColorButton, QColor::fromRgbF(_pathColor.r(), _pathColor.g(), _pathColor.b(), _pathColor.a()));
      }

      //Get path clamping
      const osgEarth::Symbology::AltitudeSymbol* altSymbol = feat->style()->get<AltitudeSymbol>();
      if (altSymbol)
        _drapeCheckbox->setChecked(altSymbol->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN);

      //Get path points
      const osgEarth::Symbology::LineString* pathLine = dynamic_cast<const osgEarth::Symbology::LineString*>(feat->getGeometry());
      if (pathLine)
      {
        for (osgEarth::Symbology::LineString::const_iterator it = pathLine->begin(); it != pathLine->end(); ++it)
          addPoint(osgEarth::GeoPoint(_mapNode->getMapSRS(), (*it).x(), (*it).y(), (*it).z(), ALTMODE_RELATIVE));
      }
    }
  }

  if (_mapNode.valid() && _views.size() > 0)
  {
    _guiHandler = new AddPointsMouseHandler(this, _mapNode.get(), _root);
    for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
      (*it)->addEventHandler(_guiHandler.get());
  }
}

void AddPathDialog::initialize()
{
  _okButton->setEnabled(false);

  _nameEdit->setText(tr("New Path"));

  // add path color selection button
  _lineColorButton = new QPushButton(tr("Path Color"));
  _lineColorButton->setStyleSheet("QPushButton { color: black; background-color: white }");
  _customLayout->addWidget(_lineColorButton);

  // add name display checkbox to the dialog
  _drapeCheckbox = new QCheckBox(tr("Clamp to terrain"));
  _drapeCheckbox->setCheckState(Qt::Checked);
  _customLayout->addWidget(_drapeCheckbox);

  // wire up UI events
  connect(_drapeCheckbox, SIGNAL(stateChanged(int)), this, SLOT(onDrapeCheckStateChanged(int)));
  connect(_lineColorButton, SIGNAL(clicked()), this, SLOT(onLineColorButtonClicked()));
}

void AddPathDialog::clearDisplay()
{
  if (_root.valid())
  {
    if (!_editing)
      _root->removeChild(_pathNode);

    _root->removeChild(_draggers);
  }

  if (_guiHandler.valid())
  {
    for (ViewVector::const_iterator it = _views.begin(); it != _views.end(); ++it)
      (*it)->removeEventHandler(_guiHandler.get());

    _guiHandler->clearDisplay();
  }
}

void AddPathDialog::resetValues()
{
  _nameEdit->setText(tr(_inName.c_str()));
  _descriptionEdit->setText(tr(_inDescription.c_str()));

  if (_inFeature.valid())
  {
    _pathNode->setFeature(_inFeature);
  }
  else
  {
    if (_root.valid())
    {
      _root->removeChild(_pathNode);
      _pathNode = 0L;
      _pathLine = 0L;
    }
  }
}

void AddPathDialog::mapMouseClick(const osgEarth::GeoPoint& point, int button)
{
  if (button == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
  {
    addPoint(point);
  }
}

void AddPathDialog::refreshFeatureNode()
{
  if (_pathNode.valid() && _pathFeature.valid())  
  {
    _pathFeature->style()->getOrCreate<LineSymbol>()->stroke()->color() = _pathColor;
    _pathFeature->style()->getOrCreate<AltitudeSymbol>()->clamping() = _drapeCheckbox->checkState() == Qt::Checked ? AltitudeSymbol::CLAMP_TO_TERRAIN : AltitudeSymbol::CLAMP_ABSOLUTE;
    _pathNode->setFeature(_pathFeature);
  }
}

void AddPathDialog::createPointDragger(int index, const osgEarth::GeoPoint& point)
{
  osgEarth::SphereDragger* sd = new osgEarth::SphereDragger(_mapNode);
  sd->setSize(4.0f);
  sd->setColor(Color::Magenta);
  sd->setPickColor(Color::Green);
  sd->setPosition(point);
  PointDraggerCallback* callback = new PointDraggerCallback(index, this);
  sd->addPositionChangedCallback(callback);

  if (!_draggers)
  {
    _draggers = new osg::Group();
    _root->addChild(_draggers);
  }

  _draggers->addChild(sd);
}

void AddPathDialog::movePoint(int index, const osgEarth::GeoPoint& position)
{
  (*_pathLine.get())[index] = position.vec3d();
  refreshFeatureNode();
}

void AddPathDialog::addPoint(const osgEarth::GeoPoint& point)
{
  if (!_pathLine.valid())
    _pathLine = new osgEarth::Symbology::LineString();

  _pathLine->push_back(point.vec3d());

  if (!_pathFeature.valid() && _pathLine->size() > 1)
  {
    osgEarth::Symbology::Style pathStyle;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::White;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 2.0f;
    pathStyle.getOrCreate<LineSymbol>()->tessellation() = 20;
    pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    pathStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;

    _pathFeature = new osgEarth::Features::Feature(_pathLine, _mapNode->getMapSRS(), pathStyle);
    //_pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;

    if (!_pathNode.valid())
    {
      _pathNode = new osgEarth::Annotation::FeatureNode(_mapNode, _pathFeature);
      _root->addChild(_pathNode);
    }

    _okButton->setEnabled(true);
  }

  refreshFeatureNode();
  createPointDragger(_pathLine->size() - 1, point);
}

void AddPathDialog::accept()
{
  clearDisplay();

  if (_pathNode.valid())
  {
    osgEarth::Annotation::AnnotationData* annoData = new osgEarth::Annotation::AnnotationData();
    annoData->setName(getName());
    annoData->setDescription(getDescription());
    //annoData->setViewpoint(osgEarth::Viewpoint(_pathNode->getPosition().vec3d(), 0.0, -90.0, 1e5, _pathNode->getPosition().getSRS()));

    _pathNode->setAnnotationData(annoData);
  }

  QDialog::accept();
}

void AddPathDialog::reject()
{
  resetValues();
  clearDisplay();
  QDialog::reject();
}

void AddPathDialog::closeEvent(QCloseEvent* event)
{
  clearDisplay();
  QDialog::closeEvent(event);
}

void AddPathDialog::onDrapeCheckStateChanged(int state)
{
  refreshFeatureNode();
}

void AddPathDialog::onLineColorButtonClicked()
{
  QColor color = QColorDialog::getColor(QColor::fromRgbF(_pathColor.r(), _pathColor.g(), _pathColor.b(), _pathColor.a()), this);
  if (color.isValid())
  {
    _pathColor = osgEarth::Symbology::Color(color.redF(), color.greenF(), color.blueF());
    refreshFeatureNode();

    updateButtonColorStyle(_lineColorButton, color);
  }
}


//---------------------------------------------------------------------------

AddPolygonDialog::AddPolygonDialog(osg::Group* root, osgEarth::MapNode* mapNode, const ViewVector& views, osgEarth::Annotation::FeatureNode* polygon, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(mapNode, views, parent, f), _root(root), _draggers(0L), _pathColor(Color(Color::White, 0.0)), _fillColor(Color(Color::Black, 0.5)), _editing(polygon ? true : false)
{
  _polygon = new osgEarth::Symbology::Polygon();
  
  initialize();

  //If editing an existing polygon
  if (polygon)
  {
    osgEarth::Annotation::AnnotationData* data = polygon->getAnnotationData();

    //Get path name
    _inName = data ? data->getName() : "";
    _nameEdit->setText(tr(_inName.c_str()));

    //Get path description
    _inDescription = data ? data->getDescription() : "";
    _descriptionEdit->setText(tr(_inDescription.c_str()));

    _polyNode = polygon;

    const osgEarth::Features::Feature* feat = polygon->getFeature();
    if (feat)
    {
      _inFeature = const_cast<osgEarth::Features::Feature*>(feat);

      //Get path color
      const osgEarth::Symbology::LineSymbol* lineSymbol = feat->style()->get<LineSymbol>();
      if (lineSymbol)
      {
        if (lineSymbol->stroke()->width() != 0.0)
        {
          _pathColor = lineSymbol->stroke()->color();
          updateButtonColorStyle(_lineColorButton, QColor::fromRgbF(_pathColor.r(), _pathColor.g(), _pathColor.b(), _pathColor.a()));
        }
      }

      //Get fill color
      const osgEarth::Symbology::PolygonSymbol* polySymbol = feat->style()->get<PolygonSymbol>();
      if (polySymbol)
      {
        _fillColor = polySymbol->fill()->color();
        updateButtonColorStyle(_fillColorButton, QColor::fromRgbF(_fillColor.r(), _fillColor.g(), _fillColor.b(), _fillColor.a()));
      }

      //Get draping
      _drapeCheckbox->setChecked(polygon->isDraped());

      //Get path points
      const osgEarth::Symbology::Polygon* polyGeom = dynamic_cast<const osgEarth::Symbology::Polygon*>(feat->getGeometry());
      if (polyGeom)
      {
        for (osgEarth::Symbology::LineString::const_iterator it = polyGeom->begin(); it != polyGeom->end(); ++it)
          addPoint(osgEarth::GeoPoint(_mapNode->getMapSRS(), (*it).x(), (*it).y(), (*it).z(), ALTMODE_RELATIVE));
      }
    }
  }
  else
  {
    //Not editing an existing polygon
    
    //Add an end point to follow the mouse cursor
    _polygon->push_back(osg::Vec3d(0.0, 0.0, 0.0));

    //Add mouse handlers
    if (_mapNode.valid() && _views.size() > 0)
    {
      _guiHandler = new AddPointsMouseHandler(this, _mapNode.get(), _root);
      for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
        (*it)->addEventHandler(_guiHandler.get());
    }
  }
}

void AddPolygonDialog::initialize()
{
  _okButton->setEnabled(false);

  _nameEdit->setText(tr("New Polygon"));

  // add line color selection button
  _lineColorButton = new QPushButton(tr("Line Color"));
  _lineColorButton->setStyleSheet("QPushButton { color: black }");
  _customLayout->addWidget(_lineColorButton);

  // add fill color selection button
  _fillColorButton = new QPushButton(tr("Fill Color"));
  _fillColorButton->setStyleSheet("QPushButton { color: white; background-color: black }");
  _customLayout->addWidget(_fillColorButton);

  // add name display checkbox to the dialog
  _drapeCheckbox = new QCheckBox(tr("Drape on terrain"));
  _drapeCheckbox->setCheckState(Qt::Checked);
  _customLayout->addWidget(_drapeCheckbox);

  // wire up UI events
  connect(_drapeCheckbox, SIGNAL(stateChanged(int)), this, SLOT(onDrapeCheckStateChanged(int)));
  connect(_lineColorButton, SIGNAL(clicked()), this, SLOT(onLineColorButtonClicked()));
  connect(_fillColorButton, SIGNAL(clicked()), this, SLOT(onFillColorButtonClicked()));
}

void AddPolygonDialog::clearDisplay()
{
  if (_root.valid())
  {
    if (!_editing)
      _root->removeChild(_polyNode);

    _root->removeChild(_draggers);
  }

  if (_guiHandler.valid())
  {
    for (ViewVector::const_iterator it = _views.begin(); it != _views.end(); ++it)
      (*it)->removeEventHandler(_guiHandler.get());

    _guiHandler->clearDisplay();
  }
}

void AddPolygonDialog::resetValues()
{
  _nameEdit->setText(tr(_inName.c_str()));
  _descriptionEdit->setText(tr(_inDescription.c_str()));

  if (_inFeature.valid())
  {
    _polyNode->setFeature(_inFeature);
  }
  else
  {
    if (_root.valid())
    {
      _root->removeChild(_polyNode);
      _polyNode = 0L;
      _polygon = 0L;
    }
  }
}

void AddPolygonDialog::mapMouseClick(const osgEarth::GeoPoint& point, int button)
{
  if (button == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
  {
    addPoint(point);
  }
}

void AddPolygonDialog::mapMouseMove(const osgEarth::GeoPoint& point)
{
  _polygon->back().set(point.vec3d());
  refreshFeatureNode(true);
}

void AddPolygonDialog::refreshFeatureNode(bool geometryOnly)
{
  if (_polyNode.valid() && _polyFeature.valid())  
  {
    if (!geometryOnly)
    {
      if (_pathColor.a() == 0.0)
      {
        _polyFeature->style()->getOrCreate<LineSymbol>()->stroke()->color() = _fillColor;
        _polyFeature->style()->getOrCreate<LineSymbol>()->stroke()->width() = 0.0;
      }
      else
      {
        _polyFeature->style()->getOrCreate<LineSymbol>()->stroke()->color() = _pathColor;
        _polyFeature->style()->getOrCreate<LineSymbol>()->stroke()->width() = ANNOTATION_PATH_WIDTH;
      }
      

      _polyFeature->style()->getOrCreate<PolygonSymbol>()->fill()->color() = _fillColor;
      _polyFeature->style()->getOrCreate<AltitudeSymbol>()->clamping() = _drapeCheckbox->checkState() == Qt::Checked ? AltitudeSymbol::CLAMP_TO_TERRAIN : AltitudeSymbol::CLAMP_ABSOLUTE;
      _polyFeature->style()->getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    }

    _polyNode->setFeature(_polyFeature);
  }
}

void AddPolygonDialog::createPointDragger(int index, const osgEarth::GeoPoint& point)
{
  osgEarth::SphereDragger* sd = new osgEarth::SphereDragger(_mapNode);
  sd->setSize(4.0f);
  sd->setColor(Color::Magenta);
  sd->setPickColor(Color::Green);
  sd->setPosition(point);
  PointDraggerCallback* callback = new PointDraggerCallback(index, this);
  sd->addPositionChangedCallback(callback);

  if (!_draggers)
  {
    _draggers = new osg::Group();
    _root->addChild(_draggers);
  }

  _draggers->addChild(sd);
}

void AddPolygonDialog::movePoint(int index, const osgEarth::GeoPoint& position)
{
  (*_polygon.get())[index] = position.vec3d();
  refreshFeatureNode();
}

void AddPolygonDialog::addPoint(const osgEarth::GeoPoint& point)
{
  if (_editing)
    _polygon->push_back(point.vec3d());
  else
    _polygon->insert(_polygon->end() - 1, point.vec3d());

  if (!_polyFeature.valid() && _polygon->size() > 2)
  {
    osgEarth::Symbology::Style polyStyle;
    polyStyle.getOrCreate<LineSymbol>()->stroke()->color() = _pathColor;
    polyStyle.getOrCreate<LineSymbol>()->stroke()->width() = ANNOTATION_PATH_WIDTH;
    polyStyle.getOrCreate<LineSymbol>()->tessellation() = 20;
    polyStyle.getOrCreate<PolygonSymbol>()->fill()->color() = _fillColor;
    polyStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    _polyFeature = new osgEarth::Features::Feature(_polygon, _mapNode->getMapSRS(), polyStyle);

    if (!_polyNode.valid())
    {
      _polyNode = new osgEarth::Annotation::FeatureNode(_mapNode, _polyFeature, _drapeCheckbox->checkState() == Qt::Checked);
      _root->addChild(_polyNode);
    }

    _okButton->setEnabled(true);
  }

  refreshFeatureNode();
  createPointDragger(_polygon->size() - 1, point);
}

void AddPolygonDialog::accept()
{
  clearDisplay();

  if (_polyNode.valid())
  {
    //Remove active cursor point
    if (!_editing)
    {
      _polygon->pop_back();
      refreshFeatureNode(true);
    }

    //Create AnnotationData object for this annotation
    osgEarth::Annotation::AnnotationData* annoData = new osgEarth::Annotation::AnnotationData();
    annoData->setName(getName());
    annoData->setDescription(getDescription());
    //annoData->setViewpoint(osgEarth::Viewpoint(_pathNode->getPosition().vec3d(), 0.0, -90.0, 1e5, _pathNode->getPosition().getSRS()));

    _polyNode->setAnnotationData(annoData);
  }

  QDialog::accept();
}

void AddPolygonDialog::reject()
{
  resetValues();
  clearDisplay();
  QDialog::reject();
}

void AddPolygonDialog::closeEvent(QCloseEvent* event)
{
  clearDisplay();
  QDialog::closeEvent(event);
}

void AddPolygonDialog::onDrapeCheckStateChanged(int state)
{
  if (_polyNode.valid())
  {
    _root->removeChild(_polyNode);
    _polyNode = new osgEarth::Annotation::FeatureNode(_mapNode, _polyFeature, _drapeCheckbox->checkState() == Qt::Checked);
    _root->addChild(_polyNode);
  }
}

void AddPolygonDialog::onLineColorButtonClicked()
{
  QColor color = QColorDialog::getColor(QColor::fromRgbF(_pathColor.r(), _pathColor.g(), _pathColor.b(), _pathColor.a()), this, tr("Select outline color..."), QColorDialog::ShowAlphaChannel);
  if (color.isValid())
  {
    _pathColor = osgEarth::Symbology::Color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    refreshFeatureNode();

    updateButtonColorStyle(_lineColorButton, color);
  }
}

void AddPolygonDialog::onFillColorButtonClicked()
{
  QColor color = QColorDialog::getColor(QColor::fromRgbF(_fillColor.r(), _fillColor.g(), _fillColor.b(), _fillColor.a()), this, tr("Select fill color..."), QColorDialog::ShowAlphaChannel);
  if (color.isValid())
  {
    _fillColor = osgEarth::Symbology::Color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    refreshFeatureNode();

    updateButtonColorStyle(_fillColorButton, color);
  }
}

//---------------------------------------------------------------------------

AddEllipseDialog::AddEllipseDialog(osg::Group* root, osgEarth::MapNode* mapNode, const ViewVector& views, osgEarth::Annotation::EllipseNode* ellipse, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(mapNode, views, parent, f), _root(root), _pathColor(Color(Color::White, 0.0)), _fillColor(Color(Color::Black, 0.5)),
_editing(ellipse ? true : false), _inStyle(ellipse ? ellipse->getStyle() : osgEarth::Symbology::Style())
{
  initialize();

  if (ellipse)
  {
    osgEarth::Annotation::AnnotationData* data = ellipse->getAnnotationData();

    //Get path name
    _inName = data ? data->getName() : "";
    _nameEdit->setText(tr(_inName.c_str()));

    //Get path description
    _inDescription = data ? data->getDescription() : "";
    _descriptionEdit->setText(tr(_inDescription.c_str()));

    _inPosition = ellipse->getPosition();
    _inRadiusMajor = ellipse->getRadiusMajor();
    _inRadiusMinor = ellipse->getRadiusMinor();
    _inRotationAngle = ellipse->getRotationAngle();

    _ellipseNode = ellipse;

    //Get path color
    const osgEarth::Symbology::LineSymbol* lineSymbol = ellipse->getStyle().get<LineSymbol>();
    if (lineSymbol)
    {
      if (lineSymbol->stroke()->width() != 0.0)
      {
        _pathColor = lineSymbol->stroke()->color();
        updateButtonColorStyle(_lineColorButton, QColor::fromRgbF(_pathColor.r(), _pathColor.g(), _pathColor.b(), _pathColor.a()));
      }
    }

    //Get fill color
    const osgEarth::Symbology::PolygonSymbol* polySymbol = ellipse->getStyle().get<PolygonSymbol>();
    if (polySymbol)
    {
      _fillColor = polySymbol->fill()->color();
      updateButtonColorStyle(_fillColorButton, QColor::fromRgbF(_fillColor.r(), _fillColor.g(), _fillColor.b(), _fillColor.a()));
    }

    //Get draping
    _drapeCheckbox->setChecked(ellipse->isDraped());

    //Call refreshFeatureNode to update _ellipseStyle
    refreshFeatureNode();

    //Add the ellipse
    addEllipse(ellipse->getPosition(), ellipse->getRadiusMajor(), ellipse->getRadiusMinor(), ellipse->getRotationAngle());
  }
  else
  {
    if (_mapNode.valid() && _views.size() > 0)
    {
      _guiHandler = new AddEllipseMouseHandler(this, _mapNode.get(), _root);
      for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
        (*it)->addEventHandler(_guiHandler.get());
    }
  }
}

void AddEllipseDialog::initialize()
{
  //Define the default ellipse style
  _ellipseStyle.getOrCreate<LineSymbol>()->stroke()->color() = _fillColor;
  _ellipseStyle.getOrCreate<LineSymbol>()->stroke()->width() = 0.0;
  _ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = _fillColor;
  _ellipseStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()->clamping() = osgEarth::Symbology::AltitudeSymbol::CLAMP_TO_TERRAIN;

  //Set _okButton disabled until an ellipse is drawn
  _okButton->setEnabled(false);

  _nameEdit->setText(tr("New Ellipse"));

  // add line color selection button
  _lineColorButton = new QPushButton(tr("Line Color"));
  _lineColorButton->setStyleSheet("QPushButton { color: black }");
  _customLayout->addWidget(_lineColorButton);

  // add fill color selection button
  _fillColorButton = new QPushButton(tr("Fill Color"));
  _fillColorButton->setStyleSheet("QPushButton { color: white; background-color: black }");
  _customLayout->addWidget(_fillColorButton);

  // add name display checkbox to the dialog
  _drapeCheckbox = new QCheckBox(tr("Drape on terrain"));
  _drapeCheckbox->setCheckState(Qt::Checked);
  _customLayout->addWidget(_drapeCheckbox);

  // wire up UI events
  connect(_drapeCheckbox, SIGNAL(stateChanged(int)), this, SLOT(onDrapeCheckStateChanged(int)));
  connect(_lineColorButton, SIGNAL(clicked()), this, SLOT(onLineColorButtonClicked()));
  connect(_fillColorButton, SIGNAL(clicked()), this, SLOT(onFillColorButtonClicked()));
}

void AddEllipseDialog::clearDisplay()
{
  if (_root.valid())
  {
    if (!_editing)
      _root->removeChild(_ellipseNode);

    _root->removeChild(_editor);
  }

  removeGuiHandlers();
}

void AddEllipseDialog::resetValues()
{
  _nameEdit->setText(tr(_inName.c_str()));
  _descriptionEdit->setText(tr(_inDescription.c_str()));

  if (_editing && _ellipseNode.valid())
  {
    _ellipseNode->setPosition(_inPosition);
    _ellipseNode->setRadii(_inRadiusMajor, _inRadiusMinor);
    _ellipseNode->setRotationAngle(_inRotationAngle);
    _ellipseNode->setStyle(_inStyle);
  }
}

void AddEllipseDialog::removeGuiHandlers()
{
  if (_guiHandler.valid())
  {
    for (ViewVector::const_iterator it = _views.begin(); it != _views.end(); ++it)
      (*it)->removeEventHandler(_guiHandler.get());

    _guiHandler = 0L;
  }
}

void AddEllipseDialog::addEllipse(const osgEarth::GeoPoint& position, const Linear& radiusMajor, const Linear& radiusMinor, const Angular& rotationAngle)
{
  if (!_ellipseNode.valid())
  {
    Style style = _ellipseStyle;
    if ( _drapeCheckbox->checkState() == Qt::Checked ) {
        style.getOrCreate<AltitudeSymbol>()->clamping()  = AltitudeSymbol::CLAMP_TO_TERRAIN;
        style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    }
    else {
        style.getOrCreate<AltitudeSymbol>()->clamping()  = AltitudeSymbol::CLAMP_NONE;
    }
    _ellipseNode = new osgEarth::Annotation::EllipseNode(_mapNode, position, radiusMajor, radiusMinor, rotationAngle, style);
    _root->addChild(_ellipseNode);
  }
  else
  {
    _ellipseNode->setPosition(position);
    _ellipseNode->setRadii(radiusMajor, radiusMinor);
    _ellipseNode->setRotationAngle(rotationAngle);
  }

  if (!_editor.valid())
  {
    _editor = new osgEarth::Annotation::EllipseNodeEditor(_ellipseNode);
    _root->addChild(_editor);
  }

  if (_guiHandler.valid())
    _guiHandler->setCapturing(false);

  _okButton->setEnabled(true);
}

void AddEllipseDialog::refreshFeatureNode(bool geometryOnly)
{
  if (_pathColor.a() == 0.0)
  {
    _ellipseStyle.getOrCreate<LineSymbol>()->stroke()->color() = _fillColor;
    _ellipseStyle.getOrCreate<LineSymbol>()->stroke()->width() = 0.0;
  }
  else
  {
    _ellipseStyle.getOrCreate<LineSymbol>()->stroke()->color() = _pathColor;
    _ellipseStyle.getOrCreate<LineSymbol>()->stroke()->width() = ANNOTATION_PATH_WIDTH;
  }

  _ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = _fillColor;

  if (_ellipseNode.valid())
    _ellipseNode->setStyle(_ellipseStyle);
}

void AddEllipseDialog::accept()
{
  clearDisplay();

  if (_ellipseNode.valid())
  {
    //Create AnnotationData object for this annotation
    osgEarth::Annotation::AnnotationData* annoData = new osgEarth::Annotation::AnnotationData();
    annoData->setName(getName());
    annoData->setDescription(getDescription());
    annoData->setViewpoint(osgEarth::Viewpoint(_ellipseNode->getPosition().vec3d(), 0.0, -90.0, 1e5, _ellipseNode->getPosition().getSRS()));

    _ellipseNode->setAnnotationData(annoData);
  }

  QDialog::accept();
}

void AddEllipseDialog::reject()
{
  resetValues();
  clearDisplay();
  QDialog::reject();
}

void AddEllipseDialog::closeEvent(QCloseEvent* event)
{
  clearDisplay();
  QDialog::closeEvent(event);
}

void AddEllipseDialog::onDrapeCheckStateChanged(int state)
{
  if (_ellipseNode.valid())
    addEllipse(_ellipseNode->getPosition(), _ellipseNode->getRadiusMajor(), _ellipseNode->getRadiusMinor(), _ellipseNode->getRotationAngle());
}

void AddEllipseDialog::onLineColorButtonClicked()
{
  QColor color = QColorDialog::getColor(QColor::fromRgbF(_pathColor.r(), _pathColor.g(), _pathColor.b(), _pathColor.a()), this, tr("Select outline color..."), QColorDialog::ShowAlphaChannel);
  if (color.isValid())
  {
    _pathColor = osgEarth::Symbology::Color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    refreshFeatureNode();

    updateButtonColorStyle(_lineColorButton, color);
  }
}

void AddEllipseDialog::onFillColorButtonClicked()
{
  QColor color = QColorDialog::getColor(QColor::fromRgbF(_fillColor.r(), _fillColor.g(), _fillColor.b(), _fillColor.a()), this, tr("Select fill color..."), QColorDialog::ShowAlphaChannel);
  if (color.isValid())
  {
    _fillColor = osgEarth::Symbology::Color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    refreshFeatureNode();

    updateButtonColorStyle(_fillColorButton, color);
  }
}
