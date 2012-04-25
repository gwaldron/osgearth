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

#include <osgEarthQt/AnnotationDialogs>
#include <osgEarthQt/Common>

#include <osgEarth/Draggers>
#include <osgEarth/Pickers>
#include <osgEarthAnnotation/AnnotationData>
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

//---------------------------------------------------------------------------

AddMarkerDialog::AddMarkerDialog(osg::Group* root, osgEarth::MapNode* mapNode, const ViewVector& views, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(mapNode, views, parent, f), _root(root)
{
  initialize();

  if (_mapNode.valid() && _views.size() > 0)
  {
    _guiHandler = new AddMarkerMouseHandler(this, _mapNode.get());
    for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
      (*it)->addEventHandler(_guiHandler.get());
  }
}

void AddMarkerDialog::initialize()
{
  _okButton->setEnabled(false);

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
  if (_root.valid())
    _root->removeChild(_placeNode);

  if (_guiHandler.valid())
    for (ViewVector::const_iterator it = _views.begin(); it != _views.end(); ++it)
      (*it)->removeEventHandler(_guiHandler.get());
}

void AddMarkerDialog::mapClick(const osgEarth::GeoPoint& point)
{
  if (_placeNode.valid() && _root.valid())
    _root->removeChild(_placeNode);

  _placeNode = new osgEarth::Annotation::PlaceNode(_mapNode, point, _markerImage, _nameCheckbox->checkState() == Qt::Checked ? getName() : "", _placeStyle);

  if (_root.valid())
    _root->addChild(_placeNode);

  _okButton->setEnabled(true);
}

void AddMarkerDialog::accept()
{
  clearDisplay();

  if (_placeNode.valid())
  {
    osgEarth::Annotation::AnnotationData* annoData = new osgEarth::Annotation::AnnotationData();
    annoData->setName(getName());
    annoData->setDescription(getDescription());
    annoData->setViewpoint(osgEarth::Viewpoint(_placeNode->getPosition().vec3d(), 0.0, -90.0, 1e5, _placeNode->getPosition().getSRS()));
    _placeNode->setAnnotationData(annoData);
  }

  QDialog::accept();
}

void AddMarkerDialog::reject()
{
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

AddPathDialog::AddPathDialog(osg::Group* root, osgEarth::MapNode* mapNode, const ViewVector& views, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(mapNode, views, parent, f), _root(root), _draggers(0L), _pathColor(Color::White)
{
  initialize();

  if (_mapNode.valid() && _views.size() > 0)
  {
    _guiHandler = new AddPathMouseHandler(this, _mapNode.get(), _root);
    for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
      (*it)->addEventHandler(_guiHandler.get());
  }
}

void AddPathDialog::initialize()
{
  _okButton->setEnabled(false);

  _nameEdit->setText(tr("New Path"));

  // add color selection button
  _colorButton = new QPushButton(tr("Line Color"));
  _colorButton->setStyleSheet("QPushButton { color: black; background-color: white }");
  _customLayout->addWidget(_colorButton);

  // add name display checkbox to the dialog
  _drapeCheckbox = new QCheckBox(tr("Clamp to terrain"));
  _drapeCheckbox->setCheckState(Qt::Checked);
  _customLayout->addWidget(_drapeCheckbox);

  // wire up UI events
  connect(_drapeCheckbox, SIGNAL(stateChanged(int)), this, SLOT(onDrapeCheckStateChanged(int)));
  connect(_colorButton, SIGNAL(clicked()), this, SLOT(onColorButtonClicked()));
}

void AddPathDialog::clearDisplay()
{
  if (_root.valid())
  {
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

void AddPathDialog::mapClick(const osgEarth::GeoPoint& point, int button)
{
  if (button == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
  {
    addPoint(point);
  }
}

void AddPathDialog::refreshFeatureNode()
{
  if (_pathNode.valid())  
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

  if (!_pathNode.valid() && _pathLine->size() > 1)
  {
    osgEarth::Symbology::Style pathStyle;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::White;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 2.0f;
    pathStyle.getOrCreate<LineSymbol>()->tessellation() = 20;
    pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    _pathFeature = new osgEarth::Features::Feature(_pathLine, _mapNode->getMapSRS(), pathStyle);
    //_pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;

    _pathNode = new osgEarth::Annotation::FeatureNode(_mapNode, _pathFeature);
    _root->addChild(_pathNode);

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

void AddPathDialog::onColorButtonClicked()
{
  QColor color = QColorDialog::getColor(QColor::fromRgba(_pathColor.asRGBA()), this);
  if (color.isValid())
  {
    _pathColor = osgEarth::Symbology::Color(color.redF(), color.greenF(), color.blueF());
    refreshFeatureNode();

    int invR = 255 - color.red();
    int invG = 255 - color.green();
    int invB = 255 - color.blue();
    QColor invColor(invR, invG, invB);

    _colorButton->setStyleSheet("QPushButton { color: " + invColor.name() + "; background-color: " + color.name() + " }");
  }
}