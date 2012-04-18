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

#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/PlaceNode>

#include <QCheckBox>
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

BaseAnnotationDialog::BaseAnnotationDialog(QWidget* parent, Qt::WindowFlags f)
: QDialog(parent, f)
{
  initDefaultUi();
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

AddMarkerDialog::AddMarkerDialog(osg::Group* root, osgEarth::MapNode* mapNode, QWidget* parent, Qt::WindowFlags f)
: BaseAnnotationDialog(parent, f), _root(root), _mapNode(mapNode)
{
  initialize();
}

void AddMarkerDialog::initialize()
{
  _okButton->setEnabled(false);

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

void AddMarkerDialog::cleanupNodes()
{
  if (_root.valid())
    _root->removeChild(_placeNode);
}

osgEarth::Annotation::AnnotationNode* AddMarkerDialog::getAnnotation()
{
  return _placeNode.get();
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
  cleanupNodes();

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
  cleanupNodes();
  QDialog::reject();
}

void AddMarkerDialog::closeEvent(QCloseEvent* event)
{
  cleanupNodes();
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