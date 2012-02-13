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

#include <osgEarthQt/LOSCreationDialog>
#include <osgEarthQt/Common>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>

#include <osgEarthUtil/LineOfSight>

using namespace osgEarth;
using namespace osgEarth::QtGui;

LOSCreationDialog::LOSCreationDialog(osgEarth::MapNode* mapNode, int losCount, osgEarth::QtGui::DataManager *manager, ViewVector* views)
  : _mapNode(mapNode), _manager(manager), _views(views), _activeButton(0L)
{
  if (_mapNode.valid())
    _map = _mapNode->getMap();

  if (_manager.valid())
    _manager->getAnnotations(_annotations);

  initUi(losCount);
}

void LOSCreationDialog::initUi(int losCount)
{
  // setup UI
  _ui.setupUi(this);

  // set a default name
  std::stringstream losName;
  losName << "Line-of-Sight " << losCount;
  _ui.nameBox->setText(tr(losName.str().c_str()));

  // fill annotation list
  // TODO: if more node types beyond Annotation are added, this will need to be moved
  //       to the on__TypeChange() methods.  Currently doing here for efficiency.
  for (AnnotationVector::const_iterator it = _annotations.begin(); it != _annotations.end(); ++it)
  {
    osgEarth::Annotation::AnnotationData* annoData = (*it)->getAnnotationData();
    std::string annoName = annoData && annoData->getName().size() > 0 ? annoData->getName() : "Annotation";
    _ui.p1NodeCombo->addItem(tr(annoName.c_str()));
    _ui.p2NodeCombo->addItem(tr(annoName.c_str()));
    _ui.radNodeCombo->addItem(tr(annoName.c_str()));
  }

  // connect type combobox signals
  connect(_ui.p1TypeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onP1TypeChange(const QString&)));
  connect(_ui.p2TypeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onP2TypeChange(const QString&)));
  connect(_ui.radTypeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onRadTypeChange(const QString&)));

  // connect map-click button signals
  connect(_ui.p1PointButton, SIGNAL(clicked(bool)), this, SLOT(onP1MapButtonClicked(bool)));
  connect(_ui.p2PointButton, SIGNAL(clicked(bool)), this, SLOT(onP2MapButtonClicked(bool)));
  connect(_ui.radPointButton, SIGNAL(clicked(bool)), this, SLOT(onRadMapButtonClicked(bool)));

  // connect find node button signals
  connect(_ui.p1NodeButton, SIGNAL(clicked(bool)), this, SLOT(onP1FindNodeButtonClicked(bool)));
  connect(_ui.p2NodeButton, SIGNAL(clicked(bool)), this, SLOT(onP2FindNodeButtonClicked(bool)));
  connect(_ui.radNodeButton, SIGNAL(clicked(bool)), this, SLOT(onRadFindNodeButtonClicked(bool)));

  // simulate type change to force UI update
  onP1TypeChange("Point");
  onP2TypeChange("Point");
  onRadTypeChange("Point");

  this->resize(721, this->height());
}

void LOSCreationDialog::mapClick(const osg::Vec3d& location)
{
  if (_activeButton)
  {
    if (_activeButton == _ui.p1PointButton)
    {
      _ui.p1LonBox->setValue(location.x());
      _ui.p1LatBox->setValue(location.y());
      _ui.p1AltBox->setValue((int)(location.z()) + 1.0);
    }
    else if (_activeButton == _ui.p2PointButton)
    {
      _ui.p2LonBox->setValue(location.x());
      _ui.p2LatBox->setValue(location.y());
      _ui.p2AltBox->setValue((int)(location.z()) + 1.0);
    }
    else if (_activeButton == _ui.radPointButton)
    {
      _ui.radLonBox->setValue(location.x());
      _ui.radLatBox->setValue(location.y());
      _ui.radAltBox->setValue((int)(location.z()) + 1.0);
    }
    
    _activeButton = 0L;
    this->setEnabled(true);
  }
}

void LOSCreationDialog::accept()
{
  if (doClose())
    QDialog::accept();
}

bool LOSCreationDialog::doClose()
{
  bool ok = false;

  if (_mapNode.valid())
  {
    if (_ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Point-to-Point")
    {
      osgEarth::Util::LineOfSightNode* los = new osgEarth::Util::LineOfSightNode(_mapNode.get());

      if (_ui.depthTestCheckBox->checkState() == Qt::Unchecked)
        los->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

      bool p1Set = false;
      bool p2Set = false;
      osg::Node* p1Node = 0L;
      osg::Node* p2Node = 0L;

      // get start point or node
      if (_ui.p1TypeCombo->currentText() == "Point")
      {
        los->setStart(osg::Vec3d(_ui.p1LonBox->value(), _ui.p1LatBox->value(), _ui.p1AltBox->value()));

        if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
          los->setAltitudeMode( AltitudeMode::RELATIVE_TO_TERRAIN );

        p1Set = true;
      }
      else if (_ui.p1TypeCombo->currentText() == "Annotation")
      {
        if (_ui.p1NodeCombo->currentIndex() >= 0 && _annotations.size() > _ui.p1NodeCombo->currentIndex())
        {
          p1Node = _annotations[_ui.p1NodeCombo->currentIndex()];
          p1Set = true;
        }
      }

      // get end point or node
      if (_ui.p2TypeCombo->currentText() == "Point")
      {
        los->setEnd(osg::Vec3d(_ui.p2LonBox->value(), _ui.p2LatBox->value(), _ui.p2AltBox->value()));

        if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
          los->setAltitudeMode(AltitudeMode::RELATIVE_TO_TERRAIN);

        p2Set = true;
      }
      else if (_ui.p2TypeCombo->currentText() == "Annotation")
      {
        if (_ui.p2NodeCombo->currentIndex() >= 0 && _annotations.size() > _ui.p2NodeCombo->currentIndex())
        {
          p2Node = _annotations[_ui.p2NodeCombo->currentIndex()];
          p2Set = true;
        }
      }

      // set _node to los if ok, else clean up allocated memory
      ok = p1Set && p2Set;
      if (ok)
      {
        _node = los;

        if (p1Node || p2Node)
          los->setUpdateCallback(new osgEarth::Util::LineOfSightTether(p1Node, p2Node));
        else
        _editor = new osgEarth::Util::LineOfSightEditor(los);
      }
      else
      {
        delete los;
      }
    }
    else if (_ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Radial")
    {
      osgEarth::Util::RadialLineOfSightNode* los = new osgEarth::Util::RadialLineOfSightNode(_mapNode.get());
      los->setRadius(_ui.radiusSpinBox->value());
      los->setNumSpokes(_ui.spokesSpinBox->value());

      if (_ui.depthTestCheckBox->checkState() == Qt::Unchecked)
        los->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

      // get center point or node to attach to
      if (_ui.radTypeCombo->currentText() == "Point")
      {
        los->setCenter(osg::Vec3d(_ui.radLonBox->value(), _ui.radLatBox->value(), _ui.radAltBox->value()));

        if (_ui.radRelativeCheckBox->checkState() == Qt::Checked)
          los->setAltitudeMode(AltitudeMode::RELATIVE_TO_TERRAIN);

        _editor = new osgEarth::Util::RadialLineOfSightEditor(los);

        ok = true;
      }
      else if (_ui.radTypeCombo->currentText() == "Annotation")
      {
        if (_ui.radNodeCombo->currentIndex() >= 0 && _annotations.size() > _ui.radNodeCombo->currentIndex())
        {
          los->setUpdateCallback(new osgEarth::Util::RadialLineOfSightTether(_annotations[_ui.radNodeCombo->currentIndex()]));
          ok = true;
        }
      }

      // set _node to los if ok, else clean up allocated memory
      if (ok)
        _node = los;
      else
        delete los;
    }
  }

  return ok;
}

void LOSCreationDialog::centerMapOnNode(osg::Node* node)
{
  if (node && _map.valid() && _manager.valid() && _views)
  {
    AnnotationNode* annoNode = dynamic_cast<AnnotationNode*>(node);
    if (annoNode && annoNode->getAnnotationData() && annoNode->getAnnotationData()->getViewpoint())
    {
      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(*annoNode->getAnnotationData()->getViewpoint()), *_views));
    }
    else
    {
      osg::Vec3d center = node->getBound().center();

      GeoPoint output;
      _map->worldPointToMapPoint(center, output);

      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(output.vec3d(), 0.0, -90.0, 1e5), *_views));
    }
  }
}

void LOSCreationDialog::onP1TypeChange(const QString& text)
{
  if (text == "Point")
  {
    _ui.p1PointOptions->setVisible(true);
    _ui.p1NodeOptions->setVisible(false);
    _ui.p2pRelativeCheckBox->setEnabled(true);
  }
  else
  {
    _ui.p1PointOptions->setVisible(false);
    _ui.p1NodeOptions->setVisible(true);
    _ui.p2pRelativeCheckBox->setEnabled(_ui.p2TypeCombo->currentText() == "Point");
  }
}

void LOSCreationDialog::onP2TypeChange(const QString& text)
{
  if (text == "Point")
  {
    _ui.p2PointOptions->setVisible(true);
    _ui.p2NodeOptions->setVisible(false);
    _ui.p2pRelativeCheckBox->setEnabled(true);
  }
  else
  {
    _ui.p2PointOptions->setVisible(false);
    _ui.p2NodeOptions->setVisible(true);
    _ui.p2pRelativeCheckBox->setEnabled(_ui.p1TypeCombo->currentText() == "Point");
  }
}

void LOSCreationDialog::onRadTypeChange(const QString& text)
{
  if (text == "Point")
  {
    _ui.radPointOptions->setVisible(true);
    _ui.radNodeOptions->setVisible(false);
  }
  else
  {
    _ui.radPointOptions->setVisible(false);
    _ui.radNodeOptions->setVisible(true);
  }
}

void LOSCreationDialog::onP1MapButtonClicked(bool checked)
{
  _activeButton = _ui.p1PointButton;
  this->setEnabled(false);
}

void LOSCreationDialog::onP2MapButtonClicked(bool checked)
{
  _activeButton = _ui.p2PointButton;
  this->setEnabled(false);
}

void LOSCreationDialog::onRadMapButtonClicked(bool checked)
{
  _activeButton = _ui.radPointButton;
  this->setEnabled(false);
}

void LOSCreationDialog::onP1FindNodeButtonClicked(bool checked)
{
  if (_ui.p1NodeCombo->currentIndex() >= 0 && _annotations.size() > _ui.p1NodeCombo->currentIndex())
    centerMapOnNode(_annotations[_ui.p1NodeCombo->currentIndex()]);
}

void LOSCreationDialog::onP2FindNodeButtonClicked(bool checked)
{
  if (_ui.p2NodeCombo->currentIndex() >= 0 && _annotations.size() > _ui.p2NodeCombo->currentIndex())
    centerMapOnNode(_annotations[_ui.p2NodeCombo->currentIndex()]);
}

void LOSCreationDialog::onRadFindNodeButtonClicked(bool checked)
{
  if (_ui.radNodeCombo->currentIndex() >= 0 && _annotations.size() > _ui.radNodeCombo->currentIndex())
    centerMapOnNode(_annotations[_ui.radNodeCombo->currentIndex()]);
}