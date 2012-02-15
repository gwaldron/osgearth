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

namespace
{
  class LOSPointDraggerCallback : public osgManipulator::DraggerCallback
  {
  public:
    LOSPointDraggerCallback(osgEarth::Map* map, LOSCreationDialog* dialog,  LOSCreationDialog::LOSPoint point):
      _dialog(dialog),
      _point(point)
    {
        _ellipsoid = map->getProfile()->getSRS()->getEllipsoid();
    }

    osg::Vec3d getLocation(const osg::Matrixd& matrix)
    {
        osg::Vec3d trans = matrix.getTrans();
        double lat, lon, height;
        _ellipsoid->convertXYZToLatLongHeight(trans.x(), trans.y(), trans.z(), lat, lon, height);
        return osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), height);
    }


    virtual bool receive(const osgManipulator::MotionCommand& command)
    {
        switch (command.getStage())
        {
          case osgManipulator::MotionCommand::START:
          {
            // Save the current matrix
            osg::Vec3d location;
            _dialog->getLOSPoint(_point, location);

            double x, y, z;
            _ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(location.y()), osg::DegreesToRadians(location.x()), location.z(), x, y, z);
            _startMotionMatrix = osg::Matrixd::translate(x, y, z);

            // Get the LocalToWorld and WorldToLocal matrix for this node.
            osg::NodePath nodePathToRoot;
            _localToWorld = osg::Matrixd::identity();
            _worldToLocal = osg::Matrixd::identity();

            return true;
          }
          case osgManipulator::MotionCommand::MOVE:
          {                  
            // Transform the command's motion matrix into local motion matrix.
            osg::Matrix localMotionMatrix = _localToWorld * command.getWorldToLocal()
                * command.getMotionMatrix()
                * command.getLocalToWorld() * _worldToLocal;

            osg::Matrixd newMatrix = localMotionMatrix * _startMotionMatrix;

            osg::Vec3d location = getLocation( newMatrix );
            if (_dialog->isAltitudeRelative(_point))
            {
              osg::Vec3d losLoc;
              _dialog->getLOSPoint(_point, losLoc, true);
              double z = losLoc.z();
              location = osg::Vec3d(location.x(), location.y(), z);
            }

            _dialog->setLOSPoint(_point, location);
            return true;
          }
          case osgManipulator::MotionCommand::FINISH:
          {
            return true;
          }
          case osgManipulator::MotionCommand::NONE:
          default:
            return false;
        }
    }


    osg::ref_ptr<const osg::EllipsoidModel> _ellipsoid;
    LOSCreationDialog* _dialog;
    LOSCreationDialog::LOSPoint _point;
    bool _start;

    osg::Matrix _startMotionMatrix;

    osg::Matrix _localToWorld;
    osg::Matrix _worldToLocal;
  };
}

LOSCreationDialog::LOSCreationDialog(osgEarth::MapNode* mapNode, osg::Group* root, int losCount, osgEarth::QtGui::DataManager *manager, ViewVector* views)
  : _mapNode(mapNode), _root(root), _manager(manager), _views(views), _activeButton(0L), _updatingUi(false)
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

  // create map point draggers
  _p1Dragger  = new LOSIntersectingDragger;
  _p1Dragger->setNode( _mapNode );    
  _p1Dragger->setHandleEvents( true );
  _p1Dragger->addDraggerCallback(new LOSPointDraggerCallback(_map, this, LOSPoint::P2P_START));
  _p1Dragger->setColor(osg::Vec4(0,1,0,0));
  _p1Dragger->setPickColor(osg::Vec4(1,0,1,0));
  _p1Dragger->setupDefaultGeometry();
  _p1BaseAlt = 0.0;

  _p2Dragger  = new LOSIntersectingDragger;
  _p2Dragger->setNode( _mapNode );    
  _p2Dragger->setHandleEvents( true );
  _p2Dragger->addDraggerCallback(new LOSPointDraggerCallback(_map, this, LOSPoint::P2P_END));    
  _p2Dragger->setColor(osg::Vec4(0,1,0,0));
  _p2Dragger->setPickColor(osg::Vec4(1,0,1,0));
  _p2Dragger->setupDefaultGeometry();
  _p2BaseAlt = 0.0;

  _radDragger  = new LOSIntersectingDragger;
  _radDragger->setNode( _mapNode );    
  _radDragger->setHandleEvents( true );
  _radDragger->addDraggerCallback(new LOSPointDraggerCallback(_map, this, LOSPoint::RADIAL_CENTER));    
  _radDragger->setColor(osg::Vec4(0,1,0,0));
  _radDragger->setPickColor(osg::Vec4(1,0,1,0));
  _radDragger->setupDefaultGeometry();
  _radBaseAlt = 0.0;

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

  // connect lat/lon/alt value change signals
  connect(_ui.p1LatBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));
  connect(_ui.p1LonBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));
  connect(_ui.p1AltBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));

  connect(_ui.p2LatBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));
  connect(_ui.p2LonBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));
  connect(_ui.p2AltBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));

  connect(_ui.radLatBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));
  connect(_ui.radLonBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));
  connect(_ui.radAltBox, SIGNAL(valueChanged(double)), this, SLOT(onLocationValueChanged(double)));

  connect(_ui.typeTabs, SIGNAL(currentChanged(int)), this, SLOT(onCurrentTabChanged(int)));

  // simulate type change to force UI update
  onP1TypeChange("Point");
  onP2TypeChange("Point");
  onRadTypeChange("Point");

  onCurrentTabChanged(0);

  this->resize(721, this->height());
}

void LOSCreationDialog::mapClick(const osg::Vec3d& point)
{
  if (_activeButton)
  {
    // transform point to map coordinates:
    osg::Vec3d lla;
    _map->worldPointToMapPoint(point, lla);

    if (_activeButton == _ui.p1PointButton)
    {
      _p1BaseAlt = lla.z();

      _ui.p1LonBox->setValue(lla.x());
      _ui.p1LatBox->setValue(lla.y());

      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Unchecked)
        _ui.p1AltBox->setValue((int)(lla.z()) + 1.0);
    }
    else if (_activeButton == _ui.p2PointButton)
    {
      _p2BaseAlt = lla.z();

      _ui.p2LonBox->setValue(lla.x());
      _ui.p2LatBox->setValue(lla.y());

      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Unchecked)
        _ui.p2AltBox->setValue((int)(lla.z()) + 1.0);
    }
    else if (_activeButton == _ui.radPointButton)
    {
      _radBaseAlt = lla.z();

      _ui.radLonBox->setValue(lla.x());
      _ui.radLatBox->setValue(lla.y());

      if (_ui.radRelativeCheckBox->checkState() == Qt::Unchecked)
        _ui.radAltBox->setValue((int)(lla.z()) + 1.0);
    }
    
    _activeButton = 0L;
    this->setEnabled(true);
  }
}

void LOSCreationDialog::updateDragger(osgEarth::Annotation::IntersectingDragger* dragger, const osg::Vec3d& point)
{
  const osg::EllipsoidModel* em = _map->getProfile()->getSRS()->getEllipsoid();

  osg::Matrixd matrix;
  em->computeLocalToWorldTransformFromXYZ(point.x(), point.y(), point.z(), matrix);
  dragger->setMatrix(matrix);
}

void LOSCreationDialog::getLOSPoint(LOSPoint point, osg::Vec3d& out_point, bool relative)
{
  double alt = 0.0;

  switch(point)
  {
    case LOSPoint::P2P_START:
      alt = _ui.p1AltBox->value();
      if (!relative && _ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
        alt += _p1BaseAlt;
      out_point.set(_ui.p1LonBox->value(), _ui.p1LatBox->value(), alt);
      break;
    case LOSPoint::P2P_END:
      alt = _ui.p2AltBox->value();
      if (!relative && _ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
        alt += _p2BaseAlt;
      out_point.set(_ui.p2LonBox->value(), _ui.p2LatBox->value(), alt);
      break;
    case LOSPoint::RADIAL_CENTER:
      alt = _ui.radAltBox->value();
      if (!relative && _ui.radRelativeCheckBox->checkState() == Qt::Checked)
        alt += _radBaseAlt;
      out_point.set(_ui.radLonBox->value(), _ui.radLatBox->value(), alt);
      break;
  }
}

void LOSCreationDialog::setLOSPoint(LOSPoint point, const osg::Vec3d& value)
{
  _updatingUi = true;
  switch(point)
  {
    case LOSPoint::P2P_START:
      _ui.p1LatBox->setValue(value.y());
      _ui.p1LonBox->setValue(value.x());
      _ui.p1AltBox->setValue(value.z());
      break;
    case LOSPoint::P2P_END:
      _ui.p2LatBox->setValue(value.y());
      _ui.p2LonBox->setValue(value.x());
      _ui.p2AltBox->setValue(value.z());
      break;
    case LOSPoint::RADIAL_CENTER:
      _ui.radLatBox->setValue(value.y());
      _ui.radLonBox->setValue(value.x());
      _ui.radAltBox->setValue(value.z());
      break;
  }
  _updatingUi = false;
}

bool LOSCreationDialog::isAltitudeRelative(LOSPoint point)
{
  switch(point)
  {
    case LOSPoint::P2P_START:
    case LOSPoint::P2P_END:
      return _ui.p2pRelativeCheckBox->checkState() == Qt::Checked;
    case LOSPoint::RADIAL_CENTER:
      return _ui.radRelativeCheckBox->checkState() == Qt::Checked;
  }

  return false;
}

void LOSCreationDialog::closeEvent(QCloseEvent* event)
{
  cleanupDraggers();
  QDialog::closeEvent(event);
}

void LOSCreationDialog::accept()
{
  if (doClose())
  {
    cleanupDraggers();
    QDialog::accept();
  }
}

void LOSCreationDialog::reject()
{
  cleanupDraggers();
  QDialog::reject();
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
          los->setAltitudeMode(osgEarth::Util::ALTITUDE_RELATIVE);

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
          los->setAltitudeMode(osgEarth::Util::ALTITUDE_RELATIVE);

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
          los->setAltitudeMode(osgEarth::Util::ALTITUDE_RELATIVE);

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

void LOSCreationDialog::cleanupDraggers()
{
  if (_root.valid())
  {
    _root->removeChild(_p1Dragger);
    _root->removeChild(_p2Dragger);
    _root->removeChild(_radDragger);
  }
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

      osg::Vec3d output;
      _map->worldPointToMapPoint(center, output);

      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(output, 0.0, -90.0, 1e5), *_views));
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

void LOSCreationDialog::onLocationValueChanged(double d)
{
  if (!_updatingUi)
  {
    QObject* s = sender();

    if (s == _ui.p1LatBox || s == _ui.p1LonBox || s == _ui.p1AltBox)
    {
      double alt = _ui.p1AltBox->value();
      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
      {
        _p1Dragger->setHeightAboveTerrain(_ui.p1AltBox->value());
        alt += _p1BaseAlt;
      }

      double x, y, z;
      _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(osg::DegreesToRadians(_ui.p1LatBox->value()), osg::DegreesToRadians(_ui.p1LonBox->value()), alt, x, y, z);
      updateDragger(_p1Dragger, osg::Vec3d(x, y, z));
    }
    else if (s == _ui.p2LatBox || s == _ui.p2LonBox || s == _ui.p2AltBox)
    {
      double alt = _ui.p2AltBox->value();
      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
      {
        _p2Dragger->setHeightAboveTerrain(_ui.p2AltBox->value());
        alt += _p2BaseAlt;
      }

      double x, y, z;
      _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(osg::DegreesToRadians(_ui.p2LatBox->value()), osg::DegreesToRadians(_ui.p2LonBox->value()), alt, x, y, z);
      updateDragger(_p2Dragger, osg::Vec3d(x, y, z));
    }
    else if (s == _ui.radLatBox || s == _ui.radLonBox || s == _ui.radAltBox)
    {
      double alt = _ui.radAltBox->value();
      if (_ui.radRelativeCheckBox->checkState() == Qt::Checked)
      {
        _radDragger->setHeightAboveTerrain(_ui.radAltBox->value());
        alt += _radBaseAlt;
      }

      double x, y, z;
      _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(osg::DegreesToRadians(_ui.radLatBox->value()), osg::DegreesToRadians(_ui.radLonBox->value()), alt, x, y, z);
      updateDragger(_radDragger, osg::Vec3d(x, y, z));
    }
  }
}

void LOSCreationDialog::onCurrentTabChanged(int index)
{
  if (_root.valid())
  {
    _root->removeChild(_radDragger);

    if (_ui.typeTabs->tabText(index) == "Point-to-Point")
    {
      if (!_root->containsNode(_p1Dragger))
        _root->addChild(_p1Dragger);

      if (!_root->containsNode(_p2Dragger))
        _root->addChild(_p2Dragger);
    }
    else if (_ui.typeTabs->tabText(index) == "Radial")
    {
      _root->removeChild(_p1Dragger);
      _root->removeChild(_p2Dragger);

      if (!_root->containsNode(_radDragger))
        _root->addChild(_radDragger);
    }
  }
}