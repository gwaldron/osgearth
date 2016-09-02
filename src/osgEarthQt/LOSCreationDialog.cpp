/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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
    class LOSPointDraggerCallback : public Dragger::PositionChangedCallback
  {
  public:
    LOSPointDraggerCallback(osgEarth::Map* map, LOSCreationDialog* dialog,  LOSCreationDialog::LOSPoint point):
      _dialog(dialog),
      _point(point)
    {
    }

      virtual void onPositionChanged(const Dragger* sender, const osgEarth::GeoPoint& position)
      {
          GeoPoint location(position);
          if (_dialog->isAltitudeRelative(_point))
          {
              osg::Vec3d losLoc;
              _dialog->getLOSPoint(_point, losLoc, true);
              double z = losLoc.z();
              //location = osg::Vec3d(location.x(), location.y(), location.z() - z);
              location.z() = z;
          }
          _dialog->setLOSPoint(_point, location.vec3d());
      }

    LOSCreationDialog* _dialog;
    LOSCreationDialog::LOSPoint _point;
    bool _start;
  };
}

LOSCreationDialog::LOSCreationDialog(osgEarth::MapNode* mapNode, osg::Group* root, int losCount, osgEarth::QtGui::DataManager *manager, ViewVector* views)
  : _mapNode(mapNode), _root(root), _manager(manager), _views(views), _activeButton(0L), _updatingUi(false), _updateAlt(true)
{
  if (_mapNode.valid())
    _map = _mapNode->getMap();

  if (_manager.valid())
    _manager->getAnnotations(_annotations);

  // create a default name
  std::stringstream losName;
  losName << "Line-of-Sight " << losCount;

  initUi(losName.str());
}

LOSCreationDialog::LOSCreationDialog(osgEarth::MapNode* mapNode, osg::Group* root, osg::Group* losNode, const std::string& name, osgEarth::QtGui::DataManager *manager, ViewVector* views)
  : _mapNode(mapNode), _root(root), _manager(manager), _views(views), _activeButton(0L), _updatingUi(false), _updateAlt(true)
{
  if (_mapNode.valid())
    _map = _mapNode->getMap();

  if (_manager.valid())
    _manager->getAnnotations(_annotations);

  initUi(name, losNode);
}

void LOSCreationDialog::initUi(const std::string& name, osg::Group* los)
{
  // setup UI
  _ui.setupUi(this);

  // set the name
  _ui.nameBox->setText(tr(name.c_str()));

  // fill annotation list
  // TODO: if more node types beyond Annotation are added, this will need to be moved
  //       to the on__TypeChange() methods.  Currently doing here for efficiency.
  for (AnnotationVector::const_iterator it = _annotations.begin(); it != _annotations.end(); ++it)
  {
    std::string annoName = (*it)->getName();
    if ( annoName.empty() ) annoName = "Annotation";
    _ui.p1NodeCombo->addItem(tr(annoName.c_str()));
    _ui.p2NodeCombo->addItem(tr(annoName.c_str()));
    _ui.radNodeCombo->addItem(tr(annoName.c_str()));
  }


  // create map point draggers
  _p1Dragger  = new LOSIntersectingDragger( _mapNode );
  _p1Dragger->addPositionChangedCallback(new LOSPointDraggerCallback(_map, this, P2P_START));
  _p1Dragger->setColor(osg::Vec4(0,1,1,0));
  _p1Dragger->setPickColor(osg::Vec4(1,0,1,0));
  _p1BaseAlt = 0.0;

  _p2Dragger  = new LOSIntersectingDragger(_mapNode);
  _p2Dragger->addPositionChangedCallback(new LOSPointDraggerCallback(_map, this, P2P_END));    
  _p2Dragger->setColor(osg::Vec4(0,1,1,0));
  _p2Dragger->setPickColor(osg::Vec4(1,0,1,0));
  _p2BaseAlt = 0.0;

  _radDragger  = new LOSIntersectingDragger(_mapNode);
  _radDragger->addPositionChangedCallback(new LOSPointDraggerCallback(_map, this, RADIAL_CENTER));    
  _radDragger->setColor(osg::Vec4(0,1,1,0));
  _radDragger->setPickColor(osg::Vec4(1,0,1,0));
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

  connect(_ui.p2pRelativeCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onRelativeCheckChanged(int)));
  connect(_ui.radRelativeCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onRelativeCheckChanged(int)));

  // connect annotation combobox signals
  connect(_ui.p1NodeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onNodeComboChange(const QString&)));
  connect(_ui.p2NodeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onNodeComboChange(const QString&)));
  connect(_ui.radNodeCombo, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onNodeComboChange(const QString&)));

  connect(_ui.depthTestCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onDepthTestChanged(int)));
  connect(_ui.typeTabs, SIGNAL(currentChanged(int)), this, SLOT(onCurrentTabChanged(int)));

  // connect radial specific signals
  connect(_ui.spokesSpinBox, SIGNAL(valueChanged(int)), this, SLOT(onSpokesBoxChanged(int)));
  connect(_ui.radiusSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onRadiusBoxChanged(double)));


  // Create los nodes and initialize
  _p2p = new osgEarth::Util::LinearLineOfSightNode(_mapNode.get());
  _radial = new osgEarth::Util::RadialLineOfSightNode(_mapNode.get());

  // simulate type change to force UI update
  onP1TypeChange("Point");
  onP2TypeChange("Point");
  onRadTypeChange("Point");

  onCurrentTabChanged(0);


  // If an los node was passed in, initialize components accordingly
  if (los)
  {
    osgEarth::Util::LinearLineOfSightNode* p2pNode = dynamic_cast<osgEarth::Util::LinearLineOfSightNode*>(los);
    if (p2pNode)
    {
      _ui.typeTabs->setCurrentIndex(0);
      _ui.typeTabs->setTabEnabled(1, false);

      _ui.depthTestCheckBox->setCheckState(p2pNode->getOrCreateStateSet()->getMode(GL_DEPTH_TEST) == osg::StateAttribute::OFF ? Qt::Unchecked : Qt::Checked);

      bool p1Set=false, p2Set=false;
      
      osgEarth::Util::LineOfSightTether* tether = dynamic_cast<osgEarth::Util::LineOfSightTether*>(p2pNode->getUpdateCallback());
      if (tether)
      {
        if (tether->startNode())
        {
          _ui.p1TypeCombo->setCurrentIndex(1); // "Annotation"

          int i = findAnnotationIndex(tether->startNode());
          if (i >= 0)
            _ui.p1NodeCombo->setCurrentIndex(i);
          else
            _ui.p1NodeCombo->setCurrentIndex(0);  // unlikely case where annotation is not found (perhaps deleted)

          p1Set = true;
        }

        if (tether->endNode())
        {
          _ui.p2TypeCombo->setCurrentIndex(1); // "Annotation"

          int i = findAnnotationIndex(tether->endNode());
          if (i >= 0)
            _ui.p2NodeCombo->setCurrentIndex(i);
          else
            _ui.p2NodeCombo->setCurrentIndex(0);  // unlikely case where annotation is not found (perhaps deleted)

          p2Set = true;
        }
      }

      if (!p1Set)
      {
        _ui.p1TypeCombo->setCurrentIndex(0); // "Point"
        onP1TypeChange("Point");  // Necessary to init UI because above setCurrentIndex call will not
        updateDraggerNodes();     // fire an event since the index defaults to 0

        bool isRelative = (p2pNode->getStart().altitudeMode() == ALTMODE_RELATIVE);
        _ui.p2pRelativeCheckBox->setChecked( isRelative );
        //_ui.p2pRelativeCheckBox->setChecked(p2pNode->getStartAltitudeMode() == ALTMODE_RELATIVE);

        osg::Vec3d pos(p2pNode->getStart().vec3d());

        if (isRelative) //p2pNode->getStartAltitudeMode() == ALTMODE_RELATIVE)
        {
          double hat = pos.z();

          double height;
          if (_mapNode->getTerrain()->getHeight(_mapNode->getMapSRS(), pos.x(), pos.y(), &height))
            pos.set(pos.x(), pos.y(), height);

          setLOSPoint(P2P_START, pos, true);

          if (_ui.p1AltBox->value() == hat)
          {
            updatePoint(P2P_START);
            updateLOSNodes();
          }
          else
          {
            _ui.p1AltBox->setValue(hat);
          }
        }
        else
        {
          setLOSPoint(P2P_START, pos, true);
        }
      }

      if (!p2Set)
      {
        _ui.p2TypeCombo->setCurrentIndex(0); // "Point"
        onP2TypeChange("Point");  // Necessary to init UI because above setCurrentIndex call will not
        updateDraggerNodes();     // fire an event since the index defaults to 0

        bool isRelative = (p2pNode->getEnd().altitudeMode() == ALTMODE_RELATIVE);
        _ui.p2pRelativeCheckBox->setChecked(isRelative);

        osg::Vec3d pos(p2pNode->getEnd().vec3d());

        if (isRelative) //p2pNode->getEndAltitudeMode() == ALTMODE_RELATIVE)
        {
          double hat = pos.z();

          double height;
          if (_mapNode->getTerrain()->getHeight(_mapNode->getMapSRS(), pos.x(), pos.y(), &height))
            pos.set(pos.x(), pos.y(), height);

          setLOSPoint(P2P_END, pos, true);

          if (_ui.p2AltBox->value() == hat)
          {
            updatePoint(P2P_END);
            updateLOSNodes();
          }
          else
          {
            _ui.p2AltBox->setValue(hat);
          }
        }
        else
        {
          setLOSPoint(P2P_END, pos, true);
        }
      }
    }
    else
    {
      osgEarth::Util::RadialLineOfSightNode* radNode = dynamic_cast<osgEarth::Util::RadialLineOfSightNode*>(los);
      if (radNode)
      {
        _ui.typeTabs->setCurrentIndex(1);
        _ui.typeTabs->setTabEnabled(0, false);

        _ui.radiusSpinBox->setValue(radNode->getRadius());
        _ui.spokesSpinBox->setValue(radNode->getNumSpokes());

        _ui.depthTestCheckBox->setCheckState(radNode->getOrCreateStateSet()->getMode(GL_DEPTH_TEST) == osg::StateAttribute::OFF ? Qt::Unchecked : Qt::Checked);

        osgEarth::Util::RadialLineOfSightTether* tether = dynamic_cast<osgEarth::Util::RadialLineOfSightTether*>(radNode->getUpdateCallback());
        if (tether)
        {
          _ui.radTypeCombo->setCurrentIndex(1); // "Annotation"

          int i = findAnnotationIndex(tether->node());
          if (i >= 0)
            _ui.radNodeCombo->setCurrentIndex(i);
          else
            _ui.radNodeCombo->setCurrentIndex(0);  // unlikely case where annotation is not found (perhaps deleted)
        }
        else
        {
          _ui.radTypeCombo->setCurrentIndex(0); // "Point"
          
          onRadTypeChange("Point"); // Necessary to init UI because above setCurrentIndex call will not
          updateDraggerNodes();     // fire an event since the index defaults to 0

          //_ui.radRelativeCheckBox->setChecked(radNode->getAltitudeMode() == ALTMODE_RELATIVE);
          bool isRelative = (radNode->getCenter().altitudeMode() == ALTMODE_RELATIVE);
          _ui.radRelativeCheckBox->setChecked( isRelative );

          osg::Vec3d pos(radNode->getCenter().vec3d());

          if (isRelative) //radNode->getAltitudeMode() == ALTMODE_RELATIVE)
          {
            double hat = pos.z();

            double height;
            if (_mapNode->getTerrain()->getHeight(_mapNode->getMapSRS(), pos.x(), pos.y(), &height))
              pos.set(pos.x(), pos.y(), height);

            setLOSPoint(RADIAL_CENTER, pos, true);
            
            if (_ui.radAltBox->value() == hat)
            {
              updatePoint(RADIAL_CENTER);
              updateLOSNodes();
            }
            else
            {
              _ui.radAltBox->setValue(hat);
            }
          }
          else
          {
            setLOSPoint(RADIAL_CENTER, pos, true);
          }
        }
      }
    }
  }

  this->resize(721, this->height());
}

void LOSCreationDialog::mapClick(const osg::Vec3d& point)
{
  if (_activeButton)
  {
    // transform point to map coordinates:
    osgEarth::GeoPoint outPoint;
    outPoint.fromWorld( _map->getSRS(), point );
    //_map->worldPointToMapPoint(point, outPoint);

    if (_activeButton == _ui.p1PointButton)
    {
      _p1BaseAlt = outPoint.z();

      _updateAlt = false;
      _ui.p1LonBox->setValue(outPoint.x());
      _ui.p1LatBox->setValue(outPoint.y());
      _updateAlt = true;

      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Unchecked)
        _ui.p1AltBox->setValue((int)(outPoint.z()) + 1.0);
    }
    else if (_activeButton == _ui.p2PointButton)
    {
      _p2BaseAlt = outPoint.z();

      _updateAlt = false;
      _ui.p2LonBox->setValue(outPoint.x());
      _ui.p2LatBox->setValue(outPoint.y());
      _updateAlt = true;

      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Unchecked)
        _ui.p2AltBox->setValue((int)(outPoint.z()) + 1.0);
    }
    else if (_activeButton == _ui.radPointButton)
    {
      _radBaseAlt = outPoint.z();

      _updateAlt = false;
      _ui.radLonBox->setValue(outPoint.x());
      _ui.radLatBox->setValue(outPoint.y());
      _updateAlt = true;

      if (_ui.radRelativeCheckBox->checkState() == Qt::Unchecked)
        _ui.radAltBox->setValue((int)(outPoint.z()) + 1.0);
    }
    
    _activeButton = 0L;
    this->setEnabled(true);
  }
}

void LOSCreationDialog::updateDragger(Dragger* dragger, const GeoPoint& point)
{
    dragger->setPosition( point, false );
}

void LOSCreationDialog::updateDraggerNodes()
{
  if (_root.valid())
  {
    if (_ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Point-to-Point")
    {
      _root->removeChild(_radDragger);

      if (_ui.p1TypeCombo->currentText() == "Point")
      {
        if (!_root->containsNode(_p1Dragger))
          _root->addChild(_p1Dragger);
      }
      else
      {
        _root->removeChild(_p1Dragger);
      }
        
      if (_ui.p2TypeCombo->currentText() == "Point")
      {
        if (!_root->containsNode(_p2Dragger))
          _root->addChild(_p2Dragger);
      }
      else
      {
        _root->removeChild(_p2Dragger);
      }
    }
    else if (_ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Radial")
    {
      _root->removeChild(_p1Dragger);
      _root->removeChild(_p2Dragger);

      if (_ui.radTypeCombo->currentText() == "Point")
      {
        if (!_root->containsNode(_radDragger))
          _root->addChild(_radDragger);
      }
      else
      {
        _root->removeChild(_radDragger);
      }
    }
  }
}

void LOSCreationDialog::updatePoint(LOSPoint point)
{
  switch(point)
  {
    case P2P_START:
      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
        _p1Dragger->setHeightAboveTerrain(_ui.p1AltBox->value());
      else
        _p1Dragger->setHeightAboveTerrain(0.0);

      updateDragger(_p1Dragger, GeoPoint(_mapNode->getMapSRS(), _ui.p1LonBox->value(), _ui.p1LatBox->value(), _p1BaseAlt, ALTMODE_RELATIVE));
      break;
    case P2P_END:
      if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
        _p2Dragger->setHeightAboveTerrain(_ui.p2AltBox->value());
      else
        _p2Dragger->setHeightAboveTerrain(0.0);

      updateDragger(_p2Dragger, GeoPoint(_mapNode->getMapSRS(), _ui.p2LonBox->value(), _ui.p2LatBox->value(), _p2BaseAlt, ALTMODE_RELATIVE));
      break;
    case RADIAL_CENTER:
      if (_ui.radRelativeCheckBox->checkState() == Qt::Checked)
        _radDragger->setHeightAboveTerrain(_ui.radAltBox->value());
      else
        _radDragger->setHeightAboveTerrain(0.0);

      updateDragger(_radDragger, GeoPoint(_mapNode->getMapSRS(), _ui.radLonBox->value(), _ui.radLatBox->value(), _radBaseAlt, ALTMODE_RELATIVE));
      break;
  }
}

int LOSCreationDialog::findAnnotationIndex(osg::Node* annotation)
{
  int index = 0;
  for (AnnotationVector::const_iterator it = _annotations.begin(); it != _annotations.end(); ++it)
  {
    if ((*it).get() == annotation)
      return index;

    index++;
  }

  return -1;
}

void LOSCreationDialog::getLOSPoint(LOSPoint point, osg::Vec3d& out_point, bool relative)
{
  double alt = 0.0;

  switch(point)
  {
    case P2P_START:
      alt = _ui.p1AltBox->value();
      if (!relative && _ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
        alt += _p1BaseAlt;
      out_point.set(_ui.p1LonBox->value(), _ui.p1LatBox->value(), alt);
      break;
    case P2P_END:
      alt = _ui.p2AltBox->value();
      if (!relative && _ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
        alt += _p2BaseAlt;
      out_point.set(_ui.p2LonBox->value(), _ui.p2LatBox->value(), alt);
      break;
    case RADIAL_CENTER:
      alt = _ui.radAltBox->value();
      if (!relative && _ui.radRelativeCheckBox->checkState() == Qt::Checked)
        alt += _radBaseAlt;
      out_point.set(_ui.radLonBox->value(), _ui.radLatBox->value(), alt);
      break;
  }
}

void LOSCreationDialog::setLOSPoint(LOSPoint point, const osg::Vec3d& value, bool updateUi)
{
  _updatingUi = !updateUi;
  switch(point)
  {
    case P2P_START:
      _ui.p1LatBox->setValue(value.y());
      _ui.p1LonBox->setValue(value.x());

      _p1BaseAlt = value.z();

      if (!isAltitudeRelative(point))
        _ui.p1AltBox->setValue(value.z());

      break;
    case P2P_END:
      _ui.p2LatBox->setValue(value.y());
      _ui.p2LonBox->setValue(value.x());

      _p2BaseAlt = value.z();

      if (!isAltitudeRelative(point))
        _ui.p2AltBox->setValue(value.z());

      break;
    case RADIAL_CENTER:
      _ui.radLatBox->setValue(value.y());
      _ui.radLonBox->setValue(value.x());

      _radBaseAlt = value.z();

      if (!isAltitudeRelative(point))
        _ui.radAltBox->setValue(value.z());

      break;
  }
  _updatingUi = false;
}

bool LOSCreationDialog::isAltitudeRelative(LOSPoint point)
{
  switch(point)
  {
    case P2P_START:
    case P2P_END:
      return _ui.p2pRelativeCheckBox->checkState() == Qt::Checked;
    case RADIAL_CENTER:
      return _ui.radRelativeCheckBox->checkState() == Qt::Checked;
  }

  return false;
}

void LOSCreationDialog::closeEvent(QCloseEvent* event)
{
  cleanupNodes();
  QDialog::closeEvent(event);
}

void LOSCreationDialog::accept()
{
  cleanupNodes();
  QDialog::accept();
}

void LOSCreationDialog::reject()
{
  cleanupNodes();
  QDialog::reject();
}

void LOSCreationDialog::updateLOSNodes(bool updateAll)
{
  if (_p2p.valid() && (updateAll || _ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Point-to-Point"))
  {
    if (_ui.depthTestCheckBox->checkState() == Qt::Checked)
      _p2p->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    else
      _p2p->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    bool p1Set = false;
    bool p2Set = false;
    osg::Node* p1Node = 0L;
    osg::Node* p2Node = 0L;

    // get start point or node
    if (_ui.p1TypeCombo->currentText() == "Point")
    {
      _p2p->setStart(GeoPoint(
          _mapNode->getMapSRS(), 
          _ui.p1LonBox->value(), 
          _ui.p1LatBox->value(), 
          _ui.p1AltBox->value(),
          _ui.p2pRelativeCheckBox->checkState() == Qt::Checked ? ALTMODE_RELATIVE : ALTMODE_ABSOLUTE) );

      //if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
      //  _p2p->setStartAltitudeMode(ALTMODE_RELATIVE);
      //else
      //  _p2p->setStartAltitudeMode(ALTMODE_ABSOLUTE);

      p1Set = true;
    }
    else if (_ui.p1TypeCombo->currentText() == "Annotation")
    {
      GeoPoint p = _p2p->getStart();
      p.altitudeMode() = ALTMODE_ABSOLUTE;
      _p2p->setStart( p );
      //_p2p->setStartAltitudeMode(ALTMODE_ABSOLUTE);
      p1Node = _annotations[_ui.p1NodeCombo->currentIndex()];
      p1Set = true;
    }

    // get end point or node
    if (_ui.p2TypeCombo->currentText() == "Point")
    {
      _p2p->setEnd(GeoPoint(
          _mapNode->getMapSRS(),
          _ui.p2LonBox->value(), 
          _ui.p2LatBox->value(), 
          _ui.p2AltBox->value(),
          _ui.p2pRelativeCheckBox->checkState() == Qt::Checked ? ALTMODE_RELATIVE : ALTMODE_ABSOLUTE) );

      //if (_ui.p2pRelativeCheckBox->checkState() == Qt::Checked)
      //  _p2p->setEndAltitudeMode(ALTMODE_RELATIVE);
      //else
      //  _p2p->setEndAltitudeMode(ALTMODE_ABSOLUTE);

      p2Set = true;
    }
    else if (_ui.p2TypeCombo->currentText() == "Annotation")
    {
      GeoPoint p = _p2p->getEnd();
      p.altitudeMode() = ALTMODE_ABSOLUTE;
      _p2p->setEnd( p );
      //_p2p->setEndAltitudeMode(ALTMODE_ABSOLUTE);
      p2Node = _annotations[_ui.p2NodeCombo->currentIndex()];
      p2Set = true;
    }

    // set update callback if tethered, else clear it
    if (p1Node || p2Node)
      _p2p->setUpdateCallback(new osgEarth::Util::LineOfSightTether(p1Node, p2Node));
    else
      _p2p->setUpdateCallback(0L);
  }
  
  if (_radial.valid() && (updateAll || _ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Radial"))
  {
    _radial->setRadius(_ui.radiusSpinBox->value());
    _radial->setNumSpokes(_ui.spokesSpinBox->value());

    if (_ui.depthTestCheckBox->checkState() == Qt::Checked)
      _radial->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    else
      _radial->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    // get center point or node to attach to
    if (_ui.radTypeCombo->currentText() == "Point")
    {
      _radial->setCenter( GeoPoint(
          _mapNode->getMapSRS(),
          _ui.radLonBox->value(),
          _ui.radLatBox->value(),
          _ui.radAltBox->value(),
          _ui.radRelativeCheckBox->checkState() == Qt::Checked ? ALTMODE_RELATIVE : ALTMODE_ABSOLUTE) );

      //_radial->setCenter(osg::Vec3d(_ui.radLonBox->value(), _ui.radLatBox->value(), _ui.radAltBox->value()));

      //if (_ui.radRelativeCheckBox->checkState() == Qt::Checked)
      //  _radial->setAltitudeMode(ALTMODE_RELATIVE);
      //else
      //  _radial->setAltitudeMode(ALTMODE_ABSOLUTE);

      // clear update callback
      _radial->setUpdateCallback(0L);
    }
    else if (_ui.radTypeCombo->currentText() == "Annotation")
    {
      GeoPoint p = _radial->getCenter();
      p.altitudeMode() = ALTMODE_ABSOLUTE;
      _radial->setCenter( p );
      //_radial->setAltitudeMode(ALTMODE_ABSOLUTE);
      _radial->setUpdateCallback(new osgEarth::Util::RadialLineOfSightTether(_annotations[_ui.radNodeCombo->currentIndex()]));
    }
  }
}

void LOSCreationDialog::cleanupNodes()
{
  if (_root.valid())
  {
    _root->removeChild(_p1Dragger);
    _root->removeChild(_p2Dragger);
    _root->removeChild(_radDragger);

    _root->removeChild(_p2p);
    _root->removeChild(_radial);
  }
}

void LOSCreationDialog::centerMapOnNode(osg::Node* node)
{
  if (node && _map.valid() && _manager.valid() && _views)
  {
    AnnotationNode* annoNode = dynamic_cast<AnnotationNode*>(node);
    if (annoNode)
    {
      osgEarth::Viewpoint vp;
      vp.setNode( annoNode );
      _manager->doAction(this, new SetViewpointAction(vp, *_views));
    }
    else
    {
      osg::Vec3d center = node->getBound().center();

      GeoPoint output;
      output.fromWorld( _map->getSRS(), center );
      //_map->worldPointToMapPoint(center, output);

      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(
          "center", output.x(), output.y(), output.z(), 0.0, -90.0, 1e5), *_views));
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

  updateDraggerNodes();
  updateLOSNodes();
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

  updateDraggerNodes();
  updateLOSNodes();
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

  updateDraggerNodes();
  updateLOSNodes();
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
  if (_ui.p1NodeCombo->currentIndex() >= 0 && (int)_annotations.size() > _ui.p1NodeCombo->currentIndex())
    centerMapOnNode(_annotations[_ui.p1NodeCombo->currentIndex()]);
}

void LOSCreationDialog::onP2FindNodeButtonClicked(bool checked)
{
  if (_ui.p2NodeCombo->currentIndex() >= 0 && (int)_annotations.size() > _ui.p2NodeCombo->currentIndex())
    centerMapOnNode(_annotations[_ui.p2NodeCombo->currentIndex()]);
}

void LOSCreationDialog::onRadFindNodeButtonClicked(bool checked)
{
  if (_ui.radNodeCombo->currentIndex() >= 0 && (int)_annotations.size() > _ui.radNodeCombo->currentIndex())
    centerMapOnNode(_annotations[_ui.radNodeCombo->currentIndex()]);
}

void LOSCreationDialog::onLocationValueChanged(double d)
{
  if (!_updatingUi)
  {
    QObject* s = sender();

    if (s == _ui.p1LatBox || s == _ui.p1LonBox)
    {
      if (_updateAlt)
      {
        double alt;
        if (_mapNode->getTerrain()->getHeight(_mapNode->getMapSRS(), _ui.p1LonBox->value(), _ui.p1LatBox->value(), &alt))
          _p1BaseAlt = alt;
      }

      updatePoint(P2P_START);
    }
    else if (s == _ui.p1AltBox)
    {
      updatePoint(P2P_START);
    }
    else if (s == _ui.p2LatBox || s == _ui.p2LonBox)
    {
      if (_updateAlt)
      {
        double alt;
        if (_mapNode->getTerrain()->getHeight(_mapNode->getMapSRS(), _ui.p2LonBox->value(), _ui.p2LatBox->value(), &alt))
          _p2BaseAlt = alt;
      }

      updatePoint(P2P_END);
    }
    else if (s == _ui.p2AltBox)
    {
      updatePoint(P2P_END);
    }
    else if (s == _ui.radLatBox || s == _ui.radLonBox)
    {
      if (_updateAlt)
      {
        double alt;
        if (_mapNode->getTerrain()->getHeight(_mapNode->getMapSRS(), _ui.radLonBox->value(), _ui.radLatBox->value(), &alt))
          _radBaseAlt = alt;
      }

      updatePoint(RADIAL_CENTER);
    }
    else if (s == _ui.radAltBox)
    {
      updatePoint(RADIAL_CENTER);
    }
  }

  updateLOSNodes();
}

void LOSCreationDialog::onRelativeCheckChanged(int state)
{
  if (!_updatingUi)
  {
    QObject* s = sender();

    if (s == _ui.p2pRelativeCheckBox)
    {
      updatePoint(P2P_START);
      updatePoint(P2P_END);
    }
    else if (s == _ui.radRelativeCheckBox)
    {
      updatePoint(RADIAL_CENTER);
    }
  }

  updateDraggerNodes();
  updateLOSNodes();
}

void LOSCreationDialog::onNodeComboChange(const QString& text)
{
  updateLOSNodes();
}

void LOSCreationDialog::onDepthTestChanged(int state)
{
  updateLOSNodes();
}

void LOSCreationDialog::onCurrentTabChanged(int index)
{
  if (_ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Point-to-Point")
  {
    _node = _p2p;

    if (_root.valid())
    {
      _root->removeChild(_radial);
      _root->addChild(_p2p);
    }
  }
  else if (_ui.typeTabs->tabText(_ui.typeTabs->currentIndex()) == "Radial")
  {
    _node = _radial;

    if (_root.valid())
    {
      _root->removeChild(_p2p);
      _root->addChild(_radial);
    }
  }

  updateDraggerNodes();
}

void LOSCreationDialog::onSpokesBoxChanged(int value)
{
  updateLOSNodes();
}

void LOSCreationDialog::onRadiusBoxChanged(double value)
{
  updateLOSNodes();
}
