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

#include <osgEarthQt/LOSControlWidget>
#include <osgEarthQt/Common>
#include <osgEarthQt/GuiActions>
#include <osgEarthQt/LOSCreationDialog>

#include <osgEarthUtil/LineOfSight>

#include <QAction>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QScrollArea>
#include <QSizePolicy>
#include <QString>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>


using namespace osgEarth;
using namespace osgEarth::QtGui;

namespace
{
  class LOSListWidgetItem : public QListWidgetItem
  {
  public:
    LOSListWidgetItem(osg::Group* los, const std::string& name) : QListWidgetItem(LOSControlWidget::tr(name.c_str())), _los(los), _name(name) { }

    osg::Group* los() { return _los.get(); }
    const std::string& name() { return _name; }

    void setLOS(osg::Group* los) { _los = los; }

    void setName(const std::string& name)
    {
      _name = name;
      this->setText(QString(name.c_str()));
    }

  private:
    osg::ref_ptr<osg::Group> _los;
    std::string _name;
  };
}


LOSControlWidget::LOSControlWidget(osg::Group* root, osgEarth::MapNode* mapNode, DataManager* dm)
  : _root(root), _mapNode(mapNode), _manager(dm), _losCounter(1)
{
  if (_mapNode.valid())
  {
    _map = _mapNode->getMap();
    _guiHandler = new LOSControlMouseHandler(this, _mapNode.get());
  }

  initialize();
}

void LOSControlWidget::setActiveView(osgViewer::View* view)
{
  removeViews();
  addView(view);
}

void LOSControlWidget::setActiveViews(const ViewVector& views)
{
  removeViews();

  for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
    addView(*it);
}

void LOSControlWidget::addView(osgViewer::View* view)
{
  view->addEventHandler(_guiHandler.get());
  _views.push_back(view);
}

void LOSControlWidget::removeViews()
{
  for (ViewVector::iterator it = _views.begin(); it != _views.end(); ++it)
    (*it)->removeEventHandler(_guiHandler.get());

  _views.clear();
}

void LOSControlWidget::initialize()
{
  //create list container
  QWidget* listBox = new QWidget;
  QVBoxLayout* listBoxLayout = new QVBoxLayout;
  listBoxLayout->setSpacing(0);
  listBoxLayout->setContentsMargins(0, 0, 0, 0);
  listBox->setLayout(listBoxLayout);

  //create toolbar with add/remove actions
  QToolBar* listBar = new QToolBar;
  listBar->setIconSize(QSize(16, 16));

  QAction* addAction = new QAction(QIcon(":/images/plus.png"), tr("Add LOS"), this);
  addAction->setStatusTip(tr("Add LOS"));
  addAction->setEnabled(_root.valid());
  connect(addAction, SIGNAL(triggered()), this, SLOT(onAddLOS()));
  listBar->addAction(addAction);

  _removeAction = new QAction(QIcon(":/images/minus.png"), tr("Remove LOS"), this);
  _removeAction->setStatusTip(tr("Remove selected LOS"));
  _removeAction->setEnabled(false);
  connect(_removeAction, SIGNAL(triggered()), this, SLOT(onRemoveSelectedLOS()));
  listBar->addAction(_removeAction);

  _editAction = new QAction(QIcon(":/images/edit.png"), tr("Edit LOS"), this);
  _editAction->setStatusTip(tr("Edit selected LOS"));
  _editAction->setEnabled(false);
  connect(_editAction, SIGNAL(triggered()), this, SLOT(onEditSelectedLOS()));
  listBar->addAction(_editAction);


  listBoxLayout->addWidget(listBar);

  //create list
  _losList = new QListWidget();
  _losList->setSelectionMode(QAbstractItemView::ExtendedSelection);
  listBoxLayout->addWidget(_losList);

  setPrimaryWidget(listBox);
  setPrimaryTitle("Line-of-Sight");

  //create details panel
  _detailsBox = new QFrame;
  QGridLayout* detailsLayout = new QGridLayout;
  detailsLayout->setSpacing(4);
  detailsLayout->setContentsMargins(2, 2, 2, 2);
  _detailsBox->setLayout(detailsLayout);
  _detailsBox->setObjectName("oeFrameContainer");

  detailsLayout->addWidget(new QLabel(tr("Name:")), 0, 0, Qt::AlignLeft);
  _nameField = new QLabel(tr("-----"));
  detailsLayout->addWidget(_nameField, 0, 1, Qt::AlignLeft);

  detailsLayout->addWidget(new QLabel(tr("Type:")), 1, 0, Qt::AlignLeft);
  _typeField = new QLabel(tr("-----"));
  detailsLayout->addWidget(_typeField, 1, 1, Qt::AlignLeft);

  _depthBox = new QCheckBox(tr("Enable depth test"));
  detailsLayout->addWidget(_depthBox, 2, 0, Qt::AlignLeft);
  _depthBox->setEnabled(false);

  detailsLayout->addWidget(new QLabel(tr("Radius:")), 4, 0, Qt::AlignLeft);
  _radiusBox = new QDoubleSpinBox;
  _radiusBox->setMinimum(0.1);
  _radiusBox->setMaximum(40075160.0);
  _radiusBox->setSingleStep(1.0);
  _radiusBox->setValue(2000.0);
  detailsLayout->addWidget(_radiusBox, 4, 1, Qt::AlignLeft);
  _radiusBox->setEnabled(false);

  detailsLayout->addWidget(new QLabel(tr("Spokes:")), 5, 0, Qt::AlignLeft);
  _spokesBox = new QSpinBox;
  _spokesBox->setMinimum(3);
  _spokesBox->setMaximum(10000);
  _spokesBox->setSingleStep(1);
  _spokesBox->setValue(100);
  detailsLayout->addWidget(_spokesBox, 5, 1, Qt::AlignLeft);
  _spokesBox->setEnabled(false);

  setSecondaryWidget(_detailsBox);
  setSecondaryTitle("Details");

  //connect list events
  connect(_losList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
  connect(_losList, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(onItemChanged(QListWidgetItem*)));
  connect(_losList, SIGNAL(itemSelectionChanged()), this, SLOT(onItemSelectionChanged()));

  //connect option signals
  connect(_depthBox, SIGNAL(stateChanged(int)), this, SLOT(onDepthBoxChanged(int)));
  connect(_radiusBox, SIGNAL(valueChanged(double)), this, SLOT(onRadiusValueChanged(double)));
  connect(_spokesBox, SIGNAL(valueChanged(int)), this, SLOT(onSpokesValueChanged(int)));
}

void LOSControlWidget::addLOSNode(osg::Group* los, const std::string& name)
{
  if (!los)
    return;

  //_losNodes.push_back(los);

  LOSListWidgetItem* losItem = new LOSListWidgetItem(los, name);
  losItem->setCheckState(Qt::Checked);
  _losList->addItem(losItem);

  _root->addChild(los);
}

void LOSControlWidget::mapClick(const osg::Vec3d& point)
{
  if (!_newDialog.isNull())
    _newDialog->mapClick(point);
}

void LOSControlWidget::onItemDoubleClicked(QListWidgetItem* item)
{
  if (!_manager.valid() || !_map.valid() || _views.size() == 0)
    return;

  LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(item);
  if (losItem && losItem->los())
  {
    osg::Vec3d center = losItem->los()->getBound().center();

    GeoPoint output;
    output.fromWorld( _map->getSRS(), center );
    //_map->worldPointToMapPoint(center, output);

    double range = losItem->los()->getBound().radius() / 0.267949849;

    _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(output.vec3d(), 0.0, -90.0, range), _views));
  }
}

void LOSControlWidget::onItemChanged(QListWidgetItem* item)
{
  if (!_manager.valid())
    return;

  LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(item);
  if (losItem && losItem->los())
    _manager->doAction(this, new ToggleNodeAction(losItem->los(), item->checkState() == Qt::Checked));
}

void LOSControlWidget::onItemSelectionChanged()
{
  _activeRadial = 0L;
  _nameField->setText("-----");
  _typeField->setText("-----");
  _depthBox->setEnabled(false);
  _radiusBox->setEnabled(false);
  _spokesBox->setEnabled(false);
  _removeAction->setEnabled(false);
  _editAction->setEnabled(false);

  QListWidgetItem* item = _losList->currentItem();
  if (item)
  {
    _nameField->setText(item->text());

    LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(_losList->currentItem());
    if (losItem)
    {
      _removeAction->setEnabled(true);
      _editAction->setEnabled(true);

      osgEarth::Util::LinearLineOfSightNode* p2pNode = dynamic_cast<osgEarth::Util::LinearLineOfSightNode*>(losItem->los());
      if (p2pNode)
      {
        _typeField->setText("Point-to-Point");
        _depthBox->setCheckState(p2pNode->getOrCreateStateSet()->getMode(GL_DEPTH_TEST) == osg::StateAttribute::OFF ? Qt::Unchecked : Qt::Checked);
        _depthBox->setEnabled(true);
      }
      else
      {
        osgEarth::Util::RadialLineOfSightNode* radNode = dynamic_cast<osgEarth::Util::RadialLineOfSightNode*>(losItem->los());
        if (radNode)
        {
          _typeField->setText("Radial");
          _depthBox->setCheckState(radNode->getOrCreateStateSet()->getMode(GL_DEPTH_TEST) == osg::StateAttribute::OFF ? Qt::Unchecked : Qt::Checked);
          _depthBox->setEnabled(true);

          _radiusBox->setEnabled(true);
          _radiusBox->setValue(radNode->getRadius());
          _spokesBox->setEnabled(true);
          _spokesBox->setValue(radNode->getNumSpokes());
          _activeRadial = radNode;
        }
      }
    }
  }
}

void LOSControlWidget::onDepthBoxChanged(int state)
{
  QListWidgetItem* item = _losList->currentItem();
  
  LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(item);
  if (losItem)
    losItem->los()->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, state == Qt::Checked ? osg::StateAttribute::ON : osg::StateAttribute::OFF);
}

void LOSControlWidget::onRadiusValueChanged(double value)
{
  if (_activeRadial.valid())
    _activeRadial->setRadius(value);
}

void LOSControlWidget::onSpokesValueChanged(int value)
{
  if (_activeRadial.valid())
    _activeRadial->setNumSpokes(value);
}

void LOSControlWidget::onAddLOS()
{
  _newDialog = new LOSCreationDialog(_mapNode.get(), _root.get(), _losCounter, _manager.get(), &_views);

  if (!_newDialog.isNull())
  {
    this->setEnabled(false);

    connect(_newDialog, SIGNAL(finished(int)), this, SLOT(onCreateFinished(int)));

    _newDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
    _newDialog->setAttribute(Qt::WA_DeleteOnClose);
    _newDialog->show();
  }
}

void LOSControlWidget::onCreateFinished(int result)
{
  this->setEnabled(true);

  if (result == QDialog::Accepted)
  {
    _losCounter++;
    addLOSNode(_newDialog->losNode(), _newDialog->losName());
  }
}

void LOSControlWidget::onRemoveSelectedLOS()
{
  QListWidgetItem* item = _losList->currentItem();
  
  LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(item);
  if (losItem && losItem->los())
    _root->removeChild(losItem->los());

  delete item;
}

void LOSControlWidget::onEditSelectedLOS()
{
  QListWidgetItem* item = _losList->currentItem();
  
  LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(item);
  if (losItem && losItem->los())
  {
    _newDialog = new LOSCreationDialog(_mapNode.get(), _root.get(), losItem->los(), losItem->name(), _manager.get(), &_views);

    if (!_newDialog.isNull())
    {
      this->setEnabled(false);

      _root->removeChild(losItem->los());

      connect(_newDialog, SIGNAL(finished(int)), this, SLOT(onEditFinished(int)));

      _newDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
      _newDialog->setAttribute(Qt::WA_DeleteOnClose);
      _newDialog->show();
    }
  }
}

void LOSControlWidget::onEditFinished(int result)
{
  this->setEnabled(true);

  LOSListWidgetItem* losItem = dynamic_cast<LOSListWidgetItem*>(_losList->currentItem());
  if (losItem)
  {
    if (result == QDialog::Accepted)
    {
      losItem->setName(_newDialog->losName());
      _newDialog->losNode()->setNodeMask(losItem->los()->getNodeMask());
      losItem->setLOS(_newDialog->losNode());
      _root->addChild(_newDialog->losNode());

      onItemSelectionChanged();
    }
    else
    {
      _root->addChild(losItem->los());
    }
  }
}
