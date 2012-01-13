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
#include <QListWidget>
#include <QScrollArea>
#include <QSizePolicy>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>


using namespace osgEarth;
using namespace osgEarth::QtGui;

namespace
{
  class LOSListItem : public QListWidgetItem
  {
  public:
    LOSListItem(osg::Group* los, const QString& name, osg::Group* editor) : QListWidgetItem(name), _los(los), _editor(editor), _editing(false) { }

    osg::Group* los() { return _los.get(); }
    osg::Group* editor() { return _editor.get(); }
    bool getEditing() { return _editing; }
    void setEditing(bool value) { _editing = value; }

  private:
    osg::ref_ptr<osg::Group> _los;
    osg::ref_ptr<osg::Group> _editor;
    bool                     _editing;
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

  detailsLayout->addWidget(new QLabel(tr("Type:")), 0, 0, Qt::AlignLeft);
  _typeField = new QLabel(tr("-----"));
  detailsLayout->addWidget(_typeField, 0, 1, Qt::AlignLeft);

  _dragBox = new QCheckBox(tr("Allow drag"));
  detailsLayout->addWidget(_dragBox, 1, 0, Qt::AlignLeft);
  _dragBox->setEnabled(false);

  detailsLayout->addWidget(new QLabel(tr("Radius:")), 2, 0, Qt::AlignLeft);
  _radiusBox = new QDoubleSpinBox;
  _radiusBox->setMinimum(0.1);
  _radiusBox->setMaximum(40075160.0);
  _radiusBox->setSingleStep(1.0);
  _radiusBox->setValue(2000.0);
  detailsLayout->addWidget(_radiusBox, 2, 1, Qt::AlignLeft);
  _radiusBox->setEnabled(false);

  detailsLayout->addWidget(new QLabel(tr("Spokes:")), 3, 0, Qt::AlignLeft);
  _spokesBox = new QSpinBox;
  _spokesBox->setMinimum(2);
  _spokesBox->setMaximum(10000);
  _spokesBox->setSingleStep(1);
  _spokesBox->setValue(100);
  detailsLayout->addWidget(_spokesBox, 3, 1, Qt::AlignLeft);
  _spokesBox->setEnabled(false);

  setSecondaryWidget(_detailsBox);
  setSecondaryTitle("Details");

  //connect list events
  connect(_losList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
  connect(_losList, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(onItemChanged(QListWidgetItem*)));
  connect(_losList, SIGNAL(itemSelectionChanged()), this, SLOT(onItemSelectionChanged()));

  //connect editor checkbox signal
  connect(_dragBox, SIGNAL(stateChanged(int)), this, SLOT(onDragBoxChanged(int)));

  //connect radial options signals
  connect(_radiusBox, SIGNAL(valueChanged(double)), this, SLOT(onRadiusValueChanged(double)));
  connect(_spokesBox, SIGNAL(valueChanged(int)), this, SLOT(onSpokesValueChanged(int)));
}

void LOSControlWidget::addLOSNode(osg::Group* los, const QString& name, osg::Group* editor)
{
  if (!los)
    return;

  //_losNodes.push_back(los);


  LOSListItem* losItem = new LOSListItem(los, name, editor);
  losItem->setCheckState(Qt::Checked);
  _losList->addItem(losItem);

  _root->addChild(los);
}

void LOSControlWidget::mapClick(const osg::Vec3d& location)
{
  if (!_newDialog.isNull())
    _newDialog->mapClick(location);
}

void LOSControlWidget::onItemDoubleClicked(QListWidgetItem* item)
{
  if (!_manager.valid() || !_map.valid() || _views.size() == 0)
    return;

  LOSListItem* losItem = dynamic_cast<LOSListItem*>(item);
  if (losItem && losItem->los())
  {
    osg::Vec3d center = losItem->los()->getBound().center();

    osg::Vec3d output;
    _map->worldPointToMapPoint(center, output);

    double range = losItem->los()->getBound().radius() / 0.267949849;

    _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(output, 0.0, -90.0, range), _views));
  }
}

void LOSControlWidget::onItemChanged(QListWidgetItem* item)
{
  if (!_manager.valid())
    return;

  LOSListItem* losItem = dynamic_cast<LOSListItem*>(item);
  if (losItem && losItem->los())
    _manager->doAction(this, new ToggleNodeAction(losItem->los(), item->checkState() == Qt::Checked));
}

void LOSControlWidget::onItemSelectionChanged()
{
  _activeRadial = 0L;
  _typeField->setText("-----");
  _dragBox->setEnabled(false);
  _radiusBox->setEnabled(false);
  _spokesBox->setEnabled(false);
  _removeAction->setEnabled(false);

  LOSListItem* losItem = dynamic_cast<LOSListItem*>(_losList->currentItem());
  if (losItem)
  {
    _removeAction->setEnabled(true);
    _dragBox->setEnabled(losItem->editor());
    _dragBox->setCheckState(losItem->getEditing() ? Qt::Checked : Qt::Unchecked);

    osgEarth::Util::LineOfSightNode* p2pNode = dynamic_cast<osgEarth::Util::LineOfSightNode*>(losItem->los());
    if (p2pNode)
    {
      _typeField->setText("Point-to-Point");
    }
    else
    {
      osgEarth::Util::RadialLineOfSightNode* radNode = dynamic_cast<osgEarth::Util::RadialLineOfSightNode*>(losItem->los());
      if (radNode)
      {
        _typeField->setText("Radial");
        _radiusBox->setEnabled(true);
        _radiusBox->setValue(radNode->getRadius());
        _spokesBox->setEnabled(true);
        _spokesBox->setValue(radNode->getNumSpokes());
        _activeRadial = radNode;
      }
    }
  }
}

void LOSControlWidget::onDragBoxChanged(int state)
{
  QListWidgetItem* item = _losList->currentItem();
  
  LOSListItem* losItem = dynamic_cast<LOSListItem*>(item);
  if (losItem && losItem->editor())
  {
    if (state == Qt::Checked)
    {
      if (!losItem->getEditing())
      {
        losItem->setEditing(true);
        _root->addChild(losItem->editor());
      }
    }
    else
    {
      losItem->setEditing(false);
      _root->removeChild(losItem->editor());
    }
  }
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
  _newDialog = new LOSCreationDialog(_mapNode.get(), _losCounter, _manager.get(), &_views);

  if (!_newDialog.isNull())
  {
    this->setEnabled(false);

    connect(_newDialog, SIGNAL(finished(int)), this, SLOT(onCreateFinished(int)));

    _newDialog->setWindowFlags(Qt::WindowStaysOnTopHint);
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
    addLOSNode(_newDialog->losNode(), _newDialog->losName(), _newDialog->losEditor());
  }
}

void LOSControlWidget::onRemoveSelectedLOS()
{
  QListWidgetItem* item = _losList->currentItem();
  
  LOSListItem* losItem = dynamic_cast<LOSListItem*>(item);
  if (losItem)
  {
    if (losItem->los())
      _root->removeChild(losItem->los());

    if (losItem->editor() && losItem->getEditing())
      _root->removeChild(losItem->editor());
  }

  delete item;
}