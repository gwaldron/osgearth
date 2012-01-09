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

#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWidget>


using namespace osgEarth;
using namespace osgEarth::QtGui;

namespace
{
  //class LOSListItem : public QListWidgetItem
  //{
  //public:
  //  LOSListItem(osgEarth::Util::LOSNode* los) : _los(los) { }

  //  osgEarth::Util::LOSNode* los() { return _los.get(); }

  //private:
	 // osg::ref_ptr<osgEarth::Util::LOSNode> _los;
  //};
}

LOSControlWidget::LOSControlWidget(DataManager* dm)
  : _manager(dm)
{
  if (_manager.valid())
    _map = _manager->map();

  initialize();
}

LOSControlWidget::LOSControlWidget(osgEarth::Map* map)
  : _map(map)
{
  initialize();
}

void LOSControlWidget::setActiveView(osgViewer::View* view)
{
  _views.clear();
  _views.push_back(view);
}

void LOSControlWidget::setActiveViews(const ViewVector& views)
{
  _views.clear();
  _views.insert(_views.end(), views.begin(), views.end());
}

void LOSControlWidget::initialize()
{
  //create list
  _losList = new QListWidget();
  _losList->setSelectionMode(QAbstractItemView::ExtendedSelection);
  setPrimaryWidget(_losList);

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

  detailsLayout->addWidget(new QLabel(tr("Radius:")), 1, 0, Qt::AlignLeft);
  _radiusBox = new QDoubleSpinBox;
  _radiusBox->setMinimum(0.01);
  _radiusBox->setMaximum(100000.0);
  _radiusBox->setSingleStep(1.0);
  detailsLayout->addWidget(_radiusBox, 1, 1, Qt::AlignLeft);

  setSecondaryWidget(_detailsBox);

  //connect list events
  connect(_losList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
  connect(_losList, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(onItemChanged(QListWidgetItem*)));
}

void LOSControlWidget::onItemDoubleClicked(QListWidgetItem* item)
{
  if (!_manager.valid() || _views.size() == 0)
    return;

  //LOSListItem* losItem = dynamic_cast<LOSListItem*>(item);
  //if (losItem && losItem->los())
  //{
  //  if (losItem->annotation()->getAnnotationData() && losItem->annotation()->getAnnotationData()->getViewpoint())
  //  {
  //    _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(*annoItem->annotation()->getAnnotationData()->getViewpoint()), _views));
  //  }
  //  else if (_manager->map())
  //  {
  //    osg::Vec3d center = annoItem->annotation()->getBound().center();

  //    osg::Vec3d output;
  //    _manager->map()->worldPointToMapPoint(center, output);

  //    _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(output, 0.0, -90.0, 1e5), _views));
  //  }
  //}
}

void LOSControlWidget::onItemChanged(QListWidgetItem* item)
{
  if (!_manager.valid())
    return;

  //LOSListItem* losItem = dynamic_cast<LOSListItem*>(item);
  //if (losItem && losItem->los())
  //  _manager->doAction(this, new ToggleNodeAction(losItem->los(), item->checkState() == Qt::Checked));
}