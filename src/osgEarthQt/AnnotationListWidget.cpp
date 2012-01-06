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

#include <osgEarthQt/AnnotationListWidget>
#include <osgEarthQt/Common>
#include <osgEarthQt/GuiActions>

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWidget>


using namespace osgEarth;
using namespace osgEarth::QtGui;

namespace
{
  class AnnotationListItem : public QListWidgetItem
  {
  public:
    AnnotationListItem(osgEarth::Annotation::AnnotationNode* annotation) : _annotation(annotation) { }

    osgEarth::Annotation::AnnotationNode* annotation() { return _annotation.get(); }

  private:
	  osg::ref_ptr<osgEarth::Annotation::AnnotationNode> _annotation;
  };
}

namespace osgEarth { namespace QtGui
{
  class AnnotationListActionCallbackProxy : public ActionCallback
  {
  public:
    AnnotationListActionCallbackProxy(AnnotationListWidget* annoList) : _annoList(annoList) { }

    void operator()( void* sender, Action* action )
    {
      if (_annoList)
      {
        Action* foundAction = dynamic_cast<ToggleNodeAction*>(action);
        if (foundAction)
          _annoList->refresh();
      }
    }

  private:
    AnnotationListWidget* _annoList;
  };
} }

const std::string AnnotationListWidget::DEFAULT_STYLESHEET = "#oeFrameContainer, #oeFrameContainer * { background-color: rgba(255, 255, 255, 100%) } #oeItemHeader, #oeItemHeader * { background-color: grey; color: white; }";

AnnotationListWidget::AnnotationListWidget(DataManager* dm)
  : _manager(dm), _updating(0), _showIcon(":/images/plus.png"), _hideIcon(":/images/minus.png")
{
  initialize();

  if (_manager.valid())
  {
    connect(_manager.get(), SIGNAL(mapChanged()), this, SLOT(onMapChanged()));
    connect(_manager.get(), SIGNAL(selectionChanged(/*const AnnotationVector&*/)), this, SLOT(onSelectionChanged(/*const AnnotationVector&*/)));

    _manager->addAfterActionCallback(new AnnotationListActionCallbackProxy(this));
  }
}

void AnnotationListWidget::setActiveView(osgViewer::View* view)
{
  _views.clear();
  _views.push_back(view);
}

void AnnotationListWidget::setActiveViews(const ViewVector& views)
{
  _views.clear();
  _views.insert(_views.end(), views.begin(), views.end());
}

void AnnotationListWidget::resetStyleSheet()
{
  setStyleSheet(tr(DEFAULT_STYLESHEET.c_str()));
}

void AnnotationListWidget::initialize()
{
  // object name for custom stylesheets
  setObjectName("oeFrameContainer");

  // create the primary vertical layout
  QVBoxLayout* primaryLayout = new QVBoxLayout;
	primaryLayout->setSpacing(0);
	primaryLayout->setContentsMargins(0, 0, 0, 0);
  setLayout(primaryLayout);

  // create parent widget to hold list and header
  _listGroup = new QWidget;
  QVBoxLayout* listGroupLayout = new QVBoxLayout;
  listGroupLayout->setSpacing(0);
  listGroupLayout->setContentsMargins(0, 0, 0, 0);
  _listGroup->setLayout(listGroupLayout);

  // create list header
  QFrame* listHeader = new QFrame;
  listHeader->setFrameStyle(QFrame::Box | QFrame::Plain);
  listHeader->setLineWidth(1);
  listHeader->setMaximumHeight(20);

  QHBoxLayout* listHeaderLayout = new QHBoxLayout;
  listHeaderLayout->setSpacing(4);
  listHeaderLayout->setContentsMargins(2, 2, 2, 2);
  listHeader->setLayout(listHeaderLayout);
  listHeader->setObjectName("oeItemHeader");

  listHeaderLayout->addWidget(new QLabel(tr("Annotations")));
  listHeaderLayout->addStretch();

  _listHideButton = new QPushButton(_hideIcon, tr(""));
  _listHideButton->setFlat(true);
  _listHideButton->setMaximumSize(16, 16);
  listHeaderLayout->addWidget(_listHideButton);

  listGroupLayout->addWidget(listHeader);

  //create list
  _annoList = new QListWidget();
  _annoList->setSelectionMode(QAbstractItemView::ExtendedSelection);
  listGroupLayout->addWidget(_annoList);

  primaryLayout->addWidget(_listGroup);

  // create parent widget to hold details and header
  _detailsGroup = new QWidget;
  _detailsGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  QVBoxLayout* detailsGroupLayout = new QVBoxLayout;
  detailsGroupLayout->setSpacing(0);
  detailsGroupLayout->setContentsMargins(0, 0, 0, 0);
  _detailsGroup->setLayout(detailsGroupLayout);

  //create details header
  QFrame* detailsHeader = new QFrame;
  detailsHeader->setFrameStyle(QFrame::Box | QFrame::Plain);
  detailsHeader->setLineWidth(1);
  detailsHeader->setMaximumHeight(20);

  QHBoxLayout* detailsHeaderLayout = new QHBoxLayout;
  detailsHeaderLayout->setSpacing(4);
  detailsHeaderLayout->setContentsMargins(2, 2, 2, 2);
  detailsHeader->setLayout(detailsHeaderLayout);
  detailsHeader->setObjectName("oeItemHeader");

  detailsHeaderLayout->addWidget(new QLabel(tr("Details")));
  detailsHeaderLayout->addStretch();

  _detailsHideButton = new QPushButton(_hideIcon, tr(""));
  _detailsHideButton->setFlat(true);
  _detailsHideButton->setMaximumSize(16, 16);
  detailsHeaderLayout->addWidget(_detailsHideButton);

  detailsGroupLayout->addWidget(detailsHeader);

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

  detailsLayout->addWidget(new QLabel(tr("Priority:")), 1, 0, Qt::AlignLeft);
  _priorityField = new QLabel(tr("-----"));
  detailsLayout->addWidget(_priorityField, 1, 1, Qt::AlignLeft);

  detailsLayout->addWidget(new QLabel(tr("Viewpoint:")), 2, 0, Qt::AlignLeft);
  _viewpointField = new QLabel(tr("-----"));
  detailsLayout->addWidget(_viewpointField, 2, 1, Qt::AlignLeft);

  detailsLayout->addWidget(new QLabel(tr("Description:")), 3, 0, Qt::AlignLeft);
  _descriptionField = new QLabel;
  _descriptionField->setContentsMargins(4, 4, 4, 4);
  _descriptionField->setIndent(6);
  detailsLayout->addWidget(_descriptionField, 4, 0, 1, 2, Qt::AlignLeft);

  detailsGroupLayout->addWidget(_detailsBox);
  primaryLayout->addWidget(_detailsGroup);

  //create widget with a stretch child for layout purposes (not ideal)
  _stretchBox = new QWidget;
  QVBoxLayout* stretchLayout = new QVBoxLayout;
  _stretchBox->setLayout(stretchLayout);
  stretchLayout->addStretch();
  primaryLayout->addWidget(_stretchBox);
  _stretchBox->setVisible(false);

  //connect show/hide button click events
  connect(_listHideButton, SIGNAL(clicked(bool)), this, SLOT(onListHideClicked(bool)));
  connect(_detailsHideButton, SIGNAL(clicked(bool)), this, SLOT(onDetailsHideClicked(bool)));

  //connect list events
  connect(_annoList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
  connect(_annoList, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(onItemChanged(QListWidgetItem*)));
  connect(_annoList, SIGNAL(itemSelectionChanged()), this, SLOT(onListSelectionChanged()));

  resetStyleSheet();
  refresh();
}

void AnnotationListWidget::refresh()
{
  _updating++;

  if (_manager.valid())
  {
    _annoList->clear();

    osgEarth::Annotation::AnnotationData* annoData = 0L;
    bool annoDataSet = false;

    AnnotationVector annos;
    _manager->getAnnotations(annos);
    for (AnnotationVector::const_iterator it = annos.begin(); it != annos.end(); ++it)
    {
      AnnotationListItem* item = new AnnotationListItem(*it);
      item->setText(QString(tr((*it)->getAnnotationData() ? (*it)->getAnnotationData()->getName().c_str() : "Annotation")));
      item->setCheckState((*it)->getNodeMask() != 0 ? Qt::Checked : Qt::Unchecked);

      _annoList->addItem(item);

      if (_manager->isSelected(*it))
      {
        if (!annoDataSet)
        {
          annoData = (*it)->getAnnotationData();
          annoDataSet = true;
        }
        else
          annoData = 0L;

        item->setSelected(true);
      }
    }

    _nameField->setText(tr(annoData ? annoData->getName().c_str() : "-----"));
    _priorityField->setText(annoData ? QString::number(annoData->getPriority()) : tr("-----"));
    _viewpointField->setText(tr(annoData ? annoData->getViewpoint()->toString().c_str() : "-----"));
    _descriptionField->setText(tr(annoData ? annoData->getDescription().c_str() : ""));
  }

  _updating--;
}

void AnnotationListWidget::onListHideClicked(bool checked)
{
  bool show = _annoList->isHidden();
  _listHideButton->setIcon(show ? _hideIcon : _showIcon);
  _annoList->setHidden(!show);
  _stretchBox->setVisible(!show);
}

void AnnotationListWidget::onDetailsHideClicked(bool checked)
{
  bool show = _detailsBox->isHidden();
  _detailsHideButton->setIcon(show ? _hideIcon : _showIcon);
  _detailsBox->setHidden(!show);
}

void AnnotationListWidget::onMapChanged()
{
  refresh();
}

void AnnotationListWidget::onSelectionChanged()
{
  refresh();
}

void AnnotationListWidget::onItemDoubleClicked(QListWidgetItem* item)
{
  if (!_manager.valid() || _views.size() == 0)
    return;

  AnnotationListItem* annoItem = dynamic_cast<AnnotationListItem*>(item);
  if (annoItem && annoItem->annotation())
  {
    if (annoItem->annotation()->getAnnotationData() && annoItem->annotation()->getAnnotationData()->getViewpoint())
    {
      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(*annoItem->annotation()->getAnnotationData()->getViewpoint()), _views));
    }
    else if (_manager->map())
    {
      osg::Vec3d center = annoItem->annotation()->getBound().center();

      osg::Vec3d output;
      _manager->map()->worldPointToMapPoint(center, output);

      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(output, 0.0, -90.0, 1e5), _views));
    }
  }
}

void AnnotationListWidget::onItemChanged(QListWidgetItem* item)
{
  if (!_manager.valid())
    return;

  AnnotationListItem* annoItem = dynamic_cast<AnnotationListItem*>(item);
  if (annoItem && annoItem->annotation())
    _manager->doAction(this, new ToggleNodeAction(annoItem->annotation(), item->checkState() == Qt::Checked));
}

void AnnotationListWidget::onListSelectionChanged()
{
  if (_updating || !_manager.valid())
    return;

  AnnotationVector annos;

  QList<QListWidgetItem*> items = _annoList->selectedItems();
  for (QList<QListWidgetItem*>::iterator it = items.begin(); it != items.end(); ++it)
  {
    AnnotationListItem* annoItem = dynamic_cast<AnnotationListItem*>(*it);
    if (annoItem && annoItem->annotation())
      annos.push_back(annoItem->annotation());
  }

  _manager->setSelectedAnnotations(annos);
}