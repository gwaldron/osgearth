/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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

#include <osgEarthQt/AnnotationListWidget>
#include <osgEarthQt/AnnotationDialogs>
#include <osgEarthQt/Common>
#include <osgEarthQt/GuiActions>

#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/PlaceNode>

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QToolBar>
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

AnnotationListWidget::AnnotationListWidget(DataManager* dm)
  : _manager(dm), _updating(0)
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

void AnnotationListWidget::initialize()
{
  //create list layout
  QWidget* listBox = new QWidget;
  QVBoxLayout* listLayout = new QVBoxLayout;
  listLayout->setSpacing(0);
  listLayout->setContentsMargins(0, 0, 0, 0);
  listBox->setLayout(listLayout);

  //create toolbar with add/remove actions
  QToolBar* listBar = new QToolBar;
  listBar->setIconSize(QSize(16, 16));

  _removeAction = new QAction(QIcon(":/images/minus.png"), tr("Remove Annotation"), this);
  _removeAction->setStatusTip(tr("Remove selected annotation"));
  _removeAction->setEnabled(false);
  connect(_removeAction, SIGNAL(triggered()), this, SLOT(onRemoveSelected()));
  listBar->addAction(_removeAction);

  _editAction = new QAction(QIcon(":/images/edit.png"), tr("Edit Annotation"), this);
  _editAction->setStatusTip(tr("Edit selected annotation"));
  _editAction->setEnabled(false);
  connect(_editAction, SIGNAL(triggered()), this, SLOT(onEditSelected()));
  listBar->addAction(_editAction);

  listLayout->addWidget(listBar);


  //create list
  _annoList = new QListWidget();
  _annoList->setSelectionMode(QAbstractItemView::ExtendedSelection);
  listLayout->addWidget(_annoList);

  setPrimaryWidget(listBox);
  setPrimaryTitle("Annotations");

  //create details panel
  _detailsScroll = new QScrollArea;
  _detailsScroll->setWidgetResizable(true);
  _detailsBox = new QFrame;
  QGridLayout* detailsLayout = new QGridLayout;
  detailsLayout->setSpacing(4);
  detailsLayout->setContentsMargins(2, 2, 2, 2);
  _detailsBox->setLayout(detailsLayout);
  _detailsBox->setObjectName("oeFrameContainer");
  _detailsScroll->setWidget(_detailsBox);

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

  setSecondaryWidget(_detailsScroll);
  setSecondaryTitle("Details");

  //connect list events
  connect(_annoList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
  connect(_annoList, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(onItemChanged(QListWidgetItem*)));
  connect(_annoList, SIGNAL(itemSelectionChanged()), this, SLOT(onListSelectionChanged()));

  refresh();
}

void AnnotationListWidget::refresh()
{
  _updating++;

  if (_manager.valid())
  {
    _annoList->clear();

    bool annoDataSet = false;

    AnnotationVector annos;
    _manager->getAnnotations(annos);
    for (AnnotationVector::const_iterator it = annos.begin(); it != annos.end(); ++it)
    {
      AnnotationListItem* item = new AnnotationListItem(*it);
      item->setText( QString(tr((*it)->getName())) );
      item->setCheckState((*it)->getNodeMask() != 0 ? Qt::Checked : Qt::Unchecked);

      _annoList->addItem(item);

      if (_manager->isSelected(*it))
      {
        item->setSelected(true);
      }
    }

    _nameField->setText(tr(annoData ? annoData->getName().c_str() : "-----"));
    _priorityField->setText(annoData ? QString::number(annoData->getPriority()) : tr("-----"));
    _descriptionField->setText(tr(annoData ? annoData->getDescription().c_str() : ""));
  }

  _updating--;
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

      GeoPoint output;
      output.fromWorld( _manager->map()->getSRS(), center );
      //_manager->map()->worldPointToMapPoint(center, output);

      _manager->doAction(this, new SetViewpointAction(osgEarth::Viewpoint(
          "doubleclick", output.x(), output.y(), output.z(), 0.0, -90.0, 1e5), _views));
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

  _removeAction->setEnabled(items.count() > 0);
  _editAction->setEnabled(items.count() == 1);

  _manager->setSelectedAnnotations(annos);
}

void AnnotationListWidget::onRemoveSelected()
{
  AnnotationVector annos;

  QList<QListWidgetItem*> items = _annoList->selectedItems();
  for (QList<QListWidgetItem*>::iterator it = items.begin(); it != items.end(); ++it)
  {
    AnnotationListItem* annoItem = dynamic_cast<AnnotationListItem*>(*it);
    if (annoItem && annoItem->annotation())
      annos.push_back(annoItem->annotation());
  }

  for (AnnotationVector::iterator it = annos.begin(); it != annos.end(); ++it)
    _manager->removeAnnotation(*it);
}

void AnnotationListWidget::onEditSelected()
{
  QListWidgetItem* item = _annoList->selectedItems()[0];

  AnnotationListItem* annoItem = dynamic_cast<AnnotationListItem*>(item);
  if (annoItem && annoItem->annotation())
  {
    osgEarth::Annotation::PlaceNode* placeNode = dynamic_cast<osgEarth::Annotation::PlaceNode*>(annoItem->annotation());
    if (placeNode)
    {
      _activeDialog = new osgEarth::QtGui::AddMarkerDialog(placeNode->getParent(0), _manager->MapNode(), _views, placeNode);
    }
    else
    {
      osgEarth::Annotation::FeatureNode* featureNode = dynamic_cast<osgEarth::Annotation::FeatureNode*>(annoItem->annotation());
      if (featureNode)
      {
        const osgEarth::Features::Feature* feat = featureNode->getFeature();
        if (feat)
        {
          const osgEarth::Symbology::LineString* pathLine = dynamic_cast<const osgEarth::Symbology::LineString*>(feat->getGeometry());
          if (pathLine)
          {
            _activeDialog = new osgEarth::QtGui::AddPathDialog(featureNode->getParent(0), _manager->MapNode(), _views, featureNode);
          }
          else
          {
            const osgEarth::Symbology::Polygon* polygon = dynamic_cast<const osgEarth::Symbology::Polygon*>(feat->getGeometry());
            if (polygon)
            {
              _activeDialog = new osgEarth::QtGui::AddPolygonDialog(featureNode->getParent(0), _manager->MapNode(), _views, featureNode);
            }
          }
        }
      }
      else
      {
        osgEarth::Annotation::EllipseNode* ellipse = dynamic_cast<osgEarth::Annotation::EllipseNode*>(annoItem->annotation());
        if (ellipse)
        {
          _activeDialog = new osgEarth::QtGui::AddEllipseDialog(ellipse->getParent(0), _manager->MapNode(), _views, ellipse);
        }
      }
    }

    if (!_activeDialog.isNull())
    {
      this->setEnabled(false);

      connect(_activeDialog, SIGNAL(finished(int)), this, SLOT(onAddFinished(int)));

      _activeDialog->setWindowTitle(tr("Edit annotation"));
      _activeDialog->setWindowFlags(Qt::Tool | Qt::WindowTitleHint | Qt::CustomizeWindowHint| Qt::WindowStaysOnTopHint);
      _activeDialog->setAttribute(Qt::WA_DeleteOnClose);
      _activeDialog->show();
    }
  }
}

void AnnotationListWidget::onAddFinished(int result)
{
  this->setEnabled(true);

  if (result == QDialog::Accepted)
    refresh();
}

