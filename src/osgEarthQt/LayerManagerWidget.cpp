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
#include <osgEarthQt/LayerManagerWidget>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>


#include <osgEarth/Map>
#include <osgEarth/Viewpoint>
#include <osgEarthAnnotation/AnnotationNode>

#include <osg/MatrixTransform>

#include <QAbstractItemDelegate>
#include <QApplication>
#include <QCheckBox>
#include <QFrame>
#include <QGraphicsOpacityEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QSlider>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QVBoxLayout>
#include <QWidget>
#include <QDrag>

using namespace osgEarth;
using namespace osgEarth::QtGui;

//---------------------------------------------------------------------------
namespace
{
  class WidgetElevationLayerCallback : public ElevationLayerCallback
  {
  public:
    WidgetElevationLayerCallback(ElevationLayerControlWidget* widget) : _widget(widget) {}

    void onEnabledChanged(TerrainLayer* layer)
    {
      if (_widget)
        _widget->setLayerVisible(layer->getVisible());
    }

  private:
    ElevationLayerControlWidget* _widget;
  };

  class WidgetImageLayerCallback : public ImageLayerCallback
  {
  public:
    WidgetImageLayerCallback(ImageLayerControlWidget* widget) : _widget(widget) {}

    void onOpacityChanged(ImageLayer* layer)
    {
      if (_widget)
        _widget->setLayerOpacity(layer->getOpacity());
    }

    void onEnabledChanged(TerrainLayer* layer)
    {
      if (_widget)
        _widget->setLayerVisible(layer->getVisible());
    }

  private:
    ImageLayerControlWidget* _widget;
  };

  class WidgetModelLayerCallback : public ModelLayerCallback
  {
  public:
    WidgetModelLayerCallback(ModelLayerControlWidget* widget) : _widget(widget) {}

    void onEnabledChanged(ModelLayer* layer)
    {
      if (_widget && layer)
        _widget->setLayerVisible(layer->getVisible());
    }

  private:
    ModelLayerControlWidget* _widget;
  };
}


//---------------------------------------------------------------------------
namespace osgEarth { namespace QtGui
{
  struct LayerManagerMapCallback : public osgEarth::MapCallback
  {
    LayerManagerMapCallback(LayerManagerWidget* manager) : _manager(manager) { }

    //void onMapInfoEstablished( const MapInfo& mapInfo ) { } 

    //void onMapModelChanged( const MapModelChange& change );

    void onImageLayerAdded(ImageLayer* layer, unsigned int index)
    {
      _manager->addImageLayerItem(layer, index);
    }

    void onImageLayerRemoved(ImageLayer* layer, unsigned int index)
    {
      _manager->removeLayerItem(layer);
    }

    void onImageLayerMoved(ImageLayer* layer, unsigned int oldIndex, unsigned int newIndex)
    {
      _manager->moveLayerItem(layer, oldIndex, newIndex);
    }

    void onElevationLayerAdded(ElevationLayer* layer, unsigned int index)
    {
      _manager->addElevationLayerItem(layer, index);
    }

    void onElevationLayerRemoved(ElevationLayer* layer, unsigned int index)
    {
      _manager->removeLayerItem(layer);
    }

    void onElevationLayerMoved(ElevationLayer* layer, unsigned int oldIndex, unsigned int newIndex)
    {
      _manager->moveLayerItem(layer, oldIndex, newIndex);
    }

    void onModelLayerAdded(ModelLayer* layer, unsigned int index)
    {
      _manager->addModelLayerItem(layer, index);
    }

    void onModelLayerRemoved(ModelLayer* layer)
    {
      _manager->removeLayerItem(layer);
    }

    void onModelLayerMoved(ModelLayer* layer, unsigned int oldIndex, unsigned int newIndex)
    {
      _manager->moveLayerItem(layer, oldIndex, newIndex);
    }

    //void onMaskLayerAdded( MaskLayer* mask ) { }
    //void onMaskLayerRemoved( MaskLayer* mask ) { }

    LayerManagerWidget* _manager;
  };
} }


//---------------------------------------------------------------------------
const QString LayerWidgetMimeData::MIME_TYPE = tr("application/LayerWidgetMimeData");


//---------------------------------------------------------------------------
LayerControlWidgetBase::~LayerControlWidgetBase()
{
}

void LayerControlWidgetBase::initUi(bool hasContent, bool showRemove)
{
  // object name for custom stylesheets
  setObjectName("oeItem");

  // create the primary vertical layout
  _primaryLayout = new QVBoxLayout;
  _primaryLayout->setSpacing(0);
  _primaryLayout->setContentsMargins(0, 0, 0, 0);
  setLayout(_primaryLayout);


  //create drop decoration box
  _dropBox = new QFrame;
  QVBoxLayout* dbLayout = new QVBoxLayout();
  dbLayout->setSpacing(0);
  dbLayout->setContentsMargins(0, 0, 0, 0);
  _dropBox->setLayout(dbLayout);

  QFrame* dropBoxInternal = new QFrame();
  dropBoxInternal->setFixedHeight(40);
  dropBoxInternal->setObjectName("oeDropTarget");
  dbLayout->addWidget(dropBoxInternal);
  dbLayout->addSpacing(4);

  QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(_dropBox);
  dbEffect->setOpacity(0.35);
  _dropBox->setGraphicsEffect(dbEffect);
  _dropBox->setVisible(false);
  _primaryLayout->addWidget(_dropBox);


  // create the header boxes and layouts
  _headerBox = new QFrame;
  _headerBoxLayout = new QHBoxLayout;
  //_headerBoxLayout->setSpacing(4);
  //_headerBoxLayout->setContentsMargins(2, 2, 2, 2);
  _headerBox->setLayout(_headerBoxLayout);

  _headerTitleBox = new QFrame;
  _headerTitleBoxLayout = new QHBoxLayout;
  _headerTitleBoxLayout->setSpacing(4);
  _headerTitleBoxLayout->setContentsMargins(2, 2, 2, 2);
  _headerTitleBox->setLayout(_headerTitleBoxLayout);
  _headerBoxLayout->addWidget(_headerTitleBox);

  _headerBoxLayout->addStretch();

  _headerButtonBox = new QFrame;
  _headerButtonBoxLayout = new QHBoxLayout;
  //_headerButtonBoxLayout->setSpacing(4);
  _headerButtonBoxLayout->setContentsMargins(2, 2, 2, 2);
  _headerButtonBox->setLayout(_headerButtonBoxLayout);
  _headerBoxLayout->addWidget(_headerButtonBox);

  if (showRemove)
  {
    // add remove button to the header button box
    _removeButton = new QPushButton(QIcon(":/images/close.png"), tr(""));
    _removeButton->setFlat(true);
    _removeButton->setMaximumSize(16, 16);
    _headerButtonBoxLayout->addWidget(_removeButton);

    connect(_removeButton, SIGNAL(clicked(bool)), this, SLOT(onRemoveClicked(bool)));
  }

  _primaryLayout->addWidget(_headerBox);


  // create the content box and layout
  if (hasContent)
  {
    _headerBox->setObjectName("oeItemHeader");

    _contentBox = new QFrame();
    _contentBoxLayout = new QHBoxLayout;
    _contentBoxLayout->setContentsMargins(4, 4, 4, 4);
    _contentBox->setLayout(_contentBoxLayout);
    _primaryLayout->addWidget(_contentBox);
  }

  setAcceptDrops(true); 
}

void LayerControlWidgetBase::onRemoveClicked(bool checked)
{
  //NOP
}

void LayerControlWidgetBase::mouseDoubleClickEvent(QMouseEvent *event)
{
  emit doubleClicked();
}

void LayerControlWidgetBase::mousePressEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton)
    _dragStartPosition = event->pos();
}

void LayerControlWidgetBase::mouseMoveEvent(QMouseEvent *event)
{
  if (!(event->buttons() & Qt::LeftButton))
    return;

  if ((event->pos() - _dragStartPosition).manhattanLength() < QApplication::startDragDistance())
    return;

  QDrag *drag = new QDrag(this);
  drag->setMimeData(new LayerWidgetMimeData(this));

  Qt::DropAction dropAction = drag->exec(Qt::MoveAction);
}

void LayerControlWidgetBase::dragEnterEvent(QDragEnterEvent* event)
{
  if (_parent && event->mimeData()->hasFormat(LayerWidgetMimeData::MIME_TYPE) && event->source()->parent() == this->parent())
  {
    if (event->source() != this)
    {
      _dropBox->setFixedWidth(_headerBox->width());
      _dropBox->setVisible(true);
    }

    event->acceptProposedAction();
  }
}

void LayerControlWidgetBase::dragLeaveEvent(QDragLeaveEvent* event)
{
  _dropBox->setVisible(false);
}

void LayerControlWidgetBase::dropEvent(QDropEvent* event)
{
  _dropBox->setVisible(false);

  const LayerWidgetMimeData* widgetData = qobject_cast<const LayerWidgetMimeData*>(event->mimeData());
  if (widgetData)
    _parent->doLayerWidgetDrop(widgetData->getWidget(), this);
}


//---------------------------------------------------------------------------
ElevationLayerControlWidget::ElevationLayerControlWidget(osgEarth::ElevationLayer* layer, LayerManagerWidget* parentManager) : LayerControlWidgetBase(parentManager, false), _layer(layer)
{
  initUi();

  if (_layer.valid())
  {
    _layerCallback = new WidgetElevationLayerCallback(this);
    _layer->addCallback(_layerCallback.get());
  }
}

ElevationLayerControlWidget::~ElevationLayerControlWidget()
{
  if (_layer.valid())
    _layer->removeCallback(_layerCallback.get());

  _layer = 0L;
  _layerCallback = 0L;
  _doubleClick = 0L;
}

void ElevationLayerControlWidget::initUi()
{
  if (_layer.valid())
  {
    // create enabled checkbox
    _visibleCheckBox = new QCheckBox();
    _visibleCheckBox->setCheckState(_layer->getVisible() ? Qt::Checked : Qt::Unchecked);
    connect(_visibleCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onEnabledCheckStateChanged(int)));
    _headerTitleBoxLayout->addWidget(_visibleCheckBox);

    // create name label
    QLabel* label = new QLabel(tr(!_layer->getName().empty() ? _layer->getName().c_str() : "Elevation Layer"));
    _headerTitleBoxLayout->addWidget(label);

    if (!_layer->getProfile())
    {
      _headerBox->setStyleSheet("background-color: #FF6666; color: white;");
      _visibleCheckBox->setEnabled(false);
    }
  }
  else
  {
    // _layer is null so just add a label stating so
    QLabel* label = new QLabel(tr("LAYER IS NULL"));
    _contentBoxLayout->addWidget(label);
  }
}

void ElevationLayerControlWidget::onEnabledCheckStateChanged(int state)
{
  bool checked = state == Qt::Checked;
  if (_layer.valid() && _layer->getVisible() != checked)
    _layer->setVisible(checked);
}

void ElevationLayerControlWidget::onRemoveClicked(bool checked)
{
  if (_parent && _parent->getMap())
    _parent->getMap()->removeElevationLayer(_layer);
}

void ElevationLayerControlWidget::setLayerVisible(bool visible)
{
  if ((_visibleCheckBox->checkState() == Qt::Checked) != visible)
    _visibleCheckBox->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
}

osgEarth::UID ElevationLayerControlWidget::getUID()
{
  if (!_layer)
    return -1;

  return _layer->getUID();
}

Action* ElevationLayerControlWidget::getDoubleClickAction(const ViewVector& views)
{
  if (!_doubleClick.valid() && _layer.valid() && _layer->getProfile())
  {
    const osgEarth::GeoExtent llExt = _layer->getProfile()->getLatLongExtent();

    osg::Vec3d focalPoint((llExt.xMax() + llExt.xMin()) / 2.0,
                          (llExt.yMax() + llExt.yMin()) / 2.0,
                          0L);

	  double rangeFactor = llExt.yMax() != llExt.yMin() ? llExt.yMax() - llExt.yMin() : llExt.xMax() - llExt.xMin();
	  double range = ((0.5 * rangeFactor) / 0.267949849) * 111000.0;
	  if (range == 0.0)
		  range = 20000000.0;

    _doubleClick = new SetViewpointAction(osgEarth::Viewpoint("ClickAction", focalPoint.x(), focalPoint.y(), focalPoint.z(), 0.0, -90.0, range), views);
  }

  return _doubleClick.get();
}


//---------------------------------------------------------------------------
ImageLayerControlWidget::ImageLayerControlWidget(osgEarth::ImageLayer* layer, LayerManagerWidget* parentManager) : LayerControlWidgetBase(parentManager), _layer(layer)
{
  initUi();

  if (_layer.valid())
  {
    _layerCallback = new WidgetImageLayerCallback(this);
    _layer->addCallback(_layerCallback.get());
  }
}

ImageLayerControlWidget::~ImageLayerControlWidget()
{
  if (_layer.valid())
    _layer->removeCallback(_layerCallback.get());

  _layer = 0L;
  _layerCallback = 0L;
  _doubleClick = 0L;
}

void ImageLayerControlWidget::initUi()
{
  if (_layer.valid())
  {
    // create enabled checkbox
    _visibleCheckBox = new QCheckBox();
    _visibleCheckBox->setCheckState(_layer->getVisible() ? Qt::Checked : Qt::Unchecked);
    connect(_visibleCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onCheckStateChanged(int)));
    _headerTitleBoxLayout->addWidget(_visibleCheckBox);

    // create name label
    QLabel* label = new QLabel(tr(!_layer->getName().empty() ? _layer->getName().c_str() : "Image Layer"));
    _headerTitleBoxLayout->addWidget(label);

    // create opacity slider
    _opacitySlider = new QSlider(Qt::Horizontal);
    _opacitySlider->setMinimum(0);
    _opacitySlider->setMaximum(100);
    _opacitySlider->setValue((int)(_layer->getOpacity() * 100.0));
    _opacitySlider->setTracking(true);
    connect(_opacitySlider, SIGNAL(valueChanged(int)), this, SLOT(onSliderValueChanged(int)));
    _contentBoxLayout->addWidget(_opacitySlider);

    if (!_layer->getProfile())
    {
      _headerBox->setStyleSheet("background-color: #FF6666; color: white;");
      _visibleCheckBox->setEnabled(false);
      _opacitySlider->setEnabled(false);
    }
  }
  else
  {
    // _layer is null so just add a label stating so
    QLabel* label = new QLabel(tr("LAYER IS NULL"));
    _contentBoxLayout->addWidget(label);
  }
}

void ImageLayerControlWidget::onCheckStateChanged(int state)
{
  bool checked = state == Qt::Checked;
  if (_layer.valid() && _layer->getVisible() != checked)
    _layer->setVisible(checked);
}

void ImageLayerControlWidget::onSliderValueChanged(int value)
{
  float opacity = ((float)value) / 100.0f;
  if (_layer.valid() && _layer->getOpacity() != opacity)
    _layer->setOpacity(opacity);
}

void ImageLayerControlWidget::onRemoveClicked(bool checked)
{
  if (_parent && _parent->getMap())
    _parent->getMap()->removeImageLayer(_layer);
}

void ImageLayerControlWidget::setLayerVisible(bool visible)
{
  if ((_visibleCheckBox->checkState() == Qt::Checked) != visible)
    _visibleCheckBox->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
}

void ImageLayerControlWidget::setLayerOpacity(float opacity)
{
  int val = (int)(opacity * 100.0);
  if (_opacitySlider->value() != val)
    _opacitySlider->setValue(val);
}

osgEarth::UID ImageLayerControlWidget::getUID()
{
  if (!_layer)
    return -1;

  return _layer->getUID();
}

Action* ImageLayerControlWidget::getDoubleClickAction(const ViewVector& views)
{
  if (!_doubleClick.valid() && _layer.valid() && _layer->getProfile())
  {
    const osgEarth::GeoExtent llExt = _layer->getProfile()->getLatLongExtent();

    osg::Vec3d focalPoint((llExt.xMax() + llExt.xMin()) / 2.0,
                          (llExt.yMax() + llExt.yMin()) / 2.0,
                          0L);

	  double rangeFactor = llExt.yMax() != llExt.yMin() ? llExt.yMax() - llExt.yMin() : llExt.xMax() - llExt.xMin();
	  double range = ((0.5 * rangeFactor) / 0.267949849) * 111000.0;
	  if (range == 0.0)
		  range = 20000000.0;

    _doubleClick = new SetViewpointAction(osgEarth::Viewpoint("DoubleClick", focalPoint.x(), focalPoint.y(), focalPoint.z(), 0.0, -90.0, range), views);
  }

  return _doubleClick.get();
}


//---------------------------------------------------------------------------
ModelLayerControlWidget::ModelLayerControlWidget(osgEarth::ModelLayer* layer, LayerManagerWidget* parentManager, osgEarth::Map* map) : LayerControlWidgetBase(parentManager), _layer(layer), _map(map)
{
  initUi();

  if (_layer.valid())
  {
    _layerCallback = new WidgetModelLayerCallback(this);
    _layer->addCallback(_layerCallback.get());
  }
}

ModelLayerControlWidget::~ModelLayerControlWidget()
{
  if (_layer.valid())
    _layer->removeCallback(_layerCallback.get());

  _layer = 0L;
  _layerCallback = 0L;
  _doubleClick = 0L;
}

void ModelLayerControlWidget::initUi()
{
  if (_layer.valid())
  {
    // create enabled checkbox
    _visibleCheckBox = new QCheckBox();
    _visibleCheckBox->setCheckState(_layer->getVisible() ? Qt::Checked : Qt::Unchecked);
    connect(_visibleCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onEnabledCheckStateChanged(int)));
    _headerTitleBoxLayout->addWidget(_visibleCheckBox);

    // create name label
    QLabel* label = new QLabel(tr(!_layer->getName().empty() ? _layer->getName().c_str() : "Model Layer"));
    _headerTitleBoxLayout->addWidget(label);
  }
  else
  {
    // _layer is null so just add a label stating so
    QLabel* label = new QLabel(tr("LAYER IS NULL"));
    _contentBoxLayout->addWidget(label);
  }
}

void ModelLayerControlWidget::onEnabledCheckStateChanged(int state)
{
  bool checked = state == Qt::Checked;
  if (_layer.valid() && _layer->getVisible() != checked)
    _layer->setVisible(checked);
}

void ModelLayerControlWidget::onRemoveClicked(bool checked)
{
  if (_parent && _parent->getMap())
    _parent->getMap()->removeModelLayer(_layer);
}

void ModelLayerControlWidget::setLayerVisible(bool visible)
{
  if ((_visibleCheckBox->checkState() == Qt::Checked) != visible)
    _visibleCheckBox->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
}


osgEarth::UID ModelLayerControlWidget::getUID()
{
  if (!_layer)
    return -1;

  return _layer->getUID();
}

Action* ModelLayerControlWidget::getDoubleClickAction(const ViewVector& views)
{
  if (!_doubleClick.valid() && _layer.valid() && _map.valid())
  {
    osg::ref_ptr<osg::Node> temp = _layer->getOrCreateSceneGraph( _map.get(), _map->getReadOptions(), 0L );
    if (temp.valid())
    {
      osg::NodePathList nodePaths = temp->getParentalNodePaths();
      if (!nodePaths.empty())
      {
        osg::NodePath path = nodePaths[0];

        osg::Matrixd localToWorld = osg::computeLocalToWorld( path );
        osg::Vec3d center = osg::Vec3d(0,0,0) * localToWorld;

        const osg::BoundingSphere& bs = temp->getBound();

        // if the tether node is a MT, we are set. If it's not, we need to get the
        // local bound and add its translation to the localToWorld.
        if ( !dynamic_cast<osg::MatrixTransform*>( temp.get() ) )
          center += bs.center();

        GeoPoint output;
        output.fromWorld( _map->getSRS(), center );
        //_map->worldPointToMapPoint(center, output);

        //TODO: make a better range calculation
        return new SetViewpointAction(osgEarth::Viewpoint("DoubleClick", output.x(), output.y(), output.z(), 0.0, -90.0, bs.radius() * 4.0), views);
      }
    }
  }

  return _doubleClick.get();
}


//---------------------------------------------------------------------------
const std::string LayerManagerWidget::DEFAULT_STYLESHEET = "#oeFrameContainer, #oeFrameContainer * { background-color: rgba(255, 255, 255, 100%) } #oeItem, #oeItem * { background-color: lightgrey; } #oeItemHeader, #oeItemHeader * { background-color: grey; color: white; } #oeDropTarget, #oeDropTarget * { background: qlineargradient(x1:0 y1:0, x2:0 y2:1, stop:0 blue, stop:1 white); }";

LayerManagerWidget::LayerManagerWidget(DataManager* dm, LayerType type) : QScrollArea(), _manager(dm), _type(type)
{
  if (_manager.valid())
    _map = _manager->map();

  initialize();
}

LayerManagerWidget::LayerManagerWidget(osgEarth::Map* map, LayerType type) : QScrollArea(), _map(map), _type(type)
{
  initialize();
}

void LayerManagerWidget::setActiveView(osgViewer::View* view)
{
  _views.clear();
  _views.push_back(view);
}

void LayerManagerWidget::setActiveViews(const ViewVector& views)
{
  _views.clear();
  _views.insert(_views.end(), views.begin(), views.end());
}

void LayerManagerWidget::resetStyleSheet()
{
  setStyleSheet(tr(DEFAULT_STYLESHEET.c_str()));
}

void LayerManagerWidget::initialize()
{
  setWidgetResizable(true);
  setObjectName("oeFrameContainer");

  _stack = new QVBoxLayout;
	_stack->setSpacing(4);
	_stack->setContentsMargins(4, 4, 4, 4);
  _stack->setSizeConstraint(QLayout::SetMinimumSize);

  QWidget* stackWidget = new QWidget();
  stackWidget->setLayout(_stack);
  stackWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
  setWidget(stackWidget);


  //create drop decoration box
  _dropBox = new QFrame;
  QVBoxLayout* dbLayout = new QVBoxLayout();
  dbLayout->setSpacing(0);
  dbLayout->setContentsMargins(0, 0, 0, 0);
  _dropBox->setLayout(dbLayout);

  QFrame* dropBoxInternal = new QFrame();
  dropBoxInternal->setFixedHeight(40);
  dropBoxInternal->setObjectName("oeDropTarget");
  dbLayout->addSpacing(4);
  dbLayout->addWidget(dropBoxInternal);

  QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(_dropBox);
  dbEffect->setOpacity(0.35);
  _dropBox->setGraphicsEffect(dbEffect);


  resetStyleSheet();
  setAcceptDrops(true);

  if (_map.valid())
  {
    refresh();
    _map->addMapCallback(new LayerManagerMapCallback(this));
  }
}

void LayerManagerWidget::onItemDoubleClicked()
{
  if (!_manager.valid())
    return;

  Action* action = ((LayerControlWidgetBase*)sender())->getDoubleClickAction(_views);
  if (action)
    _manager->doAction(this, action);
}

void LayerManagerWidget::refresh()
{
  //TODO: Clear all items in _stack?
  //      Currently refresh() is only called from initialize() so clearing is not necessary.

  if (!_map.valid())
    return;

  if (_type == IMAGE_LAYERS)
  {
    osgEarth::ImageLayerVector layers;
    _map->getImageLayers(layers);
    for (osgEarth::ImageLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
      addImageLayerItem(*it);
  }
  else if (_type == MODEL_LAYERS)
  {
    osgEarth::ModelLayerVector layers;
    _map->getModelLayers(layers);
    for (osgEarth::ModelLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
      addModelLayerItem(*it);
  }
  else if (_type == ELEVATION_LAYERS)
  {
    osgEarth::ElevationLayerVector layers;
    _map->getElevationLayers(layers);
    for (osgEarth::ElevationLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
      addElevationLayerItem(*it);
  }
}

QWidget* LayerManagerWidget::findItemByUID(osgEarth::UID uid, int* out_row)
{
  for (int i = 0; i < _stack->count(); i++)
  {
    LayerControlWidgetBase* widget = dynamic_cast<LayerControlWidgetBase*>(_stack->itemAt(i)->widget());
    if (widget && widget->getUID() == uid)
    {
      if (out_row)
        (*out_row) = i;

      return widget;
    }
  }

  return 0L;
}

void LayerManagerWidget::addElevationLayerItem(osgEarth::ElevationLayer* layer, int index)
{
  if (_type != ELEVATION_LAYERS)
    return;

  ElevationLayerControlWidget* itemWidget = new ElevationLayerControlWidget(layer, this);
  _stack->insertWidget(index, itemWidget);
  
  connect(itemWidget, SIGNAL(doubleClicked()), this, SLOT(onItemDoubleClicked()));
}

void LayerManagerWidget::addImageLayerItem(osgEarth::ImageLayer* layer, int index)
{
  if (_type != IMAGE_LAYERS)
    return;

  ImageLayerControlWidget* itemWidget = new ImageLayerControlWidget(layer, this);
  _stack->insertWidget(index, itemWidget);
  connect(itemWidget, SIGNAL(doubleClicked()), this, SLOT(onItemDoubleClicked()));
}

void LayerManagerWidget::addModelLayerItem(osgEarth::ModelLayer* layer, int index)
{
  if (_type != MODEL_LAYERS)
    return;

  ModelLayerControlWidget* itemWidget = new ModelLayerControlWidget(layer, this, _map);
  _stack->insertWidget(index, itemWidget);
  connect(itemWidget, SIGNAL(doubleClicked()), this, SLOT(onItemDoubleClicked()));
}

void LayerManagerWidget::removeLayerItem(osgEarth::Layer* layer)
{
  if (!layer)
    return;

  QWidget* item = findItemByUID(layer->getUID());
  if (item)
    delete item;
}

void LayerManagerWidget::moveLayerItem(osgEarth::Layer* layer, int oldIndex, int newIndex)
{
  if (!layer)
    return;

  int row;
  QWidget* item = findItemByUID(layer->getUID(), &row);
  if (item && row != newIndex)
  {
    //_stack->takeAt(row);
    _stack->insertWidget(newIndex, item);
  }
}


void LayerManagerWidget::dragEnterEvent(QDragEnterEvent* event)
{
  if (event->mimeData()->hasFormat(LayerWidgetMimeData::MIME_TYPE))// && event->source()->parent() == this)
  {
    const LayerWidgetMimeData* widgetData = qobject_cast<const LayerWidgetMimeData*>(event->mimeData());
    if (widgetData && widgetData->getWidget()->getParentManager() == this)
    {
      _stack->addWidget(_dropBox);
      _dropBox->setVisible(true);

      event->acceptProposedAction();
    }
  }
}

void LayerManagerWidget::dragLeaveEvent(QDragLeaveEvent* event)
{
  _dropBox->setVisible(false);
  _stack->removeWidget(_dropBox);
}

void LayerManagerWidget::dropEvent(QDropEvent* event)
{
  _dropBox->setVisible(false);
  _stack->removeWidget(_dropBox);

  const LayerWidgetMimeData* widgetData = qobject_cast<const LayerWidgetMimeData*>(event->mimeData());
  if (widgetData)
    doLayerWidgetDrop(widgetData->getWidget());
}

void LayerManagerWidget::doLayerWidgetDrop(LayerControlWidgetBase* widget, LayerControlWidgetBase* dropOn)
{
  if (!widget)
    return;

  int oldRow = -1;
  findItemByUID(widget->getUID(), &oldRow);

  int newRow = -1;
  if (dropOn)
    findItemByUID(dropOn->getUID(), &newRow);

  if (oldRow >= 0 && oldRow < newRow)
    newRow--;

  if (oldRow != newRow)
  {
    _stack->insertWidget(newRow, widget);

    if (_type == ELEVATION_LAYERS)
    {
      ElevationLayerControlWidget* elevWidget = dynamic_cast<ElevationLayerControlWidget*>(widget);
      if (elevWidget)
        _map->moveElevationLayer(elevWidget->layer(), newRow >= 0 ? newRow : _stack->count() - 1);
    }
    else if (_type == IMAGE_LAYERS)
    {
      ImageLayerControlWidget* imageWidget = dynamic_cast<ImageLayerControlWidget*>(widget);
      if (imageWidget)
        _map->moveImageLayer(imageWidget->layer(), newRow >= 0 ? newRow : _stack->count() - 1);
    }
    else if (_type == MODEL_LAYERS)
    {
      ModelLayerControlWidget* modelWidget = dynamic_cast<ModelLayerControlWidget*>(widget);
      if (modelWidget)
        _map->moveModelLayer(modelWidget->layer(), newRow >= 0 ? newRow : _stack->count() - 1);
    }
  }
}
