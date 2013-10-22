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
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>


#include <osgEarth/Map>
#include <osgEarth/Viewpoint>
#include <osgEarthAnnotation/AnnotationNode>

#include <osg/MatrixTransform>

#include <QList>
#include <QWidget>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>

using namespace osgEarth::QtGui;

//---------------------------------------------------------------------------
namespace
{
  //---------------------------------------------------------------------------
  class ActionableTreeItem
  {
  public:
    virtual Action* getDoubleClickAction(const ViewVector& views) { return 0L; }
    virtual Action* getCheckStateAction(ViewVector& views) { return 0L; }
    virtual Action* getSelectionAction(bool selected) { return 0L; }
  };

  //---------------------------------------------------------------------------
  class CustomActionTreeItem : public QTreeWidgetItem, public ActionableTreeItem
  {
  public:
	  CustomActionTreeItem(osg::Referenced* obj) : _obj(obj), QTreeWidgetItem() {};
	  CustomActionTreeItem(osg::Referenced* obj, const QStringList &strings) : _obj(obj), QTreeWidgetItem(strings) {};
  	
	  osg::Referenced* getObj() const { return _obj.get(); }
	  void setSource(osg::Referenced* obj) { _obj = obj; }

    void setDoubleClickAction(Action* action) { _doubleClick = action; }
    Action* getDoubleClickAction(const ViewVector& views) { return _doubleClick.get(); }

    void setCheckStateAction(Action* action) { _checkState = action; }
    Action* getCheckStateAction() { return _checkState.get(); }
  	
  private:
	  osg::ref_ptr<osg::Referenced> _obj;
    osg::ref_ptr<Action> _doubleClick;
    osg::ref_ptr<Action> _checkState;
  };

  //---------------------------------------------------------------------------

  class LayerTreeItem : public QTreeWidgetItem, public ActionableTreeItem
  {
  public:
    LayerTreeItem(osgEarth::Layer* layer, osgEarth::Map* map) : _layer(layer), _map(map), QTreeWidgetItem() {};
	  LayerTreeItem(osgEarth::Layer* layer, osgEarth::Map* map, const QStringList &strings) : _layer(layer), _map(map), QTreeWidgetItem(strings) {};
  	
	  osgEarth::Layer* getLayer() const { return _layer.get(); }

    Action* getCheckStateAction(ViewVector& views)
    { 
        return new SetLayerVisibleAction(views, _layer.get(), checkState(0) == Qt::Checked); 
    }

    Action* getDoubleClickAction(const ViewVector& views)
    {
      if (!_doubleClick.valid() && _layer.valid())
      {
        osgEarth::TerrainLayer* terrain = dynamic_cast<osgEarth::TerrainLayer*>(_layer.get());
        if (terrain)
        {
          const osgEarth::GeoExtent llExt = terrain->getProfile()->getLatLongExtent();

          osg::Vec3d focalPoint((llExt.xMax() + llExt.xMin()) / 2.0,
                                (llExt.yMax() + llExt.yMin()) / 2.0,
                                0L);

          double rangeFactor = llExt.yMax() != llExt.yMin() ? llExt.yMax() - llExt.yMin() : llExt.xMax() - llExt.xMin();
          double range = ((0.5 * rangeFactor) / 0.267949849) * 111000.0;
          if (range == 0.0)
              range = 20000000.0;

          _doubleClick = new SetViewpointAction(osgEarth::Viewpoint(focalPoint, 0.0, -90.0, range), views);
        }
        else
        {
          osgEarth::ModelLayer* model = dynamic_cast<osgEarth::ModelLayer*>(_layer.get());
          if (model && _map.valid())
          {
            osg::ref_ptr<osg::Node> temp = model->createSceneGraph( _map.get(), _map->getDBOptions(), 0L );
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
                return new SetViewpointAction(osgEarth::Viewpoint(output.vec3d(), 0.0, -90.0, bs.radius() * 4.0), views);
              }
            }
          }
        }
      }

      return _doubleClick.get();
    }
  	
  private:
	  osg::ref_ptr<osgEarth::Layer> _layer;
    osg::ref_ptr<osgEarth::Map> _map;
    osg::ref_ptr<Action> _doubleClick;
  };

  //---------------------------------------------------------------------------
  class ToggleNodeTreeItem : public QTreeWidgetItem, public ActionableTreeItem
  {
  public:
    ToggleNodeTreeItem(osg::Node* node) : _node(node), QTreeWidgetItem() {};
	  ToggleNodeTreeItem(osg::Node* node, const QStringList &strings) : _node(node), QTreeWidgetItem(strings) {};
  	
	  osg::Node* getNode() const { return _node.get(); }

    Action* getCheckStateAction() { return new ToggleNodeAction(_node.get(), checkState(0) == Qt::Checked); }
  	
  private:
	  osg::ref_ptr<osg::Node> _node;
  };

  //---------------------------------------------------------------------------
  class AnnotationTreeItem : public ToggleNodeTreeItem
  {
  public:
    AnnotationTreeItem(osgEarth::Annotation::AnnotationNode* annotation, osgEarth::Map* map) : _annotation(annotation), _map(map), ToggleNodeTreeItem(annotation) {};
	  AnnotationTreeItem(osgEarth::Annotation::AnnotationNode* annotation, osgEarth::Map* map, const QStringList &strings) : _annotation(annotation), _map(map), ToggleNodeTreeItem(annotation, strings) {};
  	
	  osgEarth::Annotation::AnnotationNode* getAnnotation() const { return _annotation.get(); }

    Action* getSelectionAction(bool selected) { return 0L; } //new SetAnnotationHighlightAction(_annotation.get(), selected); }

    Action* getDoubleClickAction(const ViewVector& views)
    {
      if (_annotation.valid())
      {
        if (_annotation->getAnnotationData() && _annotation->getAnnotationData()->getViewpoint())
        {
          return new SetViewpointAction(osgEarth::Viewpoint(*_annotation->getAnnotationData()->getViewpoint()), views);
        }
        else if (_map.valid())
        {
          osg::Vec3d center = _annotation->getBound().center();

          GeoPoint output;
          output.fromWorld( _map->getSRS(), center );
          //_map->worldPointToMapPoint(center, output);

          return new SetViewpointAction(osgEarth::Viewpoint(output.vec3d(), 0.0, -90.0, 1e5), views);
        }
      }

      return 0L;
    }
  	
  private:
	  osg::ref_ptr<osgEarth::Annotation::AnnotationNode> _annotation;
    osg::ref_ptr<osgEarth::Map> _map;
    osgEarth::Viewpoint _vp;
  };

  //---------------------------------------------------------------------------
  class ViewpointTreeItem : public QTreeWidgetItem, public ActionableTreeItem
  {
  public:
    ViewpointTreeItem(osgEarth::Viewpoint vp) : _vp(vp), QTreeWidgetItem() {};
	  ViewpointTreeItem(osgEarth::Viewpoint vp, const QStringList &strings) : _vp(vp), QTreeWidgetItem(strings) {};
  	
	  const osgEarth::Viewpoint& getViewpoint() const { return _vp; }

    Action* getDoubleClickAction(const ViewVector& views)
    {
      if (!_doubleClick.valid())
        _doubleClick = new SetViewpointAction(_vp, views);

      return _doubleClick.get();
    }
  	
  private:
	  osgEarth::Viewpoint _vp;
    osg::ref_ptr<Action> _doubleClick;
  };
}

namespace osgEarth { namespace QtGui
{
  class MapCatalogActionCallbackProxy : public ActionCallback
  {
  public:
    MapCatalogActionCallbackProxy(MapCatalogWidget* catalog) : _catalog(catalog) { }

    void operator()( void* sender, Action* action )
    {
      if (_catalog)
      {
        Action* foundAction = dynamic_cast<ToggleNodeAction*>(action);
        if (foundAction)
          _catalog->refreshAll();
      }
    }

  private:
    MapCatalogWidget* _catalog;
  };
} }

//---------------------------------------------------------------------------
MapCatalogWidget::MapCatalogWidget(DataManager* dm, unsigned int fields)
  : _manager(dm), _fields(fields)
{
  if (_manager.valid())
  {
    _map = dm->map();
    connect(_manager.get(), SIGNAL(mapChanged()), this, SLOT(onMapChanged()));

    //if (_fields & ANNOTATIONS)
      connect(_manager.get(), SIGNAL(selectionChanged(/*const AnnotationVector&*/)), this, SLOT(onSelectionChanged(/*const AnnotationVector&*/)));

    _manager->addAfterActionCallback(new MapCatalogActionCallbackProxy(this));
  }

  initUi();
}

MapCatalogWidget::MapCatalogWidget(osgEarth::Map* map, unsigned int fields)
  : _map(map), _fields(fields)
{
  initUi();
}

void MapCatalogWidget::setActiveView(osgViewer::View* view)
{
  _views.clear();
  _views.push_back(view);
}

void MapCatalogWidget::setActiveViews(const ViewVector& views)
{
  _views.clear();
  _views.insert(_views.end(), views.begin(), views.end());
}

void MapCatalogWidget::setHideEmptyGroups(bool hide)
{
  if (_hideEmptyGroups == hide)
    return;

  _hideEmptyGroups = hide;
  refreshAll();
}

void MapCatalogWidget::initUi()
{
  _hideEmptyGroups = false;
  _updating = false;

  _tree = new QTreeWidget();
  _tree->setColumnCount(1);
  _tree->setHeaderHidden(true);
  _tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
  _tree->setObjectName("oeFrameContainer");
  connect(_tree, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(onTreeItemDoubleClicked(QTreeWidgetItem*, int)));
  connect(_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(onTreeItemChanged(QTreeWidgetItem*, int)));
  connect(_tree, SIGNAL(itemSelectionChanged()), this, SLOT(onTreeSelectionChanged()));

  _elevationsItem = 0;
  _imagesItem = 0;
  _modelsItem = 0;
  _annotationsItem = 0;
  _masksItem = 0;
  _viewpointsItem = 0;

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setSpacing(2);
  layout->setContentsMargins(3, 0, 3, 3);
  layout->addWidget(_tree);
  setLayout(layout);

  refreshAll();
}

void MapCatalogWidget::onMapChanged()
{
  refreshAll();
}

void MapCatalogWidget::onSelectionChanged(/*const AnnotationVector& selection*/)
{
  if (_fields & ANNOTATIONS)
    refreshAnnotations();
}

void MapCatalogWidget::onTreeItemDoubleClicked(QTreeWidgetItem* item, int col)
{
  if (!_manager.valid())
    return;

  ActionableTreeItem* actionable = dynamic_cast<ActionableTreeItem*>(item);
  if (actionable)
  {
      Action* action = actionable->getDoubleClickAction(_views);
      if (action)
          _manager->doAction(this, action);
  }
}

void MapCatalogWidget::onTreeItemChanged(QTreeWidgetItem* item, int col)
{
  if (!_manager.valid())
    return;

  ActionableTreeItem* actionable = dynamic_cast<ActionableTreeItem*>(item);
  if (actionable)
  {
      Action* action = actionable->getCheckStateAction(_views);
      if (action)
          _manager->doAction(this, action);
  }
}


void MapCatalogWidget::onTreeSelectionChanged()
{
  if (_fields & ANNOTATIONS && !_updating && _manager.valid())
  {
    AnnotationVector annos;

    QList<QTreeWidgetItem*> items = _tree->selectedItems();
    for (QList<QTreeWidgetItem*>::iterator it = items.begin(); it != items.end(); ++it)
    {
      AnnotationTreeItem* annoItem = dynamic_cast<AnnotationTreeItem*>(*it);
      if (annoItem)
        annos.push_back(annoItem->getAnnotation());
    }

    _manager->setSelectedAnnotations(annos);
  }
}


void MapCatalogWidget::refreshAll()
{
  if (!_map)
    return;

  _updating = true;

  refreshElevationLayers();
  refreshImageLayers();
  refreshModelLayers();
  refreshMaskLayers();
  refreshAnnotations();
  refreshViewpoints();

  _updating = false;
}

void MapCatalogWidget::refreshElevationLayers()
{
  bool wasUpdating = _updating;
  _updating = true;

  if (_fields & ELEVATION_LAYERS)
  {
    if (!_elevationsItem)
    {
      _elevationsItem = new QTreeWidgetItem();
      //_elevationsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _elevationsItem->setText(0, "Elevation Layers");
	    _tree->addTopLevelItem(_elevationsItem);
      _elevationsItem->setExpanded(true);
    }

    _elevationsItem->takeChildren();
	  
    osgEarth::ElevationLayerVector layers;
    _map->getElevationLayers(layers);
    for (osgEarth::ElevationLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      LayerTreeItem* layerItem = new LayerTreeItem(*it, _map);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
      //layerItem->setCheckState(0, (*it)->getVisible() ? Qt::Checked : Qt::Unchecked);
			_elevationsItem->addChild(layerItem);
    }

    _elevationsItem->setHidden(_hideEmptyGroups && _elevationsItem->childCount() == 0);
  }

  _updating = wasUpdating;
}

void MapCatalogWidget::refreshImageLayers()
{
  bool wasUpdating = _updating;
  _updating = true;

  if (_fields & IMAGE_LAYERS)
  {
    if (!_imagesItem)
    {
      _imagesItem = new QTreeWidgetItem();
      //_imagesItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _imagesItem->setText(0, "Image Layers");
	    _tree->addTopLevelItem(_imagesItem);
      _imagesItem->setExpanded(true);
    }

    _imagesItem->takeChildren();

    osgEarth::ImageLayerVector layers;
    _map->getImageLayers(layers);
    for (osgEarth::ImageLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      LayerTreeItem* layerItem = new LayerTreeItem(*it, _map);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
			layerItem->setCheckState(0, (*it)->getVisible() ? Qt::Checked : Qt::Unchecked);
			_imagesItem->addChild(layerItem);
    }

    _imagesItem->setHidden(_hideEmptyGroups && _imagesItem->childCount() == 0);
  }

  _updating = wasUpdating;
}

void MapCatalogWidget::refreshModelLayers()
{
  bool wasUpdating = _updating;
  _updating = true;

  if (_fields & MODEL_LAYERS)
  {
    if (!_modelsItem)
    {
      _modelsItem = new QTreeWidgetItem();
      //_modelsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _modelsItem->setText(0, "Model Layers");
	    _tree->addTopLevelItem(_modelsItem);
      _modelsItem->setExpanded(true);
    }

    _modelsItem->takeChildren();

    osgEarth::ModelLayerVector layers;
    _map->getModelLayers(layers);
    for (osgEarth::ModelLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      LayerTreeItem* layerItem = new LayerTreeItem(*it, _map);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
			layerItem->setCheckState(0, (*it)->getVisible() ? Qt::Checked : Qt::Unchecked);
			_modelsItem->addChild(layerItem);
    }

    _modelsItem->setHidden(_hideEmptyGroups && _modelsItem->childCount() == 0);
  }

  _updating = wasUpdating;
}

void MapCatalogWidget::refreshAnnotations()
{
  bool wasUpdating = _updating;
  _updating = true;

  if (_manager.valid() && (_fields & ANNOTATIONS))
  {
    if (!_annotationsItem)
    {
      _annotationsItem = new QTreeWidgetItem();
      //_annotationsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _annotationsItem->setText(0, "Annotations");
	    _tree->addTopLevelItem(_annotationsItem);
      _annotationsItem->setExpanded(true);
    }

    _annotationsItem->takeChildren();

    AnnotationVector annos;
    _manager->getAnnotations(annos);
    for (AnnotationVector::const_iterator it = annos.begin(); it != annos.end(); ++it)
    {
      AnnotationTreeItem* annoItem = new AnnotationTreeItem(*it, _map);
      annoItem->setText(0, QString(((*it)->getAnnotationData() ? (*it)->getAnnotationData()->getName().c_str() : "Annotation")));
			annoItem->setCheckState(0, (*it)->getNodeMask() != 0 ? Qt::Checked : Qt::Unchecked);

			_annotationsItem->addChild(annoItem);

      if (_manager->isSelected(*it))
        annoItem->setSelected(true);
    }

    _annotationsItem->setHidden(_hideEmptyGroups && _annotationsItem->childCount() == 0);
  }

  _updating = wasUpdating;
}

void MapCatalogWidget::refreshMaskLayers()
{
  bool wasUpdating = _updating;
  _updating = true;

  if (_fields & MASK_LAYERS)
  {
    if (!_masksItem)
    {
      _masksItem = new QTreeWidgetItem();
      //_masksItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _masksItem->setText(0, "Mask Layers");
	    _tree->addTopLevelItem(_masksItem);
      _masksItem->setExpanded(true);
    }

    _masksItem->takeChildren();

    osgEarth::MaskLayerVector layers;
    _map->getTerrainMaskLayers(layers);
    for (osgEarth::MaskLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      CustomActionTreeItem* layerItem = new CustomActionTreeItem(*it);
      layerItem->setText(0, QString((*it)->getName().c_str()));
			_masksItem->addChild(layerItem);
    }

    _masksItem->setHidden(_hideEmptyGroups && _masksItem->childCount() == 0);
  }

  _updating = wasUpdating;
}

void MapCatalogWidget::refreshViewpoints()
{
  bool wasUpdating = _updating;
  _updating = true;

  if (_manager.valid() && (_fields & VIEWPOINTS))
  {
    if (!_viewpointsItem)
    {
      _viewpointsItem = new QTreeWidgetItem();
      //_viewpointsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _viewpointsItem->setText(0, "Viewpoints");
	    _tree->addTopLevelItem(_viewpointsItem);
      _viewpointsItem->setExpanded(true);
    }

    _viewpointsItem->takeChildren();

    std::vector<osgEarth::Viewpoint> viewpoints;
    _manager->getViewpoints(viewpoints);
    for (std::vector<osgEarth::Viewpoint>::const_iterator it = viewpoints.begin(); it != viewpoints.end(); ++it)
    {
      ViewpointTreeItem* vpItem = new ViewpointTreeItem(*it);
      vpItem->setText(0, QString((*it).getName().c_str()));
			_viewpointsItem->addChild(vpItem);
    }

    _viewpointsItem->setHidden(_hideEmptyGroups && _viewpointsItem->childCount() == 0);
  }

  _updating = wasUpdating;
}
