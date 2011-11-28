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
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>


#include <osgEarth/Map>
#include <osgEarth/Viewpoint>
#include <osgEarthAnnotation/AnnotationNode>

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
    virtual Action* getDoubleClickAction() { return 0L; }
    virtual Action* getCheckStateAction() { return 0L; }
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
    Action* getDoubleClickAction() { return _doubleClick.get(); }

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
    LayerTreeItem(osgEarth::Layer* layer) : _layer(layer), QTreeWidgetItem() {};
	  LayerTreeItem(osgEarth::Layer* layer, const QStringList &strings) : _layer(layer), QTreeWidgetItem(strings) {};
  	
	  osgEarth::Layer* getLayer() const { return _layer.get(); }

    Action* getCheckStateAction() { return new SetLayerEnabledAction(_layer.get(), checkState(0) == Qt::Checked); }

    Action* getDoubleClickAction()
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

          _doubleClick = new SetViewpointAction(osgEarth::Viewpoint(focalPoint, 0.0, -90.0, range));
        }
        else
        {
          osgEarth::ModelLayer* model = dynamic_cast<osgEarth::ModelLayer*>(_layer.get());
          if (model)
          {
            //TODO: creat SetViewpointAction for ModelLayers
          }
        }

        //_doubleClick = new SetViewpointAction(osgEarth::Viewpoint(_layer->getBound().center(), 0.0, -90.0, 1e5));
      }

      return _doubleClick.get();
    }
  	
  private:
	  osg::ref_ptr<osgEarth::Layer> _layer;
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
  	
	  osg::Node* getAnnotation() const { return _annotation.get(); }

    Action* getSelectionAction(bool selected) { return new SetAnnotationHighlightAction(_annotation.get(), selected); }

    Action* getDoubleClickAction()
    {
      if (_annotation.valid() && _annotation->getAnnotationData() && _map.valid())
      {
        if (_annotation->getAnnotationData() && _annotation->getAnnotationData()->getViewpoint())
        {
          return new SetViewpointAction(osgEarth::Viewpoint(*_annotation->getAnnotationData()->getViewpoint()));
        }
        else
        {
          osg::Vec3d center = _annotation->getBound().center();

          osg::Vec3d output;
          _map->worldPointToMapPoint(center, output);

          return new SetViewpointAction(osgEarth::Viewpoint(output, 0.0, -90.0, 1e5));
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

    Action* getDoubleClickAction()
    {
      if (!_doubleClick.valid())
        _doubleClick = new SetViewpointAction(_vp);

      return _doubleClick.get();
    }
  	
  private:
	  osgEarth::Viewpoint _vp;
    osg::ref_ptr<Action> _doubleClick;
  };
}

//---------------------------------------------------------------------------
MapCatalogWidget::MapCatalogWidget(DataManager* dm, unsigned int fields) : _manager(dm), _fields(fields)
{
  if (_manager)
  {
    _map = dm->map();
    connect(_manager.get(), SIGNAL(mapChanged()), this, SLOT(onMapChanged()));
    _manager->addAfterActionCallback(this);
  }

  initUi();
  refresh();
}

MapCatalogWidget::MapCatalogWidget(osgEarth::Map* map, unsigned int fields) : _map(map), _fields(fields)
{
  initUi();
  refresh();
}

MapCatalogWidget::~MapCatalogWidget()
{
}

void MapCatalogWidget::initUi()
{
	_tree = new QTreeWidget();
	_tree->setColumnCount(1);
	_tree->setHeaderHidden(true);
  _tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
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
}

void MapCatalogWidget::onMapChanged()
{
  refresh();
}

void MapCatalogWidget::onTreeItemDoubleClicked(QTreeWidgetItem* item, int col)
{
  if (!_manager)
    return;

  ActionableTreeItem* actionable = dynamic_cast<ActionableTreeItem*>(item);
	if (actionable)
	{
    Action* action = actionable->getDoubleClickAction();
    if (action)
      _manager->doAction(this, action);
  }
}

void MapCatalogWidget::onTreeItemChanged(QTreeWidgetItem* item, int col)
{
  if (!_manager)
    return;

  ActionableTreeItem* actionable = dynamic_cast<ActionableTreeItem*>(item);
	if (actionable)
	{
    Action* action = actionable->getCheckStateAction();
    if (action)
      _manager->doAction(this, action);
  }
}

void MapCatalogWidget::onTreeSelectionChanged()
{
  if (!_manager)
    return;

  //TODO: figure out how to handle selection changes

  for (int i=0; i < _tree->topLevelItemCount(); i++)
  {
    QTreeWidgetItem *item = _tree->topLevelItem(i);
    fireSelectionAction(item);
  }
}

void MapCatalogWidget::fireSelectionAction(QTreeWidgetItem* item)
{
  //TODO: do for current item and recurse
}

void MapCatalogWidget::refresh()
{
  //_tree->clear();

  if (!_map)
    return;

  if (_fields & ELEVATION_LAYERS)
  {
    if (!_elevationsItem)
    {
      _elevationsItem = new QTreeWidgetItem();
      //_elevationsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _elevationsItem->setText(0, "Elevation Layers");
	    _tree->addTopLevelItem(_elevationsItem);
    }

    _elevationsItem->takeChildren();
	  
    osgEarth::ElevationLayerVector layers;
    _map->getElevationLayers(layers);
    for (osgEarth::ElevationLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      LayerTreeItem* layerItem = new LayerTreeItem(*it);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
      //layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
			_elevationsItem->addChild(layerItem);
    }
  }

  if (_fields & IMAGE_LAYERS)
  {
    if (!_imagesItem)
    {
      _imagesItem = new QTreeWidgetItem();
      //_imagesItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _imagesItem->setText(0, "Image Layers");
	    _tree->addTopLevelItem(_imagesItem);
    }

    _imagesItem->takeChildren();

    osgEarth::ImageLayerVector layers;
    _map->getImageLayers(layers);
    for (osgEarth::ImageLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      LayerTreeItem* layerItem = new LayerTreeItem(*it);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
			layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
			_imagesItem->addChild(layerItem);
    }
  }

  if (_fields & MODEL_LAYERS)
  {
    if (!_modelsItem)
    {
      _modelsItem = new QTreeWidgetItem();
      //_modelsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _modelsItem->setText(0, "Model Layers");
	    _tree->addTopLevelItem(_modelsItem);
    }

    _modelsItem->takeChildren();

    osgEarth::ModelLayerVector layers;
    _map->getModelLayers(layers);
    for (osgEarth::ModelLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      LayerTreeItem* layerItem = new LayerTreeItem(*it);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
			layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
			_modelsItem->addChild(layerItem);
    }
  }

  if (_manager && (_fields & ANNOTATIONS))
  {
    if (!_annotationsItem)
    {
      _annotationsItem = new QTreeWidgetItem();
      //_annotationsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _annotationsItem->setText(0, "Annotations");
	    _tree->addTopLevelItem(_annotationsItem);
    }

    _annotationsItem->takeChildren();

    AnnotationVector annos;
    _manager->getAnnotations(annos);
    for (AnnotationVector::const_iterator it = annos.begin(); it != annos.end(); ++it)
    {
      AnnotationTreeItem* annoItem = new AnnotationTreeItem(*it, _map);
      annoItem->setText(0, QString(((*it)->getAnnotationData() ? (*it)->getAnnotationData()->getName().c_str() : "Annotation")));
			annoItem->setCheckState(0, (*it)->getNodeMask() != 0 ? Qt::Checked : Qt::Unchecked);
      annoItem->setSelected((*it)->getHightlight());
			_annotationsItem->addChild(annoItem);
    }
  }

  if (_fields & MASK_LAYERS)
  {
    if (!_masksItem)
    {
      _masksItem = new QTreeWidgetItem();
      //_masksItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _masksItem->setText(0, "Mask Layers");
	    _tree->addTopLevelItem(_masksItem);
    }

    _masksItem->takeChildren();

    osgEarth::MaskLayerVector layers;
    _map->getTerrainMaskLayers(layers);
    for (osgEarth::MaskLayerVector::const_iterator it = layers.begin(); it != layers.end(); ++it)
    {
      CustomActionTreeItem* layerItem = new CustomActionTreeItem(*it);
      layerItem->setText(0, QString((*it)->getName().c_str()));
			//layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
			_masksItem->addChild(layerItem);
    }
  }

  if (_manager && (_fields & VIEWPOINTS))
  {
    if (!_viewpointsItem)
    {
      _viewpointsItem = new QTreeWidgetItem();
      //_viewpointsItem->setIcon(0, QIcon(":/resources/images/globe.png"));
	    _viewpointsItem->setText(0, "Viewpoints");
	    _tree->addTopLevelItem(_viewpointsItem);
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
  }
}

//ActionCallback
void MapCatalogWidget::operator()( void* sender, Action* action )
{
  Action* foundAction = dynamic_cast<SetLayerEnabledAction*>(action);
  if (!foundAction)
  {
    foundAction = dynamic_cast<ToggleNodeAction*>(action);
    if (!foundAction)
    {
      foundAction = dynamic_cast<SetAnnotationHighlightAction*>(action);
    }
  }

  if (foundAction)
    refresh();
}