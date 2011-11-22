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
#include <osgEarthQt/DataManager>

#include <osgEarth/Map>

#include <QWidget>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>

using namespace osgEarth::QtGui;

namespace
{
  class CustomObjectTreeItem : public QTreeWidgetItem
  {
  public:
	  CustomObjectTreeItem(osg::Referenced* obj) : _obj(obj), QTreeWidgetItem() {};
	  CustomObjectTreeItem(osg::Referenced* obj, const QStringList &strings) : _obj(obj), QTreeWidgetItem(strings) {};
  	
	  osg::Referenced* getObj() const { return _obj.get(); }
	  void setSource(osg::Referenced* obj) { _obj = obj; }
  	
  private:
	  osg::ref_ptr<osg::Referenced> _obj;
  };
}

MapCatalogWidget::MapCatalogWidget(DataManager* dm, unsigned int fields) : _manager(dm), _fields(fields)
{
  if (_manager)
  {
    _map = dm->map();
    connect(_manager.get(), SIGNAL(mapChanged()), this, SLOT(onMapChanged()));
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
  //TODO clean up _tree
}

void MapCatalogWidget::initUi()
{
	_tree = new QTreeWidget();
	_tree->setColumnCount(1);
	_tree->setHeaderHidden(true);

  _elevationsItem = 0;
  _imagesItem = 0;
  _modelsItem = 0;
  _masksItem = 0;

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
      CustomObjectTreeItem* layerItem = new CustomObjectTreeItem(*it);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
      layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
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
      CustomObjectTreeItem* layerItem = new CustomObjectTreeItem(*it);
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
      CustomObjectTreeItem* layerItem = new CustomObjectTreeItem(*it);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
			layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
			_modelsItem->addChild(layerItem);
    }
  }


  //TODO: Annotations


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
      CustomObjectTreeItem* layerItem = new CustomObjectTreeItem(*it);
      layerItem->setText(0, QString( (*it)->getName().c_str() ) );
			//layerItem->setCheckState(0, (*it)->getEnabled() ? Qt::Checked : Qt::Unchecked);
			_masksItem->addChild(layerItem);
    }
  }
}
