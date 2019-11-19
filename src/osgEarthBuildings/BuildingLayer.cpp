/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "BuildingLayer"
#include "BuildingCatalog"
#include "BuildingFactory"
#include "BuildingCompiler"
#include "BuildingPager"

#include <osgEarth/Registry>
#include <osgEarth/FeatureSourceIndexNode>

using namespace osgEarth;
using namespace osgEarth::Buildings;

#define LC "[BuildingLayer] "

REGISTER_OSGEARTH_LAYER(buildings, BuildingLayer);

//...................................................................

BuildingLayer::~BuildingLayer()
{
    //nop
}

void
BuildingLayer::init()
{
    VisibleLayer::init();

    // Create the root group
    _root = new osg::Group();
    _root->setName(getName());
}

void
BuildingLayer::setFeatureSource(FeatureSource* source)
{
    if (_featureSource.getLayer() != source)
    {
        _featureSource.setLayer(source);

        if (source && source->getStatus().isError())
        {
            setStatus(source->getStatus());
            return;
        }

        createSceneGraph();
    }
}

osg::Node*
BuildingLayer::getNode() const
{
    return _root.get();
}

Status
BuildingLayer::openImplementation()
{
    Status fsStatus = _featureSource.open(options().featureSource(), getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = _styleSheet.open(options().styleSheet(), getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    if (options().buildingCatalog().isSet())
    {
        _catalog = new BuildingCatalog();
        if (_catalog->load(options().buildingCatalog().get(), getReadOptions(), 0L) == false)
        {
            return Status(Status::ResourceUnavailable, "Cannot open building catalog");
            _catalog = 0L;
        }
    }
    else
    {
        return Status(Status::ConfigurationError, "Missing required catalog");
    }

    return VisibleLayer::openImplementation();
}

void
BuildingLayer::addedToMap(const Map* map)
{
    // Hang on to the Map reference
    _map = map;

    _featureSource.connect(map, options().featureSourceLayer());
    _styleSheet.connect(map, options().styleSheetLayer());

    // Set up a feature session with a cache:
    _session = new Session(
        map, 
        _styleSheet.getLayer(),
        _featureSource.getLayer(),
        getReadOptions() );
    
    // Install a resource cache that we will use for instanced models,
    // but not for skins; b/c we want to cache skin statesets per tile. So there is
    // a separate resource cache in the CompilerOutput class for that.
    _session->setResourceCache( new ResourceCache() );

    // Recreate the scene graph
    createSceneGraph();
}

void
BuildingLayer::createSceneGraph()
{
    const Profile* profile = 0L;

    // reinitialize the graph:
    _root->removeChildren(0, _root->getNumChildren());

    // resolve observer reference:
    osg::ref_ptr<const Map> map;
    _map.lock(map);

    // assertion:
    FeatureSource* fs = _featureSource.getLayer();
    if (!fs || !_session.valid() || !map.valid())
    {
        //if (getStatus().isOK())
        //    setStatus(Status(Status::ServiceUnavailable, "Internal assertion failure, call support"));
        return;
    }
    
    // Try to page against the feature profile, otherwise fallback to the map
    profile = fs->getFeatureProfile()->getTilingProfile();

    if (profile == 0L)
    {
        profile = _map->getProfile();
    }

    BuildingPager* pager = new BuildingPager( profile );
    pager->setElevationPool   ( _map->getElevationPool() );
    pager->setSession         ( _session.get() );
    pager->setFeatureSource   ( fs );
    pager->setCatalog         ( _catalog.get() );
    pager->setCompilerSettings( options().compilerSettings().get() );
    pager->setPriorityOffset  ( options().priorityOffset().get() );
    pager->setPriorityScale   ( options().priorityScale().get() );
    pager->setSceneGraphCallbacks(getSceneGraphCallbacks());

    if (options().filterUsage().isSet())
    {
       pager->setFilterUsage(options().filterUsage().get());
    }
    
    if (options().enableCancelation().isSet())
    {
        pager->setEnableCancelation(options().enableCancelation().get());
    }

    pager->build();

    if ( options().createIndex() == true )
    {
        // create a feature index.
        FeatureSourceIndex* index = new FeatureSourceIndex(
            fs,
            Registry::objectIndex(),
            FeatureSourceIndexOptions() );

        // ..and a node to house it.
        FeatureSourceIndexNode* inode = new FeatureSourceIndexNode( index );

        // tell the pager to generate an index
        pager->setIndex( inode );

        // install in the scene graph.
        inode->addChild( pager );
        _root->addChild( inode );
    }

    else
    {
        _root->addChild( pager );
    }
}

BuildingPager* BuildingLayer::pager()
{
   for (size_t i = 0; i < _root->getNumChildren(); ++i)
   {
      osg::Node* node = _root->getChild(i);
      BuildingPager* pager = dynamic_cast<BuildingPager*>(node);
      if (pager)
      {
         return pager;
      }
   }
   return 0;
}

void
BuildingLayer::removedFromMap(const Map* map)
{
    _featureSource.disconnect(map);
    _styleSheet.disconnect(map);
}

const GeoExtent&
BuildingLayer::getExtent() const
{
    const FeatureSource* fs = _featureSource.getLayer();

    if (fs && fs->getFeatureProfile())
    {
        return fs->getFeatureProfile()->getExtent();
    }

    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        return map->getProfile()->getExtent();
    }

    return GeoExtent::INVALID;
}
