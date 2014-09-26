/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include "EarthFileSerializer"
#include <osgEarth/FileUtils>
#include <osgEarth/MapFrame>

using namespace osgEarth_osgearth;
using namespace osgEarth;

MapNode*
EarthFileSerializer2::deserialize( const Config& conf, const std::string& referenceURI ) const
{
    MapOptions mapOptions( conf.child( "options" ) );

    // legacy: check for name/type in top-level attrs:
    if ( conf.hasValue( "name" ) || conf.hasValue( "type" ) )
    {
        Config legacy;
        if ( conf.hasValue("name") ) legacy.add( "name", conf.value("name") );
        if ( conf.hasValue("type") ) legacy.add( "type", conf.value("type") );
        mapOptions.mergeConfig( legacy );
    }

    Map* map = new Map( mapOptions );

    // Yes, MapOptions and MapNodeOptions share the same Config node. Weird but true.
    MapNodeOptions mapNodeOptions( conf.child( "options" ) );

    // Read the layers in LAST (otherwise they will not benefit from the cache/profile configuration)

    // Image layers:
    ConfigSet images = conf.children( "image" );
    for( ConfigSet::const_iterator i = images.begin(); i != images.end(); i++ )
    {
        Config layerDriverConf = *i;        

        ImageLayerOptions layerOpt( layerDriverConf );
        layerOpt.name() = layerDriverConf.value("name");
        //layerOpt.driver() = TileSourceOptions( layerDriverConf );

        map->addImageLayer( new ImageLayer(layerOpt) );
    }

    // Elevation layers:
    for( int k=0; k<2; ++k )
    {
        std::string tagName = k == 0 ? "elevation" : "heightfield"; // support both :)

        ConfigSet heightfields = conf.children( tagName );
        for( ConfigSet::const_iterator i = heightfields.begin(); i != heightfields.end(); i++ )
        {
            Config layerDriverConf = *i;            

            ElevationLayerOptions layerOpt( layerDriverConf );
            layerOpt.name() = layerDriverConf.value( "name" );

            map->addElevationLayer( new ElevationLayer(layerOpt) );
        }
    }

    // Model layers:
    ConfigSet models = conf.children( "model" );
    for( ConfigSet::const_iterator i = models.begin(); i != models.end(); i++ )
    {
        const Config& layerDriverConf = *i;

        ModelLayerOptions layerOpt( layerDriverConf );
        layerOpt.name() = layerDriverConf.value( "name" );
        layerOpt.driver() = ModelSourceOptions( layerDriverConf );

        map->addModelLayer( new ModelLayer(layerOpt) );
    }

    // Mask layer:
    ConfigSet masks = conf.children( "mask" );
    for( ConfigSet::const_iterator i = masks.begin(); i != masks.end(); i++ )
    {
        Config maskLayerConf = *i;

        MaskLayerOptions options(maskLayerConf);
        options.name() = maskLayerConf.value( "name" );
        options.driver() = MaskSourceOptions(options);

        map->addTerrainMaskLayer( new MaskLayer(options) );
    }

    
    //Add any addition paths specified in the options/osg_file_paths element to the file path.  Useful for pointing osgEarth at resource folders.
    Config osg_file_paths = conf.child( "options" ).child("osg_file_paths");
    ConfigSet urls = osg_file_paths.children("url");
    for (ConfigSet::const_iterator i = urls.begin(); i != urls.end(); i++) 
    {
        std::string path = osgEarth::getFullPath( referenceURI, (*i).value());
        OE_DEBUG << "Adding OSG file path " << path << std::endl;
        osgDB::Registry::instance()->getDataFilePathList().push_back( path );
    }

    MapNode* mapNode = new MapNode( map, mapNodeOptions );

    // External configs:
    Config ext = conf.child( "external" );
    if ( !ext.empty() )
    {
        mapNode->externalConfig() = ext;
    }

    return mapNode;
}


Config
EarthFileSerializer2::serialize( MapNode* input ) const
{
    Config mapConf("map");
    mapConf.set("version", "2");

    if ( !input || !input->getMap() )
        return mapConf;

    Map* map = input->getMap();
    MapFrame mapf( map, Map::ENTIRE_MODEL );

    // the map and node options:
    Config optionsConf = map->getInitialMapOptions().getConfig();
    optionsConf.merge( input->getMapNodeOptions().getConfig() );
    mapConf.add( "options", optionsConf );

    // the layers
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();
        //Config layerConf = layer->getInitialOptions().getConfig();
        Config layerConf = layer->getImageLayerOptions().getConfig();
        layerConf.set("name", layer->getName());
        layerConf.set("driver", layer->getInitialOptions().driver()->getDriver());        
        mapConf.add( "image", layerConf );
    }

    for( ElevationLayerVector::const_iterator i = mapf.elevationLayers().begin(); i != mapf.elevationLayers().end(); ++i )
    {
        ElevationLayer* layer = i->get();
        //Config layerConf = layer->getInitialOptions().getConfig();
        Config layerConf = layer->getElevationLayerOptions().getConfig();
        layerConf.set("name", layer->getName());
        layerConf.set("driver", layer->getInitialOptions().driver()->getDriver());        
        mapConf.add( "elevation", layerConf );
    }

    for( ModelLayerVector::const_iterator i = mapf.modelLayers().begin(); i != mapf.modelLayers().end(); ++i )
    {
        ModelLayer* layer = i->get();
        Config layerConf = layer->getModelLayerOptions().getConfig();
        layerConf.set("name", layer->getName());
        layerConf.set("driver", layer->getModelLayerOptions().driver()->getDriver());
        mapConf.add( "model", layerConf );
    }

    Config ext = input->externalConfig();
    if ( !ext.empty() )
    {
        ext.key() = "external";
        mapConf.add( ext );
    }

    return mapConf;
}
