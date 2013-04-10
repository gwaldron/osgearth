/* -*-c++-*- */
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
#include "EarthFileSerializer"

using namespace osgEarth_osgearth;
using namespace osgEarth;

MapNode*
EarthFileSerializer1::deserialize( const Config& conf, const std::string& referenceURI ) const
{
    // piece together a MapOptions, TerrainOptions, and MapNodeOptions:
    Config mapOptionsConf;
    mapOptionsConf.setReferrer( conf.referrer() );

    if ( conf.hasValue("name") )
        mapOptionsConf.update("name", conf.value("name"));
    if ( conf.hasValue("type") )
        mapOptionsConf.update("type", conf.value("type"));

    Config terrainOptionsConf;
    terrainOptionsConf.setReferrer( conf.referrer() );

    Config mapNodeOptionsConf;
    mapNodeOptionsConf.setReferrer( conf.referrer() );

    for( ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i )
    {
        const Config& child = *i;

        if (child.key() == "profile" || 
            child.key() == "cache" )
        {
            if (child.key() == "cache")
            {
                std::string type = child.value("type");
                if (type.empty())
                    type = "filesystem";
                Config cacheConfig(child);
                cacheConfig.set("driver", type );
                mapOptionsConf.add( cacheConfig );
            }
            else
            {
                mapOptionsConf.add( child );
            }
        }
        else if (
            child.key() == "proxy" ||
            child.key() == "cache_only" )
        {
            mapNodeOptionsConf.add( child );
        }
        else if (
            child.key() == "vertical_scale" ||
            child.key() == "sample_ratio" ||
            child.key() == "min_tile_range_factor" ||
            child.key() == "normalize_edges" ||
            child.key() == "combine_layers" ||
            child.key() == "loading_policy" || 
            child.key() == "max_lod" ||
            child.key() == "lighting" )
        {
            terrainOptionsConf.add( child );
        }
        else if ( child.key() == "layering_technique" )
        {
            if ( child.value() == "multipass" )
                terrainOptionsConf.update( "compositor", "multipass");
        }
    }
    MapOptions mapOptions( mapOptionsConf );
    MapNodeOptions mapNodeOptions( mapNodeOptionsConf );
    mapNodeOptions.setTerrainOptions( TerrainOptions(terrainOptionsConf) );

    Map* map = new Map( mapOptions );

    // Read the layers in LAST (otherwise they will not benefit from the cache/profile configuration)

    // Image layers:
    ConfigSet images = conf.children( "image" );
    for( ConfigSet::const_iterator i = images.begin(); i != images.end(); i++ )
    {
        Config layerDriverConf = *i;
        layerDriverConf.add( "default_tile_size", "256" );

        ImageLayerOptions layerOpt( layerDriverConf );
        layerOpt.name() = layerDriverConf.value("name");
        layerOpt.driver() = TileSourceOptions( layerDriverConf );

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
            layerDriverConf.add( "default_tile_size", "16" );

            ElevationLayerOptions layerOpt( layerDriverConf );
            layerOpt.name() = layerDriverConf.value( "name" );
            layerOpt.driver() = TileSourceOptions( layerDriverConf );

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
        //map->addModelLayer( new ModelLayer( i->value("name"), ModelSourceOptions(*i) ) );
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

    return new MapNode( map, mapNodeOptions );
}

