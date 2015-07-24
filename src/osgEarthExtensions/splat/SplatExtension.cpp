/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include "SplatExtension"
#include "SplatCatalog"
#include "Biome"
#include "SplatCoverageLegend"
#include "SplatTerrainEffect"

#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/XmlUtils>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[SplatExtension] "

//.........................................................................

namespace
{
}

//.........................................................................

SplatExtension::SplatExtension()
{
    //nop
}

SplatExtension::SplatExtension(const SplatOptions& options) :
_options( options )
{
    //nop
}

SplatExtension::~SplatExtension()
{
    //nop
}

void
SplatExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
SplatExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

    OE_INFO << LC << "Connecting to MapNode.\n";

    if ( !_options.catalogURI().isSet() && !_options.biomesURI().isSet() )
    {
        OE_WARN << LC << "Illegal: either a catalog URI or a biomes URI is required.\n";
        return false;
    }

    if ( !_options.legendURI().isSet() )
    {
        OE_WARN << LC << "Illegal: a legend URI is required.\n";
        return false;
    }

    if ( !_options.coverageLayerName().isSet() )
    {
        OE_WARN << LC << "Illegal: a coverage layer name is required.\n";
        return false;
    }

    // Locate the coverage layer in the map.
    const Map* map = mapNode->getMap();
    const ImageLayer* coverageLayer = map->getImageLayerByName( _options.coverageLayerName().get() );
    if ( !coverageLayer )
    {
        OE_WARN << LC << "Coverage layer \""
            << _options.coverageLayerName().get()
            << "\" not found in map.\n";
        return false;
    }

    // Read in the legend.
    osg::ref_ptr<SplatCoverageLegend> legend = new SplatCoverageLegend();
    {
        osg::ref_ptr<XmlDocument> doc = XmlDocument::load(
            _options.legendURI().get(),
            _dbOptions.get() );

        if ( doc.valid() )
        {
            legend->fromConfig( doc->getConfig().child("legend") );
        }

        if ( legend->empty() )
        {
            OE_WARN << LC
                << "Failed to read required legend from \""
                << _options.legendURI()->full() << "\"\n";
            return false;
        }
        else
        {
            OE_INFO << LC << "Legend: found " << legend->getPredicates().size() << " mappings \n";
        }
    }

    BiomeVector biomes;

    // If there is a biome file, read it; it will point to one or more catalog files.
    if ( _options.biomesURI().isSet() )
    {
        osg::ref_ptr<XmlDocument> doc = XmlDocument::load(
            _options.biomesURI().get(),
            _dbOptions.get() );

        if ( doc.valid() )
        {
            Config conf = doc->getConfig().child("biomes");
            if ( !conf.empty() )
            {
                for(ConfigSet::const_iterator i = conf.children().begin();
                    i != conf.children().end();
                    ++i)
                {
                    Biome biome( *i );
                    if ( biome.catalogURI().isSet() )
                    {
                        SplatCatalog* catalog = SplatCatalog::read( biome.catalogURI().get(), _dbOptions.get() );
                        if ( catalog )
                        {
                            biome.setCatalog( catalog );
                            biomes.push_back( biome );
                        }
                    }
                }
            }
        }
    }

    // Otherwise, no biome file, so read a single catalog directly:
    if ( biomes.empty() )
    {
        // Read in the catalog.
        SplatCatalog* catalog = SplatCatalog::read(
            _options.catalogURI().get(),
            _dbOptions.get() );

        if ( catalog )
        {
            biomes.push_back( Biome() );
            biomes.back().setCatalog( catalog );
        }
    }

    if ( !biomes.empty() )
    {
        // Terrain effect that implements splatting.
        _effect = new SplatTerrainEffect( biomes, legend, _dbOptions.get() );

        // set the coverage layer (mandatory)
        _effect->setCoverageLayer( coverageLayer );

        // set the render order (optional)
        if ( _options.drawAfterImageLayers() == true )
            _effect->setRenderOrder( 1.0f );

        // set the various rendering options.
        if ( _options.coverageWarp().isSet() )
            _effect->getCoverageWarpUniform()->set( _options.coverageWarp().get() );
    
        if ( _options.coverageBlur().isSet() )
            _effect->getCoverageBlurUniform()->set( _options.coverageBlur().get() );

        if ( _options.scaleLevelOffset().isSet() )
            _effect->getScaleLevelOffsetUniform()->set( (float)_options.scaleLevelOffset().get() );

        // add it to the terrain.
        mapNode->getTerrainEngine()->addEffect( _effect.get() );
    }

    else
    {
        OE_WARN << LC << "Extension not installed become there are no valid biomes.\n";
    }

    return true;
}

bool
SplatExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode && _effect.valid() )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
    }
    _effect = 0L;
    return true;
}

bool
SplatExtension::connect(Control* control)
{
    //TODO add a UI.
    Container* container = dynamic_cast<Container*>(control);
    if ( container )
    {
        container->addControl( new LabelControl("Splatting is on!") );
    }
    return true;
}

bool
SplatExtension::disconnect(Control* control)
{
    // NOP
    return true;
}
