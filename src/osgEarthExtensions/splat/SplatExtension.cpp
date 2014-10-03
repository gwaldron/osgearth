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
#include "SplatExtension"
#include "SplatCatalog"
#include "SplatCoverageLegend"
#include "SplatTerrainEffect"

#include <osgEarth/MapNode>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[SplatExtension] "


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

    if ( !_options.catalogURI().isSet() )
    {
        OE_WARN << LC << "Illegal: catalog URI is required" << std::endl;
        return false;
    }

    if ( !_options.legendURI().isSet() )
    {
        OE_WARN << LC << "Illegal: legend URI is required" << std::endl;
        return false;
    }

    if ( !_options.coverageLayerName().isSet() )
    {
        OE_WARN << LC << "Illegal: coverage layer name is required" << std::endl;
        return false;
    }

    // Locate the coverage layer in the map.
    const Map* map = mapNode->getMap();
    const ImageLayer* coverageLayer = map->getImageLayerByName( _options.coverageLayerName().get() );
    if ( !coverageLayer )
    {
        OE_WARN << LC << "Coverage layer \""
            << _options.coverageLayerName().get()
            << "\" not found in map."
            << std::endl;
        return false;
    }

    // Read in the catalog.
    osg::ref_ptr<SplatCatalog> catalog = new SplatCatalog();
    {
        ReadResult result = _options.catalogURI()->readString( _dbOptions.get() );
        if ( result.succeeded() )
        {
            Config conf;
            conf.setReferrer(_options.catalogURI()->full());

            std::string json = result.getString();
            conf.fromJSON( json );
            catalog->fromConfig( conf );

            OE_INFO << LC << "Catalog: " << catalog->getClasses().size() << " classes\n";
        }
        else
        {
            OE_WARN << LC
                << "Failed to read catalog from \""
                << _options.catalogURI()->full() << "\"\n";
            return false;
        }
    }

    // Read in the legend.
    osg::ref_ptr<SplatCoverageLegend> legend = new SplatCoverageLegend();
    {
        ReadResult result = _options.legendURI()->readString( _dbOptions.get() );
        if ( result.succeeded() )
        {
            Config conf;
            conf.setReferrer(_options.legendURI()->full());

            conf.fromJSON( result.getString() );
            legend->fromConfig( conf );

            OE_INFO << LC << "Legend: " << legend->getPredicates().size() << " mappings \n";
        }
        else
        {
            OE_WARN << LC
                << "Failed to read legend from \""
                << _options.legendURI()->full() << "\"\n";
            return false;
        }
    }

    // Install the splatter on the terrain engine.
    _effect = new SplatTerrainEffect( catalog, legend, _dbOptions.get() );

    // set the coverage layer (mandatory)
    _effect->setCoverageLayer( coverageLayer );

    // set the render order (optional)
    if ( _options.drawAfterImageLayers() == true )
        _effect->setRenderOrder( 1.0f );

    mapNode->getTerrainEngine()->addEffect( _effect.get() );

    return true;
}

bool
SplatExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
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
