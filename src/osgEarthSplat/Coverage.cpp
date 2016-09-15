/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "Coverage"
#include "SplatCoverageLegend"
#include <osgEarth/Map>
#include <osgEarth/XmlUtils>
#include <osgDB/Options>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[Coverage] "

Coverage::Coverage()
{
    //nop
}

void
Coverage::setLayer(ImageLayer* layer)
{
    _layer = layer;
}

bool 
Coverage::lockLayer(osg::ref_ptr<ImageLayer>& layer) const
{
    return _layer.lock(layer);
}

void
Coverage::setLegend(SplatCoverageLegend* value)
{
    _legend = value;
}

SplatCoverageLegend*
Coverage::getLegend() const
{
    return _legend.get();
}

bool
Coverage::configure(const ConfigOptions& conf, const Map* map, const osgDB::Options* dbo)
{
    CoverageOptions in( conf );
    
    if ( !in.layer().isSet() || in.layer()->empty() )
    {
        OE_WARN << LC << "Coverage much reference a map layer.\n";
        return false;
    }

    // Find the classification layer in the map:
    _layer = map->getImageLayerByName( in.layer().get() );
    if ( !_layer.valid() )
    {
        OE_WARN << LC << "Layer \"" << in.layer().get() << "\" not found in the map\n";
        return false;
    }

    if ( !in.legend().isSet() )
    {
        OE_WARN << LC << "Legend is required\n";
        return false;
    }

    // Load the legend from XML:
    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in.legend().get(), dbo );
    if ( doc.valid() )
    {
        _legend = new SplatCoverageLegend();
        _legend->fromConfig( doc->getConfig().child("legend") );
    }

    if ( !_legend.valid() || _legend->empty() )
    {
        OE_WARN << LC << "Failed to read required legend from \"" << in.legend()->full() << "\"\n";
        return false;
    }
    else
    {
        OE_INFO << LC << "Legend: found " << _legend->getPredicates().size() << " mappings \n";
    }

    return true;
}
