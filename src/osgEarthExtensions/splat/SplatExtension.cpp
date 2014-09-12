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
#include "SplatClassificationMap"

#include <osgEarth/MapNode>

using namespace osgEarth;
using namespace osgEarth::Extensions::Splat;

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
SplatExtension::startup(MapNode* mapNode, const osgDB::Options* dbOptions)
{
    OE_INFO << LC << "Startup.\n";

    if ( !_options.catalogURI().isSet() )
    {
        OE_WARN << LC << "Illegal: catalog URI is required" << std::endl;
        return;
    }

    if ( !_options.classMapURI().isSet() )
    {
        OE_WARN << LC << "Illegal: classification map URI is required" << std::endl;
        return;
    }

    // Read in the catalog.
    osg::ref_ptr<SplatCatalog> catalog = new SplatCatalog();
    {
        ReadResult result = _options.catalogURI()->readString( dbOptions );
        if ( result.succeeded() )
        {
            Config conf;
            conf.fromJSON( result.getString() );
            catalog->fromConfig( conf );
        }
        else
        {
            OE_WARN << LC
                << "Failed to read catalog from \""
                << _options.catalogURI()->full() << "\"\n";
            return;
        }
    }

    // Read in the classification map.
    osg::ref_ptr<SplatClassificationMap> classmap = new SplatClassificationMap();
    {
        ReadResult result = _options.classMapURI()->readString( dbOptions );
        if ( result.succeeded() )
        {
            Config conf;
            conf.fromJSON( result.getString() );
            classmap->fromConfig( conf );
        }
        else
        {
            OE_WARN << LC
                << "Failed to read classification map from \""
                << _options.classMapURI()->full() << "\"\n";
            return;
        }
    }
}

void
SplatExtension::shutdown(MapNode* mapNode)
{
    OE_INFO << LC << "Shutdown.\n";
}
