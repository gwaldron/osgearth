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
#include "Surface"
#include "Biome"
#include <osgEarth/Map>
#include <osgEarth/XmlUtils>
#include <osgDB/Options>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[Surface] "

Surface::Surface()
{
    //nop
}

void
Surface::setCatalog(SplatCatalog* catalog)
{
    _biomeRegions.clear();
    if ( catalog )
    {
        _biomeRegions.push_back( BiomeRegion() );
        BiomeRegion& br = _biomeRegions.back();
        br.setCatalog( catalog );
    }
}

SplatCatalog*
Surface::getCatalog() const
{
    return _biomeRegions.size() > 0 ? _biomeRegions.front().getCatalog() : 0L;
}

bool
Surface::configure(const ConfigOptions& conf, const Map* map, const osgDB::Options* dbo)
{
    SurfaceOptions in(conf);

    _biomeRegions.clear();
    
    if ( in.biomesURI().isSet() )
    {
        osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in.biomesURI().get(), dbo );
        if ( doc.valid() )
        {
            Config conf = doc->getConfig().child("biomes");
            if ( !conf.empty() )
            {
                for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
                {
                    BiomeRegion br;
                    if ( br.configure(*i) )
                    {
                        if ( br.catalogURI().isSet() )
                        {
                            SplatCatalog* catalog = SplatCatalog::read( br.catalogURI().get(), dbo );
                            if ( catalog )
                            {
                                br.setCatalog( catalog );
                                _biomeRegions.push_back( br );
                            }
                        }
                    }
                }
            }
        }
    }

    // Otherwise, no biome file, so read a single catalog directly:
    if ( _biomeRegions.empty() )
    {
        // Read in the catalog.
        SplatCatalog* catalog = SplatCatalog::read( in.catalogURI().get(), dbo );
        if ( catalog )
        {
            _biomeRegions.push_back( BiomeRegion() );
            _biomeRegions.back().setCatalog( catalog );
        }
    }
    
    return true;
}
