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
#include "KML_Polygon"
#include "KML_LinearRing"
#include <iterator>

using namespace osgEarth_kml;

void
KML_Polygon::parseStyle(const Config& conf, KMLContext& cx, Style& style)
{
    KML_Geometry::parseStyle(conf, cx, style);

    // need at minimum a poly symbol.
    if ( !style.has<PolygonSymbol>() )
    {
        style.getOrCreate<PolygonSymbol>()->fill()->color() = osg::Vec4f(1,1,1,1);
    }
}

void
KML_Polygon::parseCoords( const Config& conf, KMLContext& cx )
{
    Polygon* poly = new Polygon();

    Config outerConf = conf.child("outerboundaryis");
    if ( !outerConf.empty() )
    {
        Config outerRingConf = outerConf.child("linearring");
        if ( !outerRingConf.empty() )
        {
            KML_LinearRing outer;
            outer.parseCoords( outerRingConf, cx );
            if ( outer._geom.valid() )
            {
                dynamic_cast<Ring*>(outer._geom.get())->rewind( Ring::ORIENTATION_CCW );
                poly->reserve( outer._geom->size() );
                std::copy( outer._geom->begin(), outer._geom->end(), std::back_inserter(*poly) );
            }
        }

        ConfigSet innerConfs = conf.children("innerboundaryis");
        for( ConfigSet::const_iterator i = innerConfs.begin(); i != innerConfs.end(); ++i )
        {
            Config innerRingConf = i->child("linearring");
            if ( !innerRingConf.empty() )
            {
                KML_LinearRing inner;
                inner.parseCoords( innerRingConf, cx );
                if ( inner._geom.valid() )
                {
                    Geometry* innerGeom = inner._geom.get();
                    dynamic_cast<Ring*>(innerGeom)->rewind( Ring::ORIENTATION_CW );
                    poly->getHoles().push_back( dynamic_cast<Ring*>(innerGeom) );
                }
            }
        }
    }

    _geom = poly;
}
