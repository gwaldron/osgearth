
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
#include "Parapet"
#include "BuildContext"

#define LC "[Parapet] "

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace osgEarth::Buildings;

Parapet::Parapet() :
_width( 0.0f )
{
    _numFloors = 1u;
    setTag("parapet");
}

Parapet::Parapet(const Parapet& rhs) :
Elevation( rhs ),
_width   ( rhs._width )
{
}

Elevation*
Parapet::clone() const
{
    return new Parapet(*this);
}


void
Parapet::setWidth(float width)
{
    _width = width;

    if ( width > 0.0f )
    {
        _roof = new Roof();
        _roof->setType( Roof::TYPE_FLAT );
        _roof->setTag("parapet");
    }
    else
    {
        _roof = 0L;
    }
}

bool
Parapet::build(const Polygon* footprint, BuildContext& bc)
{
    if ( getWidth() > 0.0f )
    {
        // copy the outer ring of the footprint. Ignore any holes.
        osg::ref_ptr<Polygon> copy = dynamic_cast<Polygon*>(footprint->clone());
    
        // apply a negative buffer to the outer ring:
        osg::ref_ptr<Geometry> hole;
        BufferParameters bp(BufferParameters::CAP_DEFAULT, BufferParameters::JOIN_MITRE);
        if ( copy->buffer(-getWidth(), hole, bp) )
        {
            Ring* ring = dynamic_cast<Ring*>( hole.get() );
            if ( ring )
            {
                // rewind the new geometry CW and add it as a hole:
                ring->rewind(Geometry::ORIENTATION_CW);
                copy->getHoles().push_back( ring );
                return Elevation::build( copy.get(), bc );
            }
        }
    }

    return Elevation::build( footprint, bc );
}

Config
Parapet::getConfig() const
{
    Config conf = Elevation::getConfig();
    conf.add("type", "parapet");
    conf.add("width", getWidth());
    return conf;
}