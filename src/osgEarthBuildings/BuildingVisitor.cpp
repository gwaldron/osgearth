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
#include "BuildingVisitor"

using namespace osgEarth;
using namespace osgEarth;
using namespace osgEarth::Buildings;

void
BuildingVisitor::traverse(Building* building)
{
    for(ElevationVector::iterator i = building->getElevations().begin();
        i != building->getElevations().end();
        ++i)
    {
        apply( i->get() );
    }
}

void
BuildingVisitor::traverse(Elevation* elevation)
{
    for(ElevationVector::iterator i = elevation->getElevations().begin();
        i != elevation->getElevations().end();
        ++i)
    {
        apply( i->get() );
    }

    if ( elevation->getRoof() )
    {
        apply( elevation->getRoof() );
    }
}

void
BuildingVisitor::traverse(Roof* roof)
{
    //nop
}
