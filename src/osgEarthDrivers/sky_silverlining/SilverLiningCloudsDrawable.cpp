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
#include "SilverLiningCloudsDrawable"
#include "SilverLiningContext"
#include <osgEarth/SpatialReference>
#include <SilverLining.h>

#define LC "[SilverLining:SkyDrawable] "

using namespace osgEarth;
using namespace osgEarth::Drivers::SilverLining;

CloudsDrawable::CloudsDrawable(SilverLiningContext* SL) :
_SL( SL )
{
    // call this to ensure draw() gets called every frame.
    setSupportsDisplayList( false );
    
    // not MT-safe (camera updates, etc)
    this->setDataVariance(osg::Object::DYNAMIC);    
}

void
CloudsDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    if(_SL->ready())
    {
        renderInfo.getState()->disableAllVertexArrays();
        _SL->getAtmosphere()->DrawObjects( true, true, true );
        renderInfo.getState()->dirtyAllVertexArrays();
    }
}

osg::BoundingBox
#if OSG_VERSION_GREATER_THAN(3,3,1)
CloudsDrawable::computeBoundingBox() const
#else
CloudsDrawable::computeBound() const
#endif
{
    osg::BoundingBox cloudBoundBox;
    if ( !_SL->ready() )
        return cloudBoundBox;
    
    double minX, minY, minZ, maxX, maxY, maxZ;
    _SL->getAtmosphere()->GetCloudBounds( minX, minY, minZ, maxX, maxY, maxZ );
    cloudBoundBox.set( osg::Vec3d(minX, minY, minZ), osg::Vec3d(maxX, maxY, maxZ) );
    return cloudBoundBox;
}
