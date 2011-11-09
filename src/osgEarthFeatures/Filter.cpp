/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/Filter>
#include <osgEarth/ECEF>
#include <osg/MatrixTransform>

using namespace osgEarth;
using namespace osgEarth::Features;

void
FeaturesToNodeFilter::computeLocalizers( const FilterContext& context )
{
    if ( context.isGeoreferenced() )
    {
        const SpatialReference* geogSRS = context.profile()->getSRS()->getGeographicSRS();
        GeoExtent geodExtent = context.extent()->transform( geogSRS );
        if ( geodExtent.width() < 180.0 )
        {
            osg::Vec3d centroid, centroidECEF;
            geodExtent.getCentroid( centroid.x(), centroid.y() );
            geogSRS->transformToECEF( centroid, centroidECEF );
            _local2world = ECEF::createInverseRefFrame( centroidECEF );
            _world2local.invert( _local2world );
        }
    }
}

osg::Node*
FeaturesToNodeFilter::delocalize( osg::Node* node ) const
{
    if ( !_local2world.isIdentity() ) 
        return delocalizeAsGroup( node );
    else
        return node;
}

osg::Group*
FeaturesToNodeFilter::delocalizeAsGroup( osg::Node* node ) const
{
    osg::Group* group = createDelocalizeGroup();
    if ( node )
        group->addChild( node );
    return group;
}

osg::Group*
FeaturesToNodeFilter::createDelocalizeGroup() const
{
    osg::Group* group = _local2world.isIdentity() ?
        new osg::Group() :
        new osg::MatrixTransform( _local2world );

    return group;
}
