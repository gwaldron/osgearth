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

#include <osgEarthAnnotation/LocalizedNode>
#include <osgEarth/Utils>

using namespace osgEarth;
using namespace osgEarth::Annotation;


LocalizedNode::LocalizedNode( MapNode* mapNode, const osg::Vec3d& pos ) :
_mapNode       ( mapNode ),
_horizonCulling( false )
{
    _xform = new osg::MatrixTransform();
    if ( mapNode )
    {
        setHorizonCulling( true );
        setPosition( pos );
    }
}

bool
LocalizedNode::setPosition( const osg::Vec3d& pos )
{
    return setPosition( pos, 0L );
}

bool
LocalizedNode::setPosition( const osg::Vec3d& pos, const SpatialReference* srs )
{
    osg::ref_ptr<MapNode> mapNode = _mapNode.get();
    if ( mapNode.valid() )
    {
        Map* map = mapNode->getMap();

        osg::Vec3d mapPos = pos;
        if ( srs )
        {
            if ( ! map->toMapPoint( pos, srs, mapPos ) )
                return false;
        }

        // update the transform:
        osg::Matrixd local2world;
        map->getProfile()->getSRS()->createLocal2World( mapPos, local2world );
        _xform->setMatrix( local2world );

        // and update the culler:
        CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(_xform->getCullCallback());
        if ( culler )
        {
            culler->_world = local2world.getTrans();
        }
        return true;
    }   
    else return false;
}

osg::Vec3d
LocalizedNode::getPosition() const
{
    return getPosition( 0L );
}

osg::Vec3d
LocalizedNode::getPosition( const SpatialReference* srs ) const
{
    osg::Vec3d output;
    osg::ref_ptr<MapNode> mapNode = _mapNode.get();
    if ( mapNode.valid() )
    {
        mapNode->getMap()->worldPointToMapPoint( _xform->getMatrix().getTrans(), output );
        if ( srs )
        {
            mapNode->getMap()->getProfile()->getSRS()->transform( output, srs, output );
        }
    }
    return output;
}

void
LocalizedNode::setHorizonCulling( bool value )
{
    if ( _horizonCulling != value )
    {
        _horizonCulling = value;

        if ( _horizonCulling )
        {
            osg::ref_ptr<MapNode> mapNode = _mapNode.get();
            if ( mapNode.valid() )
            {
                osg::Vec3d world = _xform->getMatrix().getTrans();
                const osg::EllipsoidModel* em = mapNode->getMap()->getProfile()->getSRS()->getEllipsoid();
                _xform->setCullCallback( new CullNodeByHorizon(world, em) );
            }
        }
        else
        {
            _xform->removeCullCallback( _xform->getCullCallback() );
        }
    }
}
