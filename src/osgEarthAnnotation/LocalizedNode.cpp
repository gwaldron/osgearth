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


LocalizedNode::LocalizedNode(const SpatialReference* mapSRS,
                             const osg::Vec3d&       pos,
                             bool                    is2D ) :
_mapSRS        ( mapSRS ),
_horizonCulling( false ),
_autoTransform ( is2D )
{
    if ( _autoTransform )
    {
        osg::AutoTransform* at = new osg::AutoTransform();
        at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
        at->setAutoScaleToScreen( true );
        at->setCullingActive( false ); // just for the first pass
        _xform = at;
    }
    else
    {
        _xform = new osg::MatrixTransform();
    }
    _xform->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );

    if ( mapSRS )
    {
        setHorizonCulling( true );
        setPosition( pos );
    }
}

void
LocalizedNode::traverse( osg::NodeVisitor& nv )
{
    if ( _autoTransform && nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
       _xform->setCullingActive( true );
    }
    osg::Group::traverse( nv );
}

bool
LocalizedNode::setPosition( const osg::Vec3d& pos )
{
    return setPosition( pos, 0L );
}

bool
LocalizedNode::setPosition( const osg::Vec3d& pos, const SpatialReference* posSRS )
{
    // first transform the point to the map's SRS:
    osg::Vec3d mapPos = pos;
    if ( posSRS && !posSRS->transform(pos, _mapSRS.get(), mapPos) )
        return false;

    // update the transform:
    osg::Matrixd local2world;
    _mapSRS->createLocal2World( mapPos, local2world );

    if ( _autoTransform )
        static_cast<osg::AutoTransform*>(_xform.get())->setPosition( local2world.getTrans() );
    else
        static_cast<osg::MatrixTransform*>(_xform.get())->setMatrix( local2world );

    // and update the culler:
    CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(_xform->getCullCallback());
    if ( culler )
        culler->_world = local2world.getTrans();

    return true;
}

#if 0
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

        if ( _autoTransform )
            static_cast<osg::AutoTransform*>(_xform.get())->setPosition( local2world.getTrans() );
        else
            static_cast<osg::MatrixTransform*>(_xform.get())->setMatrix( local2world );

        // and update the culler:
        CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(_xform->getCullCallback());
        if ( culler )
            culler->_world = local2world.getTrans();

        return true;
    }   
    else return false;
}
#endif

osg::Vec3d
LocalizedNode::getPosition() const
{
    return getPosition( 0L );
}

osg::Vec3d
LocalizedNode::getPosition( const SpatialReference* srs ) const
{
    osg::Vec3d world;
    if ( _autoTransform )
        world = static_cast<osg::AutoTransform*>(_xform.get())->getPosition();
    else
        world = static_cast<osg::MatrixTransform*>(_xform.get())->getMatrix().getTrans();

    osg::Vec3d output = world;
    if ( _mapSRS->isGeographic() )
        _mapSRS->transformFromECEF( world, output );

    if ( srs )
        _mapSRS->transform( output, srs, output );

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
            osg::Vec3d world;
            if ( _autoTransform )
                world = static_cast<osg::AutoTransform*>(_xform.get())->getPosition();
            else
                world = static_cast<osg::MatrixTransform*>(_xform.get())->getMatrix().getTrans();

            _xform->setCullCallback( new CullNodeByHorizon(world, _mapSRS->getEllipsoid()) );
        }
        else
        {
            _xform->removeCullCallback( _xform->getCullCallback() );
        }
    }
}
