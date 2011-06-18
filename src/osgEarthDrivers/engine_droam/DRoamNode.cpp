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
#include "DRoamNode"
#include "CubeManifold"
#include "GeodeticManifold"
#include <osgEarth/Cube>
#include <osgEarthDrivers/tms/TMSOptions>

using namespace osgEarth::Drivers;

#undef USE_GEODETIC_MANIFOLD

DRoamNode::DRoamNode( Map* map ) :
_map( map )
{
    this->setCullCallback( new MyCullCallback );
    this->setUpdateCallback( new MyUpdateCallback );

    // TODO: provide the ellipsoid in the ctor (like with MapNode->Map)
    this->setCoordinateSystem( "EPSG:4326" );
    this->setFormat( "WKT" );
    this->setEllipsoidModel( new osg::EllipsoidModel );

#ifdef USE_GEODETIC_MANIFOLD
    _manifold = new GeodeticManifold();
#else
    _manifold = new CubeManifold();
#endif
    _mesh = new MeshManager( _manifold.get(), _map.get() );

    _mesh->_maxActiveLevel = MAX_ACTIVE_LEVEL;

    this->setInitialBound( _manifold->initialBound() );

    this->addChild( _mesh->_amrGeode.get() );

    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
}

DRoamNode::DRoamNode( const DRoamNode& rhs, const osg::CopyOp& op ) :
osg::CoordinateSystemNode( rhs, op ),
_manifold( rhs._manifold.get() )
{
    //nop
}

void
DRoamNode::cull( osg::NodeVisitor* nv )
{
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( nv );

    // reset the drawable set list. we will populate it by traversing the
    // manifold and collecting drawables that are visible. The new drawables
    // list does not actually take effect until the next frame. So yes, the culling is
    // one frame out of sync with the draw. We should probably think of a way to fix that.
    //_mesh->_activeDrawables.clear();

    _mesh->_amrDrawList.clear();

    _manifold->cull( static_cast<osgUtil::CullVisitor*>( nv ) );

    // I know is not strictly kosher to modify the scene graph from the CULL traversal. But
    // we need frame-coherence, and both the Geode and all Geometry's are marked with DYNAMIC
    // data variance .. so hopefully this is safe.
    _mesh->_amrGeom->setDrawList( _mesh->_amrDrawList );
    _mesh->_amrGeode->dirtyBound();
}

void 
DRoamNode::update( osg::NodeVisitor* nv )
{
    _mesh->update();
}
