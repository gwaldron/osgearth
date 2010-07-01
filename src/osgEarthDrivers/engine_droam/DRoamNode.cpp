/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/EarthFile>
#include <osgEarth/Cube>
#include <osgEarthDrivers/tms/TMSOptions>

using namespace osgEarth::Drivers;


DRoamNode::DRoamNode( Map* map ) :
_map( map )
{
    this->setCullCallback( new MyCullCallback );
    this->setUpdateCallback( new MyUpdateCallback );

    // TODO: provide the ellipsoid in the ctor (like with MapNode->Map)
    this->setCoordinateSystem( "EPSG:4326" );
    this->setFormat( "WKT" );
    this->setEllipsoidModel( new osg::EllipsoidModel );

    const Profile* profile = new osgEarth::UnifiedCubeProfile();
    //Profile::createCube2( SpatialReference::create( "epsg:4326" ) );

    _manifold = new CubeManifold( profile );
    _mesh = new MeshManager( _manifold.get(), _map.get() );

    _mesh->_maxActiveLevel = MAX_ACTIVE_LEVEL;

    this->setInitialBound( _manifold->initialBound() );

#ifdef USE_AMR
    this->addChild( _mesh->_amrGeode.get() );
#else // USE_AMR
    this->addChild( _mesh->_geode.get() );
#endif

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
    _mesh->_activeDrawables.clear();

#ifdef USE_AMR
    _mesh->_amrDrawList.clear();
#endif // USE_AMR

    _manifold->cull( static_cast<osgUtil::CullVisitor*>( nv ) );

    // I know is not strictly kosher to modify the scene graph from the CULL traversal. But
    // we need frame-coherence, and both the Geode and all Geometry's are marked with DYNAMIC
    // data variance .. so hopefully this is safe.
    _mesh->_geode->removeDrawables( 0, _mesh->_geode->getNumDrawables() );
    for( osg::Geode::DrawableList::iterator i = _mesh->_activeDrawables.begin(); i != _mesh->_activeDrawables.end(); ++i )
        _mesh->_geode->addDrawable( i->get() );
    _mesh->_geode->dirtyBound();

#ifdef USE_AMR
    _mesh->_amrGeom->setDrawList( _mesh->_amrDrawList );
    _mesh->_amrGeode->dirtyBound();
#endif // USE_AMR
}

void 
DRoamNode::update( osg::NodeVisitor* nv )
{
    _mesh->update();
}
