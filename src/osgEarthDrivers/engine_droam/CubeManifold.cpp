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
#include "CubeManifold"
#include <osgEarth/Cube>

#define NEG_Y 0
#define POS_Y 1
#define NEG_X 2
#define POS_X 3
#define NEG_Z 4
#define POS_Z 5

#define LC "[CubeManifold] "

CubeManifold::CubeManifold()
{
    _profile = new osgEarth::UnifiedCubeProfile();
    _ellipsoid = _profile->getSRS()->getGeographicSRS()->getEllipsoid();
}

void
CubeManifold::initialize( MeshManager* mesh )
{
    // diamonds at or below level 3 are static and cannot be removed.
    // Level 1 is the first Quadtree Ancestor. Level 3 contains the first Quadtree
    // decendants (the decendants of the Level 1 quadtrees).
    mesh->_minGeomLevel = 1;
    mesh->_minActiveLevel = 3;

    // Construct the eight root "vertex diamonds". These are used only for their
    // vertex positions (as grandparents of the root edge diamonds). They have
    // no children or ancestors of their own.
    _vd[0] = new Diamond(mesh, TileKey(), 0, "vd0"); // X, -Y, Z
    _vd[0]->setCoord( p(1, -1, 1) );
    _vd[0]->_childValence = 3;

    _vd[1] = new Diamond(mesh, TileKey(), 0, "vd1"); // -X, Y, Z
    _vd[1]->setCoord( p(-1, 1, 1) );
    _vd[0]->_childValence = 3;

    _vd[2] = new Diamond(mesh, TileKey(), 0, "vd2"); // -X, -Y, -Z
    _vd[2]->setCoord( p(-1, -1, -1) );
    _vd[0]->_childValence = 3;

    _vd[3] = new Diamond(mesh, TileKey(), 0, "vd3"); // X, Y, -Z
    _vd[3]->setCoord( p(1, 1, -1) );
    _vd[0]->_childValence = 3;

    _vd[4] = new Diamond(mesh, TileKey(), 0, "vd4"); // X, Y, Z
    _vd[4]->setCoord( p(1, 1, 1) );
    _vd[0]->_childValence = 3;

    _vd[5] = new Diamond(mesh, TileKey(), 0, "vd5"); // -X, -Y, Z
    _vd[5]->setCoord( p(-1, -1, 1) );
    _vd[0]->_childValence = 3;

    _vd[6] = new Diamond(mesh, TileKey(), 0, "vd6"); // X, -Y, -Z
    _vd[6]->setCoord( p(1, -1, -1) );
    _vd[0]->_childValence = 3;

    _vd[7] = new Diamond(mesh, TileKey(), 0, "vd7"); // -X, Y, -Z
    _vd[7]->setCoord( p(-1, 1, -1) );
    _vd[0]->_childValence = 3;

    // Construct the 6 face diamonds. Exactly 2 faces parent each edge diamond (constructed
    // later). The vertex position of each face diamond resides as the center of that face
    // on the cube. The faces are carefully oriented so that child diamonds are anchored
    // (i.e. have their QUADTREE ancestor) at the PARENT_L and PARENT_R positions. This is
    // critical for proper subdivision and orientation propagation.

    // The first 3 share vd0 as a common PARENT_L ancestor:

    _fd[NEG_Y] = new Diamond(mesh, TileKey(0,1,0,_profile.get()), 1, "fd -y"); // -Y face (-90=>0 long)
    _fd[NEG_Y]->setCoord( p(0, -1, 0) );
    _fd[NEG_Y]->_a[PARENT_R] = _vd[2];
    _fd[NEG_Y]->_a[PARENT_L] = _vd[0];
    _fd[NEG_Y]->_a[QUADTREE] = _vd[5];
    _fd[NEG_Y]->_a[GDPARENT] = _vd[6];
    _fd[NEG_Y]->_orientation = 6;

    _fd[POS_X] = new Diamond(mesh, TileKey(0,2,0,_profile.get()), 1, "fd +x"); // +X face (0=>90 long)
    _fd[POS_X]->setCoord( p(1, 0, 0) );
    _fd[POS_X]->_a[PARENT_R] = _vd[3];
    _fd[POS_X]->_a[PARENT_L] = _vd[0];
    _fd[POS_X]->_a[QUADTREE] = _vd[6];
    _fd[POS_X]->_a[GDPARENT] = _vd[4];
    _fd[POS_X]->_orientation = 0;

    _fd[POS_Z] = new Diamond(mesh, TileKey(0,4,0,_profile.get()), 1, "fd +z"); // +Z face (north polar)
    _fd[POS_Z]->setCoord( p(0, 0, 1) );
    _fd[POS_Z]->_a[PARENT_R] = _vd[1];
    _fd[POS_Z]->_a[PARENT_L] = _vd[0];
    _fd[POS_Z]->_a[QUADTREE] = _vd[4];
    _fd[POS_Z]->_a[GDPARENT] = _vd[5];
    _fd[POS_Z]->_orientation = 6; // 4

    // The next 3 share vd7 as a common QUADTREE ancestor:

    _fd[POS_Y] = new Diamond(mesh, TileKey(0,3,0,_profile.get()), 1, "fd +y"); // +Y face (90=>180 long)
    _fd[POS_Y]->setCoord( p(0, 1, 0) );
    _fd[POS_Y]->_a[PARENT_R] = _vd[1];
    _fd[POS_Y]->_a[PARENT_L] = _vd[3];
    _fd[POS_Y]->_a[QUADTREE] = _vd[7];
    _fd[POS_Y]->_a[GDPARENT] = _vd[4];
    _fd[POS_Y]->_orientation = 2;

    _fd[NEG_X] = new Diamond(mesh, TileKey(0,0,0,_profile.get()), 1, "fd -x"); // -X face (-180=>-90 long)
    _fd[NEG_X]->setCoord( p(-1, 0, 0) );
    _fd[NEG_X]->_a[PARENT_R] = _vd[2];
    _fd[NEG_X]->_a[PARENT_L] = _vd[1];
    _fd[NEG_X]->_a[QUADTREE] = _vd[7];
    _fd[NEG_X]->_a[GDPARENT] = _vd[5];
    _fd[NEG_X]->_orientation = 0;

    _fd[NEG_Z] = new Diamond(mesh, TileKey(0,5,0,_profile.get()), 1, "fd -z"); // -Z face (south polar)
    _fd[NEG_Z]->setCoord( p(0, 0, -1) );
    _fd[NEG_Z]->_a[PARENT_R] = _vd[3];
    _fd[NEG_Z]->_a[PARENT_L] = _vd[2];
    _fd[NEG_Z]->_a[QUADTREE] = _vd[7];
    _fd[NEG_Z]->_a[GDPARENT] = _vd[6];
    _fd[NEG_Z]->_orientation = 0;

    // Next, construct the 12 root edge diamonds. These are the roots of the global
    // bintree, and are the first elements that actually get drawn. Each edge diamond
    // represents an edge of the cube, with its vertex position at the midpoint of that
    // edge. Each edge diamond has two face diamonds as parents, and one vertex diamond
    // as its "quadtree" ancestor. That adds up to the 4 verts of the diamond.

    // GROUP OF THREE under the vd[0] "quadtree":
    _ed[0] = new Diamond(mesh, TileKey(), 2, "ed0");
    _ed[0]->setCoord( p(0, -1, 1) );
    _ed[0]->_a[PARENT_R] = _fd[POS_Z];
    _ed[0]->_a[PARENT_L] = _fd[NEG_Y];
    _ed[0]->_a[QUADTREE] = _vd[0];
    _ed[0]->_a[GDPARENT] = _vd[5];
//    _ed[0]->_color = RED;

    _ed[1] = new Diamond(mesh, TileKey(), 2, "ed1");
    _ed[1]->setCoord( p(1, -1, 0 ) );
    _ed[1]->_a[PARENT_R] = _fd[NEG_Y];
    _ed[1]->_a[PARENT_L] = _fd[POS_X];
    _ed[1]->_a[QUADTREE] = _vd[0];
    _ed[1]->_a[GDPARENT] = _vd[6];
    //_ed[1]->_color = RED;

    _ed[2] = new Diamond(mesh, TileKey(), 2, "ed2");
    _ed[2]->setCoord( p(1, 0, 1) );
    _ed[2]->_a[PARENT_R] = _fd[POS_X];
    _ed[2]->_a[PARENT_L] = _fd[POS_Z];
    _ed[2]->_a[QUADTREE] = _vd[0];
    _ed[2]->_a[GDPARENT] = _vd[4];
    //_ed[2]->_color = RED;

    // GROUP OF THREE under the vd[1] "quadtree":
    _ed[3] = new Diamond(mesh, TileKey(), 2, "ed3");
    _ed[3]->setCoord( p(0, 1, 1) );
    _ed[3]->_a[PARENT_R] = _fd[POS_Z];
    _ed[3]->_a[PARENT_L] = _fd[POS_Y];
    _ed[3]->_a[QUADTREE] = _vd[1];
    _ed[3]->_a[GDPARENT] = _vd[4];
    //_ed[3]->_color = GREEN;

    _ed[4] = new Diamond(mesh, TileKey(), 2, "ed4");
    _ed[4]->setCoord( p(-1, 1, 0) );
    _ed[4]->_a[PARENT_R] = _fd[POS_Y];
    _ed[4]->_a[PARENT_L] = _fd[NEG_X];
    _ed[4]->_a[QUADTREE] = _vd[1];
    _ed[4]->_a[GDPARENT] = _vd[7];
    //_ed[4]->_color = GREEN;

    _ed[5] = new Diamond(mesh, TileKey(), 2, "ed5");
    _ed[5]->setCoord( p(-1, 0, 1) );
    _ed[5]->_a[PARENT_R] = _fd[NEG_X];
    _ed[5]->_a[PARENT_L] = _fd[POS_Z];
    _ed[5]->_a[QUADTREE] = _vd[1];
    _ed[5]->_a[GDPARENT] = _vd[5];
    //_ed[5]->_color = GREEN;

    // GROUP OF THREE under the vd[2] "quadtree":
    _ed[6] = new Diamond(mesh, TileKey(), 2, "ed6");
    _ed[6]->setCoord( p(-1, -1, 0) );
    _ed[6]->_a[PARENT_R] = _fd[NEG_Y];
    _ed[6]->_a[PARENT_L] = _fd[NEG_X];
    _ed[6]->_a[QUADTREE] = _vd[2];
    _ed[6]->_a[GDPARENT] = _vd[5];
    //_ed[6]->_color = BLUE;

    _ed[7] = new Diamond(mesh, TileKey(), 2, "ed7");
    _ed[7]->setCoord( p(-1, 0, -1) );
    _ed[7]->_a[PARENT_R] = _fd[NEG_X];
    _ed[7]->_a[PARENT_L] = _fd[NEG_Z];
    _ed[7]->_a[QUADTREE] = _vd[2];
    _ed[7]->_a[GDPARENT] = _vd[7];
    //_ed[7]->_color = BLUE;

    _ed[8] = new Diamond(mesh, TileKey(), 2, "ed8");
    _ed[8]->setCoord( p(0, -1, -1) );
    _ed[8]->_a[PARENT_R] = _fd[NEG_Z];
    _ed[8]->_a[PARENT_L] = _fd[NEG_Y];
    _ed[8]->_a[QUADTREE] = _vd[2];
    _ed[8]->_a[GDPARENT] = _vd[6];
    //_ed[8]->_color = BLUE;

    // GROUP OF THREE under the vd[3] "quadtree":
    _ed[9] = new Diamond(mesh, TileKey(), 2, "ed9");
    _ed[9]->setCoord( p(1, 1, 0) );
    _ed[9]->_a[PARENT_R] = _fd[POS_Y];
    _ed[9]->_a[PARENT_L] = _fd[POS_X];
    _ed[9]->_a[QUADTREE] = _vd[3];
    _ed[9]->_a[GDPARENT] = _vd[4];
    //_ed[9]->_color = YELLOW;

    _ed[10] = new Diamond(mesh, TileKey(), 2, "ed10");
    _ed[10]->setCoord( p(1, 0, -1) );
    _ed[10]->_a[PARENT_R] = _fd[POS_X];
    _ed[10]->_a[PARENT_L] = _fd[NEG_Z];
    _ed[10]->_a[QUADTREE] = _vd[3];
    _ed[10]->_a[GDPARENT] = _vd[6];
    //_ed[10]->_color = YELLOW;

    _ed[11] = new Diamond(mesh, TileKey(), 2, "ed11");
    _ed[11]->setCoord( p(0, 1, -1) );
    _ed[11]->_a[PARENT_R] = _fd[NEG_Z];
    _ed[11]->_a[PARENT_L] = _fd[POS_Y];
    _ed[11]->_a[QUADTREE] = _vd[3];
    _ed[11]->_a[GDPARENT] = _vd[7];
    //_ed[11]->_color = YELLOW;

    // NEXT, set the pointers from each face diamond to its 4 children. We can double-
    // check these by looking at the ancestor assignments above. They should match up.
    // At the same time, we assign the back pointers from each of the edge diamonds to
    // their 2 parents.

    _fd[POS_X]->setChild( 0, _ed[10].get() );
    _fd[POS_X]->setChild( 1, _ed[9].get() );
    _fd[POS_X]->setChild( 2, _ed[2].get() );
    _fd[POS_X]->setChild( 3, _ed[1].get() );

    _fd[NEG_Y]->setChild( 0, _ed[6].get() );
    _fd[NEG_Y]->setChild( 1, _ed[8].get() );
    _fd[NEG_Y]->setChild( 2, _ed[1].get() );
    _fd[NEG_Y]->setChild( 3, _ed[0].get() );

    _fd[POS_Z]->setChild( 0, _ed[3].get() );
    _fd[POS_Z]->setChild( 1, _ed[5].get() );
    _fd[POS_Z]->setChild( 2, _ed[0].get() );
    _fd[POS_Z]->setChild( 3, _ed[2].get() );

    _fd[POS_Y]->setChild( 0, _ed[4].get() );
    _fd[POS_Y]->setChild( 1, _ed[3].get() );
    _fd[POS_Y]->setChild( 2, _ed[9].get() );
    _fd[POS_Y]->setChild( 3, _ed[11].get() );

    _fd[NEG_X]->setChild( 0, _ed[7].get() );
    _fd[NEG_X]->setChild( 1, _ed[6].get() );
    _fd[NEG_X]->setChild( 2, _ed[5].get() );
    _fd[NEG_X]->setChild( 3, _ed[4].get() );

    _fd[NEG_Z]->setChild( 0, _ed[11].get() );
    _fd[NEG_Z]->setChild( 1, _ed[10].get() );
    _fd[NEG_Z]->setChild( 2, _ed[8].get() );
    _fd[NEG_Z]->setChild( 3, _ed[7].get() );

    // seed the bouding spheres of the manifold diamonds:
    for( unsigned short f=0; f<6; ++f )
        _fd[f]->activate();

    for( unsigned short e=0; e<12; ++e )
        _ed[e]->activate();

    // generate Level 3 (the first renderable quadtree decendants).
    seed( 3 );

    // hopefully, that's it!
}


osg::Vec3d
CubeManifold::p( double x, double y, double z ) const
{
    osg::Vec3d v( x, y, z );
    return v;
}

static bool
toLonLatRad( const osg::Vec3d& coord, osg::Vec3d& out_lonLat )
{
    double lat, lon;

    // normalize all coordinates into the [0..1] range:
    double x = (1.0+coord.x())*0.5;
    double y = (1.0+coord.y())*0.5;
    double z = (1.0+coord.z())*0.5;

    // now convert into lat/long; this is different for each face:

    if ( coord.x() == 1.0 ) // positive X ( 0 <= lon <= 90, -45 <= lat <= 45 )
    {
        if ( !osgEarth::CubeUtils::faceCoordsToLatLon( y, z, 2, lat, lon ) )
            OE_WARN << LC << "+X: fc2ll failed" << std::endl;
    }
    else if ( coord.x() == -1.0 ) // negative X ( -180 <= lon <= -90, -45 <= lat <= 45 )
    {
        if ( !osgEarth::CubeUtils::faceCoordsToLatLon( 1.0-y, z, 0, lat, lon ) )
            OE_WARN << LC << "-X: fc2ll failed" << std::endl;
    }
    else if ( coord.y() == 1.0 ) // positive Y ( 90 <= lon <= 180, -45 <= lat <= 45 )
    {
        if ( !osgEarth::CubeUtils::faceCoordsToLatLon( 1.0-x, z, 3, lat, lon ) )
            OE_WARN << LC << "+Y: fc2ll failed" << std::endl;
    }
    else if ( coord.y() == -1.0 ) // negative Y ( -90 <= lon <= 0, -45 <= lat <= 45 )
    {
        if ( !osgEarth::CubeUtils::faceCoordsToLatLon( x, z, 1, lat, lon ) )
            OE_WARN << LC << "-Y: fc2ll failed" << std::endl;
    }
    else if ( coord.z() == 1.0 ) // positive Z ( -180 <= lon < 180, 45 <= lat <= 90 )
    {
        if ( !osgEarth::CubeUtils::faceCoordsToLatLon( 1.0-y, x, 4, lat, lon ) )
            OE_WARN << LC << "+Z: fc2ll failed" << std::endl;
    }
    else //if ( coord.z() == -1.0 ) // negative Z ( -180 <= lon < 180, -90 <= lat <= -45 )
    {
        if ( !osgEarth::CubeUtils::faceCoordsToLatLon( x, 1.0-y, 5, lat, lon ) )
            OE_WARN << LC << "-Z: fc2ll failed" << std::endl;
    }

    out_lonLat.x() = osg::DegreesToRadians( lon );
    out_lonLat.y() = osg::DegreesToRadians( lat );
    out_lonLat.z() = 0.0;

    return true;
}


osg::Vec3d
CubeManifold::midpoint( const osg::Vec3d& p0, const osg::Vec3d& p1 ) const
{
    return (p0+p1)*0.5;
}

MeshNode
CubeManifold::createNode( const osg::Vec3d& manCoord ) const
{
    MeshNode node;

    node._manifoldCoord = manCoord;
    
    toLonLatRad( manCoord, node._geodeticCoord ); // our manifold coords are already geodetic

    osg::Matrixd local2world;
    _ellipsoid->computeLocalToWorldTransformFromLatLongHeight(
        node._geodeticCoord.y(), node._geodeticCoord.x(), node._geodeticCoord.z(),
        local2world );

    node._vertex = local2world.getTrans();

    node._normal = node._vertex;
    node._normal.normalize();
    //node._normal = _ellipsoid->computeLocalUpVector(
    //    node._vertex.x(), node._vertex.y(), node._vertex.z() );

    node._geodeticRot = local2world.getRotate();
    //OE_INFO << LC
    //    << "ROT: x=" << node._geodeticRot.x() << ", y=" << node._geodeticRot.y()
    //    << ", z=" << node._geodeticRot.z() << ", w=" << node._geodeticRot.w()
    //    << std::endl;

    return node;
}

osg::BoundingSphere
CubeManifold::initialBound() const
{
    return osg::BoundingSphere( osg::Vec3d(0,0,0), _ellipsoid->getRadiusEquator() * 1.2 );
    //return osg::BoundingSphere( osg::Vec3d(0,0,0), 2.0 );
}

void
CubeManifold::seed( Level maxLevel )
{
    for( unsigned short e=0; e<12; ++e )
    {
        _ed[e]->seed( maxLevel );
    }
}

void
CubeManifold::cull( osgUtil::CullVisitor* cv )
{
    for( unsigned short e=0; e<12; ++e )
    {
        _ed[e]->cull( cv );
    }
}
