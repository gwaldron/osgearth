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
#include "GeodeticManifold"
#include <osgEarth/Registry>

#define LC "[osgEarth::GeodeticManifold] "

GeodeticManifold::GeodeticManifold()
{
    _profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    _ellipsoid = _profile->getSRS()->getGeographicSRS()->getEllipsoid();
}

void
GeodeticManifold::initialize( MeshManager* mesh )
{
    mesh->_minGeomLevel = 1;
    mesh->_minActiveLevel = 3;

    // Construct the "vertex diamonds".
    _vd[0] = new Diamond(mesh, TileKey(), 0, "vd0"); // north pole 1
    _vd[0]->setCoord( -90, 90, 0 );

    _vd[1] = new Diamond(mesh, TileKey(), 0, "vd1"); // north pole 2
    _vd[1]->setCoord( 90, 90, 0 );

    _vd[2] = new Diamond(mesh, TileKey(), 0, "vd2"); // south pole 1
    _vd[2]->setCoord( -90, -90, 0 );

    _vd[3] = new Diamond(mesh, TileKey(), 0, "vd3"); // south pole 2
    _vd[3]->setCoord( 90, -90, 0 );

    _vd[4] = new Diamond(mesh, TileKey(), 0, "vd4");
    _vd[4]->setCoord( -180, 0, 0 );

    _vd[5] = new Diamond(mesh, TileKey(), 0, "vd5");
    _vd[5]->setCoord( 0, 0, 0 );

    // The 4 "face diamonds":
    _fd[0] = new Diamond(mesh, TileKey(), 0, "fd0");
    _fd[0]->setCoord( -90, 0, 0 );
    _fd[0]->_a[GDPARENT] = _vd[0].get();
    _fd[0]->_a[QUADTREE] = _vd[2].get();
    _fd[0]->_a[PARENT_L] = _vd[4].get();
    _fd[0]->_a[PARENT_R] = _vd[5].get();

    _fd[1] = new Diamond(mesh, TileKey(), 0, "fd1");
    _fd[1]->setCoord( 90, 0, 0 );
    _fd[1]->_a[GDPARENT] = _vd[3].get();
    _fd[1]->_a[QUADTREE] = _vd[1].get();
    _fd[1]->_a[PARENT_L] = _vd[4].get();
    _fd[1]->_a[PARENT_R] = _vd[5].get();

    _fd[2] = new Diamond(mesh, TileKey(), 0, "fd2"); // virtual north pole diamond
    _fd[2]->setCoord( 0, 90, 0 );
    _fd[2]->_a[GDPARENT] = _vd[0].get();
    _fd[2]->_a[QUADTREE] = _vd[1].get();
    _fd[2]->_a[PARENT_L] = _vd[5].get();
    _fd[2]->_a[PARENT_R] = _vd[4].get();

    _fd[3] = new Diamond(mesh, TileKey(), 0, "fd3"); // virtual south pole diamond
    _fd[3]->setCoord( 0, -90, 0 );
    _fd[3]->_a[GDPARENT] = _vd[3].get();
    _fd[3]->_a[QUADTREE] = _vd[2].get();
    _fd[3]->_a[PARENT_L] = _vd[5].get();
    _fd[3]->_a[PARENT_R] = _vd[4].get();

    // the 8 "edge diamonds" (first with geometry)
    _ed[0] = new Diamond(mesh, TileKey(1,0,0,_profile.get()), 1, "ed0");
    _ed[0]->setCoord( -135, 45, 0 );
    _ed[0]->_a[GDPARENT] = _vd[0].get();
    _ed[0]->_a[QUADTREE] = _vd[4].get();
    _ed[0]->_a[PARENT_L] = _fd[2].get();
    _ed[0]->_a[PARENT_R] = _fd[0].get();
    _ed[0]->_orientation = 0;

    _ed[1] = new Diamond(mesh, TileKey(1,1,0,_profile.get()), 1, "ed1");
    _ed[1]->setCoord( -45, 45, 0 );
    _ed[1]->_a[GDPARENT] = _vd[0].get();
    _ed[1]->_a[QUADTREE] = _vd[5].get();
    _ed[1]->_a[PARENT_L] = _fd[0].get();
    _ed[1]->_a[PARENT_R] = _fd[2].get();
    _ed[1]->_orientation = 2;

    _ed[2] = new Diamond(mesh, TileKey(1,0,1,_profile.get()), 1, "ed2");
    _ed[2]->setCoord( -135, -45, 0 );
    _ed[2]->_a[GDPARENT] = _vd[2].get();
    _ed[2]->_a[QUADTREE] = _vd[4].get();
    _ed[2]->_a[PARENT_L] = _fd[0].get();
    _ed[2]->_a[PARENT_R] = _fd[3].get();
    _ed[2]->_orientation = 6;

    _ed[3] = new Diamond(mesh, TileKey(1,1,1,_profile.get()), 1, "ed3");
    _ed[3]->setCoord( -45, -45, 0 );
    _ed[3]->_a[GDPARENT] = _vd[2].get();
    _ed[3]->_a[QUADTREE] = _vd[5].get();
    _ed[3]->_a[PARENT_L] = _fd[3].get();
    _ed[3]->_a[PARENT_R] = _fd[0].get();
    _ed[3]->_orientation = 4;

    _ed[4] = new Diamond(mesh, TileKey(1,2,0,_profile.get()), 1, "ed4");
    _ed[4]->setCoord( 45, 45, 0 );
    _ed[4]->_a[GDPARENT] = _vd[1].get();
    _ed[4]->_a[QUADTREE] = _vd[5].get();
    _ed[4]->_a[PARENT_L] = _fd[2].get();
    _ed[4]->_a[PARENT_R] = _fd[1].get();
    _ed[4]->_orientation = 0;
    
    _ed[5] = new Diamond(mesh, TileKey(1,3,0,_profile.get()), 1, "ed5");
    _ed[5]->setCoord( 135, 45, 0 );
    _ed[5]->_a[GDPARENT] = _vd[1].get();
    _ed[5]->_a[QUADTREE] = _vd[4].get();
    _ed[5]->_a[PARENT_L] = _fd[1].get();
    _ed[5]->_a[PARENT_R] = _fd[2].get();
    _ed[5]->_orientation = 2;

    _ed[6] = new Diamond(mesh, TileKey(1,2,1,_profile.get()), 1, "ed6");
    _ed[6]->setCoord( 45, -45, 0 );
    _ed[6]->_a[GDPARENT] = _vd[3].get();
    _ed[6]->_a[QUADTREE] = _vd[5].get();
    _ed[6]->_a[PARENT_L] = _fd[1].get();
    _ed[6]->_a[PARENT_R] = _fd[3].get();
    _ed[6]->_orientation = 6;

    _ed[7] = new Diamond(mesh, TileKey(1,3,1,_profile.get()), 1, "ed7");
    _ed[7]->setCoord( 135, -45, 0 );
    _ed[7]->_a[GDPARENT] = _vd[3].get();
    _ed[7]->_a[QUADTREE] = _vd[4].get();
    _ed[7]->_a[PARENT_L] = _fd[3].get();
    _ed[7]->_a[PARENT_R] = _fd[1].get();
    _ed[7]->_orientation = 7;

    // set child pointers:
    _fd[0]->setChild( 0, _ed[3].get() );
    _fd[0]->setChild( 1, _ed[1].get() );
    _fd[0]->setChild( 2, _ed[0].get() );
    _fd[0]->setChild( 3, _ed[2].get() );

    _fd[1]->setChild( 0, _ed[4].get() );
    _fd[1]->setChild( 1, _ed[6].get() );
    _fd[1]->setChild( 2, _ed[7].get() );
    _fd[1]->setChild( 3, _ed[5].get() );

    _fd[2]->setChild( 0, _ed[5].get() );
    _fd[2]->setChild( 1, _ed[0].get() );
    _fd[2]->setChild( 2, _ed[1].get() );
    _fd[2]->setChild( 3, _ed[4].get() );

    _fd[3]->setChild( 0, _ed[2].get() );
    _fd[3]->setChild( 1, _ed[7].get() );
    _fd[3]->setChild( 2, _ed[6].get() );
    _fd[3]->setChild( 3, _ed[3].get() );
    

    // seed the bouding spheres of the manifold diamonds:
    for( unsigned short f=0; f<4; ++f )
        _fd[f]->activate();

    for( unsigned short e=0; e<8; ++e )
        _ed[e]->activate();

    // generate Level 3 (the first renderable quadtree decendants).
    seed( 3 );

    // hopefully, that's it!
}

//osg::Vec3d
//GeodeticManifold::project( const osg::Vec3d& coord ) const
//{
//    osg::Vec3d out;
//
//    _ellipsoid->convertLatLongHeightToXYZ( 
//        osg::DegreesToRadians( coord.y() ),
//        osg::DegreesToRadians( coord.x() ), 0, 
//        out.x(), out.y(), out.z() );
//
//    return out;
//}

osg::Vec3d
GeodeticManifold::midpoint( const osg::Vec3d& p0, const osg::Vec3d& p1 ) const
{
    //TODO account for date line crossing
    return (p0+p1)*0.5;
}

//osg::Vec3d
//GeodeticManifold::normal( const osg::Vec3d& vert ) const
//{
//    //TODO: this is spherical. adjust for ellipsoid if necessary.
//    osg::Vec3d n = vert;
//    n.normalize();
//    return n;
//}

MeshNode
GeodeticManifold::createNode( const osg::Vec3d& manCoord ) const
{
    MeshNode node;

    node._manifoldCoord = manCoord;

    node._geodeticCoord.set( 
        osg::DegreesToRadians(manCoord.x()), osg::DegreesToRadians(manCoord.y()), manCoord.z() );

    osg::Vec3d temp;
    _ellipsoid->convertLatLongHeightToXYZ(
        node._geodeticCoord.y(), node._geodeticCoord.x(), node._geodeticCoord.z(),
        temp.x(), temp.y(), temp.z() );

    node._vertex = temp;

    node._normal = _ellipsoid->computeLocalUpVector(
        temp.x(), temp.y(), temp.z() );

    return node;
}

osg::BoundingSphere
GeodeticManifold::initialBound() const
{
    return osg::BoundingSphere( osg::Vec3d(0,0,0), _ellipsoid->getRadiusEquator() * 1.2 );
}

void
GeodeticManifold::seed( Level maxLevel )
{
    for( unsigned short e=0; e<8; ++e )
    {
        _ed[e]->seed( maxLevel );
    }
}

void
GeodeticManifold::cull( osgUtil::CullVisitor* cv )
{
    for( unsigned short e=0; e<8; ++e )
    {
        _ed[e]->cull( cv );
    }
}
