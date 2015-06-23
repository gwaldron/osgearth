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
#include "SurfaceNode"
#include "GeometryPool"
#include "TileDrawable"

#include <osgEarth/TileKey>

#include <osg/CullStack>
#include <osg/Geode>

#include <osg/Geometry>
#include <osgText/Text>

#include <numeric>
using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SurfaceNode] "

//..............................................................

namespace
{
    osg::Geode* makeBBox(const osg::BoundingBox& bbox)
    {        
        osg::Geode* geode = new osg::Geode();
        std::string sizeStr = "(empty)";
        float zpos = 0.0f;

        if ( bbox.valid() )
        {
            osg::Geometry* geom = new osg::Geometry();
            geom->setName("bbox");
        
            osg::Vec3Array* v = new osg::Vec3Array();
            for(int i=0; i<8; ++i)
                v->push_back(bbox.corner(i));
            geom->setVertexArray(v);

            osg::DrawElementsUByte* de = new osg::DrawElementsUByte(GL_LINES);
            de->push_back(0); de->push_back(1);
            de->push_back(1); de->push_back(3);
            de->push_back(3); de->push_back(2);
            de->push_back(2); de->push_back(0);
            de->push_back(4); de->push_back(5);
            de->push_back(5); de->push_back(7);
            de->push_back(7); de->push_back(6);
            de->push_back(6); de->push_back(4);
            de->push_back(0); de->push_back(4);
            de->push_back(1); de->push_back(5);
            de->push_back(3); de->push_back(7);
            de->push_back(2); de->push_back(6);
            geom->addPrimitiveSet(de);

            osg::Vec4Array* c= new osg::Vec4Array();
            c->push_back(osg::Vec4(0,1,1,1));
            geom->setColorArray(c);
            geom->setColorBinding(geom->BIND_OVERALL);

            geode->addDrawable(geom);

            sizeStr = Stringify() << bbox.xMax()-bbox.xMin();
            sizeStr = Stringify() << "min="<<bbox.zMin()<<"\nmax="<<bbox.zMax();
            zpos = bbox.zMax();
        }

#if 1
        osgText::Text* t = new osgText::Text();
        t->setText( sizeStr );
        //t->setFont( osgEarth::Registry::instance()->getDefaultFont() );
        t->setCharacterSizeMode(t->SCREEN_COORDS);
        t->setCharacterSize(36.0f);
        t->setAlignment(t->CENTER_CENTER);
        t->setColor(osg::Vec4(1,1,1,1));
        t->setBackdropColor(osg::Vec4(0,0,0,1));
        t->setBackdropType(t->OUTLINE);
        t->setPosition(osg::Vec3(0,0,zpos));
        geode->addDrawable(t);
#endif

        geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(),0);
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING,0);

        return geode;
    }
}

//..............................................................

SurfaceNode::SurfaceNode(const TileKey&        tilekey,
                         const MapInfo&        mapinfo,
                         const RenderBindings& bindings,
                         TileDrawable*         drawable)
{
    _tileKey = tilekey;

    _drawable = drawable;

    _surfaceGeode = new osg::Geode();
    _surfaceGeode->addDrawable( drawable );

#if 0
    if ( pool )
    {
        osg::ref_ptr<osg::Geometry> geom;
        pool->getPooledGeometry( tilekey, mapinfo, geom );

        _drawable = new TileDrawable(tilekey, bindings, geom.get());

        _surfaceGeode->addDrawable( _drawable.get() );
    }
    else
    {
        OE_WARN << LC << "INTERNAL: no geometry pool; result is an empty geode\n";
    }
#endif
    
    // Create the final node.
    addChild( _surfaceGeode.get() );
        
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    tilekey.getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix local2world;
    centroid.createLocalToWorld( local2world );
    setMatrix( local2world );

    _worldCorners.resize(8);
    _childrenCorners.resize(4);
    for(size_t i = 0; i < _childrenCorners.size(); ++i)
    {
        _childrenCorners[i].resize(8);
    }
    // Initialize the cached bounding box.
    setElevationExtrema(osg::Vec2f(0, 0));
}

float minVal(float lhs, float rhs)
{
    return std::min(lhs, rhs);
}
float maxVal(float lhs, float rhs)
{
    return std::max(lhs, rhs);
}

void GetLengthAndLengthSquared(float& diffLength, float& diffLengthSquare, const osg::Vec3& diff)
{
   diffLengthSquare = diff*diff;
   diffLength = sqrt(diffLengthSquare);
}

template<class Func>
class AccumulatePredicate
{
public:
    AccumulatePredicate(Func& func, const osg::Vec3& center): m_center(center), m_Func(func){}
    float operator()(float lhs, const osg::Vec3& corner)
    {
        float diffLength, diffLengthSquare;
        GetLengthAndLengthSquared(diffLength, diffLengthSquare, (corner-m_center));
        lhs = m_Func(lhs, diffLength);
        return lhs;
    }
private:
    const osg::Vec3& m_center;
    Func& m_Func;

};


float SurfaceNode::_minDistanceFromPointSquare(const VectorPoints& corners, const osg::Vec3& center)
{   
    float fMinDistance = std::accumulate(corners.begin(), corners.end()
        , std::numeric_limits<float>::max()
        , AccumulatePredicate< float (float, float) >(minVal, center));
    return fMinDistance;
}

float SurfaceNode::_maxDistanceFromPointSquare(const VectorPoints& corners, const osg::Vec3& center)
{
    float fMaxDistance = std::accumulate(corners.begin(), corners.end()
        , std::numeric_limits<float>::min()
        , AccumulatePredicate< float (float, float) >(maxVal, center));

    return fMaxDistance;
}

float SurfaceNode::minDistanceFromPointSquare(const osg::Vec3& center)
{
    return _minDistanceFromPointSquare(_worldCorners, center);
}

float SurfaceNode::maxDistanceFromPointSquare(const osg::Vec3& center)
{
    return _maxDistanceFromPointSquare(_worldCorners, center);
}

bool
SurfaceNode::boxIntersectsSphere(const osg::Vec3& center, float radius, float radiusSquare)
{
    float fMinDistanceSquare = minDistanceFromPointSquare(center);
 // return fMinDistanceSquare<=radiusSquare;
    return fMinDistanceSquare<=radius;
}

bool
SurfaceNode::anyChildBoxIntersectsSphere(const osg::Vec3& center, float radius, float radiusSquare)
{
    for(ChildrenCorners::const_iterator it = _childrenCorners.begin(); it != _childrenCorners.end(); ++it)
    {
        const VectorPoints& childCorners = *it;
        float fMinDistanceSquare = _minDistanceFromPointSquare(childCorners, center);
      // if (fMinDistanceSquare<=radiusSquare)
        if (fMinDistanceSquare<=radius)
        {
            return true;
        }
    }
    return false;
}
void
SurfaceNode::setElevationExtrema(const osg::Vec2f& minmax)
{
    _drawable->setElevationExtrema(minmax);

    // compute, not get.
    osg::BoundingBox box = _drawable->computeBox();

    osg::Matrix local2world;
    this->computeLocalToWorldMatrix(local2world, 0L);
    
    _bbox.init();
    for(int i=0; i<8; ++i)
    {
        _worldCorners[i] = (box.corner(i) * local2world);
        _bbox.expandBy( _worldCorners[i] );
    }

    osg::Vec3 minZMedians[4];
    osg::Vec3 maxZMedians[4];

    minZMedians[0] = (box.corner(0)+box.corner(1))*0.5;
    minZMedians[1] = (box.corner(1)+box.corner(3))*0.5;
    minZMedians[2] = (box.corner(3)+box.corner(2))*0.5;
    minZMedians[3] = (box.corner(0)+box.corner(2))*0.5;
                                  
    maxZMedians[0] = (box.corner(4)+box.corner(5))*0.5;
    maxZMedians[1] = (box.corner(5)+box.corner(7))*0.5;
    maxZMedians[2] = (box.corner(7)+box.corner(6))*0.5;
    maxZMedians[3] = (box.corner(4)+box.corner(6))*0.5;
                                  
    // Child 0 corners
    _childrenCorners[0][0] =  box.corner(0);
    _childrenCorners[0][1] =  minZMedians[0];
    _childrenCorners[0][2] =  minZMedians[3];
    _childrenCorners[0][3] = (minZMedians[0]+minZMedians[2])*0.5;

    _childrenCorners[0][4] =  box.corner(4);
    _childrenCorners[0][5] =  maxZMedians[0];
    _childrenCorners[0][6] =  maxZMedians[3];
    _childrenCorners[0][7] = (maxZMedians[0]+maxZMedians[2])*0.5;

    // Child 1 corners
    _childrenCorners[1][0] =  minZMedians[0];
    _childrenCorners[1][1] =  box.corner(1);
    _childrenCorners[1][2] = (minZMedians[0]+minZMedians[2])*0.5;
    _childrenCorners[1][3] =  minZMedians[1];
                     
    _childrenCorners[1][4] =  maxZMedians[0];
    _childrenCorners[1][5] =  box.corner(5);
    _childrenCorners[1][6] = (maxZMedians[0]+maxZMedians[2])*0.5;
    _childrenCorners[1][7] =  maxZMedians[1];

    // Child 2 corners
    _childrenCorners[2][0] =  minZMedians[3];
    _childrenCorners[2][1] = (minZMedians[0]+minZMedians[2])*0.5;
    _childrenCorners[2][2] =  box.corner(2);
    _childrenCorners[2][3] =  minZMedians[2];
                     
    _childrenCorners[2][4] =  maxZMedians[3];
    _childrenCorners[2][5] = (maxZMedians[0]+maxZMedians[2])*0.5;
    _childrenCorners[2][6] =  box.corner(6);
    _childrenCorners[2][7] =  maxZMedians[2]; 

    // Child 3 corners
    _childrenCorners[3][0] = (minZMedians[0]+minZMedians[2])*0.5;
    _childrenCorners[3][1] =  minZMedians[1];
    _childrenCorners[3][2] =  minZMedians[2];
    _childrenCorners[3][3] =  box.corner(3);
                     
    _childrenCorners[3][4] = (maxZMedians[0]+maxZMedians[2])*0.5;
    _childrenCorners[3][5] =  maxZMedians[1];
    _childrenCorners[3][6] =  maxZMedians[2];
    _childrenCorners[3][7] =  box.corner(7);

    // Transform to world space
    for(size_t childIndex = 0; childIndex < _childrenCorners.size(); ++ childIndex)
    {
         VectorPoints& childrenCorners = _childrenCorners[childIndex];
         for(size_t cornerIndex = 0; cornerIndex < childrenCorners.size(); ++cornerIndex)
         {
             osg::Vec3& childCorner = childrenCorners[cornerIndex];
             osg::Vec3 childCornerWorldSpace = childCorner*local2world;
             childCorner = childCornerWorldSpace;
         }
    }

#if 0
    if ( _debugGeode.valid() )
        removeChild( _debugGeode.get() );
    _debugGeode = makeBBox(box);
    addChild( _debugGeode.get() );
#endif
}

const osg::BoundingBox&
SurfaceNode::getAlignedBoundingBox() const
{
    return _drawable->getBox();
}
