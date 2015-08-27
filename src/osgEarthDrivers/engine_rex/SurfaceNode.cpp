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

        osgText::Text* textDrawable = new osgText::Text();
        textDrawable->setText( sizeStr );
        //textDrawable->setFont( osgEarth::Registry::instance()->getDefaultFont() );
        textDrawable->setCharacterSizeMode(textDrawable->SCREEN_COORDS);
        textDrawable->setCharacterSize(20.0f);
        textDrawable->setAlignment(textDrawable->CENTER_CENTER);
        textDrawable->setColor(osg::Vec4(1,1,1,1));
        textDrawable->setBackdropColor(osg::Vec4(0,0,0,1));
        textDrawable->setBackdropType(textDrawable->OUTLINE);
        textDrawable->setPosition(osg::Vec3(0,0,zpos));
        geode->addDrawable(textDrawable);

        geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(),0);
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING,0);

        return geode;
    }
}

//..............................................................

const bool SurfaceNode::_enableDebugNodes = false; //true;

SurfaceNode::SurfaceNode(const TileKey&        tilekey,
                         const MapInfo&        mapinfo,
                         const RenderBindings& bindings,
                         TileDrawable*         drawable)
: _debugNodeVisible(false)
{
    _tileKey = tilekey;

    _drawable = drawable;

    _surfaceGeode = new osg::Geode();
    _surfaceGeode->addDrawable( drawable );
    
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

float
SurfaceNode::minSquaredDistanceFromPoint(const VectorPoints& corners, const osg::Vec3& center, float fZoomFactor)
{   
    float mind2 = FLT_MAX;
    for( VectorPoints::const_iterator i=corners.begin(); i != corners.end(); ++i )
    {
        float d2 = (*i - center).length2()*fZoomFactor*fZoomFactor;
        if ( d2 < mind2 ) mind2 = d2;
    }
    return mind2;
}

bool
SurfaceNode::anyChildBoxIntersectsSphere(const osg::Vec3& center, float radiusSquared, float fZoomFactor)
{
    for(ChildrenCorners::const_iterator it = _childrenCorners.begin(); it != _childrenCorners.end(); ++it)
    {
        const VectorPoints& childCorners = *it;
        float fMinDistanceSquared = minSquaredDistanceFromPoint(childCorners, center, fZoomFactor);
        if (fMinDistanceSquared <= radiusSquared)
        {
            return true;
        }
    }
    return false;
}

void
SurfaceNode::setElevationExtrema(const osg::Vec2f& minmax)
{
    // communicate the extrema to the drawable so it can compute a proper bounding box:
    if ( minmax != osg::Vec2f(0,0) )
        _drawable->setElevationExtrema(minmax);

    // bounding box in local space:
    const osg::BoundingBox& box = _drawable->getBox();
    
    // compute the bounding box in world space:
    const osg::Matrix& local2world = getMatrix();    
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

    if(_enableDebugNodes)
    {
        removeDebugNode();
        addDebugNode(box);
        setDebugNodeVisible(false);
    }
}

void
SurfaceNode::setElevationRaster(const osg::Image*   raster,
                                const osg::Matrixf& scaleBias)
{
    if ( _drawable.valid() )
    {
        _drawable->setElevationRaster( raster, scaleBias );
    }
}

void
SurfaceNode::addDebugNode(const osg::BoundingBox& box)
{
    _debugText = 0;
    _debugGeode = makeBBox(box);

    for(size_t i = 0; i < _debugGeode->getNumDrawables(); ++i)
    {
        if (std::string(_debugGeode->getDrawable(i)->className())=="Text")
        {
            _debugText = static_cast<osgText::Text*>(_debugGeode->getDrawable(i));
            break;
        }
    }
    addChild( _debugGeode.get() );
}

void
SurfaceNode::removeDebugNode(void)
{
    _debugText = 0;
    if ( _debugGeode.valid() )
    {
        removeChild( _debugGeode.get() );
    }
}

void
SurfaceNode::setDebugText(const std::string& strText)
{
    if (_debugText.valid()==false)
    {
        return;
    }
    _debugText->setText(strText);
}

void
SurfaceNode::setDebugNodeVisible(bool bVisible)
{
    _debugNodeVisible = bVisible;
    if (_enableDebugNodes && _debugGeode.valid())
    {
        unsigned int mask = (_debugNodeVisible)? ~0x0 : 0;
        _debugGeode->setNodeMask(mask);
    }
}

bool
SurfaceNode::isDebugNodeVisible(void) const
{
    return _debugNodeVisible;
}

const osg::BoundingBox&
SurfaceNode::getAlignedBoundingBox() const
{
    return _drawable->getBox();
}
