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
#include <osgEarth/Registry>
#include <osgEarth/Horizon>
#include <osgEarth/ImageUtils>

#include <osg/CullStack>
#include <osg/Geode>

#include <osg/Geometry>
#include <osg/TriangleFunctor>
#include <osgText/Text>
#include <osg/CullStack>
#include <osgUtil/CullVisitor>

#include <numeric>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SurfaceNode] "

//..............................................................

namespace
{    
    osg::Geode* makeBBox(const osg::BoundingBox& bbox, const TileKey& key)
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

#if 1
            // bottom:
            de->push_back(0); de->push_back(1);
            de->push_back(1); de->push_back(3);
            de->push_back(3); de->push_back(2);
            de->push_back(2); de->push_back(0);
#endif
#if 1
            // top:
            de->push_back(4); de->push_back(5);
            de->push_back(5); de->push_back(7);
            de->push_back(7); de->push_back(6);
            de->push_back(6); de->push_back(4);
#endif
#if 0
            // corners:
            de->push_back(0); de->push_back(4);
            de->push_back(1); de->push_back(5);
            de->push_back(3); de->push_back(7);
            de->push_back(2); de->push_back(6);
#endif
            geom->addPrimitiveSet(de);

            osg::Vec4Array* c= new osg::Vec4Array();
            c->push_back(osg::Vec4(1,0,0,1));
            geom->setColorArray(c);
            geom->setColorBinding(geom->BIND_OVERALL);

            geode->addDrawable(geom);

            sizeStr = Stringify() << key.str() << "\nmax="<<bbox.zMax()<<"\nmin="<<bbox.zMin()<<"\n";
            zpos = bbox.zMax();
        }

        osgText::Text* textDrawable = new osgText::Text();
        textDrawable->setDataVariance(osg::Object::DYNAMIC);
        textDrawable->setText( sizeStr );
        textDrawable->setFont( osgEarth::Registry::instance()->getDefaultFont() );
        textDrawable->setCharacterSizeMode(textDrawable->SCREEN_COORDS);
        textDrawable->setCharacterSize(32.0f);
        textDrawable->setAlignment(textDrawable->CENTER_BOTTOM);
        textDrawable->setColor(osg::Vec4(1,1,1,1));
        textDrawable->setBackdropColor(osg::Vec4(0,0,0,1));
        textDrawable->setBackdropType(textDrawable->OUTLINE);
        textDrawable->setPosition(osg::Vec3(0,0,zpos));
        textDrawable->setAutoRotateToScreen(true);
        geode->addDrawable(textDrawable);

        geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(),0);
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING,0);
        geode->getOrCreateStateSet()->setRenderBinDetails(INT_MAX, "DepthSortedBin");

        return geode;
    }

    osg::Geode* makeSphere(const osg::BoundingSphere& bs)
    {
        osg::Geometry* geom = new osg::Geometry();
        geom->setUseVertexBufferObjects(true);

        float r = bs.radius();

        osg::Vec3Array* v = new osg::Vec3Array();
        v->reserve(6);
        v->push_back(osg::Vec3(0, 0, r)); // top
        v->push_back(osg::Vec3(0, 0, -r)); // bottom
        v->push_back(osg::Vec3(-r, 0, 0)); // left
        v->push_back(osg::Vec3(r, 0, 0)); // right
        v->push_back(osg::Vec3(0, r, 0)); // back
        v->push_back(osg::Vec3(0, -r, 0)); // front
        geom->setVertexArray(v);

        osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_LINE_STRIP);
        b->reserve(24);
        b->push_back(0); b->push_back(3); b->push_back(4);
        b->push_back(0); b->push_back(4); b->push_back(2);
        b->push_back(0); b->push_back(2); b->push_back(5);
        b->push_back(0); b->push_back(5); b->push_back(3);
        b->push_back(1); b->push_back(3); b->push_back(5);
        b->push_back(1); b->push_back(4); b->push_back(3);
        b->push_back(1); b->push_back(2); b->push_back(4);
        b->push_back(1); b->push_back(5); b->push_back(2);
        geom->addPrimitiveSet(b);

        osg::Vec3Array* n = new osg::Vec3Array();
        n->reserve(6);
        n->push_back(osg::Vec3(0, 0, 1));
        n->push_back(osg::Vec3(0, 0, -1));
        n->push_back(osg::Vec3(-1, 0, 0));
        n->push_back(osg::Vec3(1, 0, 0));
        n->push_back(osg::Vec3(0, 1, 0));
        n->push_back(osg::Vec3(0, -1, 0));
        geom->setNormalArray(n);
        geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

        //MeshSubdivider ms;
        //ms.run(*geom, osg::DegreesToRadians(maxAngle), GEOINTERP_GREAT_CIRCLE);

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0].set(1,1,0,1);
        geom->setColorArray(c);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(geom);

        return geode;
    }
}

//..............................................................

void
HorizonTileCuller::set(const SpatialReference* srs, 
                       const osg::Matrix&      local2world,
                       const osg::BoundingBox& bbox)
{
    if (!_horizon.valid() && srs->isGeographic())
    {
        _horizon = new Horizon();
    }

    if (_horizon.valid())
    {
        _horizon->setEllipsoid(*srs->getEllipsoid());
        //_radiusPolar = srs->getEllipsoid()->getRadiusPolar();
        //_radiusEquator = srs->getEllipsoid()->getRadiusEquator();
        //_local2world = local2world;

        // Adjust the horizon ellipsoid based on the minimum Z value of the tile;
        // necessary because a tile that's below the ellipsoid (ocean floor, e.g.)
        // may be visible even if it doesn't pass the horizon-cone test. In such
        // cases we need a more conservative ellipsoid.
        double zMin = (double)std::min( bbox.corner(0).z(), 0.0f );
        zMin = std::max(zMin, -25000.0); // approx the lowest point on earth * 2
        _horizon->setEllipsoid( osg::EllipsoidModel(
            srs->getEllipsoid()->getRadiusEquator() + zMin, 
            srs->getEllipsoid()->getRadiusPolar() + zMin) );

        // consider the uppermost 4 points of the tile-aligned bounding box.
        // (the last four corners of the bbox are the "zmax" corners.)
        for(unsigned i=0; i<4; ++i)
        {
            _points[i] = bbox.corner(4+i) * local2world;
        }

        //_bs.set(bbox.center() * _local2world, bbox.radius());
    }
}

bool
HorizonTileCuller::isVisible(const osg::Vec3d& from) const
{
    if (!_horizon.valid())
        return true;

    // alternate method (slower)
    //return _horizon->isVisible(from, _bs.center(), _bs.radius());

    for (unsigned i = 0; i < 4; ++i)
        if (_horizon->isVisible(from, _points[i], 0.0))
            return true;

    return false;
}

//..............................................................


const bool SurfaceNode::_enableDebugNodes = ::getenv("OSGEARTH_MP_DEBUG") != 0L;

SurfaceNode::SurfaceNode(const TileKey&        tilekey,
                         const MapInfo&        mapinfo,
                         const RenderBindings& bindings,
                         TileDrawable*         drawable)
{
    _tileKey = tilekey;

    _drawable = drawable;

    _surfaceGeode = new osg::Geode();
    _surfaceGeode->addDrawable( drawable );
    
    // Create the final node.
    addChild( _surfaceGeode.get() );

    // Establish a local reference frame for the tile:
    GeoPoint centroid;
    tilekey.getExtent().getCentroid(centroid);

    osg::Matrix local2world;
    centroid.createLocalToWorld( local2world );
    setMatrix( local2world );
    
    // Initialize the cached bounding box.
    setElevationRaster( 0L, osg::Matrixf::identity() );
}

void
SurfaceNode::setElevationRaster(const osg::Image*   raster,
                                const osg::Matrixf& scaleBias)
{
    if ( !_drawable.valid() )
        return;

    // communicate the raster to the drawable:
    if ( raster )
    {
        _drawable->setElevationRaster( raster, scaleBias );
    }

    // next compute the bounding box in local space:
#if OSG_VERSION_GREATER_OR_EQUAL(3,3,2)
    const osg::BoundingBox& box = _drawable->getBoundingBox();
#else
    const osg::BoundingBox& box = _drawable->getBound();
#endif

    // Compute the medians of each potential child node:

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

    // Transform the child corners to world space
    
    const osg::Matrix& local2world = getMatrix();
    for(int i=0; i<4; ++i)
    {
        VectorPoints& childCorners = _childrenCorners[i];
         for(int j=0; j<8; ++j)
         {
             osg::Vec3& corner = childCorners[j];
             corner = corner*local2world;
         }
    }

    if( _enableDebugNodes )
    {
        removeDebugNode();
        addDebugNode(box);
    }

    // Update the horizon culler.
    _horizonCuller.set( _tileKey.getProfile()->getSRS(), getMatrix(), box );

    // need this?
    dirtyBound();
}

const osg::Image*
SurfaceNode::getElevationRaster() const
{
    return _drawable->getElevationRaster();
}

const osg::Matrixf&
SurfaceNode::getElevationMatrix() const
{
    return _drawable->getElevationMatrix();
};

void
SurfaceNode::addDebugNode(const osg::BoundingBox& box)
{
    _debugText = 0;
    _debugGeode = makeBBox(box, _tileKey);
    //_debugGeode = makeSphere(this->getBound());
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

const osg::BoundingBox&
SurfaceNode::getAlignedBoundingBox() const
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,3,2)
    return _drawable->getBoundingBox();
#else
    return _drawable->getBound();
#endif
}
