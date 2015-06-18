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

SurfaceNode::SurfaceNode(const TileKey& tilekey,
                         const MapInfo& mapinfo,
                         const RenderBindings& bindings,
                         GeometryPool*  pool)
{
    _tileKey = tilekey;

    _surfaceGeode = new osg::Geode();
    osg::StateSet* surfaceSS = _surfaceGeode->getOrCreateStateSet();
    surfaceSS->setRenderBinDetails(0, "oe.SurfaceBin");
    surfaceSS->setNestRenderBins(false);

    // holds the control surface.
    _landCoverGeode = new osg::Geode();
    osg::StateSet* landCoverSS = _landCoverGeode->getOrCreateStateSet();
    landCoverSS->setRenderBinDetails(1, "oe.LandCoverBin");
    landCoverSS->setNestRenderBins(false);

    if ( pool )
    {
        osg::ref_ptr<osg::Geometry> geom;
        pool->getPooledGeometry( tilekey, mapinfo, geom );

        _drawable = new TileDrawable(tilekey, bindings, geom.get());

        _surfaceGeode->addDrawable( _drawable.get() );
        _landCoverGeode->addDrawable( _drawable.get() );
    }
    else
    {
        OE_WARN << LC << "INTERNAL: no geometry pool; result is an empty geode\n";
    }
    
    // Create the final node.
    addChild( _surfaceGeode.get() );
    addChild( _landCoverGeode.get() );
        
    // Establish a local reference frame for the tile:
    osg::Vec3d centerWorld;
    GeoPoint centroid;
    tilekey.getExtent().getCentroid(centroid);
    centroid.toWorld(centerWorld);
    osg::Matrix local2world;
    centroid.createLocalToWorld( local2world );
    setMatrix( local2world );

    // Initialize the cached bounding box.
    setElevationExtrema(osg::Vec2f(0, 0));
}

void
SurfaceNode::traverse(osg::NodeVisitor& nv)
{
    //_surfaceGeode->accept(nv);
    //_landCoverGeode->accept(nv);
    osg::MatrixTransform::traverse(nv);
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
        _bbox.expandBy( box.corner(i) * local2world );
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
