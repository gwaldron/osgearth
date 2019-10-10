
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "ElevationCompiler"
#include <osgEarthFeatures/Session>
#include <osgUtil/SmoothingVisitor>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Buildings;

#define LC "[ElevationCompiler] "


bool
ElevationCompiler::compile(CompilerOutput&       output,
                           const Building*       building,
                           const Elevation*      elevation,
                           const osg::Matrix&    world2local,
                           const osgDB::Options* readOptions) const
{
    if ( !building ) return false;
    if ( !elevation ) return false;

    // make sure there's something to build:
    const Elevation::Walls& walls = elevation->getWalls();
    if ( walls.size() == 0 )
    {
        OE_DEBUG << LC << "Elevation has no walls; skipping.\n";
        return false;
    }

    // precalculate the frame transformation; combining these will
    // prevent any precision loss during the transform.
    osg::Matrix frame = building->getReferenceFrame() * world2local;
        
    SkinResource* skin = elevation->getSkinResource();
    bool useTexture = skin != 0L;
    float texWidth = 0.0f;
    float texHeight = 0.0f;
    osg::Vec2f texScale(1.0f, 1.0f);
    osg::Vec2f texBias (0.0f, 0.0f);
    float texLayer = 0.0f;

    osg::ref_ptr<osg::StateSet> stateSet;
    if ( skin )
    {
        stateSet = output.getSkinStateSet(skin, readOptions);

        texWidth = skin->imageWidth().get();
        texHeight = skin->imageHeight().get();
        texScale.set(skin->imageScaleS().get(), skin->imageScaleT().get());
        texBias.set(skin->imageBiasS().get(), skin->imageBiasT().get());
        texLayer = skin->imageLayer().get();
    }

    bool genColors  = false;
    bool genNormals = false;

    //TODO
    Color upperWallColor = elevation->getColor();
    Color lowerWallColor = elevation->getColor();
    if ( !skin )
        lowerWallColor = upperWallColor.brightness(0.95f);

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    geom->setUseVertexBufferObjects( true );
    geom->setUseDisplayList( false );

    if ( stateSet.valid() )
    {
        geom->setStateSet( stateSet.get() );
    }

    // Count the total number of verts.
    unsigned totalNumVerts = 0;
    for(Elevation::Walls::const_iterator wall = walls.begin(); wall != walls.end(); ++wall)
    {
        totalNumVerts += (6 * wall->getNumPoints());
    }

    totalNumVerts *= elevation->getNumFloors();
    OE_DEBUG << LC << "Extrusion: total verts in elevation = " << totalNumVerts << "\n";

    // preallocate all attribute arrays.
    osg::Vec3Array* verts = new osg::Vec3Array();
    geom->setVertexArray( verts );

    osg::Vec4Array* colors = 0L;
    if ( genColors )
    {
        colors = new osg::Vec4Array();
        geom->setColorArray( colors );
        geom->setColorBinding( geom->BIND_PER_VERTEX );
    }

    osg::Vec3Array* texCoords = 0L;
    if ( skin )
    {
        texCoords = new osg::Vec3Array();
        geom->setTexCoordArray( 0, texCoords );
    }

    osg::Vec3Array* normals = 0L;
    if ( genNormals )
    {
        normals = new osg::Vec3Array();
        geom->setNormalArray( normals );
        geom->setNormalBinding( geom->BIND_PER_VERTEX );
    }

    unsigned vertPtr = 0;
    //TODO
    float  floorHeight = elevation->getHeight() / (float)elevation->getNumFloors();

    OE_DEBUG << LC << "...elevation has " << walls.size() << " walls\n";

    // Each elevation is a collection of walls. One outer wall and
    // zero or more inner walls (where there were holes in the original footprint).
    for(Elevation::Walls::const_iterator wall = walls.begin(); wall != walls.end(); ++wall)
    {
        osg::DrawElements* de = 
            totalNumVerts > 0xFFFF ? (osg::DrawElements*) new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
            totalNumVerts > 0xFF   ? (osg::DrawElements*) new osg::DrawElementsUShort( GL_TRIANGLES ) :
                                     (osg::DrawElements*) new osg::DrawElementsUByte ( GL_TRIANGLES );

        // pre-allocate for speed
        //de->reserveElements( numWallVerts );
        geom->addPrimitiveSet( de );

        OE_DEBUG << LC << "..elevation has " << elevation->getNumFloors() << " floors\n";

        float numFloorsF = (float)elevation->getNumFloors();
            
        for(unsigned flr=0; flr < elevation->getNumFloors(); ++flr)
        {
            float lowerZ = (float)flr * floorHeight;
    
            OE_DEBUG << LC << "...wall has " << wall->faces.size() << " faces\n";
            for(Elevation::Faces::const_iterator f = wall->faces.begin(); f != wall->faces.end(); ++f, vertPtr += 4)
            {
                osg::Vec3d Lvec = f->left.upper - f->left.lower; Lvec.normalize();
                osg::Vec3d Rvec = f->right.upper - f->right.lower; Rvec.normalize();

                float upperZ = lowerZ + floorHeight;

                osg::Vec3d LL = (f->left.lower  + Lvec*lowerZ) * frame;
                osg::Vec3d UL = (f->left.lower  + Lvec*upperZ) * frame;
                osg::Vec3d LR = (f->right.lower + Rvec*lowerZ) * frame;
                osg::Vec3d UR = (f->right.lower + Rvec*upperZ) * frame;

                verts->push_back( UL );
                verts->push_back( LL );
                verts->push_back( LR );
                verts->push_back( UR );
           
#if 0
                if ( anchors )
                {
                    float x = structure.baseCentroid.x(), y = structure.baseCentroid.y(), vo = structure.verticalOffset;

                    (*anchors)[vertPtr+1].set( x, y, vo, Clamping::ClampToGround );
                    (*anchors)[vertPtr+2].set( x, y, vo, Clamping::ClampToGround );
                    (*anchors)[vertPtr+3].set( x, y, vo, Clamping::ClampToGround );

                    (*anchors)[vertPtr+0].set( x, y, vo, Clamping::ClampToAnchor );
                    (*anchors)[vertPtr+4].set( x, y, vo, Clamping::ClampToAnchor );
                    (*anchors)[vertPtr+5].set( x, y, vo, Clamping::ClampToAnchor );
                }
#endif

                // Assign wall polygon colors.
                if ( colors )
                {
                    colors->push_back( upperWallColor );
                    colors->push_back( lowerWallColor );
                    colors->push_back( lowerWallColor );
                    colors->push_back( upperWallColor );
                }

                if ( texCoords )
                {
                    // Calculate the texture coordinates at each corner. The structure builder
                    // will have spaced the verts correctly for this to work.
                    float uL = fmod( f->left.offsetX,  texWidth ) / texWidth;
                    float uR = fmod( f->right.offsetX, texWidth ) / texWidth;

                    // Correct for the case in which the rightmost corner is exactly on a
                    // texture boundary.
                    if ( uR < uL || (uL == 0.0 && uR == 0.0))
                        uR = 1.0f;

                    osg::Vec2f texLL( uL, 0.0f );
                    osg::Vec2f texLR( uR, 0.0f );
                    osg::Vec2f texUL( uL, 1.0f );
                    osg::Vec2f texUR( uR, 1.0f );

                    texUL = texBias + osg::componentMultiply(texUL, texScale);
                    texUR = texBias + osg::componentMultiply(texUR, texScale);
                    texLL = texBias + osg::componentMultiply(texLL, texScale);
                    texLR = texBias + osg::componentMultiply(texLR, texScale);
                        
                    texCoords->push_back( osg::Vec3f(texUL.x(), texUL.y(), texLayer) );
                    texCoords->push_back( osg::Vec3f(texLL.x(), texLL.y(), texLayer) );
                    texCoords->push_back( osg::Vec3f(texLR.x(), texLR.y(), texLayer) );
                    texCoords->push_back( osg::Vec3f(texUR.x(), texUR.y(), texLayer) );
                }

                // build the triangles.
                de->addElement( vertPtr+0 );
                de->addElement( vertPtr+1 );
                de->addElement( vertPtr+2 );
                de->addElement( vertPtr+0 );
                de->addElement( vertPtr+2 );
                de->addElement( vertPtr+3 );

            } // faces loop

        } // floors loop

    } // walls loop

    // TODO - temporary, doesn't smooth disconnected edges
    osgUtil::SmoothingVisitor::smooth( *geom.get(), 15.0f );
    
    if ( !genColors )
    {
        colors = new osg::Vec4Array();
        geom->setColorArray( colors );
        geom->setColorBinding( geom->BIND_OVERALL );
        colors->push_back(osg::Vec4(1,1,1,1));
    }
    
    output.addDrawable( geom.get(), elevation->getTag() );

    return true;
}
