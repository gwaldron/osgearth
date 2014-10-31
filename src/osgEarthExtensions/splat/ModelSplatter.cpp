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
#include "ModelSplatter"

#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/VirtualProgram>
#include <osgEarth/DrawInstanced>
#include <osgEarth/TerrainTileNode>
#include <osgEarth/NodeUtils>

#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[ModelSplatter] "

namespace
{
    const char* vs_model = 
        "#version 120 \n" 
        "#extension GL_EXT_gpu_shader4 : enable \n" 
        "#extension GL_ARB_draw_instanced: enable \n" 

        "uniform sampler2D oe_terrain_tex; \n"
        "uniform mat4 oe_terrain_tex_matrix; \n"
        "uniform mat4 oe_trees_up; \n"
        "uniform vec2 oe_trees_span; \n"
        "uniform mat4 oe_trees_local2ltp; \n"
        "uniform mat4 oe_trees_ltp2local; \n"

        "uniform float osg_FrameTime; \n"
        "uniform vec4 oe_tile_key; \n"

        "varying float oe_modelsplat_dist; \n"
        
        "float oe_modelsplat_rand(vec2 co) \n"
        "{\n"
        "   return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);\n"
        "}\n"

        "void oe_modelsplat_vert_model(inout vec4 VertexMODEL) \n" 
        "{ \n"
        "    vec4 v = oe_trees_local2ltp * VertexMODEL; \n"
        
        //"    v.xyz /= v.w; \n"
        //"    float delta = clamp(20.0 - oe_tile_key.z, 1.0, 8.0); \n"
        //"    v.xyz *= delta; \n"

        "    vec2 span = oe_trees_span; \n"
        "    float fInstanceID = float(gl_InstanceID); \n"
        "    float rx = oe_modelsplat_rand( vec2(fInstanceID, oe_tile_key.z)); \n"
        "    float ry = oe_modelsplat_rand( vec2(rx, -fInstanceID)); \n"
        "    vec2 rxy = vec2(rx, ry); \n"
        "    vec2 offset = -0.5*span + span*rxy; \n"
        "    v.xy += offset; \n"

        //"    float a = osg_FrameTime+rx; \n"
        //"    float c = sin(a); \n"
        //"    v.xy += 0.1*c*v.z; \n"

        // matrix mult probably unnecessary 
        "    vec4 rc = oe_terrain_tex_matrix * vec4(rx, ry, 0.0, 1.0); \n"
     
        // scale and bias the tex coords for heightfield sampling:
        // "16" is the tile size. Obviously this needs to be parameterized.
        "    rc *= 15.0/16.0; \n"
        "    rc += 0.5/16.0; \n"

        "    float h = texture2D(oe_terrain_tex, rc.st).r; \n"
        "    v.z += h; \n"

        "    VertexMODEL = oe_trees_ltp2local * v; \n"
        "} \n";

#if 0
    const char* vs_view =
        "#version 110 \n"
        "uniform vec4 oe_tile_key; \n"
        "uniform float oe_min_tile_range_factor; \n"
        "varying float oe_modelsplat_dist; \n"
        "void oe_modelsplat_vertView(inout vec4 VertexVIEW) { \n"        
        "    float radius       = oe_tile_key.w; \n"
        "    float near         = oe_min_tile_range_factor*radius; \n"
        "    float far          = near + radius*2.0; \n"
        "    float d            = length(VertexVIEW.xyz/VertexVIEW.w); \n"
        "    oe_modelsplat_dist = clamp((d-near)/(far-near), 0.0, 1.0); \n"
        "} \n";
#endif

    const char* fs =
        "#version 110\n"
        "void oe_modelsplat_frag(inout vec4 color) { \n"
        "    if (color.a < 0.2) discard; \n"
        "} \n";
}

//..........................................................................


ModelSplatter::ModelSplatter() :
_dirty( true ),
_count( 65 ),
_minLOD( 14 )
{
    //nop
}

ModelSplatter::~ModelSplatter()
{
    //nop
}

void 
ModelSplatter::setModel(osg::Node* node)
{
    _model = node;
    _dirty = true;
}

void
ModelSplatter::setNumInstances(unsigned num)
{
    _count = num;
    _dirty = true;
}

void 
ModelSplatter::setMinLOD(unsigned lod)
{
    _minLOD = lod;
    _dirty = true;
}

void
ModelSplatter::establish()
{
    if ( _dirty && _model.valid() )
    {
        Threading::ScopedMutexLock lock(_modelMutex);
        if ( _dirty && _model.valid() )
        {
            _dirty = false;
            
            // have to set unref-after-apply to false:
            osgUtil::Optimizer::TextureVisitor texvis(
                true, false,
                false, false,
                false, false);
            _model->accept( texvis );

            // have to enimilate the scale matrix:
            osgUtil::Optimizer::FlattenStaticTransformsVisitor flatten;
            _model->accept( flatten );

            osg::BoundingBox bbox;
            DrawInstanced::ConvertToDrawInstanced cdi( _count, bbox, true );
            _model->accept( cdi );

            osg::ref_ptr<StateSetCache> cache = new StateSetCache();
            Registry::shaderGenerator().run(_model, cache.get());

            VirtualProgram* vp = VirtualProgram::getOrCreate( _model->getOrCreateStateSet() );

            vp->setFunction("oe_modelsplat_vert_model", vs_model, ShaderComp::LOCATION_VERTEX_MODEL);
            vp->setFunction("oe_modelsplat_frag", fs, ShaderComp::LOCATION_FRAGMENT_COLORING, 2.0f);
        }
    }
}

void
ModelSplatter::operator()(const TileKey& key, osg::Node* node)
{
    TerrainTileNode* tile = osgEarth::findTopMostNodeOfType<TerrainTileNode>(node);
    if ( !tile )
        return;

    if ( key.getLOD() >= _minLOD && _model.valid() )
    {
        // make sure the correct model is loaded
        establish();

        // elevation texture and matrix are required
        osg::Texture* elevationTex = tile->getElevationTexture();
        if ( !elevationTex )
        {
            //OE_WARN << LC << "No elevation texture for key " << key.str() << "\n";
            return;
        }

        osg::RefMatrix* elevationTexMat = tile->getElevationTextureMatrix();
        if ( !elevationTexMat )
        {
            //OE_WARN << LC << "No elevation texture matrix for key " << key.str() << "\n";
            return;
        }
        
        tile->addChild( _model.get() );

        osg::StateSet* ss = tile->getOrCreateStateSet();

        // first, a rotation vector to make trees point up.
        GeoPoint p;
        key.getExtent().getCentroid(p);
        osg::Vec3d up;
        p.createWorldUpVector(up);
        osg::Quat q;
        q.makeRotate(osg::Vec3d(0,0,1), up);
        osg::Matrixd zup = osg::Matrixd::rotate(q);

        // matrices to resolve the weird terrain localization into a usable LTP.
        osg::Matrix tile2world = tile->getMatrix();
        osg::Matrix world2ltp;
        p.createWorldToLocal(world2ltp);
        osg::Matrix local2ltp = tile2world * world2ltp;
        osg::Matrix ltp2local;
        ltp2local.invert(local2ltp);

        // after inverting the matrix, combine the ZUP (optimization)
        local2ltp.preMult( zup );

        ss->addUniform( new osg::Uniform("oe_trees_local2ltp", osg::Matrixf(local2ltp)) );
        ss->addUniform( new osg::Uniform("oe_trees_ltp2local", osg::Matrixf(ltp2local)) );

        // calculate the scatter area:
        float h = key.getExtent().height() * 111320.0f;
        float w = key.getExtent().width() * 111320.0f * cos(fabs(osg::DegreesToRadians(p.y())));
        ss->addUniform( new osg::Uniform("oe_trees_span", osg::Vec2f(w,h)) );
        
        ss->setTextureAttributeAndModes(2, tile->getElevationTexture(), 1);
        ss->addUniform( new osg::Uniform("oe_terrain_tex_matrix", osg::Matrixf(*elevationTexMat)) );        
    }
}
