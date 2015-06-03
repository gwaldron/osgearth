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
#include <osgEarth/ShaderFactory>
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

        "uniform sampler2D oe_tile_elevationTex; \n"
        "uniform mat4 oe_tile_elevationTexMatrix; \n"
        "uniform vec2 oe_trees_span; \n"

        "uniform vec4 oe_tile_key; \n"

        "varying float oe_modelsplat_dist; \n"
        
        "float oe_modelsplat_rand(vec2 co) \n"
        "{\n"
        "   return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);\n"
        "}\n"

        "void oe_modelsplat_vert_model(inout vec4 VertexMODEL) \n" 
        "{ \n"
        "    vec2 span = oe_trees_span; \n"
        "    float fInstanceID = float(gl_InstanceID); \n"
        "    float rx = oe_modelsplat_rand( vec2(fInstanceID, oe_tile_key.z)); \n"
        "    float ry = oe_modelsplat_rand( vec2(rx, -fInstanceID)); \n"
        "    vec2 rxy = vec2(rx, ry); \n"
        "    vec2 offset = -0.5*span + span*rxy; \n"
        "    VertexMODEL.xy += offset; \n"

        // matrix mult probably unnecessary 
        "    vec4 rc = oe_tile_elevationTexMatrix * vec4(rx, ry, 0.0, 1.0); \n"
     
        // scale and bias the tex coords for heightfield sampling:
        // "17" is the tile size. Obviously this needs to be parameterized.
        "    rc *= 16.0/17.0; \n"
        "    rc += 0.5/17.0; \n"

        "    float h = texture2D(oe_tile_elevationTex, rc.st).r; \n"
        "    VertexMODEL.z += h; \n"
        "} \n";

    const char* fs =
        "#version 110\n"
        "void oe_modelsplat_frag(inout vec4 color) { \n"
        "    if (color.a < 0.2) discard; \n"
        "} \n";
}

//..........................................................................


ModelSplatter::ModelSplatter() :
_dirty( true ),
_count( 128 ),
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
        }
    }
}

void
ModelSplatter::operator()(const TileKey& key, osg::Node* node)
{
#if 0
    TerrainTileNode* tile = osgEarth::findTopMostNodeOfType<TerrainTileNode>(node);
    if ( !tile )
        return;

    if ( key.getLOD() >= _minLOD && _model.valid() )
    {
        // make sure the correct model is loaded
        establish();

        //// elevation texture and matrix are required
        //osg::Texture* elevationTex = tile->getModel()->elevationModel().get();
        //if ( !elevationTex )
        //{
        //    //OE_WARN << LC << "No elevation texture for key " << key.str() << "\n";
        //    return;
        //}

        //osg::RefMatrixf* elevationTexMat = tile->getElevationTextureMatrix();
        //if ( !elevationTexMat )
        //{
        //    //OE_WARN << LC << "No elevation texture matrix for key " << key.str() << "\n";
        //    return;
        //}
        
        osg::Group* payload = tile->getOrCreatePayloadGroup();
        payload->addChild( _model.get() );
        //tile->addChild( _model.get() );

        osg::StateSet* ss = payload->getOrCreateStateSet();

        // first, a rotation vector to make trees point up.
        GeoPoint p;
        key.getExtent().getCentroid(p);

        // calculate the scatter area:
        float h = key.getExtent().height() * 111320.0f;
        float w = key.getExtent().width() * 111320.0f * cos(fabs(osg::DegreesToRadians(p.y())));
        ss->addUniform( new osg::Uniform("oe_trees_span", osg::Vec2f(w,h)) );
        
        //// hack..
        //ss->setTextureAttributeAndModes(2, tile->getElevationTexture(), 1);
        //ss->addUniform(new osg::Uniform("oe_terrain_tex", 2));
        //ss->addUniform(new osg::Uniform("oe_terrain_tex_matrix", osg::Matrixf(*elevationTexMat)) );        
    }
#endif
}
