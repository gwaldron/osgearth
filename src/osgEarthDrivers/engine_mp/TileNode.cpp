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
#include "TileNode"

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Uniform>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/DrawInstanced>
#include <osgEarth/Registry>
#include <osgUtil/Optimizer>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "

#define TREE_LOD 15
#define TREE_COUNT 200

namespace
{
    const char* vs = 
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

        "varying vec4 osg_FrontColor; \n"
        
        "float oe_trees_rand(vec2 co) \n"
        "{\n"
        "   return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);\n"
        "}\n"

        "void oe_trees_vert(inout vec4 VertexMODEL) \n" 
        "{ \n"
        "    vec4 v = oe_trees_local2ltp * VertexMODEL; \n"
        "    vec2 span = oe_trees_span; \n"
        "    float fInstanceID = float(gl_InstanceID); \n"
        "    float rx = clamp(oe_trees_rand( vec2(fInstanceID,  fInstanceID)), 0.0, 0.99); \n"
        "    float ry = clamp(oe_trees_rand( vec2(fInstanceID, -fInstanceID)), 0.0, 0.99); \n"
        "    vec2 rxy = vec2(rx, ry); \n"
        "    vec2 offset = -0.5*span + span*rxy; \n"
        "    v.xy += offset; \n"

        "    float a = osg_FrameTime+rx; \n"
        "    float c = sin(a); \n"
        "    v.xy += 0.1*c*v.z; \n"

        // matrix mult probably unnecessary 
        "    vec4 rc = oe_terrain_tex_matrix * vec4(rx, ry, 0.0, 1.0); \n"
     
        // scale and bias the tex coords for heightfield sampling:
        "    rc *= 15.0/16.0; \n"
        "    rc += 0.5/16.0; \n"

        "    float h = texture2D(oe_terrain_tex, rc.st).r; \n"
        "    v.z += h; \n"

        "    VertexMODEL = oe_trees_ltp2local * v; \n"
        "} \n";

    const char* fs =
        "#version 110\n"
        "varying vec4 osg_FrontColor; \n"
        "void oe_trees_frag(inout vec4 color) { color = osg_FrontColor;\n}\n";

    Threading::Mutex        tree_mutex;
    osg::ref_ptr<osg::Node> tree;

    osg::Node* makeTrees(int delta)
    {
        if ( !tree.valid() )
        {
            Threading::ScopedMutexLock lock(tree_mutex);
            if ( !tree.valid() )
            { 
                //tree = osgDB::readNodeFile("H:/devel/osgearth/repo/data/loopix/grass1a.osgb");
                tree = osgDB::readNodeFile("H:/devel/osgearth/repo/data/pinetree.ive");
                //tree = osgDB::readNodeFile("H:/devel/osgearth/repo/data/bigtree.ive");
                //tree = osgDB::readNodeFile("H:/devel/osgearth/repo/data/gw3.osg");

                // have to set unref-after-apply to false:
                osgUtil::Optimizer::TextureVisitor texvis(
                    true, false,
                    false, false,
                    false, false);
                tree->accept( texvis );

                // have to enimilate the scale matrix:
                osgUtil::Optimizer::FlattenStaticTransformsVisitor flatten;
                tree->accept( flatten );

                int count = TREE_COUNT;

                //osg::BoundingBox bbox(-250,-250,-250,250,250,250);
                osg::BoundingBox bbox; //(-1e7,-1e7,-1e7,1e7,1e7,1e7);
                DrawInstanced::ConvertToDrawInstanced cdi( count, bbox, true );
                tree->accept( cdi );

                osg::ref_ptr<StateSetCache> cache = new StateSetCache();
                Registry::shaderGenerator().run(tree.get(), cache.get());

                VirtualProgram* vp = VirtualProgram::getOrCreate( tree->getOrCreateStateSet() );

                vp->setFunction("oe_trees_vert", vs, ShaderComp::LOCATION_VERTEX_MODEL);

                //vp->setFunction("oe_trees_frag", fs, ShaderComp::LOCATION_FRAGMENT_COLORING, 1.0f);

                //tree->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
                //tree->getOrCreateStateSet()->setMode(GL_BLEND, 1);
            }
        }

        return tree.get();
    };
}






//----------------------------------------------------------------------------

TileNode::TileNode(const TileKey& key, const TileModel* model, const osg::Matrixd& matrix) :
_key               ( key ),
_model             ( model ),
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    this->setName( key.str() );
    this->setMatrix( matrix );

    // revisions are initially in sync:
    if ( model )
    {
        _maprevision = model->_revision;
        if ( model->requiresUpdateTraverse() )
        {
            this->setNumChildrenRequiringUpdateTraversal(1);
        }
    }

    if ( key.getLOD() >= TREE_LOD && _model.valid() )
    {
        int delta = key.getLOD() - TREE_LOD;

        this->addChild( makeTrees(delta) );

        osg::StateSet* ss = getOrCreateStateSet();

        // first, a rotation vector to make trees point up.
        GeoPoint p;
        key.getExtent().getCentroid(p);
        osg::Vec3d up;
        p.createWorldUpVector(up);
        osg::Quat q;
        q.makeRotate(osg::Vec3d(0,0,1), up);
        osg::Matrixd zup = osg::Matrixd::rotate(q);

        // matrices to resolve the weird terrain localization into a usable LTP.
        osg::Matrix tile2world = getMatrix();
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
        //OE_INFO << "tile " << key.str() << " w = " << w << ", h = " << h << std::endl;
        
        ss->setTextureAttributeAndModes(2, _model->_elevationTexture.get(), 1);
        //ss->addUniform( new osg::Uniform("oe_terrain_tex", 2) );

        if ( model->_elevationData.getLocator() )
        {
            osg::Matrixd elevMatrix;
            model->_tileLocator->createScaleBiasMatrix(
                model->_elevationData.getLocator()->getDataExtent(),
                elevMatrix);
            ss->addUniform( new osg::Uniform("oe_terrain_tex_matrix", osg::Matrixf(elevMatrix)) );
        }
    }
}


void
TileNode::setLastTraversalFrame(unsigned frame)
{
    _lastTraversalFrame = frame;
}


void
TileNode::traverse( osg::NodeVisitor& nv )
{
    if ( _model.valid() )
    {
        if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
            if (ccc)
            {
                if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
            }

            // if this tile is marked dirty, bump the marker so the engine knows it
            // needs replacing.
            if ( _dirty || _model->_revision != _maprevision )
            {
                _outOfDate = true;
            }
        }
        else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
        {
            _model->updateTraverse(nv);
        }
    }    

    osg::MatrixTransform::traverse( nv );
}

void
TileNode::releaseGLObjects(osg::State* state) const
{
    osg::MatrixTransform::releaseGLObjects( state );

    if ( _model.valid() )
        _model->releaseGLObjects( state );
}

void
TileNode::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::MatrixTransform::resizeGLObjectBuffers( maxSize );

    if ( _model.valid() )
        const_cast<TileModel*>(_model.get())->resizeGLObjectBuffers( maxSize );
}
