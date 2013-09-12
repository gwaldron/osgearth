/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/DrawInstanced>
#include <osgEarth/CullingUtils>
#include <osgEarth/ShaderUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/BufferIndexBinding>
#include <osgUtil/MeshOptimizers>

#define MAX_COUNT_UBO   (Registry::capabilities().getMaxUniformBlockSize()/64)
#define MAX_COUNT_ARRAY 128 // max size of a mat4 uniform array...how to query?

using namespace osgEarth;
using namespace osgEarth::DrawInstanced;


//----------------------------------------------------------------------

namespace
{
    typedef std::map< osg::ref_ptr<osg::Node>, std::vector<osg::Matrix> > ModelNodeMatrices;
    
    /**
     * Simple bbox callback to return a static bbox.
     */
    struct StaticBoundingBox : public osg::Drawable::ComputeBoundingBoxCallback
    {
        osg::BoundingBox _bbox;
        StaticBoundingBox( const osg::BoundingBox& bbox ) : _bbox(bbox) { }
        osg::BoundingBox computeBound(const osg::Drawable&) const { return _bbox; }
    };
}

//----------------------------------------------------------------------


ConvertToDrawInstanced::ConvertToDrawInstanced(unsigned                numInstances,
                                               const osg::BoundingBox& bbox,
                                               bool                    optimize ) :
osg::NodeVisitor ( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_numInstances    ( numInstances ),
_optimize        ( optimize )
{
    _staticBBoxCallback = new StaticBoundingBox(bbox);
}


void 
ConvertToDrawInstanced::apply( osg::Geode& geode )
{
    for( unsigned d=0; d<geode.getNumDrawables(); ++d )
    {
        osg::Geometry* geom = geode.getDrawable(d)->asGeometry();
        if ( geom )
        {
            if ( _optimize )
            {
                // convert to triangles
                osgUtil::IndexMeshVisitor imv;
                imv.makeMesh( *geom );

                // activate VBOs
                geom->setUseDisplayList( false );
                geom->setUseVertexBufferObjects( true );
            }

            geom->setComputeBoundingBoxCallback( _staticBBoxCallback.get() ); 
            geom->dirtyBound();

            // convert to use DrawInstanced
            for( unsigned p=0; p<geom->getNumPrimitiveSets(); ++p )
            {
                geom->getPrimitiveSet(p)->setNumInstances( _numInstances );
            }
        }
    }

    traverse(geode);
}


void
DrawInstanced::install(osg::StateSet* stateset)
{
    if ( !stateset )
        return;

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

    std::stringstream buf;

    buf << "#version 120 \n"
        << "#extension GL_EXT_gpu_shader4 : enable \n";
        //<< "#extension GL_EXT_draw_instanced : enable\n";

    if ( Registry::capabilities().supportsUniformBufferObjects() )
    {
        // note: no newlines in the layout() line, please
        buf << "#extension GL_ARB_uniform_buffer_object : enable\n"
            << "layout(std140) uniform oe_di_modelData { "
            <<     "mat4 oe_di_modelMatrix[" << MAX_COUNT_UBO << "]; } ;\n";

        vp->getTemplate()->addBindUniformBlock( "oe_di_modelData", 0 );
    }
    else
    {
        buf << "uniform mat4 oe_di_modelMatrix[" << MAX_COUNT_ARRAY << "];\n";
    }

    buf << "void oe_di_setPosition(inout vec4 VertexModel)\n"
        << "{\n"
        << "    VertexModel = oe_di_modelMatrix[gl_InstanceID] * VertexModel; \n"
        << "}\n";

    std::string src;
    src = buf.str();

    vp->setFunction(
        "oe_di_setPosition",
        src,
        ShaderComp::LOCATION_VERTEX_MODEL );
}


void
DrawInstanced::remove(osg::StateSet* stateset)
{
    if ( !stateset )
        return;

    VirtualProgram* vp = VirtualProgram::get(stateset);
    if ( !vp )
        return;

    vp->removeShader( "oe_di_setPosition" );
    vp->getTemplate()->removeBindUniformBlock( "oe_di_modelData" );
}


void
DrawInstanced::convertGraphToUseDrawInstanced( osg::Group* parent )
{
    // place a static bounding sphere on the graph since we intend to alter
    // the structure of the subgraph.
    const osg::BoundingSphere& bs = parent->getBound();
    parent->setComputeBoundingSphereCallback( new StaticBound(bs) );
    parent->dirtyBound();

    ModelNodeMatrices models;

    // collect the matrices for all the MT's under the parent. Obviously this assumes
    // a particular scene graph structure.
    for( unsigned i=0; i < parent->getNumChildren(); ++i )
    {
        // each MT in the group parents the same child.
        osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>( parent->getChild(i) );
        if ( mt )
        {
            osg::Node* n = mt->getChild(0);
            models[n].push_back( mt->getMatrix() );
        }
    }

    // get rid of the old matrix transforms.
    parent->removeChildren(0, parent->getNumChildren());

    // whether to use UBOs.
    bool useUBO = Registry::capabilities().supportsUniformBufferObjects();

    // maximum size of a slice.
    // for UBOs, assume 64K / sizeof(mat4) = 1024.
    // for uniform array, assume 8K / sizeof(mat4) = 128.
    unsigned maxSliceSize = useUBO ? MAX_COUNT_UBO : MAX_COUNT_ARRAY;

    // For each model:
    for( ModelNodeMatrices::iterator i = models.begin(); i != models.end(); ++i )
    {
        osg::Node*                node     = i->first.get();
        std::vector<osg::Matrix>& matrices = i->second;

        // calculate the overall bounding box for the model:
        osg::ComputeBoundsVisitor cbv;
        node->accept( cbv );
        const osg::BoundingBox& nodeBox = cbv.getBoundingBox();

        osg::BoundingBox bbox;
        for( std::vector<osg::Matrix>::iterator m = matrices.begin(); m != matrices.end(); ++m )
        {
            osg::Matrix& matrix = *m;
            bbox.expandBy(nodeBox.corner(0) * matrix);
            bbox.expandBy(nodeBox.corner(1) * matrix);
            bbox.expandBy(nodeBox.corner(2) * matrix);
            bbox.expandBy(nodeBox.corner(3) * matrix);
            bbox.expandBy(nodeBox.corner(4) * matrix);
            bbox.expandBy(nodeBox.corner(5) * matrix);
            bbox.expandBy(nodeBox.corner(6) * matrix);
            bbox.expandBy(nodeBox.corner(7) * matrix);
        }

        // calculate slice count and sizes:
        unsigned sliceSize = std::min(matrices.size(), (size_t)maxSliceSize);
        unsigned numSlices = matrices.size() / maxSliceSize;
        unsigned lastSliceSize = matrices.size() % maxSliceSize;
        if ( lastSliceSize == 0 )
            lastSliceSize = sliceSize;
        else
            ++numSlices;

        // Convert the node's primitive sets to use "draw-instanced" rendering; at the
        // same time, assign our computed bounding box as the static bounds for all
        // geometries. (As DI's they cannot report bounds naturally.)
        ConvertToDrawInstanced cdi(sliceSize, bbox, true);
        node->accept( cdi );

        // If we don't have an even number of instance groups, make a smaller last one.
        osg::Node* lastNode = node;
        if ( numSlices > 1 && lastSliceSize < sliceSize )
        {
            // clone, but only make copies of necessary things
            lastNode = osg::clone( 
                node, 
                osg::CopyOp::DEEP_COPY_NODES | osg::CopyOp::DEEP_COPY_DRAWABLES | osg::CopyOp::DEEP_COPY_PRIMITIVES );

            ConvertToDrawInstanced cdi(lastSliceSize, bbox, false);
            lastNode->accept( cdi );
        }

        // Next, break the rendering down into "slices". GLSL will only support a limited
        // amount of pre-instance uniform data, so we have to portion the graph out into
        // slices of no more than this chunk size.
        for( unsigned slice = 0; slice < numSlices; ++slice )
        {
            unsigned   offset      = slice * sliceSize;
            unsigned   currentSize = slice == numSlices-1 ? lastSliceSize : sliceSize;
            osg::Node* currentNode = slice == numSlices-1 ? lastNode      : node;

            // this group is simply a container for the uniform:
            osg::Group* sliceGroup = new osg::Group();

            if ( useUBO ) // uniform buffer object:
            {
                osg::MatrixfArray* mats = new osg::MatrixfArray();
                mats->setBufferObject( new osg::UniformBufferObject() );
                // 64 = sizeof(mat4)
                osg::UniformBufferBinding* ubb = new osg::UniformBufferBinding( 0, mats->getBufferObject(), 0, currentSize * 64 );
                sliceGroup->getOrCreateStateSet()->setAttribute( ubb, osg::StateAttribute::ON );
                for( unsigned m=0; m < currentSize; ++m )
                {
                    mats->push_back( matrices[offset + m] );
                }
                ubb->setDataVariance( osg::Object::DYNAMIC );
            }
            else // just use a uniform array
            {
                // assign the matrices to the uniform array:
                ArrayUniform uniform(
                    "oe_di_modelMatrix", 
                    osg::Uniform::FLOAT_MAT4,
                    sliceGroup->getOrCreateStateSet(),
                    currentSize );

                for( unsigned m=0; m < currentSize; ++m )
                {
                    uniform.setElement( m, matrices[offset + m] );
                }
            }

            // add the node as a child:
            sliceGroup->addChild( currentNode );

            parent->addChild( sliceGroup );
        }
    }
}
