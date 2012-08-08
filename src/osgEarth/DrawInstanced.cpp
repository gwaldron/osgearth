/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osgUtil/MeshOptimizers>

// The maximum size of the per-instance data in the shader is 8192.
// So this should 8192/sizeof(per-instance data) = 8192/sizeof(mat4).
#define MAX_INSTANCES_PER_GROUP 128

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


VirtualProgram*
DrawInstanced::createDrawInstancedProgram()
{
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "DrawInstanced" );

    std::string src = Stringify()
        << "#extension GL_EXT_draw_instanced : enable\n"

        << "uniform mat4 osgearth_InstanceModelMatrix[" << MAX_INSTANCES_PER_GROUP << "];\n" //<< numInstances << "];\n"

        << "void osgearth_setInstancePosition()\n"
        << "{\n"
        << "    gl_Position = gl_ModelViewProjectionMatrix * osgearth_InstanceModelMatrix[gl_InstanceID] * gl_Vertex; \n"
        << "}\n";

    vp->setFunction(
        "osgearth_setInstancePosition",
        src,
        ShaderComp::LOCATION_VERTEX_PRE_COLORING );

    return vp;
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

    // maximum size of a slice:
    unsigned maxSliceSize = MAX_INSTANCES_PER_GROUP;

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

            ArrayUniform uniform(
                "osgearth_InstanceModelMatrix", 
                osg::Uniform::FLOAT_MAT4,
                sliceGroup->getOrCreateStateSet(),
                currentSize );

            // assign the matrices to the uniform array:
            for( unsigned m=0; m < currentSize; ++m )
            {
                uniform.setElement( m, matrices[offset + m] );
            }

            // add the node as a child:
            sliceGroup->addChild( currentNode );

            parent->addChild( sliceGroup );
        }
    }
}
