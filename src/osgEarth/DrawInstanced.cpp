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
#include <osgEarth/DrawInstanced>
#include <osgEarth/CullingUtils>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/StateSetCache>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>
#include <osgEarth/Utils>

#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/UserDataContainer>
#include <osg/LOD>
#include <osgUtil/MeshOptimizers>

#define LC "[DrawInstanced] "

using namespace osgEarth;
using namespace osgEarth::DrawInstanced;

// Ref: http://sol.gfxile.net/instancing.html

#define POSTEX_TEXTURE_UNIT 5
#define POSTEX_MAX_TEXTURE_SIZE 256

#define TAG_MATRIX_VECTOR "osgEarth::DrawInstanced::MatrixRefVector"

//Uncomment to experiment with instance count adjustment
//#define USE_INSTANCE_LODS

//----------------------------------------------------------------------

namespace
{
#ifdef USE_INSTANCE_LODS

    struct LODCallback : public osg::Drawable::DrawCallback
    {
        LODCallback() : _first(true), _maxInstances(0) { }

        void drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const
        {
            const osg::Geometry* geom = drawable->asGeometry();

            if ( _first && geom->getNumPrimitiveSets() > 0 )
            {
                _maxInstances = geom->getPrimitiveSet(0)->getNumInstances();
                _first = false;
            }

            const osg::BoundingBox bbox = Utils::getBoundingBox(geom);
            float radius = bbox.radius();

            osg::Vec3d centerView = bbox.center() * ri.getState()->getModelViewMatrix();
            float rangeToBS = (float)-centerView.z() - radius;

#if OSG_MIN_VERSION_REQUIRED(3,3,0)
            // check for inherit mode (3.3.0+ only)
            osg::Camera* cam = ri.getCurrentCamera();

            // Problem: the camera stack is *always* size=1. So no access to the ref cam.
            if (cam->getReferenceFrame() == cam->ABSOLUTE_RF_INHERIT_VIEWPOINT &&
                ri.getCameraStack().size() > 1)
            {
                osg::Camera* refCam = *(ri.getCameraStack().end()-2);
                if ( refCam )
                {
                    osg::Vec3d centerWorld = centerView * cam->getInverseViewMatrix();
                    osg::Vec3d centerRefView = centerWorld * refCam->getViewMatrix();
                    rangeToBS = (float)(-centerRefView.z() - radius);
                }
            }
#endif

            // these should obviously be programmable
            const float maxDistance = 2000.0f;
            const float minDistance = 100.0f;

            float ratio = (rangeToBS-minDistance)/(maxDistance-minDistance);
            ratio = 1.0 - osg::clampBetween(ratio, 0.0f, 1.0f);
            // 1 = closest, 0 = farthest

            unsigned instances = (unsigned)(ratio*(float)_maxInstances);

            if ( instances > 0 )
            {
                for(unsigned i=0; i<geom->getNumPrimitiveSets(); ++i)
                {
                    const osg::PrimitiveSet* ps = geom->getPrimitiveSet(i);
                    const_cast<osg::PrimitiveSet*>(ps)->setNumInstances(instances);
                }

                drawable->drawImplementation(ri);
            }
        }

        mutable bool     _first;
        mutable unsigned _maxInstances;
    };
#endif // USE_INSTANCE_LODS

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

    // assume x is positive
    int nextPowerOf2(int x)
    {
        --x;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x+1;
    }

    osg::Vec2f calculateIdealTextureSize(unsigned numMats, unsigned maxNumVec4sPerSpan)
    {
        unsigned numVec4s = 4 * numMats;

        bool npotOK = false; //Registry::capabilities().supportsNonPowerOfTwoTextures();
        if ( npotOK )
        {
            unsigned cols = std::min(numVec4s, maxNumVec4sPerSpan);
            unsigned rows = 
                cols < maxNumVec4sPerSpan ? 1 : 
                numVec4s % cols == 0 ? numVec4s / cols :
                1 + (numVec4s / cols);
            return osg::Vec2f( (float)cols, (float)rows );
        }
        else
        {
            // start with a square:
            int x = (int)ceil(sqrt((float)numVec4s));

            // round the x dimension up to a power of 2:
            x = nextPowerOf2( x );

            // recalculate the necessary rows, given the new column count:
            int y = numVec4s % x == 0 ? numVec4s/x : 1 + (numVec4s/x);
            y = nextPowerOf2( y );

            return osg::Vec2f((float)x, (float)y);
        }
    }
}

//----------------------------------------------------------------------


ConvertToDrawInstanced::ConvertToDrawInstanced(unsigned                numInstances,
                                               const osg::BoundingBox& bbox,
                                               bool                    optimize ) :
_numInstances    ( numInstances ),
_optimize        ( optimize )
{
    setTraversalMode( TRAVERSE_ALL_CHILDREN );
    setNodeMaskOverride( ~0 );

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
                // activate VBOs
                geom->setUseDisplayList( false );
                geom->setUseVertexBufferObjects( true );
            }

            geom->setComputeBoundingBoxCallback( _staticBBoxCallback.get() ); 
            geom->dirtyBound();

            // convert to use DrawInstanced
            for( unsigned p=0; p<geom->getNumPrimitiveSets(); ++p )
            {
                osg::PrimitiveSet* ps = geom->getPrimitiveSet(p);
                ps->setNumInstances( _numInstances );
                _primitiveSets.push_back( ps );
            }

#ifdef USE_INSTANCE_LODS
            geom->setDrawCallback( new LODCallback() );
#endif
        }
    }

    traverse(geode);
}


void
ConvertToDrawInstanced::apply(osg::LOD& lod)
{
    // find the highest LOD:
    int   minIndex = 0;
    float minRange = FLT_MAX;
    for(unsigned i=0; i<lod.getNumRanges(); ++i)
    {
        if ( lod.getRangeList()[i].first < minRange )
        {
            minRange = lod.getRangeList()[i].first;
            minIndex = i;
        }
    }

    // remove all but the highest:
    osg::ref_ptr<osg::Node> highestLOD = lod.getChild( minIndex );
    lod.removeChildren( 0, lod.getNumChildren() );

    // add it back with a full range.
    lod.addChild( highestLOD.get(), 0.0f, FLT_MAX );

    traverse(lod);
}


void
DrawInstanced::install(osg::StateSet* stateset)
{
    if ( !stateset )
        return;

    // simple vertex program to position a vertex based on its instance
    // matrix, which is stored in a texture.
     std::string src_vert = Stringify() 
        << "#version 120 \n" 
        << "#extension GL_EXT_gpu_shader4 : enable \n" 
        << "#extension GL_ARB_draw_instanced: enable \n" 
        << "uniform sampler2D oe_di_postex; \n" 
        << "uniform vec2 oe_di_postex_size; \n" 
        << "void oe_di_setInstancePosition(inout vec4 VertexMODEL) \n" 
        << "{ \n" 
        << "    float index = float(4 * gl_InstanceID) / oe_di_postex_size.x; \n" 
        << "    float s = fract(index); \n" 
        << "    float t = floor(index)/oe_di_postex_size.y; \n" 
        << "    float step = 1.0 / oe_di_postex_size.x; \n"  // step from one vec4 to the next 
        << "    vec4 m0 = texture2D(oe_di_postex, vec2(s, t)); \n" 
        << "    vec4 m1 = texture2D(oe_di_postex, vec2(s+step, t)); \n" 
        << "    vec4 m2 = texture2D(oe_di_postex, vec2(s+step+step, t)); \n" 
        << "    vec4 m3 = texture2D(oe_di_postex, vec2(s+step+step+step, t)); \n" 
        << "    VertexMODEL = VertexMODEL * mat4(m0, m1, m2, m3); \n" // why??? 
        << "} \n"; 

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

    vp->setFunction(
        "oe_di_setInstancePosition",
        src_vert,
        ShaderComp::LOCATION_VERTEX_MODEL );

    stateset->getOrCreateUniform("oe_di_postex", osg::Uniform::SAMPLER_2D)->set(POSTEX_TEXTURE_UNIT);
}


void
DrawInstanced::remove(osg::StateSet* stateset)
{
    if ( !stateset )
        return;

    VirtualProgram* vp = VirtualProgram::get(stateset);
    if ( !vp )
        return;

    vp->removeShader( "oe_di_setInstancePosition" );

    stateset->removeUniform("oe_di_postex");
    stateset->removeUniform("oe_di_postex_size");
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
            // we have to deep-copy the primitives because we're going to convert them
            // to use draw-instancing.
            osg::Node* n = mt->getChild(0);
            models[n].push_back( mt->getMatrix() );
        }
    }

    // get rid of the old matrix transforms.
    parent->removeChildren(0, parent->getNumChildren());

    // maximum size of a slice.
    unsigned maxTexSize   = POSTEX_MAX_TEXTURE_SIZE;
    unsigned maxSliceSize = (maxTexSize*maxTexSize)/4; // 4 vec4s per matrix.

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

        // If the number of instances is not an exact multiple of the number of slices,
        // replicate the node so we can draw a difference instance count in the final group.
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

        // Assign matrix vectors to the nodes, so the application can easily retrieve
        // the original position data if necessary.
        MatrixRefVector* nodeMats = new MatrixRefVector();
        nodeMats->setName(TAG_MATRIX_VECTOR);
        nodeMats->reserve(lastNode != node ? sliceSize*(numSlices-1) : sliceSize*numSlices);
        node->getOrCreateUserDataContainer()->addUserObject(nodeMats);

        // ...and a separate one for lastNode if necessary
        MatrixRefVector* lastNodeMats = 0L;
        if (lastNode != node)
        {
            lastNodeMats = new MatrixRefVector();
            lastNodeMats->setName(TAG_MATRIX_VECTOR);
            lastNodeMats->reserve(lastSliceSize);
            lastNode->getOrCreateUserDataContainer()->addUserObject(lastNodeMats);
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

            // calculate the ideal texture size for this slice:
            osg::Vec2f texSize = calculateIdealTextureSize(currentSize, maxTexSize);
            OE_DEBUG << LC << "size = " << currentSize << ", tex = " << texSize.x() << ", " << texSize.y() << std::endl;

            // sampler that will hold the instance matrices:
            osg::Image* image = new osg::Image();
            image->setName("osgearth.drawinstanced.postex");
            image->allocateImage( (int)texSize.x(), (int)texSize.y(), 1, GL_RGBA, GL_FLOAT );

            osg::Texture2D* postex = new osg::Texture2D( image );
            postex->setInternalFormat( GL_RGBA16F_ARB );
            postex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
            postex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
            postex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP );
            postex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP );
            postex->setUnRefImageDataAfterApply( true );
            if ( !ImageUtils::isPowerOfTwo(image) )
                postex->setResizeNonPowerOfTwoHint( false );

            // Tell the SG to skip the positioning texture.
            ShaderGenerator::setIgnoreHint(postex, true);

            osg::StateSet* stateset = sliceGroup->getOrCreateStateSet();
            stateset->setTextureAttributeAndModes(POSTEX_TEXTURE_UNIT, postex, 1);
            stateset->getOrCreateUniform("oe_di_postex_size", osg::Uniform::FLOAT_VEC2)->set(texSize);

            // could use PixelWriter but we know the format.
            GLfloat* ptr = reinterpret_cast<GLfloat*>( image->data() );
            for(unsigned m=0; m<currentSize; ++m)
            {
                const osg::Matrixf& mat = matrices[offset + m];
                for(int col=0; col<4; ++col)
                    for(int row=0; row<4; ++row)
                        *ptr++ = mat(row,col);

                // store them int the metadata as well
                if (currentNode == node)
                    nodeMats->push_back(mat);
                else
                    lastNodeMats->push_back(mat);
            }

            // add the node as a child:
            sliceGroup->addChild( currentNode );

            parent->addChild( sliceGroup );
        }
    }
}


const DrawInstanced::MatrixRefVector*
DrawInstanced::getMatrixVector(osg::Node* node)
{
    if ( !node )
        return 0L;

    osg::UserDataContainer* udc = node->getUserDataContainer();
    if ( !udc )
        return 0L;

    osg::Object* obj = udc->getUserObject(TAG_MATRIX_VECTOR);
    if ( !obj )
        return 0L;

    // cast is safe because of our unique tag
    return static_cast<const MatrixRefVector*>( obj );
}
