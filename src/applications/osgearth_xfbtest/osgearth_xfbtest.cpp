/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osg/Version>
#include <osg/Notify>

#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osg/Geometry>
#include <osg/Depth>
#include <osg/Point>
#include <osg/VertexAttribDivisor>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ShaderLoader>



#ifndef GL_TRANSFORM_FEEDBACK_BUFFER
    #define GL_TRANSFORM_FEEDBACK_BUFFER      0x8C8E
#endif

#ifndef TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN
    #define TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN 0x8c88
#endif


using namespace osgEarth;

const char* IG_VS =
    "#version 330\n"
    "in vec4 xfb_position;\n"
    "void oe_instancing_setPos(inout vec4 vertex) { \n"
    "    vertex.xyz += xfb_position.xyz; \n"
    "} \n";

static int s_numInstancesToDraw = 0;

/**
 * Draw callback that goes on the instanced geometry, and draws exactly the number of 
 * instances required (instead of reading the information from PrimitiveSet::getNumInstances)
 */
struct InstanceDrawCallback : public osg::Drawable::DrawCallback
{
    InstanceDrawCallback() { }

    void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
    {
        const osg::Geometry* geom = static_cast<const osg::Geometry*>(drawable);
        geom->drawVertexArraysImplementation(renderInfo);
        for(int i=0; i<geom->getNumPrimitiveSets(); ++i)
        {
            const osg::PrimitiveSet* pset = geom->getPrimitiveSet(i);
            const osg::DrawArrays* da = dynamic_cast<const osg::DrawArrays*>(pset);
            if ( da )
            {
                renderInfo.getState()->glDrawArraysInstanced( da->getMode(), da->getFirst(), da->getCount(), s_numInstancesToDraw);
            }
            else
            {
                const osg::DrawElements* de = dynamic_cast<const osg::DrawElements*>(pset);
                if ( de )
                {
                    GLenum dataType = const_cast<osg::DrawElements*>(de)->getDataType();
                    renderInfo.getState()->glDrawElementsInstanced( de->getMode(), de->getNumIndices(), dataType, de->getDataPointer(), s_numInstancesToDraw );
                }
            }
        }
    }
};


struct InstanceGroup : public osg::Group
{
    InstanceGroup()
    {
        osg::StateSet* ss = getOrCreateStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        vp->setFunction("oe_instancing_setPos", IG_VS, ShaderComp::LOCATION_VERTEX_MODEL, -FLT_MAX);

        _xfb = new osg::Vec4Array();
        _xfb->resizeArray( 1u );

        // In practice, we will pre-set a bounding sphere/box for a tile
        this->setCullingActive(false);
    }

    void configure(int slot, unsigned numInstances)
    {
        _slot = slot;
        _numInstances = numInstances;

        osg::StateSet* ss = getOrCreateStateSet();

        _xfb->resizeArray( numInstances );

        VirtualProgram* vp = VirtualProgram::get(ss);
        
        vp->removeBindAttribLocation( "xfb_position" );
        vp->addBindAttribLocation( "xfb_position", _slot );

        ss->setAttribute( new osg::VertexAttribDivisor(_slot, 1) );

        _drawCallback = new InstanceDrawCallback();

        Setup setup(this);
        this->accept( setup );
    }

    osg::Array* getControlPoints() const
    {
        return _xfb.get();
    }

    osg::BoundingSphere computeBound() const
    {
        osg::BoundingSphere bs;

        const osg::Vec4Array* points = static_cast<const osg::Vec4Array*>(_xfb.get());
        osg::BoundingSphere instanceBS = osg::Group::computeBound();
        for(int i=0; i<points->getNumElements(); ++i)
        {
            osg::Vec3f center( (*points)[i].x(), (*points)[i].y(), (*points)[i].z() );
            bs.expandBy( osg::BoundingSphere(center + instanceBS.center(), instanceBS.radius()) );
        }

        OE_WARN << "BS = " << bs.center().x() << ", " << bs.center().y() << ", " << bs.center().z() << ", r=" << bs.radius() << "\n";
        return bs;
    }

    struct Setup : public osg::NodeVisitor 
    {
        Setup(InstanceGroup* ig) : _ig(ig), osg::NodeVisitor()
        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            this->setNodeMaskOverride( ~0 );
            _count = 0;
        }

        void apply(osg::Geode& geode)
        {
            for(int i=0; i<geode.getNumDrawables(); ++i)
            {
                osg::Geometry* g = geode.getDrawable(i)->asGeometry();
                if ( g )
                {
                    // Must ensure that VBOs are used:
                    g->setUseVertexBufferObjects(true);
                    g->setUseDisplayList(false);

                    // Bind our transform feedback buffer to the geometry:
                    g->setVertexAttribArray    ( _ig->_slot, _ig->_xfb.get() );
                    g->setVertexAttribBinding  ( _ig->_slot, g->BIND_PER_VERTEX );
                    g->setVertexAttribNormalize( _ig->_slot, false );

                    // Set up a draw callback to intecept draw calls so we can vary 
                    // the instance count per frame.
                    g->setDrawCallback( _ig->_drawCallback.get() );

                    // disable frustum culling because the instance doesn't have a real location
                    g->setCullingActive(false);

                    ++_count;
                }
            }
        }

        InstanceGroup* _ig;
        int _count;
    };
    
    unsigned _numInstances;
    int _slot;
    osg::ref_ptr<osg::Array> _xfb;
    osg::ref_ptr<osg::Drawable::DrawCallback> _drawCallback;
};


#define XFB_SLOT 7

// Generator that just writes each incoming vertex to the XFB.
const char* VS_GENERATOR =
    "#version " GLSL_VERSION_STR "\n"
    "out vec4 position; \n"
    "void main(void) { \n"
    "    position = gl_Vertex; \n"
    "} \n";

// Geometry shader perform GPU culling on the control point set.
// The "cullingRadius" bit isn't working quite right; not sure why atm.
const char* GS_GENERATOR =
    "#version " GLSL_VERSION_STR "\n"
    "layout(points) in; \n"
    "layout(points, max_vertices=1) out; \n"
    "uniform float cullingRadius; \n"
    "in vec4 position[1]; \n"
    "out vec4 xfb_output; \n"
    "void main(void) { \n"
    "    vec4 vertView = gl_ModelViewMatrix * position[0]; \n"
    "    vertView.xy -= cullingRadius*sign(vertView.xy); \n"
    "    vec4 vertClip = gl_ProjectionMatrix * vertView; \n"
    "    float w = abs(vertClip.w); \n"
    "    if ( abs(vertClip.x) <= w && abs(vertClip.y) <= w && abs(vertClip.z) <= w ) { \n"
    "        xfb_output = position[0]; \n"
    "        EmitVertex(); \n"
    "        EndPrimitive(); \n"
    "    } \n"
    "} \n";

osg::Program* makeGeneratorProgram()
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader(osg::Shader::VERTEX, VS_GENERATOR) );
    program->addShader( new osg::Shader(osg::Shader::GEOMETRY, GS_GENERATOR) );
    program->addTransformFeedBackVarying( "xfb_output" );
    program->setTransformFeedBackMode( GL_INTERLEAVED_ATTRIBS );

    return program;
}

void makeVisibleVP(osg::StateSet* ss)
{
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setFunction("oe_instancing_setPos", IG_VS, ShaderComp::LOCATION_VERTEX_MODEL, -FLT_MAX);
    vp->addBindAttribLocation( "xfb_position", XFB_SLOT );
}


struct RenderToXFB : public osg::Drawable::DrawCallback
{
    /**
     * Constructs a Transform Feedback draw callback.
     *
     * @param[in ] xfb Transform Feedback control points
     *
     * NOTE: You must add the XF buffer array to its final geometry (or geometries)
     * before creating this callback. Otherwise, it will not be able to calculate
     * a correct offset into the VBO.
     */
    RenderToXFB(osg::Array* xfb)
    {
        _vbo = xfb->getVertexBufferObject();
        int numBufferData = _vbo->getNumBufferData();
        _offset = 0;
        _size = 0;
        for(int i=0; i<numBufferData; ++i)
        {
            _size = _vbo->getBufferData(i)->getTotalDataSize();
            if ( _vbo->getBufferData(i)->getDataPointer() == xfb->getDataPointer() )
                break;
            else
                _offset += _size;
        }
        OE_DEBUG << "offset = " << _offset << "; size = " << _size << "\n";

        _queries.resize(64);
        _queries.setAllElementsTo( INT_MAX );
    }
    
    void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
    {
        unsigned contextID = renderInfo.getState()->getContextID();
        osg::GLBufferObject* bo = _vbo->getOrCreateGLBufferObject(contextID);
        if ( bo->isDirty() )
            bo->compileBuffer();

        GLuint objID = bo->getGLObjectID();

        osg::GLExtensions* ext = renderInfo.getState()->get<osg::GLExtensions>();
           
        GLuint numPrims = 0;
        GLuint& query = _queries[contextID];
        if ( query == INT_MAX )
        {
            ext->glGenQueries(1, &query);
        }
        else
        {     
        
            ext->glGetQueryObjectuiv(query, GL_QUERY_RESULT, &numPrims);        
            s_numInstancesToDraw = numPrims;
        }


        ext->glBindBufferRange(GL_TRANSFORM_FEEDBACK_BUFFER, 0, objID, _offset, _size);
        //if ( renderInfo.getState()->checkGLErrors("PRE:glBindBufferBase") ) exit(0);

        glEnable(GL_RASTERIZER_DISCARD);
        //if ( renderInfo.getState()->checkGLErrors("PRE:glEnable(GL_RASTERIZER_DISCARD)") ) exit(0);


        ext->glBeginQuery(TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query);

        ext->glBeginTransformFeedback(GL_POINTS); // get from input geom?
        //if ( renderInfo.getState()->checkGLErrors("PRE:glBeginTransformFeedback(GL_POINTS)") ) exit(0);
        

        drawable->drawImplementation(renderInfo);

        
        ext->glEndTransformFeedback();
        //if ( renderInfo.getState()->checkGLErrors("POST:glEndTransformFeedback") ) exit(0);
        ext->glEndQuery(TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

        glDisable(GL_RASTERIZER_DISCARD);
        //if ( renderInfo.getState()->checkGLErrors("POST:glDisable(GL_RASTERIZER_DISCARD)") ) exit(0);

        ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 0);
        //if ( renderInfo.getState()->checkGLErrors("POST:glBindBufferBase") ) exit(0);

    }
    osg::ref_ptr<osg::BufferObject> _vbo;
    int _offset, _size;
    mutable osg::buffered_object<GLuint> _queries;
};



osg::Geometry* makeGeneratorGeometry(osg::Array* xfb, int dim, float width)
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);

    // input points -- 
    osg::Vec3Array* verts = new osg::Vec3Array();
    for(int i=0; i<dim; ++i)
        for(int j=0; j<dim; ++j)
            verts->push_back(osg::Vec3(i*width, j*width, 0));
    geom->setVertexArray( verts );

    osg::DrawArrays* da = new osg::DrawArrays(GL_POINTS, 0, dim*dim);
    geom->addPrimitiveSet( da );

    // Add a custom draw callback that will render the control points 
    // into the the XF buffer. (Must add this callbacks AFTER setting up the geometry)
    geom->setDrawCallback( new RenderToXFB(xfb) );

    // Still trying to figure out exactly how this geometry gets culled -gw
    // If you zoom to the middle of the field, it culls.
    geom->setCullingActive(false);

    return geom;
}

osg::Node* makeSceneGraph()
{
    osg::Group* root = new osg::Group();
    root->getOrCreateStateSet()->setRenderBinDetails(0, "TraversalOrderBin");
    //root->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(10.0f));
    //root->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LESS, 0, 1, false));
    
    // Model to instance:
    osg::Node* instancedModel = osgDB::readNodeFile("../data/tree.ive.osgearth_shadergen");
    float radius = instancedModel->getBound().radius();

    // Reference axis.
    //std::string axis = Stringify() << "../data/axes.osgt.(" << radius << ").scale";
    //root->addChild( osgDB::readNodeFile(axis) );

    const int dim = 256;
    const int maxNumInstances = dim*dim;
    OE_NOTICE "Rendering " << maxNumInstances << " instances; radius = " << radius << "\n";

    // This is the group containing the instanced model. The "configure" method will set up
    // the instance model for dynamic XFB instancing.
    InstanceGroup* ig = new InstanceGroup();
    ig->addChild( instancedModel );
    ig->configure( XFB_SLOT, maxNumInstances );
    root->addChild( ig );

    // construct the geometry that will generate the instancing control points and render them
    // into the Transform Feedback buffer.
    osg::Geometry* genGeom = makeGeneratorGeometry( ig->getControlPoints(), dim, radius );
    genGeom->getOrCreateStateSet()->setAttribute( makeGeneratorProgram() );
    genGeom->getOrCreateStateSet()->addUniform( new osg::Uniform("cullingRadius", radius) ); // since we don't know where ther center point is
    osg::Geode* genGeode = new osg::Geode();
    genGeode->addDrawable( genGeom );    
    
    root->addChild( ig );
    root->addChild( genGeode );

    return root;
}

int
main(int argc, char** argv)
{
    osgEarth::Registry::capabilities();

    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new osgGA::TrackballManipulator() );

    viewer.addEventHandler( new osgViewer::StatsHandler() );    
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    viewer.setSceneData( makeSceneGraph() );

    viewer.run();
}

#else
int main(int argc, char** argv)
{
    OSG_WARN << "Sorry, but XFB requires at least OSG 3.4.0." << std::endl;
    return -1;
}
#endif