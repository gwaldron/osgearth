/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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

#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osg/Geometry>
#include <osg/Camera>
#include <osg/BufferIndexBinding>
#include <osg/Point>
#include <osg/TextureBuffer>
#include <osg/VertexAttribDivisor>
#include <osgDB/ReadFile>

#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ShaderLoader>

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

struct InstanceGroup : public osg::Group
{
    InstanceGroup()
    {
        osg::StateSet* ss = getOrCreateStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        //ShaderLoader::load( "", IG_VS, 0L );
        vp->setFunction("oe_instancing_setPos", IG_VS, ShaderComp::LOCATION_VERTEX_MODEL, -FLT_MAX);

        _xfb = new osg::Vec4Array();
        _xfb->resizeArray( 1u );
    }

    void configure(int slot, unsigned numInstances)
    {
        _slot = slot;
        _numInstances = numInstances;

        osg::StateSet* ss = getOrCreateStateSet();

        _xfb->resizeArray( numInstances );

        VirtualProgram* vp = VirtualProgram::get(ss);
        
        //vp->removeBindAttribLocation( "xfb_position" );
        vp->addBindAttribLocation( "xfb_position", _slot );

        ss->setAttribute( new osg::VertexAttribDivisor(_slot, 1) );
        
        Setup setup(this);
        this->accept( setup );
    }

    osg::Array* getControlPoints() const
    {
        return _xfb.get();
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
                    osg::Geometry::PrimitiveSetList& prims = g->getPrimitiveSetList();
                    for(int p=0; p<prims.size(); ++p)
                    {
                        prims[p]->setNumInstances( _ig->_numInstances );
                    }

                    g->setVertexAttribArray    ( _ig->_slot, _ig->_xfb.get() );
                    g->setVertexAttribBinding  ( _ig->_slot, g->BIND_PER_VERTEX );
                    g->setVertexAttribNormalize( _ig->_slot, false );

                    ++_count;
                }
            }
        }

        InstanceGroup* _ig;
        int _count;
    };
    
    unsigned _numInstances;
    int      _slot;
    osg::ref_ptr<osg::Array> _xfb;
};


#define XFB_SLOT 7

// Generator that just writes each incoming vertex to the XFB.
const char* VS_XF =
    "#version 330 compatibility\n"
    "out vec4 xfb_output; \n"
    "void main(void) { \n"
    "    xfb_output = gl_Vertex; \n"
    "} \n";

osg::Program* makeXFProgram()
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader(osg::Shader::VERTEX, VS_XF) );
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


struct XFBDrawCallback : public osg::Drawable::DrawCallback
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
    XFBDrawCallback(osg::Array* xfb)
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

        ext->glBindBufferRange(GL_TRANSFORM_FEEDBACK_BUFFER, 0, objID, _offset, _size);
        if ( renderInfo.getState()->checkGLErrors("PRE:glBindBufferBase") ) exit(0);

        glEnable(GL_RASTERIZER_DISCARD);
        if ( renderInfo.getState()->checkGLErrors("PRE:glEnable(GL_RASTERIZER_DISCARD)") ) exit(0);

        GLuint& query = _queries[contextID];
        if ( query == INT_MAX )
            ext->glGenQueries(1, &query);
        ext->glBeginQuery(TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query);

        ext->glBeginTransformFeedback(GL_POINTS); // get from input geom?
        if ( renderInfo.getState()->checkGLErrors("PRE:glBeginTransformFeedback(GL_POINTS)") ) exit(0);
        

        drawable->drawImplementation(renderInfo);

        
        ext->glEndTransformFeedback();
        if ( renderInfo.getState()->checkGLErrors("POST:glEndTransformFeedback") ) exit(0);
        
        // Query the number of generated points so we can use it to draw instances.
        // NOTE: THIS IS SLOW! Can we frame-defer this, or detect when it may need to change??
        GLuint numPrims = 0;
        ext->glEndQuery(TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);
        ext->glGetQueryObjectuiv(query, GL_QUERY_RESULT, &numPrims);
        
        if ( (renderInfo.getState()->getFrameStamp()->getFrameNumber() % 300) == 0 )
        {
            OE_NOTICE << "num prims = " << numPrims << std::endl;
        }

        glDisable(GL_RASTERIZER_DISCARD);
        if ( renderInfo.getState()->checkGLErrors("POST:glDisable(GL_RASTERIZER_DISCARD)") ) exit(0);

        ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 0);
        if ( renderInfo.getState()->checkGLErrors("POST:glBindBufferBase") ) exit(0);

    }
    osg::ref_ptr<osg::BufferObject> _vbo;
    int _offset, _size;
    mutable osg::buffered_object<GLuint> _queries;
};



osg::Geometry* makeXFGeometry(osg::Array* xfb, int dim, float width)
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);

    // input points -- 
    osg::Vec3Array* verts = new osg::Vec3Array();
    for(int i=0; i<dim; ++i)
        for(int j=0; j<dim; ++j)
            verts->push_back(osg::Vec3(i*width, j*width, 0));
    geom->setVertexArray( verts );

    osg::DrawArrays* da = new osg::DrawArrays(GL_POINTS, 0, dim*dim);
    geom->addPrimitiveSet( da );

    // add callbacks AFTER setting up the geometry
    geom->setDrawCallback( new XFBDrawCallback(xfb) );

    geom->setCullingActive(false);

    return geom;
}

osg::Geometry* makeVisibleGeometry()
{
    const int numInstances = 2;

    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);

    osg::Vec4Array* controlPoints = new osg::Vec4Array();
    controlPoints->resize( numInstances );
    geom->setVertexAttribArray    ( XFB_SLOT, controlPoints );
    geom->setVertexAttribBinding  ( XFB_SLOT, geom->BIND_PER_VERTEX );
    geom->setVertexAttribNormalize( XFB_SLOT, false );

    // geometry to render at each control point:
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->push_back( osg::Vec3(-1, 0, -1) );
    verts->push_back( osg::Vec3( 1, 0, -1) );
    verts->push_back( osg::Vec3( 1, 0,  1) );
    verts->push_back( osg::Vec3(-1, 0,  1) );
    geom->setVertexArray( verts );

    osg::DrawArrays* da = new osg::DrawArrays(GL_QUADS, 0, 4);
    da->setNumInstances( numInstances );
    geom->addPrimitiveSet( da );

    //(*controlPoints)[0].set( -6, 0, 0, 1 );
    //(*controlPoints)[1].set(  6, 0, 0, 1 );
    
    osg::StateSet* ss = geom->getOrCreateStateSet();
    ss->setAttribute( new osg::VertexAttribDivisor(XFB_SLOT, 1u) );

    return geom;
}

osg::Node* makeSceneGraph()
{
    osg::Group* root = new osg::Group();
    root->getOrCreateStateSet()->setRenderBinDetails(0, "TraversalOrderBin");
    root->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(10.0f));
    
    // Mode to instance:
    osg::Node* instancedModel = osgDB::readNodeFile("../data/red_flag.osg"); //.osgearth_shadergen");
    float width = instancedModel->getBound().radius();

    // Reference axis.
    std::string axis = Stringify() << "../data/axes.osgt.(" << width << ").scale";
    root->addChild( osgDB::readNodeFile(axis) );

    const int dim = 8;
    const int numInstances = dim*dim;

    InstanceGroup* ig = new InstanceGroup();
    ig->addChild( instancedModel );
    ig->configure( XFB_SLOT, numInstances );
    root->addChild( ig );

    // construct the geometry that will generate the instancing control points:
    osg::Geometry* xfGeom = makeXFGeometry( ig->getControlPoints(), dim, width );
    xfGeom->getOrCreateStateSet()->setAttribute( makeXFProgram() );
    osg::Geode* xfGeode = new osg::Geode();
    xfGeode->addDrawable( xfGeom );
    
    root->addChild( ig );
    root->addChild( xfGeode );

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

    viewer.setSceneData( makeSceneGraph() );

    viewer.run();
}

#else
int main(int argc, char** argv)
{
    return -1;
}
#endif