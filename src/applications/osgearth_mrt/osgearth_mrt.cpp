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

#include <osg/Notify>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/TextureRectangle>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgEarth/VirtualProgram>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

// shared data.
struct App
{
    osg::TextureRectangle* gcolor;
    osg::TextureRectangle* gnormal;
    osg::TextureRectangle* gdepth;
};


osg::Node*
createMRTPass(App& app, osg::Node* sceneGraph)
{
    osg::Camera* rtt = new osg::Camera();
    rtt->setRenderOrder(osg::Camera::PRE_RENDER);
    rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    rtt->setViewport(0, 0, app.gcolor->getTextureWidth(), app.gcolor->getTextureHeight());
    rtt->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    rtt->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0), app.gcolor);
    rtt->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER1), app.gnormal);
    rtt->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER2), app.gdepth);
    rtt->setCullingMode(rtt->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING); 

    static const char* vertSource =
        "out float mrt_depth;\n"
        "void oe_mrt_vertex(inout vec4 vertexClip)\n"
        "{\n"
        "    mrt_depth = (vertexClip.z/vertexClip.w)*0.5+1.0;\n"
        "}\n";

    static const char* fragSource =
        "in float mrt_depth;\n"
        "in vec3 vp_Normal; \n"
        "void oe_mrt_fragment(inout vec4 color)\n"
        "{\n"
        "    gl_FragData[0] = color; \n"
        "    gl_FragData[1] = vec4((vp_Normal+1.0)/2.0,1.0);\n"
        "    gl_FragData[2] = vec4(mrt_depth,mrt_depth,mrt_depth,1.0); \n"
        "}\n";

    VirtualProgram* vp = VirtualProgram::getOrCreate( rtt->getOrCreateStateSet() );
    vp->setFunction( "oe_mrt_vertex",   vertSource, ShaderComp::LOCATION_VERTEX_CLIP );
    vp->setFunction( "oe_mrt_fragment", fragSource, ShaderComp::LOCATION_FRAGMENT_OUTPUT );

    rtt->addChild( sceneGraph );
    return rtt;
}

osg::Node*
createFramebufferQuad(App& app)
{
    float w = (float)app.gcolor->getTextureWidth();
    float h = (float)app.gcolor->getTextureHeight();

    osg::Geometry* g = new osg::Geometry();
    g->setSupportsDisplayList( false );
    
    osg::Vec3Array* v = new osg::Vec3Array();
    v->push_back(osg::Vec3(-w/2, -h/2, 0));
    v->push_back(osg::Vec3( w/2, -h/2, 0));
    v->push_back(osg::Vec3( w/2,  h/2, 0));
    v->push_back(osg::Vec3(-w/2,  h/2, 0));
    g->setVertexArray(v);

    osg::Vec2Array* t = new osg::Vec2Array();
    t->push_back(osg::Vec2(0,0));
    t->push_back(osg::Vec2(w,0));
    t->push_back(osg::Vec2(w,h));
    t->push_back(osg::Vec2(0,h));
    g->setTexCoordArray(0, t);

    osg::Vec4Array* c = new osg::Vec4Array();
    c->push_back(osg::Vec4(1,1,1,1));
    g->setColorArray(c);
    g->setColorBinding(osg::Geometry::BIND_OVERALL);

    g->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( g );

    return geode;
}

osg::Node*
createFramebufferPass(App& app)
{
    osg::Node* quad = createFramebufferQuad(app);
    
    osg::StateSet* stateset = quad->getOrCreateStateSet();

    static const char* vertSource =
        "varying vec4 texcoord;\n"
        "void effect_vert(inout vec4 vertexView)\n"
        "{\n"
        "    texcoord = gl_MultiTexCoord0; \n"
        "}\n";

    static const char* fragSource =
        "#version " GLSL_VERSION_STR "\n"
        "#extension GL_ARB_texture_rectangle : enable\n"
        "uniform sampler2DRect gcolor;\n"
        "uniform sampler2DRect gnormal;\n"
        "uniform sampler2DRect gdepth;\n"
        "varying vec4 texcoord;\n"

        "void effect_frag(inout vec4 color)\n"
        "{\n"
        "    color = texture2DRect(gcolor, texcoord.st); \n"
        "    float depth = texture2DRect(gdepth, texcoord.st).r; \n"
        "    vec3 normal = texture2DRect(gnormal,texcoord.st).xyz *2.0-1.0; \n"

        // sample radius in pixels:
        "    float e = 5.0; \n"

        // sample the normals around our pixel and find the approximate
        // deviation from our center normal:
        "    vec3 avgNormal =\n"
        "       texture2DRect(gnormal, texcoord.st+vec2( e, e)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2(-e, e)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2(-e,-e)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2( e,-e)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2( 0, e)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2( e, 0)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2( 0,-e)).xyz + \n"
        "       texture2DRect(gnormal, texcoord.st+vec2(-e, 0)).xyz;  \n"
        "    avgNormal = normalize((avgNormal/8.0)*2.0-1.0); \n"
        "    float deviation = clamp(dot(normal, avgNormal),0.0,1.0); \n"

        // set a blur factor based on the normal deviation, so that we
        // blur more around edges.
        "    e = 2.5 * (1.0-deviation); \n"

        "    vec4 blurColor = \n"
        "       color + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2( e, e)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2(-e, e)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2(-e,-e)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2( e,-e)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2( 0, e)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2( e, 0)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2( 0,-e)) + \n"
        "       texture2DRect(gcolor, texcoord.st+vec2(-e, 0));  \n"
        "    blurColor /= 9.0; \n"

        // blur the color and darken the edges at the same time
        "    color.rgb = blurColor.rgb * deviation; \n"
        "}\n";

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setFunction("effect_vert", vertSource, ShaderComp::LOCATION_VERTEX_VIEW);
    vp->setFunction("effect_frag", fragSource, ShaderComp::LOCATION_FRAGMENT_COLORING);

    stateset->setTextureAttributeAndModes(0, app.gcolor, 1);
    stateset->addUniform(new osg::Uniform("gcolor", 0));
    stateset->setTextureAttributeAndModes(1, app.gnormal, 1);
    stateset->addUniform(new osg::Uniform("gnormal", 1));
    stateset->setTextureAttributeAndModes(2, app.gdepth, 1);
    stateset->addUniform(new osg::Uniform("gdepth", 2));
    stateset->setMode( GL_LIGHTING, 0 );

    float w = app.gcolor->getTextureWidth();
    float h = app.gcolor->getTextureHeight();

    osg::Camera* camera = new osg::Camera();
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    camera->setViewMatrix( osg::Matrix::identity() );
    camera->setProjectionMatrixAsOrtho2D( -w/2, (-w/2)+w, -h/2, (-h/2)+h );

    camera->addChild( quad );
    return camera;
}


void
createRenderTargets(App& app, unsigned width, unsigned height)
{
    app.gcolor = new osg::TextureRectangle();
    app.gcolor->setTextureSize(width, height);
    app.gcolor->setInternalFormat(GL_RGBA);
    app.gcolor->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    app.gcolor->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    app.gnormal = new osg::TextureRectangle();
    app.gnormal->setTextureSize(width, height);
    app.gnormal->setInternalFormat(GL_RGB);
    app.gnormal->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
    app.gnormal->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

    app.gdepth = new osg::TextureRectangle();
    app.gdepth->setTextureSize(width, height);
    app.gdepth->setInternalFormat(GL_LUMINANCE);
    app.gdepth->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
    app.gdepth->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);
}


int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

struct RTTIntersectionTest : public osgGA::GUIEventHandler
{
    osgViewer::View* _view;
    osg::Node*       _node;

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
    {
        if ( ea.getEventType() == ea.PUSH )
        {
            // mouse click from [-1...1]
            float nx = ea.getXnormalized();
            float ny = ea.getYnormalized();

            // clicked point in clip space:
            osg::Vec3d pn( nx, ny, -1 ); // on near plane
            osg::Vec3d pf( nx, ny,  1 ); // on far plane

            OE_NOTICE << "clip: nx=" << nx << ", ny=" << ny << std::endl;
            
            // take the view matrix as-is:
            osg::Matrix view = _view->getCamera()->getViewMatrix();

            // adjust projection matrix to include entire earth:
            double fovy, ar, zn, zf;
            _view->getCamera()->getProjectionMatrix().getPerspective(fovy, ar, zn, zf);
            osg::Matrix proj;
            proj.makePerspective(fovy, ar, 1.0, 1e10);

            // Invert the MVP to transform points from clip to model space:
            osg::Matrix MVP = view * proj;
            osg::Matrix invMVP;
            invMVP.invert(MVP);

            pn = pn * invMVP;
            pf = pf * invMVP;

            OE_NOTICE << "model: near = " << pn.x() << ", " << pn.y() << ", " << pn.z() << std::endl;
            OE_NOTICE << "model: far  = " << pf.x() << ", " << pf.y() << ", " << pf.z() << std::endl;

            // Intersect in model space.
            osgUtil::LineSegmentIntersector* lsi = new osgUtil::LineSegmentIntersector(
                osgUtil::Intersector::MODEL, pn, pf );

            lsi->setIntersectionLimit( lsi->LIMIT_NEAREST );

            osgUtil::IntersectionVisitor iv( lsi ); 
            
            _node->accept( iv );

            if ( lsi->containsIntersections() )
            {
                osg::Vec3d p = lsi->getIntersections().begin()->getWorldIntersectPoint();
                OE_NOTICE << "i = " << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
            }
        }
        return false;
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );

    osg::Group* root = new osg::Group();

    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        App app;
        createRenderTargets( app, 1280, 1024 );

        osg::Node* pass1 = createMRTPass(app, node);
        root->addChild( pass1 );

        osg::Node* pass2 = createFramebufferPass(app);
        root->addChild( pass2 );

        // demonstrate scene intersection when using MRT/RTT.
        RTTIntersectionTest* isect = new RTTIntersectionTest();
        isect->_view = &viewer;
        isect->_node = node;
        viewer.addEventHandler( isect );

        viewer.setSceneData( root );

        viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
    return 0;
}
