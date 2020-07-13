/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/ImGuiUtils>
#include <osgEarth/OsgImGuiHandler.hpp>
#include <imgui_internal.h>

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/Geocoder>
#include <osgEarth/TMS>
#include <osgEarth/WMS>
#include <osgEarth/GDAL>
#include <osgEarth/XYZ>
#include <osgEarth/Composite>

#include <osgDB/ReadFile>
#include <osg/PointSprite>

#include <iostream>

#include <osgEarth/Metrics>

#define TEXTURE_DIM 2048
//#define TEXTURE_DIM 1024

osg::Node* makeQuad(int width, int height, const osg::Vec4& color)
{
    osg::Geometry *geometry = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->push_back(osg::Vec3(0, 0, 0));
    verts->push_back(osg::Vec3(width, 0, 0));
    verts->push_back(osg::Vec3(width, height, 0));
    verts->push_back(osg::Vec3(0, height, 0));
    geometry->setVertexArray(verts);
    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back(color);
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    auto de = new osg::DrawElementsUByte(GL_TRIANGLES);
    geometry->addPrimitiveSet(de);
    de->push_back(0); de->push_back(1); de->push_back(2);
    de->push_back(0); de->push_back(2); de->push_back(3);

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geometry);
    geode->setCullingActive(false);
    return geode;
}


static const char* particleVertSource =
"#version " GLSL_VERSION_STR "\n"
"uniform sampler2D positionSampler; \n"
"uniform vec2 resolution;\n"
"uniform float pointSize;\n"
"out vec4 particle_color;\n"
"void oe_particle_vertex(inout vec4 vertexModel)\n"
"{\n"
    // Using the instance ID, generate "texture coords" for this instance.
    "vec2 tC; \n"
    "float r = float(gl_InstanceID) / resolution.x; \n"
    "tC.s = fract( r ); tC.t = floor( r ) / resolution.y; \n"

    // Use the (scaled) tex coord to translate the position of the vertices.
    "vec4 posInfo = texture2D( positionSampler, tC );\n"
    "float life = posInfo.w;\n"

    "particle_color = mix(vec4(0.0, 1.0, 0.0, 1.0), vec4(1.0, 0.0, 0.0, 1.0), life);\n"

    "vec3 pos = posInfo.xyz;\n"

    "pos.xy *= vec2(4096, 2048);\n"

    "vertexModel.xyz = pos;\n"

    // Big points
    //"gl_PointSize = mix(3.0, 0.2, life);\n"
    "gl_PointSize = pointSize;\n"
"}\n";

static const char* particleFragSource =
"#version " GLSL_VERSION_STR "\n"
"in vec4 particle_color;\n"
"void oe_particle_frag(inout vec4 color)\n"
"{\n"
"    color = particle_color;\n"
"}\n";

osg::Geometry*
createInstancedGeometry(int nInstances)
{
    osg::Geometry* geom = new osg::Geometry;
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);

    // Points
    osg::Vec3Array* v = new osg::Vec3Array;
    v->resize( 1 );
    geom->setVertexArray( v );
    (*v)[ 0 ] = osg::Vec3( 0.0, 0.0, 0.0 );
    geom->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, 1, nInstances ) );

    VirtualProgram* vp = VirtualProgram::getOrCreate(geom->getOrCreateStateSet());
    vp->setFunction("oe_particle_vertex", particleVertSource, ShaderComp::LOCATION_VERTEX_MODEL);
    vp->setFunction("oe_particle_frag", particleFragSource, ShaderComp::LOCATION_FRAGMENT_COLORING);
    geom->getOrCreateStateSet()->addUniform(new osg::Uniform("positionSampler", 0));
    geom->getOrCreateStateSet()->addUniform(new osg::Uniform("pointSize", 1.0f));
    geom->getOrCreateStateSet()->addUniform(new osg::Uniform("resolution", osg::Vec2f(TEXTURE_DIM, TEXTURE_DIM)));
    geom->getOrCreateStateSet()->setMode(GL_PROGRAM_POINT_SIZE, 1);
    //geom->getOrCreateStateSet()->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);

    return geom;
}

osg::Texture2D* createPositionTexture()
{
    Random random;

    osg::Image* positionImage = new osg::Image;
    positionImage->allocateImage(TEXTURE_DIM, TEXTURE_DIM, 1, GL_RGBA, GL_FLOAT);
    positionImage->setInternalTextureFormat(GL_RGBA32F_ARB);
    GLfloat* ptr = reinterpret_cast<GLfloat*>(positionImage->data());

    for (unsigned int i = 0; i < TEXTURE_DIM * TEXTURE_DIM; i++)
    {
        // Start in the center and eminate out.
        float x = random.next();
        float y = random.next();
        float z = 0.0;

        *ptr++ = x;
        *ptr++ = y;
        *ptr++ = z;
        // Random life
        *ptr++ = random.next();
    }

    osg::Texture2D* tex = new osg::Texture2D(positionImage);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
    tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
    tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    return tex;
}


std::string computeVert =
"#version 330\n"
"void main() \n"
"{ \n"
"    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
"} \n";

std::string computeFrag =
"#version 330\n"
"uniform sampler2D texturePosition; \n"
"uniform sampler2D windMap;\n"
"uniform vec2 resolution;\n"
"uniform float dieSpeed;\n"
"uniform float speedFactor;\n"
"uniform float dropChance;\n"

"uniform float osg_DeltaSimulationTime; \n"
"uniform float osg_SimulationTime; \n"

"layout(location=0) out vec4 out_particle; \n"

// Generate a pseudo-random value in the specified range:
"float\n"
"oe_random(float minValue, float maxValue, vec2 co)\n"
"{\n"
"    float t = fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);\n"
"    return minValue + t*(maxValue-minValue);\n"
"}\n"

"void main() \n"
"{ \n"
"   vec2 uv = gl_FragCoord.xy / resolution.xy;\n"

"   vec4 positionInfo = texture2D( texturePosition, uv );\n"

"   vec3 position = positionInfo.xyz;\n"
"   float life = positionInfo.w;\n"
"   vec2 posUV = position.xy;\n"
"   float uMin = -21.32;\n"
"   float uMax = 26.8;\n"
"   float vMin = -21.57;\n"
"   float vMax = 21.42;\n"
"   vec2 velocity = mix(vec2(uMin, vMin), vec2(uMax, vMax), texture2D(windMap, posUV).rg);\n"
"   float distortion = cos(radians(posUV.y * 180.0 - 90.0));\n"
//"   vec2 offset = vec2(velocity.x / distortion, -velocity.y) * 0.0001 * speedFactor;\n"
"   vec2 offset = vec2(velocity.x / distortion, velocity.y) * 0.0001 * speedFactor;\n"
"   position.x += offset.x;\n"

"   if (position.x >= 1.0) position.x -= 1.0;\n"
"   else if (position.x < 0) position.x += 1.0;\n"

"   position.y += offset.y;\n"
"   if (position.y >= 1.0) position.y -= 1.0;\n"
"   else if (position.y < 0) position.y += 1.0;\n"
"   if (dieSpeed > 0)\n"
"{\n"
"   life -= (osg_DeltaSimulationTime / dieSpeed);\n"
"}\n"

"   vec2 seed = (position.xy + uv);\n"

#if 0
"   float dropChance = 0.0;\n"//oe_random(0.0, 1.0, vec2(posUV.x, posUV.y));\n"
// Reset particle
"   if (life < 0.0 || dropChance > 0.99) {\n"
//"       life = oe_random(0.0, 1.0, vec2(posUV.x, posUV.y));\n"
"       life = 1.0;\n"
"       float x = oe_random(0.0, 1.0, vec2(posUV.x, posUV.y));\n"
"       float y = oe_random(0.0, 1.0, vec2(posUV.y, posUV.x));\n"
"       float z = 0.0;\n;"
"       position = vec3(x, y, z);\n"
"   }\n"
#endif

"   float drop = oe_random(0.0, 1.0, seed);\n"
// Reset particle
"   if (life < 0.0 || dropChance > drop) {\n"
"       life = oe_random(0.0, 1.0, seed + 3.4);\n"
"       float x = oe_random(0.0, 1.0, seed + 1.3);\n"
"       float y = oe_random(0.0, 1.0, seed + 2.1);\n"
"       float z = 0.0;\n;"
"       position = vec3(x, y, z);\n"
"   }\n"

"   out_particle = vec4(position, life);\n"
"} \n";

// Helper node that will run a fragment shader, taking one texture as input and writing to another texture.
// And then it flips on each frame to use the previous input.
class ComputeNode : public osg::Group
{
public:
    ComputeNode(osg::Texture2D* windTexture) :
        _size(TEXTURE_DIM),
        _windTexture(windTexture)
    {
        _inputPosition = createPositionTexture();
        _outputPosition = createPositionTexture();

        buildCamera();
    }

    osg::StateSet* createStateSet()
    {
        osg::Program* program = new osg::Program;
        program->addShader(new osg::Shader(osg::Shader::VERTEX, computeVert));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, computeFrag));
        osg::StateSet* ss = new osg::StateSet;
        ss->setAttributeAndModes(program);

        ss->addUniform(new osg::Uniform("texturePosition", 0));
        ss->addUniform(new osg::Uniform("windMap", 1));

        ss->addUniform(new osg::Uniform("resolution", osg::Vec2f(_size, _size)));

        ss->setTextureAttributeAndModes(0, _inputPosition.get(), osg::StateAttribute::ON);
        ss->setTextureAttributeAndModes(1, _windTexture.get(), osg::StateAttribute::ON);

        return ss;
    }

    osg::Camera* createRTTCamera()
    {
        osg::Camera* camera = new osg::Camera;

        camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        camera->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0f));

        // set view
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

        // set viewport
        camera->setViewport(0, 0, _size, _size);

        // set the camera to render before the main camera.
        camera->setRenderOrder(osg::Camera::PRE_RENDER);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

        // set up projection.
        camera->setProjectionMatrixAsOrtho2D(0.0, _size, 0.0, _size);
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

        // Make a full screen quad
        _quad = makeQuad(_size, _size, osg::Vec4(1, 1, 1, 1));
        _quad->setCullingActive(false);
        _quad->setStateSet(createStateSet());
        camera->addChild(_quad);

        return camera;
    }

    void buildCamera()
    {
        if (_camera.valid())
        {
            removeChild(_camera.get());
        }
        _camera = createRTTCamera();

        _camera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0), _outputPosition.get());
        addChild(_camera.get());
    }

    void swap()
    {
        // Swap the positions
        osg::ref_ptr< osg::Texture2D > tmp = _inputPosition.get();
        _inputPosition = _outputPosition.get();
        _outputPosition = tmp.get();

        buildCamera();
    }

    osg::ref_ptr< osg::Texture2D > _inputPosition;
    osg::ref_ptr< osg::Texture2D > _outputPosition;
    osg::ref_ptr< osg::Camera > _camera;
    osg::ref_ptr<osg::Node> _quad;
    osg::ref_ptr<osg::Texture2D > _windTexture;

    unsigned int _size;
};

class WindTextureNode : public osg::Group
{
public:
    WindTextureNode(osg::Texture2D* windTexture):
        _width(4096),
        _height(2048)
    {

        _computeNode = new ComputeNode(windTexture);
        _computeNode->getOrCreateStateSet()->addUniform(new osg::Uniform("dieSpeed", 10.0f));
        _computeNode->getOrCreateStateSet()->addUniform(new osg::Uniform("speedFactor", 1.0f));
        _computeNode->getOrCreateStateSet()->addUniform(new osg::Uniform("dropChance", 0.0f));

        // Create the render target
        createOutputTexture();
        createRTTCamera();

        addChild(_rttCamera.get());
        addChild(_computeNode.get());

        setCullingActive(false);
    }

    float getDieSpeed() const
    {
        float value;
        _computeNode->getOrCreateStateSet()->getUniform("dieSpeed")->get(value);
        return value;
    }

    void setDieSpeed(float value)
    {
        _computeNode->getOrCreateStateSet()->getUniform("dieSpeed")->set(value);
    }

    float getSpeedFactor() const
    {
        float value;
        _computeNode->getOrCreateStateSet()->getUniform("speedFactor")->get(value);
        return value;
    }

    void setSpeedFactor(float value)
    {
        _computeNode->getOrCreateStateSet()->getUniform("speedFactor")->set(value);
    }

    float getDropChance() const
    {
        float value;
        _computeNode->getOrCreateStateSet()->getUniform("dropChance")->get(value);
        return value;
    }

    void setDropChance(float value)
    {
        _computeNode->getOrCreateStateSet()->getUniform("dropChance")->set(value);
    }

    float getPointSize() const
    {
        float value;
        _points->getOrCreateStateSet()->getUniform("pointSize")->get(value);
        return value;
    }

    void setPointSize(float value)
    {
        _points->getOrCreateStateSet()->getUniform("pointSize")->set(value);
    }

    osg::Texture2D* getOutputTexture() const
    {
        return _outputTexture.get();
    }

    void swap()
    {
        _computeNode->swap();
    }

private:

    void createOutputTexture()
    {
        _outputTexture = new osg::Texture2D();
        _outputTexture->setTextureSize(_width, _height);
        _outputTexture->setInternalFormat(GL_RGBA8);
        _outputTexture->setSourceFormat(GL_RGBA);
        _outputTexture->setSourceType(GL_UNSIGNED_BYTE);
        _outputTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
        _outputTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
    }

    void createRTTCamera()
    {
        _rttCamera = new osg::Camera();
        _rttCamera->setRenderOrder(osg::Camera::PRE_RENDER);
        _rttCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        _rttCamera->setViewport(0, 0, _outputTexture->getTextureWidth(), _outputTexture->getTextureHeight());
        _rttCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        _rttCamera->setClearColor(osg::Vec4(0, 0, 0, 0));
        _rttCamera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0), _outputTexture.get());
        _rttCamera->setCullingMode(_rttCamera->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
        _rttCamera->setProjectionMatrixAsOrtho2D(0.0, _width, 0.0, _height);
        _rttCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

        _points = createInstancedGeometry(TEXTURE_DIM*TEXTURE_DIM);
            // Attatch the output of the compute node as the texture to feed the positions on the instanced geometry.
        _points->getOrCreateStateSet()->setTextureAttributeAndModes(0, _computeNode->_outputPosition.get(), osg::StateAttribute::ON);
        _rttCamera->addChild(_points.get());
    }

    osg::ref_ptr< osg::Texture2D > _outputTexture;
    osg::ref_ptr< osg::Camera > _rttCamera;

    osg::ref_ptr< ComputeNode > _computeNode;

    osg::ref_ptr< osg::Geometry > _points;

    unsigned int _width;
    unsigned int _height;
};




#define LC "[viewer] "

class WindTextureLayer : public ImageLayer
{
public:
    osg::ref_ptr<WindTextureNode> _node;
    osg::ref_ptr< osg::Texture2D > _windTexture;

    META_Layer(osgEarth, WindTextureLayer, Options, ImageLayer, mytexturelayer);

    void setWindTexture(osg::Texture2D* texture)
    {
        _windTexture = texture;
    }

    float getDieSpeed() const
    {
        return _node->getDieSpeed();
    }

    void setDieSpeed(float value)
    {
        return _node->setDieSpeed(value);
    }


    float getSpeedFactor() const
    {
        return _node->getSpeedFactor();
    }

    void setSpeedFactor(float value)
    {
        return _node->setSpeedFactor(value);
    }

    float getPointSize() const
    {
        return _node->getPointSize();
    }

    void setPointSize(float value)
    {
        return _node->setPointSize(value);
    }

    float getDropChance() const
    {
        return _node->getDropChance();
    }

    void setDropChance(float value)
    {
        return _node->setDropChance(value);
    }


    Status openImplementation()
    {
        _node = new WindTextureNode(_windTexture.get());

        setProfile(Profile::create("global-geodetic"));
        setUseCreateTexture();
        dataExtents().push_back(DataExtent(getProfile()->getExtent(), 0, 0));

        return Status::OK();
    }

    TextureWindow
        createTexture(const TileKey& key, ProgressCallback* progress) const
    {
        // Set the texture matrix corresponding to the tile key:
        osg::Matrixf textureMatrix;
        key.getExtent().createScaleBias(getProfile()->getExtent(), textureMatrix);
        return TextureWindow(_node->getOutputTexture(), textureMatrix);
    }

    virtual osg::Node* getNode() const
    {
        return _node.get();
    }

    osg::Texture2D* getTexture()
    {
        return _node->getOutputTexture();
    }

    void swap()
    {
        _node->swap();
    }
};

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

class ImGuiDemo : public OsgImGuiHandler
{
public:
    ImGuiDemo(osgViewer::View* view, MapNode* mapNode, EarthManipulator* earthManip, WindTextureLayer* wind) :
        _mapNode(mapNode),
        _earthManip(earthManip),
        _view(view),
        _wind(wind)
    {
    }

protected:
    virtual void drawUi(osg::RenderInfo& renderInfo)
    {
        // ImGui code goes here...
        //ImGui::ShowDemoWindow();
        _layers.draw(_mapNode.get(), _view->getCamera(), _earthManip.get());

        ImGui::Begin("Wind");

        float dieSpeed = _wind->getDieSpeed();
        ImGui::SliderFloat("Die Speed", &dieSpeed, 0.0, 45.0f);
        _wind->setDieSpeed(dieSpeed);

        float speedFactor = _wind->getSpeedFactor();
        ImGui::SliderFloat("Speed Factor", &speedFactor, 0.01, 1.0);
        _wind->setSpeedFactor(speedFactor);

        float pointSize = _wind->getPointSize();
        ImGui::SliderFloat("Point Size", &pointSize, 0.01, 5.0);
        _wind->setPointSize(pointSize);

        float dropChance = _wind->getDropChance();
        ImGui::SliderFloat("Drop Chance", &dropChance, 0.0, 1.0);
        _wind->setDropChance(dropChance);


        osg::Texture2D *tex = _wind->getTexture();
        if (tex)
        {
            const unsigned int contextID = renderInfo.getState()->getContextID();

            // get the texture object for the current contextID.
            tex->apply(*renderInfo.getState());
            osg::Texture::TextureObject *textureObject = tex->getTextureObject(contextID);
            if (textureObject)
            {
                double ar = (double)tex->getTextureWidth() / (double)tex->getTextureHeight();
                double h = 100.0;
                double w = ar * h;
                ImGui::Image((void *)(intptr_t)textureObject->_id, ImVec2(w, h), ImVec2(0, 1), ImVec2(1, 0));
            }
        }
        ImGui::End();
    }

    osg::ref_ptr< MapNode > _mapNode;
    osg::ref_ptr<EarthManipulator> _earthManip;
    osgViewer::View* _view;
    LayersGUI _layers;
    WindTextureLayer* _wind;
};

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    /*
    ImGuiNotifyHandler* notifyHandler = new ImGuiNotifyHandler();
    osg::setNotifyHandler(notifyHandler);
    osgEarth::setNotifyHandler(notifyHandler);
    */

    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, false);

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    EarthManipulator* manip = new EarthManipulator(arguments);
    viewer.setCameraManipulator(manip);

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    // Setup the viewer for imgui
    viewer.setRealizeOperation(new ImGuiDemo::RealizeOperation);

    viewer.realize();

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        GDALImageLayer* windData = new GDALImageLayer();
        windData->setName("Wind Data");
        windData->setURL("d:/dev/osgearth/data/2016112000.png");
        windData->setProfile(Profile::create("global-geodetic"));


        // a custom layer that displays a user texture:
        auto windLayer = new WindTextureLayer();
        windLayer->setName("Wind Simulation");
        osg::Texture2D *windTexture = new osg::Texture2D(osgDB::readImageFile("d:/dev/osgearth/data/2016112000.png"));
        windTexture->setResizeNonPowerOfTwoHint(false);
        windTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
        windTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
        windLayer->setWindTexture(windTexture);
        windLayer->setOpacity(0.5f);

        MapNode* mapNode = MapNode::findMapNode(node);
        if (mapNode)
        {
            mapNode->getMap()->addLayer(windData);
            mapNode->getMap()->addLayer(windLayer);

            viewer.getEventHandlers().push_front(new ImGuiDemo(&viewer, mapNode, manip, windLayer));
        }

        viewer.setSceneData(node);

        while (!viewer.done())
        {
            viewer.frame();
            windLayer->swap();
        }
        return 0;
    }
    else
    {
        return usage(argv[0]);
    }
}
