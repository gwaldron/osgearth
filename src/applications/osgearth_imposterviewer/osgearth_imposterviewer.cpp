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
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgEarth/Notify>
#include <osgEarth/VirtualProgram>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/GLUtils>
#include <osg/Texture2D>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>

using namespace osgEarth;

#define LC "[imposter baker] "

int
fail(const std::string& msg, char** argv)
{
    OE_WARN LC << msg << std::endl;
    OE_WARN LC << argv[0]
        << "\n    --model <file>               ; model to impostor"
        << "\n    --texture <file>             ; texture created with impostor baker"
        << std::endl;
    return -1;
}

const char* imposterVSModel =
    "#version 330 \n"
    "vec3 vector; \n"
    "vec3 vp_Normal; \n"
    "uniform mat4 osg_ViewMatrixInverse; \n"
    "void imposterVSModel(inout vec4 vertexModel) { \n"
    "    vec3 eyeModel = osg_ViewMatrixInverse[3].xyz;\n"
    "    vector = normalize(eyeModel - vertexModel.xyz); \n"
    "} \n";

const char* imposterVSView =
    "#version 330 \n"
    "uniform float imposterSize; \n"
    "out vec2 imposterTC; \n"
    "vec3 vector; \n"
    "vec3 vp_Normal; \n"
    "uniform mat4 osg_ViewMatrixInverse; \n"

    // Assume normalized input on +Z hemisphere. Output is [0..1]
    "vec2 vec3_to_hemioct(in vec3 v) { \n"
    "    // Project the hemisphere onto the hemi-octahedron, and then into the xy plane \n"
    "    vec2 p = v.xy * (1.0 / (abs(v.x) + abs(v.y) + v.z)); \n"
    "    // Rotate and scale the center diamond to the unit square \n"
    "    return (vec2(p.x + p.y, p.x - p.y)+1.0)*0.5; \n"
    "} \n"

    "vec3 hemioct_to_vec3(vec2 e) { \n"
    "    // Rotate and scale the unit square back to the center diamond \n"
    "    vec2 temp = vec2(e.x + e.y, e.x - e.y) * 0.5; \n"
    "    vec3 v = vec3(temp, 1.0 - abs(temp.x) - abs(temp.y)); \n"
    "    return normalize(v); \n"
    "} \n"

    "void imposterVSView(inout vec4 vertexView) { \n"
    "    const float numFrames = 4.0; \n"
    "    vec2 offset; \n"
    "    if      (gl_VertexID == 0) offset = vec2(-1, -1); \n"
    "    else if (gl_VertexID == 1) offset = vec2( 1, -1); \n"
    "    else if (gl_VertexID == 2) offset = vec2(-1,  1); \n"
    "    else                       offset = vec2( 1,  1); \n"

    "    vec2 octo = vec3_to_hemioct(vector); \n"
    "    octo = floor(octo*numFrames)/numFrames; \n" // quantize
    "    float t = (0.5/numFrames); \n" // half the size of a frame

    "    // establish the texture coordinates \n"
    "    imposterTC = vec2(t)+octo+(offset*t); \n"

    //"    // derive the original transform used to capture the impostor \n"
    //"    vec3 quantizedVector = hemioct_to_vec3(octo); \n"
    //"    vec3 F = -quantizedVector; \n"
    //"    vec3 R = cross(F, vp_Normal); \n"
    //"    vec3 U = cross(F, R); \n"
    //"    mat3 xform = mat3(R, U, -F); \n"

    "    vertexView.xy += (offset+1.0)*imposterSize; \n"
    "} \n";

const char* imposterFS =
    "#version 330 \n"
    "uniform sampler2D imposterTex; \n"
    "in vec2 imposterTC; \n"
    "in vec3 aaa; \n"
    "void imposterFS(inout vec4 color) { \n"
    "    vec4 texel = texture(imposterTex, imposterTC); \n"
    "    color = texel; \n"
    "    if (texel.a < 0.1) discard; \n"
    "} \n";

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer(arguments);
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    std::string textureFile;
    if (!arguments.read("--texture", textureFile))
        return fail("Missing required --texture argument", argv);

    osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(textureFile);
    if (!image.valid())
        return fail("Failed to load texture", argv);

    std::string modelFile;
    if (!arguments.read("--model", modelFile))
        return fail("Missing required --model argument", argv);

    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile(modelFile);
    if (!model.valid())
        return fail("Failed to load model", argv);
    Registry::instance()->shaderGenerator().run(model.get());

    // compute the bbox of the model and compute the maximum span (x,y,z)
    osg::ComputeBoundsVisitor cbv;
    model->accept(cbv);
    osg::BoundingBox modelBB = cbv.getBoundingBox();
    double x = modelBB.xMax() - modelBB.xMin();
    double y = modelBB.yMax() - modelBB.yMin();
    double z = modelBB.zMax() - modelBB.zMin();
    double maxSpan = osg::maximum(x, osg::maximum(y, z));
    osg::Uniform* imposterSizeU = new osg::Uniform("imposterSize", (float)maxSpan/2.0f);

    // Set up the imposter texture:
    osg::Texture2D* tex = new osg::Texture2D(image.get());
    tex->setFilter(tex->MIN_FILTER, tex->LINEAR_MIPMAP_LINEAR);
    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
    tex->setWrap(tex->WRAP_S, tex->CLAMP_TO_EDGE);
    tex->setWrap(tex->WRAP_T, tex->CLAMP_TO_EDGE);
    osg::Uniform* imposterTexU = new osg::Uniform("imposterTex", (int)0);

    // Build the imposter geometry:
    osg::Geometry* imposterGeom = new osg::Geometry();
    imposterGeom->setUseVertexBufferObjects(true);
    imposterGeom->setUseDisplayList(false);
    osg::Vec3Array* verts = new osg::Vec3Array(4); // four 0,0,0 verts
    imposterGeom->setVertexArray(verts);
    osg::Vec3Array* normals = new osg::Vec3Array(4);
    normals->assign(4, osg::Vec3(0,0,1));
    imposterGeom->setNormalArray(normals);
    imposterGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, 4));
    osg::StateSet* imposterSS = imposterGeom->getOrCreateStateSet();
    VirtualProgram* geomVP = VirtualProgram::getOrCreate(imposterSS);
    geomVP->setFunction("imposterVSModel", imposterVSModel, ShaderComp::LOCATION_VERTEX_MODEL);
    geomVP->setFunction("imposterVSView", imposterVSView, ShaderComp::LOCATION_VERTEX_VIEW);
    geomVP->setFunction("imposterFS", imposterFS, ShaderComp::LOCATION_FRAGMENT_COLORING);
    imposterSS->setTextureAttribute(0, tex, osg::StateAttribute::ON);
    imposterSS->addUniform(imposterTexU);
    imposterSS->addUniform(imposterSizeU);
    imposterSS->setMode(GL_BLEND, 1);

    // Build the scene graph:
    osg::Group* root = new osg::Group();
    
    double offset = model->getBound().radius();
    osg::PositionAttitudeTransform* modelxform = new osg::PositionAttitudeTransform();
    modelxform->setPosition(osg::Vec3d(-offset, 0.0, 0.0));
    modelxform->addChild(model.get());
    root->addChild(modelxform);
    
    osg::PositionAttitudeTransform* imposterxform = new osg::PositionAttitudeTransform();
    imposterxform->setPosition(osg::Vec3d(offset, 0.0, 0.0));
    imposterxform->addChild(imposterGeom);
    root->addChild(imposterxform);

    // default uniform values:
    GLUtils::setGlobalDefaults(viewer.getCamera()->getOrCreateStateSet());

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler());
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler());


    viewer.setSceneData(root);
    return viewer.run();
}