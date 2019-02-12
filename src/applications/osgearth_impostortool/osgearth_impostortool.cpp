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
#include <osgEarth/Random>

#include <osg/Texture2D>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>

using namespace osgEarth;

#define LC "[impostertool] "

int
fail(const std::string& msg, char** argv)
{
    OE_WARN LC << msg << std::endl;
    OE_WARN LC << argv[0]
        << "\n  --bake                          ; create impostor texture"
        << "\n    --in <filename>               ; model to process"
        << "\n    --out <filename>              ; output texture filename"
        << "\n    --size <n>                    ; dimension of texture"
        << "\n    --frames <n>                  ; number of snapshots in each dimension"
        << "\n    --view                        ; view the model matrix on screen"
        << "\n"
        << "\n  --test                          ; test an impostor"
        << "\n    --model <file>                ; source model (for side-by-side view)"
        << "\n    --texture <file>              ; texture created with --bake"
        << "\n    --grid                        ; render a scattered grid of impostors"
        << "\n    --debug                       ; render impostor billboards"

        << std::endl;
    return -1;
}

osg::Camera*
createColorCamera(unsigned dim)
{
    osg::Image* image = new osg::Image();
    image->allocateImage(dim, dim, 1, GL_RGBA, GL_UNSIGNED_BYTE);

    osg::Camera* rtt = new osg::Camera();
    rtt->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    rtt->setViewport(0, 0, dim, dim);
    rtt->setClearColor(osg::Vec4(0, 0, 0, 0));
    rtt->setRenderOrder(osg::Camera::PRE_RENDER);
    rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    rtt->attach(osg::Camera::COLOR_BUFFER0, image);
    return rtt;
}

osg::Camera*
createNormalMapCamera(unsigned dim)
{
    osg::Image* image = new osg::Image();
    image->allocateImage(dim, dim, 1, GL_RGB, GL_UNSIGNED_BYTE);

    osg::Camera* rtt = new osg::Camera();
    rtt->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    rtt->setViewport(0, 0, dim, dim);
    rtt->setClearColor(osg::Vec4(0, 0, 0, 0));
    rtt->setRenderOrder(osg::Camera::PRE_RENDER);
    rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    rtt->setImplicitBufferAttachmentMask(0, 0);
    rtt->attach(osg::Camera::COLOR_BUFFER0, image);
    return rtt;
}

osg::Image*
getImageAttachment(osg::Camera* camera)
{
    osg::Camera::Attachment a = camera->getBufferAttachmentMap().at(osg::Camera::COLOR_BUFFER0);
    return a._image.get();
}

// input: hemioctahedron coords [-1..1]
osg::Vec3d
hemiOctahedronToVec3(const osg::Vec2d& e)
{
    osg::Vec2d temp = osg::Vec2d(e.x() + e.y(), e.x() - e.y())*0.5;
    osg::Vec3d v(temp.x(), temp.y(), 1.0 - fabs(temp.x()) - fabs(temp.y()));
    v.normalize();
    return v;
}

// Shaders for rendering the normal map
const char* normalMapVS =
    "#version 330 \n"
    "out vec3 modelNormal; \n"
    "void normalMapVS(inout vec4 vertex) { \n"
    "    modelNormal = gl_Normal; \n"
    "} \n";

const char* normalMapFS =
    "#version 330 \n"
    "in vec3 modelNormal; \n"
    "out vec3 encodedNormal; \n"
    "void normalMapFS(inout vec4 color) { \n"
    "    encodedNormal = (modelNormal+1.0)*0.5; \n"
    "} \n";

int
bake_main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setUpViewInWindow(30, 30, 1024, 1024);

    // SingleThreaded so the frame will finish rendering before frame() returns
    // and we can write it to a file
    viewer.setThreadingModel(viewer.SingleThreaded);

    std::string infile;
    if (!arguments.read("--in", infile))
        return fail("Missing required --in", argv);

    std::string outfile;
    if (!arguments.read("--out", outfile))
        return fail("Missing required --out", argv);

    std::string ext = osgDB::getFileExtension(outfile);
    std::string base = osgDB::getNameLessExtension(outfile);
    std::string normalMapOutFile = Stringify() << base << ".normal." << ext;

    int size = 1024;
    arguments.read("--size", size);
    if (size < 2)
        return fail("--size must be greater than or equal to 2", argv);

    int numFrames = 12;
    arguments.read("--frames", numFrames);
    if (numFrames < 2 || (numFrames & 0x01) != 0)
        return fail("--frames must be an even number greater than 1", argv);

    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile(infile);
    if (!model.valid())
        return fail("Cannot load model", argv);

    // shaders
    Registry::instance()->shaderGenerator().run(model.get());

    // test:
    osg::Vec3d test = hemiOctahedronToVec3(osg::Vec2d(0, 0));
    OE_INFO << LC << "oct(0,0) => " << test.x() << "," << test.y() << "," << test.z() << std::endl;

    // compute the bbox of the model and compute the maximum span (x,y,z)
    osg::ComputeBoundsVisitor cbv;
    model->accept(cbv);
    osg::BoundingBox modelBB = cbv.getBoundingBox();
    double x = modelBB.xMax() - modelBB.xMin();
    double y = modelBB.yMax() - modelBB.yMin();
    double z = modelBB.zMax() - modelBB.zMin();
    double maxSpan = osg::maximum(x, osg::maximum(y, z));
    double halfMaxSpan = 0.5*maxSpan;

    // offset the model so the origin is at its bounding box centroid.
    osg::PositionAttitudeTransform* modelPAT = new osg::PositionAttitudeTransform();
    modelPAT->setPosition(-modelBB.center());
    modelPAT->addChild(model);

    // number of frames in each dimension.
    double unitFrameSize = 1.0 / (double)numFrames;

    osg::Group* models = new osg::Group();
    for (int i = 0; i < numFrames; ++i)
    {
        for (int j = 0; j < numFrames; ++j)
        {
            osg::PositionAttitudeTransform* xform = new osg::PositionAttitudeTransform();
            xform->setPosition(osg::Vec3d(maxSpan*i, 0, maxSpan*j));
            xform->addChild(modelPAT);

            models->addChild(xform);

            double u = (double)i / (double)(numFrames - 1);
            double v = (double)j / (double)(numFrames - 1);
            osg::Vec2d hoc = osg::Vec2d(u, v) * 2.0 - osg::Vec2d(1.0, 1.0);
            osg::Vec3d eye = hemiOctahedronToVec3(hoc);

            osg::Vec3d eye2d(eye.x(), eye.y(), 0.0);
            osg::Quat azim;
            azim.makeRotate(eye2d, osg::Vec3d(0, -1, 0));
            double a = asin(eye.z());
            osg::Quat pitch(a, osg::Vec3d(1, 0, 0));
            xform->setAttitude(azim*pitch);
        }
        std::cout << std::endl;
    }

    osg::Group* root = new osg::Group();
    root->addChild(models);

    osg::Camera* colorCamera = createColorCamera(size);
    colorCamera->addChild(models);
    root->addChild(colorCamera);
    osg::StateSet* colorSS = colorCamera->getOrCreateStateSet();
    colorSS->setMode(GL_BLEND, 1);
    colorSS->setMode(GL_CULL_FACE, 0);

    osg::Camera* normalMapCamera = createNormalMapCamera(size);
    normalMapCamera->addChild(models);
    root->addChild(normalMapCamera);
    osg::StateSet* normalMapSS = normalMapCamera->getOrCreateStateSet();
    normalMapSS->setMode(GL_CULL_FACE, 0);

    VirtualProgram* normalMapVP = new VirtualProgram();
    normalMapCamera->getOrCreateStateSet()->setAttribute(normalMapVP, osg::StateAttribute::OVERRIDE);
    normalMapVP->setFunction("normalMapVS", normalMapVS, ShaderComp::LOCATION_VERTEX_MODEL);
    normalMapVP->setFunction("normalMapFS", normalMapFS, ShaderComp::LOCATION_FRAGMENT_OUTPUT);

    viewer.setSceneData(root);
    viewer.realize();


    osg::Matrix proj, view;
    proj.makeOrtho(-halfMaxSpan, -halfMaxSpan + maxSpan * numFrames, -halfMaxSpan, -halfMaxSpan + maxSpan * numFrames, -maxSpan, maxSpan);
    view.makeLookAt(osg::Vec3d(0, 0, 0), osg::Vec3d(0, 1, 0), osg::Vec3d(0, 0, 1));

    colorCamera->setProjectionMatrix(proj);
    colorCamera->setViewMatrix(view);

    normalMapCamera->setProjectionMatrix(proj);
    normalMapCamera->setViewMatrix(view);

    viewer.frame();

    // write color files:
    OE_INFO << LC << "Writing " << outfile << std::endl;
    osgDB::writeImageFile(*getImageAttachment(colorCamera), outfile);

    OE_INFO << LC << "Writing " << normalMapOutFile << std::endl;
    osgDB::writeImageFile(*getImageAttachment(normalMapCamera), normalMapOutFile);

    if (arguments.read("--view"))
        return viewer.run();
    else
        return 0;
}

const char* imposterVSView =
"#version 330 \n"
"uniform float imposterSize; \n"
"out vec2 imposterTC; \n"
"out vec2 rawTC; \n"
"vec3 vp_Normal; \n"
"uniform mat4 osg_ViewMatrixInverse; \n"

// given the octahedral vector, create a local coordinate system
// in which to create the billboard plane. This returns the offset
// vector for one of the billboard corners, whichever one is
// represented by offset.
"vec3 projectOffset(in vec3 vector, in vec2 offset) \n"
"{ \n"
// XYZ is the reference frame of the original viewer that was
// used to capture the impostor:
"    vec3 y = normalize(-vector); \n"
"    vec3 x = normalize(cross(y, vec3(0,0,1))); \n"
"    vec3 z = normalize(cross(x, y)); \n"
"    vec3 newX = x * offset.x * imposterSize; \n"
"    vec3 newZ = z * offset.y * imposterSize; \n"
"    return newX + newZ; \n"
"} \n"

// transform from 2D frame-grid coordinates to a 3D octahedral vector
// Assume normalized input on +Z hemisphere. Output is [0..1]
"vec2 vec3_to_hemioct(in vec3 v) { \n"
"    // Project the hemisphere onto the hemi-octahedron, and then into the xy plane \n"
"    vec2 p = v.xy * (1.0 / (abs(v.x) + abs(v.y) + v.z)); \n"
"    // Rotate and scale the center diamond to the unit square \n"
"    return (vec2(p.x + p.y, p.x - p.y)+1.0)*0.5; \n" // [0..1]
//"    return vec2(p.x + p.y, p.x - p.y); \n" // [-1..1]
"} \n"

// transform from 3D octahedral vector to a 3D frame-grid coordinate
// Input is [0..1]
"vec3 hemioct_to_vec3(in vec2 t) { \n"
"    vec2 e = t*2.0-1.0; \n"
"    // Rotate and scale the unit square back to the center diamond \n"
"    vec2 temp = vec2(e.x + e.y, e.x - e.y) * 0.5; \n"
"    vec3 v = vec3(temp, 1.0 - abs(temp.x) - abs(temp.y)); \n"
"    return normalize(v); \n"
"} \n"

"void imposterVSView(inout vec4 vertexView) { \n"

// number of frames in each dimension - make this a define or something
"    const float numFrames = 12.0; \n"

// calcuate the billboard corner offset vector for the current vertex:
"    vec2 offset; \n"
"    if      (gl_VertexID == 0) offset = vec2(-1, -1); \n"
"    else if (gl_VertexID == 1) offset = vec2( 1, -1); \n"
"    else if (gl_VertexID == 2) offset = vec2(-1,  1); \n"
"    else                       offset = vec2( 1,  1); \n"

// get grid vector is the vector from the model to the camera
// that will get mapped to 2D octahedral space. I tried to do 
// this in view space but it didn't work - need to revisit that.
"    vec3 eyeWorld = osg_ViewMatrixInverse[3].xyz;\n"
"    vec4 vertexWorld = osg_ViewMatrixInverse * vertexView; \n"
"    vec3 modelToCamera = normalize(eyeWorld - vertexWorld.xyz); \n"

// transform to 2D frame-grid space
"    vec2 octo = vec3_to_hemioct(modelToCamera); \n"
// quantize to the nearest frame
"    octo = floor(octo*numFrames)/numFrames; \n"
// calculate 1/2 the size of one frame:
"    float t = (0.5/numFrames); \n"

// establish the texture coordinates:
"    imposterTC = vec2(t)+octo+(offset*t); \n"

// use the octahedral vector to establish a reference frame for
// creating the billboard in model space, then xform that into
// view space and create the billboard by offsetting the vertices:
"    vec3 quantizedVector = hemioct_to_vec3(octo); \n"
"    vec3 vertexOffset = gl_NormalMatrix * projectOffset(quantizedVector, offset); \n"
"    vertexView.xyz += vertexOffset; \n"

"    rawTC = (offset+1.0)*0.5; \n" // debugging
"} \n";

const char* imposterFS =
"#version 330 \n"
"#pragma import_defines(IMPOSTOR_DEBUG) \n"
"uniform sampler2D imposterTex; \n"
"in vec2 imposterTC; \n"
"in vec2 rawTC; \n"
"void imposterFS(inout vec4 color) { \n"
"    vec4 texel = texture(imposterTex, imposterTC); \n"
"    color = texel; \n"
"#ifdef IMPOSTOR_DEBUG \n"
"    // debugging \n"
"    if (any(greaterThan(abs(rawTC*2.0-1.0), vec2(.99)))) color=vec4(1); \n"
"    if (rawTC.t > .99) color=vec4(1,0,0,1); \n"
"#endif \n"
"    if (color.a < 0.15) discard; \n"
"} \n";

int
test_main(int argc, char** argv)
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

    bool grid = arguments.read("--grid");
    bool random = arguments.read("--random");
    bool debug = arguments.read("--debug");

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
    osg::Uniform* imposterSizeU = new osg::Uniform("imposterSize", (float)maxSpan / 2.0f);

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
    normals->assign(4, osg::Vec3(0, 0, 1));
    imposterGeom->setNormalArray(normals);
    imposterGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, 4));
    osg::StateSet* imposterSS = imposterGeom->getOrCreateStateSet();
    VirtualProgram* geomVP = VirtualProgram::getOrCreate(imposterSS);
    geomVP->setFunction("imposterVSView", imposterVSView, ShaderComp::LOCATION_VERTEX_VIEW);
    geomVP->setFunction("imposterFS", imposterFS, ShaderComp::LOCATION_FRAGMENT_COLORING);
    imposterSS->setTextureAttribute(0, tex, osg::StateAttribute::ON);
    imposterSS->addUniform(imposterTexU);
    imposterSS->addUniform(imposterSizeU);
    imposterSS->setMode(GL_BLEND, 1);
    if (debug) imposterSS->setDefine("IMPOSTOR_DEBUG");

    // Build the scene graph:
    osg::Group* root = new osg::Group();

    double offset = model->getBound().radius() * 2.0;

    Random prng;

    if (grid)
    {
        const double DIM = 128;
        for(double i = -offset*DIM; i<= offset*DIM; i += offset)
        {
            for (double j = -offset * DIM; j <= offset * DIM; j += offset)
            {
                double x = i + offset*(prng.next()-0.5), y = j + offset*(prng.next()-0.5);
                osg::PositionAttitudeTransform* imposterxform = new osg::PositionAttitudeTransform();
                imposterxform->setPosition(osg::Vec3d(x, y, 0.0));
                imposterxform->addChild(imposterGeom);
                root->addChild(imposterxform);
            }
        }
    }
    else if (random)
    {
        const double DIM = 256;
        const double range = offset * DIM;
        for(int i=0; i<DIM*DIM; ++i)
        {
            double x = -range / 2 + (prng.next()*range);
            double y = -range / 2 + (prng.next()*range);

            osg::PositionAttitudeTransform* imposterxform = new osg::PositionAttitudeTransform();
            imposterxform->setPosition(osg::Vec3d(x, y, 0.0));
            imposterxform->addChild(imposterGeom);
            root->addChild(imposterxform);
        }
    }

    else
    {
        osg::PositionAttitudeTransform* modelxform = new osg::PositionAttitudeTransform();
        modelxform->setPosition(osg::Vec3d(-offset, 0.0, 0.0));
        modelxform->addChild(model.get());
        root->addChild(modelxform);
    
        osg::PositionAttitudeTransform* imposterxform = new osg::PositionAttitudeTransform();
        imposterxform->setPosition(osg::Vec3d(offset, 0.0, 0.0));
        imposterxform->addChild(imposterGeom);
        root->addChild(imposterxform);
    }

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



int 
main(int argc, char** argv)
{
    osg::ArgumentParser a(&argc, argv);
    if (a.read("--bake"))
        return bake_main(argc, argv);
    else if (a.read("--test"))
        return test_main(argc, argv);

    return fail("Missing required argument --bake or --test", argv);
}