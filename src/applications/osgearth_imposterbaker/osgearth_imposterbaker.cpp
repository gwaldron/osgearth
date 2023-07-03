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
#include <osgEarth/Notify>
#include <osgEarth/VirtualProgram>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
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
        << "\n    --in <filename>               ; model to process"
        << "\n    --out <filename>              ; output texture filename"
        << "\n    --size <n>                    ; dimension of texture"
        << "\n    --frames <n>                  ; number of snapshots in each dimension"
        << "\n    --view                        ; view the model matrix on screen"
        << "\n    --u <n>                       ; if frames==1, sets the u position"
        << "\n    --v <n>                       ; if frames==1, sets the v position"
        << std::endl;
    return -1;
}

osg::Camera*
createColorCamera(unsigned dimx, unsigned dimy)
{
    osg::Image* image = new osg::Image();
    image->allocateImage(dimx, dimy, 1, GL_RGBA, GL_UNSIGNED_BYTE);

    osg::Camera* rtt = new osg::Camera();
    rtt->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    rtt->setViewport(0, 0, dimx, dimy);
    rtt->setClearColor(osg::Vec4(0, 0, 0, 0));
    rtt->setRenderOrder(osg::Camera::PRE_RENDER);
    rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    //rtt->setImplicitBufferAttachmentMask(0, 0);
    rtt->attach(osg::Camera::COLOR_BUFFER0, image); //, 4u, 4u);
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

// Shaders for albedo rendering
const char* removeVertexColors =
"#version 330 \n"
"void removeVertexColors(inout vec4 color) { \n"
"    color = vec4(1,1,1,color.a); \n"
"} \n";

const char* discardAlpha =
"#version 330 \n"
"void discardAlpha(inout vec4 color) { \n"
"    color.a = step(0.15, color.a); \n"
"    if (color.a < 0.15) discard; \n"
"} \n";

// Shaders for rendering the normal map
const char* normalMapVS =
"#version 330 \n"
"out vec3 modelNormal; \n"
"void normalMapVS(inout vec4 vertex) { \n"
"    modelNormal = gl_NormalMatrix * gl_Normal; \n"
"} \n";
const char* normalMapFS = R"(
#version 330
in vec3 modelNormal;
out vec4 encodedNormal;
void normalMapFS(inout vec4 color) {
    vec3 N = normalize(gl_FrontFacing ? modelNormal : -modelNormal);
    N = mix(N, vec3(0,0,1), 0.5);
    encodedNormal = vec4((N.xyz+1.0)*0.5, 1.0);
}
)";

int
main(int argc, char** argv)
{
    osgEarth::initialize();

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
    std::string normalMapOutFile = Stringify() << base << "_NML." << ext;

    int size = 1024;
    arguments.read("--size", size);
    if (size < 2)
        return fail("--size must be greater than or equal to 2", argv);

    int numFrames = 8;
    arguments.read("--frames", numFrames);
    if (numFrames < 1 || (numFrames > 1 && (numFrames & 0x01) != 0))
        return fail("--frames must be an even number greater than 1", argv);

    double myu = 0.5, myv = 0.0;
    arguments.read("--u", myu);
    arguments.read("--v", myv);
    myu = osg::clampBetween(myu, 0.0, 0.99999);
    myv = osg::clampBetween(myv, 0.0, 0.49999);

    bool tight_fit = arguments.read("--tight");
    if (tight_fit)
        numFrames = 1;

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

    models->getOrCreateStateSet()->setMode(GL_CULL_FACE, 0);
    models->getOrCreateStateSet()->setMode(GL_BLEND, 1);

    if (numFrames == 1)
    {
        osg::PositionAttitudeTransform* xform = new osg::PositionAttitudeTransform();
        xform->setPosition(osg::Vec3d(0, 0, 0));
        xform->addChild(modelPAT);

        models->addChild(xform);

        double u = myu, v = myv;
        osg::Vec2d hoc = osg::Vec2d(u, v) * 2.0 - osg::Vec2d(1.0, 1.0);
        osg::Vec3d eye = hemiOctahedronToVec3(hoc);

        osg::Vec3d eye2d(eye.x(), eye.y(), 0.0);
        osg::Quat azim;
        azim.makeRotate(eye2d, osg::Vec3d(0, -1, 0));
        double a = asin(eye.z());
        osg::Quat pitch(a, osg::Vec3d(1, 0, 0));
        xform->setAttitude(azim*pitch);
    }

    else
    {
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
    }

    osg::Group* root = new osg::Group();
    root->addChild(models);

    osg::Camera* colorCamera;

    if (tight_fit && numFrames == 1)
    {
        unsigned cam_w, cam_h;
        if (myv == 0) {
            double m = std::max(x, y);
            if (z >= m) {
                cam_h = size;
                cam_w = float(size) * float(m) / float(z);
            }
            else {
                cam_w = size;
                cam_h = float(size) * float(z) / float(m);
            }
        }
        else cam_w = size, cam_h = size;

        colorCamera = createColorCamera(cam_w, cam_h);
    }
    else
    {
        colorCamera = createColorCamera(size, size);
    }

    colorCamera->addChild(models);
    root->addChild(colorCamera);

    VirtualProgram* colorVP = VirtualProgram::getOrCreate(colorCamera->getOrCreateStateSet());
    colorVP->setFunction("removeVertexColors", removeVertexColors, VirtualProgram::LOCATION_FRAGMENT_COLORING, 0.0f);
    colorVP->setFunction("discardAlpha", discardAlpha, VirtualProgram::LOCATION_FRAGMENT_COLORING);

    osg::Camera* normalMapCamera = createNormalMapCamera(size);
    normalMapCamera->addChild(models);
    root->addChild(normalMapCamera);

    VirtualProgram* normalMapVP = new VirtualProgram();
    normalMapCamera->getOrCreateStateSet()->setAttribute(normalMapVP, osg::StateAttribute::OVERRIDE);
    normalMapVP->setFunction("normalMapVS", normalMapVS, VirtualProgram::LOCATION_VERTEX_MODEL);
    normalMapVP->setFunction("normalMapFS", normalMapFS, VirtualProgram::LOCATION_FRAGMENT_OUTPUT);

    viewer.setSceneData(root);
    viewer.realize();


    osg::Matrix proj, view;
    if (tight_fit && numFrames == 1)
    {
        double w = std::max(x, y);
        if (myv == 0)
            proj.makeOrtho(-w * 0.5, w*0.5, -z * 0.5, z*0.5, -w, +w);
        else // myv == 1
            proj.makeOrtho(-w * 0.5, w*0.5, -w * 0.5, w*0.5, -w, +w);
    }
    else
    {
        proj.makeOrtho(-halfMaxSpan, -halfMaxSpan + maxSpan * numFrames, -halfMaxSpan, -halfMaxSpan + maxSpan * numFrames, -maxSpan, maxSpan);
    }
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
