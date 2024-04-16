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

#include <osgEarth/Notify>
#include <osgEarth/TextureArena>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ExampleResources>
#include <osgEarth/CullingUtils>
#include <osgEarth/Chonk>
#include <osgEarth/Capabilities>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/BlendFunc>
#include <osgUtil/Simplifier>

using namespace osgEarth;
using namespace osgEarth::Util;

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int main_NV(int argc, char** argv)
{
    osgEarth::initialize();

    if (!Capabilities::get().supportsNVGL()) {
        OE_WARN << "This app requires NVIDIA GL extensions" << std::endl;
        return -1;
    }

    osg::ArgumentParser arguments(&argc, argv);

    osgViewer::Viewer viewer(arguments);
    viewer.setRealizeOperation(new GL3RealizeOperation());

    MapNodeHelper().configureView(&viewer);

    if (arguments.read("--pause"))
        ::getchar();

    if (arguments.read("--novsync")) {
        CustomRealizeOperation* op = new CustomRealizeOperation();
        op->setSyncToVBlank(false);
        viewer.setRealizeOperation(op);
    }

    GLUtils::enableGLDebugging();

    int size = 1;
    arguments.read("--size", size);

    osg::Group* root = new osg::Group();
    osg::StateSet* root_ss = root->getOrCreateStateSet();

    // textures:
    osg::ref_ptr<TextureArena> arena = new TextureArena();
    arena->setBindingPoint(1);
    root_ss->setAttribute(arena);

    // need these for GPU culling to work:
    viewer.getCamera()->addCullCallback(new InstallCameraUniform());
    root_ss->addUniform(new osg::Uniform("oe_sse", 50.0f));
    root_ss->setDefine("OE_GPUCULL_DEBUG", "1");

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
#define GL_SAMPLE_ALPHA_TO_COVERAGE_ARB   0x809E
        root_ss->setMode(GL_BLEND, 0);
        root_ss->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
        root_ss->setDefine("OE_USE_ALPHA_TO_COVERAGE");
    }
    else
    {
        root_ss->setAttributeAndModes(new osg::BlendFunc(
            GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), 1);
    }

    ChonkFactory factory(arena.get());

    auto drawable = new ChonkDrawable();

    float spacing = 0.0;

    std::vector<Chonk::Ptr> chonks;

    for (int i = 1; i < argc; ++i)
    {
        osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(argv[i]);
        if (!node.valid())
            return -1;

        OE_NOTICE << "Loaded " << argv[i] << std::endl;

        Chonk::Ptr chonk = Chonk::create();
        chonk->add(node.get(), factory);
        chonks.push_back(chonk);

        float radius = chonk->getBound().radius();
        spacing = std::max(spacing, radius*2.01f);
    }

    if (chonks.empty())
    {
        OE_WARN << "No models loaded." << std::endl;
        return -1;
    }

    for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
            for (int z = 0; z < size; ++z) {
                osg::Matrixf xform = osg::Matrixf::translate(
                    spacing*float(x), 
                    spacing*float(y),
                    spacing*float(z));
                int index = (x + y + z) % chonks.size();
                drawable->add(chonks[index], xform);
            }
        }
    }

    root->addChild(drawable);
    viewer.setSceneData(root);

    return viewer.run();
}

int
main(int argc, char** argv)
{
    // new bindlessNV
    return main_NV(argc, argv);
}
