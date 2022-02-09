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
#include <osgEarth/GeometryCloud>
#include <osgEarth/CullingUtils>
#include <osgEarth/Chonk>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/BlendFunc>
#include <osgUtil/Simplifier>

using namespace osgEarth;
using namespace osgEarth::Util;

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

const char* vs = R"(
    #version 430
    #extension GL_ARB_gpu_shader_int64 : enable

    layout(binding=1, std430) buffer TextureArena {
        uint64_t tex_arena[];
    };

    layout(location=6) in int arena_index;

    out vec3 tex_coord;
    flat out uint64_t tex_handle;

    void vs(inout vec4 vertex)
    {
        tex_coord = gl_MultiTexCoord7.xyz;
        tex_handle = tex_arena[arena_index];
    };
)";

const char* fs = R"(
    #version 430
    #extension GL_ARB_gpu_shader_int64 : enable

    in vec3 tex_coord;
    flat in uint64_t tex_handle;

    void fs(inout vec4 color)
    {
        vec4 texel = texture(sampler2D(tex_handle), tex_coord.st);
        color = texel;
    }
)";

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int
test_TextureArena(osg::ArgumentParser& args)
{
    return 0;
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int
test_GeometryCloud(osg::ArgumentParser& args)
{
    return 0;
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int
test_InstanceCloud(osg::ArgumentParser& args)
{
    return 0;
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

//! Simple DrawCallback to render the contents of the GeometryCloud.
//! This is really only useful in the context of this example, but
//! you could adapt it for use in other situations. Normally you would
//! allocate a GLBuffer and subload all the commands to it, and then
//! call glMultiDrawElementsIndirect with a nullptr argument. But
//! for this example we are just being lazy.
struct GeometryCloudRenderer : public osg::Drawable::DrawCallback
{
    GeometryCloud* _cloud;

    GeometryCloudRenderer(GeometryCloud* cloud) : _cloud(cloud) { }

    void drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const override
    {
        osg::State& state = *ri.getState();
        osg::GLExtensions* ext = state.get<osg::GLExtensions>();

        // prepare the VAO, etc.
        osg::VertexArrayState* vas = state.getCurrentVertexArrayState();
        vas->setVertexBufferObjectSupported(true);
        drawable->asGeometry()->drawVertexArraysImplementation(ri);

        // bind the combined EBO to the VAO
        if (vas->getRequiresSetArrays())
        {
            osg::GLBufferObject* ebo = drawable->asGeometry()->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
            state.bindElementBufferObject(ebo);
        }

        // Render a single instance of each cloud geometry
        for (unsigned i = 0; i < _cloud->getNumDrawCommands(); ++i)
        {
            DrawElementsIndirectCommand cmd;
            _cloud->getDrawCommand(i, cmd);
            cmd.instanceCount = 1;

            osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);
            ext->glDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, &cmd);
        }
    }
};

int main_NV(int argc, char** argv)
{
    osgEarth::initialize();
    osg::ArgumentParser arguments(&argc, argv);

    osgViewer::Viewer viewer(arguments);
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

    viewer.getCamera()->addCullCallback(new InstallCameraUniform());

    osg::StateSet* root_ss = root->getOrCreateStateSet();

    osg::ref_ptr<TextureArena> arena = new TextureArena();
    arena->setBindingPoint(1);
    root_ss->setAttribute(arena);


    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
#define GL_SAMPLE_ALPHA_TO_COVERAGE_ARB   0x809E
        root_ss->setMode(GL_BLEND, 0);
        root_ss->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
    }
    else
    {
        root_ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), 1);
        root_ss->setDefine("OE_USE_ALPHA_DISCARD");
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

        float radius = node->getBound().radius();
        spacing = std::max(spacing, radius*1.1f);

        Chonk::Ptr chonk = Chonk::create();
        chonk->add(node.get(), 350, FLT_MAX, factory);

        chonks.push_back(chonk);
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

    // Simple shader that will render geometry with bindless textures
    ChonkDrawable::installDefaultShader(root_ss);

    root->addChild(drawable);
    viewer.setSceneData(root);

    EventRouter* router = new EventRouter();
    viewer.addEventHandler(router);

    router->onKeyPress(router->KEY_Leftbracket, [&arena]() {
        OE_NOTICE << "Activating " << arena->size() << " textures" << std::endl;
        for (auto& tex : arena->getTextures())
            arena->activate(tex);
        });

    router->onKeyPress(router->KEY_Rightbracket, [&arena]() {
        OE_NOTICE << "Deactivating " << arena->size() << " textures" << std::endl;
        for (auto& tex : arena->getTextures())
            arena->deactivate(tex);
        });

    OE_NOTICE
        << "\n\n\nHello. There are " << arena->size() << " textures in the arena."
        << "\n\nPress '[' to make resident, ']' to make non-resident"
        << std::endl;

    return viewer.run();
}

int
main(int argc, char** argv)
{
    // new bindlessNV
    return main_NV(argc, argv);

#if 0
    osgEarth::initialize();
    osg::ArgumentParser arguments(&argc, argv);

    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();
    osg::StateSet* root_ss = root->getOrCreateStateSet();

    // Simple shader that will render geometry with bindless textures
    VirtualProgram* vp = VirtualProgram::getOrCreate(root->getOrCreateStateSet());
    vp->addGLSLExtension("GL_ARB_gpu_shader_int64");
    vp->setFunction("vs", vs, ShaderComp::LOCATION_VERTEX_VIEW);
    vp->setFunction("fs", fs, ShaderComp::LOCATION_FRAGMENT_COLORING);

    osg::ref_ptr<TextureArena> arena = new TextureArena();
    arena->setBindingPoint(1);
    root_ss->setAttribute(arena.get(), 1);

    osg::ref_ptr<GeometryCloud> cloud = new GeometryCloud(arena.get());

    float spacing = 0.0;

    for (int i = 1; i < argc; ++i)
    {
        osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(argv[i]);
        if (!node.valid())
            return -1;

        OE_NOTICE << "Loaded " << argv[i] << ", adding to the cloud" << std::endl;

        float radius = node->getBound().radius();
        spacing += radius;
        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
        mt->setMatrix(osg::Matrix::translate(spacing, 0, 0));
        mt->addChild(node);
        auto result = cloud->add(mt.get());
        
        spacing += radius;
    }

    cloud->getGeometry()->setDrawCallback(new GeometryCloudRenderer(cloud.get()));
    root->addChild(cloud->getGeometry());
    viewer.setSceneData(root);

    viewer.addEventHandler(new osgViewer::StatsHandler());

    EventRouter* router = new EventRouter();
    viewer.addEventHandler(router);

    router->onKeyPress(router->KEY_Leftbracket, [&arena]() {
        OE_NOTICE << "Activating " << arena->size() << " textures" << std::endl;
        for (auto& tex : arena->getTextures())
            arena->activate(tex);
        });

    router->onKeyPress(router->KEY_Rightbracket, [&arena]() {
        OE_NOTICE << "Deactivating " << arena->size() << " textures" << std::endl;
        for (auto& tex : arena->getTextures())
            arena->deactivate(tex);
        });

    OE_NOTICE 
        << "\n\n\nHello. There are " << arena->size() << " textures in the arena."
        << "\n\nPress '[' to make resident, ']' to make non-resident" 
        << std::endl;

    return viewer.run();
#endif
}
