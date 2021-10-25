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

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PrimitiveSetIndirect>

using namespace osgEarth;
using namespace osgEarth::Util;

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

const char* vs = R"(
    #version 430
    #extension GL_ARB_gpu_shader_int64 : enable

    layout(binding=5, std430) buffer TextureArena {
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

int
main(int argc, char** argv)
{
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
}
