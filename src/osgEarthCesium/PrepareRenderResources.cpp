/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "PrepareRenderResources"
#include <Cesium3DTilesSelection/GltfUtilities.h>
#include <CesiumGltf/AccessorView.h>

#include <osg/Texture2D>
#include <osg/Geometry>
#include <osgEarth/Notify>
#include <osgEarth/Registry>
#include <osg/MatrixTransform>
#include <glm/gtc/type_ptr.hpp>

using namespace osgEarth::Cesium;


LoadThreadResult::LoadThreadResult()
{
}

LoadThreadResult::~LoadThreadResult()
{    
}

MainThreadResult::MainThreadResult()
{
}

MainThreadResult::~MainThreadResult()
{
}


LoadRasterThreadResult::LoadRasterThreadResult()
{
}

LoadRasterThreadResult::~LoadRasterThreadResult()
{
}

LoadRasterMainThreadResult::LoadRasterMainThreadResult()
{
}

LoadRasterMainThreadResult::~LoadRasterMainThreadResult()
{
}


/********/
// TODO:  Move to own class
class NodeBuilder
{
public:
    NodeBuilder(CesiumGltf::Model* model, const glm::dmat4& transform) :
        _model(model),
        _transform(transform)
    {
        loadArrays();
        loadTextures();        
    }

    osg::Node* build()
    {
        osg::MatrixTransform* root = new osg::MatrixTransform;
        osg::Matrixd matrix;

        glm::dmat4x4 rootTransform = _transform;
        rootTransform = Cesium3DTilesSelection::GltfUtilities::applyRtcCenter(*_model, rootTransform);
        rootTransform = Cesium3DTilesSelection::GltfUtilities::applyGltfUpAxisTransform(*_model, rootTransform);
        matrix.set(glm::value_ptr(rootTransform));

        //matrix.set(glm::value_ptr(_transform));
        root->setMatrix(matrix);
        
        if (!_model->scenes.empty())
        {
            for (auto& scene : _model->scenes)
            {
                for (int node : scene.nodes)
                {
                    root->addChild(createNode(_model->nodes[node]));
                }
            }
        }
        else
        {
            for (auto itr = _model->nodes.begin(); itr != _model->nodes.end(); ++itr)
            {
                root->addChild(createNode(*itr));
            }
        }

        osgEarth::Registry::shaderGenerator().run(root);
        osg::Group* container = new osg::Group;
        container->addChild(root);
        return container;
    }

    osg::Node* createNode(const CesiumGltf::Node& node)
    {
        osg::MatrixTransform* root = new osg::MatrixTransform;
        if (node.matrix.size() == 16)
        {
            osg::Matrixd matrix;
            matrix.set(node.matrix.data());
            root->setMatrix(matrix);
        }


        if (root->getMatrix().isIdentity())
        {
            osg::Matrixd scale, translation, rotation;
            if (node.scale.size() == 3)
            {
                scale = osg::Matrixd::scale(node.scale[0], node.scale[1], node.scale[2]);
            }

            if (node.rotation.size() == 4) {
                osg::Quat quat(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]);
                rotation.makeRotate(quat);
            }

            if (node.translation.size() == 3) {
                translation = osg::Matrixd::translate(node.translation[0], node.translation[1], node.translation[2]);
            }

            root->setMatrix(scale * rotation * translation);
        }

        if (node.mesh >= 0)
        {
            // Build the mesh and add it.
            // TODO:  This mesh needs cached since it can be reused and referenced elsewhere.
            root->addChild(createMesh(_model->meshes[node.mesh]));
        }

        for (int child : node.children)
        {
            root->addChild(createNode(_model->nodes[child]));
        }

        return root;
    }

    template<typename T>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<T>& accessorView)
    {
        return nullptr;
    }

    // TODO:  The rest of the accessor types.

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<uint16_t>>& accessorView)
    {
        osg::UShortArray* result = new osg::UShortArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<uint32_t>>& accessorView)
    {
        osg::UIntArray* result = new osg::UIntArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<uint8_t>>& accessorView)
    {
        osg::UByteArray* result = new osg::UByteArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::VEC3<float>>& accessorView)
    {
        osg::Vec3Array* result = new osg::Vec3Array(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i].set(data.value[0], data.value[1], data.value[2]);
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::VEC2<float>>& accessorView)
    {
        osg::Vec2Array* result = new osg::Vec2Array(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i].set(data.value[0], data.value[1]);
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<float>>& accessorView)
    {
        osg::FloatArray* result = new osg::FloatArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    void loadArrays()
    {
        for (unsigned int i = 0; i < _model->accessors.size(); ++i)
        {
            osg::ref_ptr< osg::Array > osgArray = nullptr;
            CesiumGltf::createAccessorView(*this->_model, i, [&](const auto& accessorView) {
                osgArray = accessorViewToArray(accessorView);
                });

            if (osgArray)
            {
                osgArray->setBinding(osg::Array::BIND_PER_VERTEX);
                osgArray->setNormalize(_model->accessors[i].normalized);
                _arrays.push_back(osgArray);
            }
            else
            {
                _arrays.push_back(nullptr);
            }
        }
    }

    void loadTextures()
    {
        for (unsigned int i = 0; i < _model->textures.size(); ++i)
        {
            auto& texture = _model->textures[i];

            osg::Texture2D* osgTexture = new osg::Texture2D;
            osgTexture->setResizeNonPowerOfTwoHint(false);
            osgTexture->setDataVariance(osg::Object::STATIC);
            // Sampler settings
            if (texture.sampler >= 0 && texture.sampler < _model->samplers.size())
            {
                auto& sampler = _model->samplers[texture.sampler];

                // TODO?  Actually set filter?
                osgTexture->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR_MIPMAP_LINEAR); //sampler.minFilter);
                osgTexture->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR); //sampler.magFilter);
                osgTexture->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)sampler.wrapS);
                osgTexture->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)sampler.wrapT);
                //osgTexture->setWrap(osg::Texture::WRAP_R, (osg::Texture::WrapMode)sampler.wrapR);
            }
            else
            {
                osgTexture->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR_MIPMAP_LINEAR);
                osgTexture->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR);
                osgTexture->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
                osgTexture->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
            }

            // Load the image
            auto& image = _model->images[texture.source].cesium;

            osg::Image* osgImage = nullptr;
            if (image.compressedPixelFormat == CesiumGltf::GpuCompressedPixelFormat::NONE)
            {
                // TODO:  Handle other channel counts
                GLenum format = GL_RGB, texFormat = GL_RGB8;
                if (image.channels == 4) format = GL_RGBA, texFormat = GL_RGBA8;

                osgImage = new osg::Image;
                //OE_NOTICE << "Loading image of size " << image.width << "x" << image.height << " components = " << image.component << " totalSize=" << image.image.size() << std::endl;
                unsigned char* imgData = new unsigned char[image.pixelData.size()];
                memcpy(imgData, &image.pixelData[0], image.pixelData.size());
                osgImage->setImage(image.width, image.height, 1, texFormat, format, GL_UNSIGNED_BYTE, imgData, osg::Image::AllocationMode::USE_NEW_DELETE);
            }
            else
            {
                OE_NOTICE << "TODO:  Compressed textures" << std::endl;
            }

            if (osgImage)
            {
                osgTexture->setImage(osgImage);
            }

            _textures.push_back(osgTexture);
        }
    }

    osg::Node* createMesh(const CesiumGltf::Mesh& mesh)
    {
        osg::Group* geode = new osg::Group;
        for (auto primitive : mesh.primitives)
        {
            osg::ref_ptr< osg::Geometry> geom = new osg::Geometry;
            geom->setUseDisplayList(false);
            geom->setUseVertexBufferObjects(true);

            for (auto& attribute : primitive.attributes)
            {
                const std::string& name = attribute.first;
                osg::Array* osgArray = _arrays[attribute.second];
                if (name == "POSITION")
                {
                    geom->setVertexArray(osgArray);
                }
                else if (name == "COLOR_0")
                {
                    geom->setColorArray(osgArray);
                }
                else if (name == "NORMAL")
                {
                    geom->setNormalArray(osgArray);
                }
                else if (name == "TEXCOORD_0")
                {
                    geom->setTexCoordArray(0, osgArray);
                }
                else if (name == "TEXCOORD_1")
                {
                    geom->setTexCoordArray(1, osgArray);
                }

                // Look for Cesium Overlay texture coordinates.
                const std::string CESIUM_OVERLAY = "_CESIUMOVERLAY_";
                if (osgEarth::startsWith(name, CESIUM_OVERLAY))
                {
                    int index = std::stoi(name.substr(CESIUM_OVERLAY.length()));

                    // Not sure how many overlays or tex coords we should support.  For now we'll support 2 texture coords and two overlays so stick the 
                    // overlay texture coordinates at an index of index + 2.  Revisit this later.
                    geom->setTexCoordArray(index + 2, osgArray);
                }
            }

            // If there is no color array just add one
            if (!geom->getColorArray())
            {
                osg::Vec4Array* colors = new osg::Vec4Array();
                osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
                for (unsigned int i = 0; i < verts->size(); i++)
                {
                    colors->push_back(osg::Vec4(1, 1, 1, 1));
                }
                geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
            }


            if (primitive.indices >= 0)
            {
                osg::Array* primitiveArray = _arrays[primitive.indices];
                GLenum mode = GL_TRIANGLES;
                switch (primitive.mode)
                {
                case CesiumGltf::MeshPrimitive::Mode::TRIANGLES:
                    mode = GL_TRIANGLES;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::TRIANGLE_FAN:
                    mode = GL_TRIANGLE_FAN;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::TRIANGLE_STRIP:
                    mode = GL_TRIANGLE_STRIP;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::LINES:
                    mode = GL_LINES;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::LINE_LOOP:
                    mode = GL_LINES;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::LINE_STRIP:
                    mode = GL_LINE_STRIP;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::POINTS:
                    mode = GL_POINTS;
                    break;
                }

                switch (primitiveArray->getType())
                {
                case osg::Array::UShortArrayType:
                {
                    osg::UShortArray* indices = static_cast<osg::UShortArray*>(primitiveArray);
                    osg::DrawElementsUShort* drawElements = new osg::DrawElementsUShort(mode, indices->begin(), indices->end());
                    geom->addPrimitiveSet(drawElements);
                    break;
                }
                case osg::Array::UIntArrayType:
                {
                    osg::UIntArray* indices = static_cast<osg::UIntArray*>(primitiveArray);
                    osg::DrawElementsUInt* drawElements
                        = new osg::DrawElementsUInt(mode, indices->begin(), indices->end());
                    geom->addPrimitiveSet(drawElements);
                    break;
                }
                case osg::Array::UByteArrayType:
                {
                    osg::UByteArray* indices = static_cast<osg::UByteArray*>(primitiveArray);
                    // DrawElementsUByte doesn't have the constructor with iterator arguments.
                    osg::DrawElementsUByte* drawElements = new osg::DrawElementsUByte(mode, indices->size());
                    std::copy(indices->begin(), indices->end(), drawElements->begin());
                    geom->addPrimitiveSet(drawElements);
                    break;
                }
                }
            }
            else
            {
                // If there are no primitives and we have a vertex array assume it is a point cloud.
                geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, geom->getVertexArray()->getNumElements()));
            }


            if (primitive.material >= 0 && primitive.material < _model->materials.size())
            {
                auto& material = _model->materials[primitive.material];
                auto pbr = material.pbrMetallicRoughness;
                unsigned int baseColorTexture = pbr->baseColorTexture->index;
                if (baseColorTexture >= 0 && baseColorTexture < _textures.size())
                {
                    osg::StateSet* stateSet = geom->getOrCreateStateSet();
                    stateSet->setTextureAttributeAndModes(0, _textures[baseColorTexture], osg::StateAttribute::ON);
                }
            }

            geode->addChild(geom);


        }
        return geode;
    }

    glm::dmat4 _transform;
    CesiumGltf::Model* _model;

    std::vector< osg::ref_ptr< osg::Array> > _arrays;
    std::vector< osg::ref_ptr< osg::Texture2D > > _textures;
};
/********/

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
PrepareRendererResources::prepareInLoadThread(
    const CesiumAsync::AsyncSystem& asyncSystem,
    Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
    const glm::dmat4& transform,
    const std::any& rendererOptions)
{
    CesiumGltf::Model* model = std::get_if<CesiumGltf::Model>(&tileLoadResult.contentKind);
    if (!model)
    {
        return asyncSystem.createResolvedFuture(
            Cesium3DTilesSelection::TileLoadResultAndRenderResources{
                std::move(tileLoadResult),
                nullptr });
    }


    NodeBuilder builder(model, transform);
    LoadThreadResult* result = new LoadThreadResult;
    result->node = builder.build();
    //result->node->setName(tileLoadResult.pCompletedRequest->url());
    return asyncSystem.createResolvedFuture(
        Cesium3DTilesSelection::TileLoadResultAndRenderResources{
            std::move(tileLoadResult),
            result });
}

void* PrepareRendererResources::prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult)
{    
    LoadThreadResult* loadThreadResult = reinterpret_cast<LoadThreadResult*>(pLoadThreadResult);
    MainThreadResult* mainThreadResult = new MainThreadResult();
    mainThreadResult->node = loadThreadResult->node;

    loadThreadResult->node = nullptr;
    delete loadThreadResult;

    return mainThreadResult;    
}

void PrepareRendererResources::free(
    Cesium3DTilesSelection::Tile& tile,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{ 
    LoadThreadResult* loadThreadResult = reinterpret_cast<LoadThreadResult*>(pLoadThreadResult);
    if (loadThreadResult)
    {
        if (loadThreadResult->node.valid())
        {
            //result->node->releaseGLObjects();
            loadThreadResult->node = nullptr;
        }
        delete loadThreadResult;
    }
 
    MainThreadResult* mainThreadResult = reinterpret_cast<MainThreadResult*>(pMainThreadResult);
    if (mainThreadResult)
    {
        if (mainThreadResult->node.valid())
        {
            mainThreadResult->node = nullptr;
        }
        delete mainThreadResult;
    }    
}

void* PrepareRendererResources::prepareRasterInLoadThread(
    CesiumGltf::ImageCesium& image,
    const std::any& rendererOptions)
{
    osg::Image* osgImage = new osg::Image;
    // TODO:  Handle other channel counts and make this a generic function.
    GLenum format = GL_RGB, texFormat = GL_RGB8;
    if (image.channels == 4) format = GL_RGBA, texFormat = GL_RGBA8;

    unsigned char* imgData = new unsigned char[image.pixelData.size()];
    memcpy(imgData, &image.pixelData[0], image.pixelData.size());
    osgImage->setImage(image.width, image.height, 1, texFormat, format, GL_UNSIGNED_BYTE, imgData, osg::Image::AllocationMode::USE_NEW_DELETE);

    LoadRasterThreadResult* result = new LoadRasterThreadResult;
    result->image = osgImage;
    return result;
}

void* PrepareRendererResources::prepareRasterInMainThread(
    Cesium3DTilesSelection::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult)
{
    LoadRasterThreadResult* loadThreadResult = reinterpret_cast<LoadRasterThreadResult*>(pLoadThreadResult);

    if (!loadThreadResult)
    {
        return nullptr;
    }

    LoadRasterMainThreadResult* result = new LoadRasterMainThreadResult;
    result->image = loadThreadResult->image;
    loadThreadResult->image = nullptr;
    delete loadThreadResult;
    return result;
}

void PrepareRendererResources::freeRaster(
    const Cesium3DTilesSelection::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    LoadRasterThreadResult* loadThreadResult = reinterpret_cast<LoadRasterThreadResult*>(pLoadThreadResult);
    if (loadThreadResult)
    {
        loadThreadResult->image = nullptr;
        delete loadThreadResult;
    }

    LoadRasterMainThreadResult* loadMainThreadResult = reinterpret_cast<LoadRasterMainThreadResult*>(pMainThreadResult);
    if (loadMainThreadResult)
    {
        loadMainThreadResult->image = nullptr;
        delete loadMainThreadResult;
    }
}

const char* raster_overlay_vs = R"(
            #version 330
            out vec4 oe_raster_overlay_coords;
            uniform int oe_raster_overlay_coord;
            void raster_overlay_model(inout vec4 vertex)
            {
                switch (oe_raster_overlay_coord) {
                    case 0:
                        oe_raster_overlay_coords = gl_MultiTexCoord0;
                        break;
                    case 1:
                        oe_raster_overlay_coords = gl_MultiTexCoord1;
                        break;
                    case 2:
                        oe_raster_overlay_coords = gl_MultiTexCoord2;
                        break;
                    case 3:
                        oe_raster_overlay_coords = gl_MultiTexCoord3;
                        break;
                }
            }
        )";

const char* raster_overlay_fs = R"(
            #version 330
            in vec4 oe_raster_overlay_coords;
            uniform sampler2D oe_raster_overlay_sampler;
            uniform vec4 oe_translation_scale;
            
            void raster_overlay_color(inout vec4 color)
            {
                vec4 texel;
                vec2 trans = oe_translation_scale.xy;
                vec2 scale = oe_translation_scale.zw;

                vec2 texcoords = oe_raster_overlay_coords.xy * scale + trans;
                texcoords.t = 1.0 - texcoords.t;
                texel = texture(oe_raster_overlay_sampler, texcoords);
                color = color * texel;
            }
        )";

void PrepareRendererResources::attachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const Cesium3DTilesSelection::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources,
    const glm::dvec2& translation,
    const glm::dvec2& scale)
{
    LoadRasterMainThreadResult* loadMainThreadResult = reinterpret_cast<LoadRasterMainThreadResult*>(pMainThreadRendererResources);
    if (!loadMainThreadResult)
    {
        return;
    }

    const Cesium3DTilesSelection::TileContent& content = tile.getContent();
    const Cesium3DTilesSelection::TileRenderContent* renderContent = content.getRenderContent();

    MainThreadResult* renderResult = reinterpret_cast<MainThreadResult*>(renderContent->getRenderResources());
    if (renderResult)
    {
        int overlayUnit = overlayTextureCoordinateID + 2;
        osg::StateSet* stateset = renderResult->node->getOrCreateStateSet();
        osg::Texture2D* osgTexture = new osg::Texture2D(loadMainThreadResult->image.get());
        osgTexture->setResizeNonPowerOfTwoHint(false);
        osgTexture->setDataVariance(osg::Object::STATIC);
        osgTexture->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR_MIPMAP_LINEAR);
        osgTexture->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR);
        osgTexture->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
        osgTexture->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
        stateset->setTextureAttributeAndModes(overlayUnit, osgTexture, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);


        stateset->getOrCreateUniform("oe_raster_overlay_coord", osg::Uniform::INT)->set(overlayUnit);        
        stateset->getOrCreateUniform("oe_raster_overlay_sampler", osg::Uniform::SAMPLER_2D)->set(overlayUnit);

        osg::Uniform* uniform_and_scale = new osg::Uniform("oe_translation_scale", osg::Vec4f(translation.x, translation.y, scale.x, scale.y));
        stateset->addUniform(uniform_and_scale);

        VirtualProgram::getOrCreate(stateset)->setFunction("raster_overlay_color", raster_overlay_fs, VirtualProgram::LOCATION_FRAGMENT_COLORING);
        VirtualProgram::getOrCreate(stateset)->setFunction("raster_overlay_model", raster_overlay_vs, VirtualProgram::LOCATION_VERTEX_MODEL);

    }
}

void PrepareRendererResources::detachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const Cesium3DTilesSelection::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources) noexcept
{
    // TODO:
    OE_NOTICE << "detachRasterInMainThread" << std::endl;
}