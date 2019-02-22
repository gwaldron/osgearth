/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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

#include <iostream>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <osg/Notify>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgEarth/Notify>
#include <osgEarth/URI>
#include <osgEarth/StringUtils>
#include <osgEarth/Endian>
#include <osgEarth/JsonUtils>

#include <iostream>
#include <iomanip>

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
// #define TINYGLTF_NOEXCEPTION // optional. disable exception handling.
#include "tiny_gltf.h"
using namespace tinygltf;
using namespace osgEarth;

namespace gltf = tinygltf;


#define LC "[gltf] "

typedef std::map<const osg::Node*, int> OsgNodeSequenceMap;
typedef std::map<const osg::BufferData*, int> ArraySequenceMap;
typedef std::map<const osg::Array*, int> AccessorSequenceMap;

//! Visitor that builds a GLTF data model from an OSG scene graph.
class OSGtoGLTF : public osg::NodeVisitor
{
private:
   gltf::Model& _model;
   OpenThreads::Atomic _nameGen;
   std::stack<gltf::Node*> _gltfNodeStack;
   OsgNodeSequenceMap _osgNodeSeqMap;
   ArraySequenceMap _buffers;
   ArraySequenceMap _bufferViews;
   ArraySequenceMap _accessors;

public:
    OSGtoGLTF(gltf::Model& model) : _model(model)
    {
        setTraversalMode(TRAVERSE_ALL_CHILDREN);
        setNodeMaskOverride(~0);

        // default root scene:
        _model.scenes.push_back(gltf::Scene());
        gltf::Scene& scene = _model.scenes.back();
        _model.defaultScene = 0;
    }

    void push(gltf::Node& gnode)
    {
        _gltfNodeStack.push(&gnode);
    }

    void pop()
    {
        _gltfNodeStack.pop();
    }

    void apply(osg::Node& node)
    {
        bool isRoot = _model.scenes[_model.defaultScene].nodes.empty();
        if (isRoot)
        {
            // put a placeholder here just to prevent any other nodes
            // from thinking they are the root
            _model.scenes[_model.defaultScene].nodes.push_back(-1);
        }

        traverse(node);

        _model.nodes.push_back(gltf::Node());
        gltf::Node& gnode = _model.nodes.back();
        int id = _model.nodes.size()-1;
        gnode.name = Stringify() << "_gltfNode_" << id;
        _osgNodeSeqMap[&node] = id;

        if (isRoot)
        {
            // replace the placeholder with the actual root id.
            _model.scenes[_model.defaultScene].nodes.back() = id;
        }
    }

    void apply(osg::Group& group)
    {
        apply(static_cast<osg::Node&>(group));

        for(unsigned i=0; i<group.getNumChildren(); ++i)
        {
            int id = _osgNodeSeqMap[group.getChild(i)];
            _model.nodes.back().children.push_back(id);
        }
    }

    void apply(osg::Transform& xform)
    {
        apply(static_cast<osg::Group&>(xform));

        osg::Matrix matrix;
        xform.computeLocalToWorldMatrix(matrix, this);
        const double* ptr = matrix.ptr();
        for(unsigned i=0; i<16; ++i)
            _model.nodes.back().matrix.push_back(*ptr++);
    }

    unsigned getBytesInDataType(GLenum dataType)
    {
        return
            dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE   ? 1 :
            dataType == GL_SHORT || dataType == GL_UNSIGNED_SHORT ? 2 :
            dataType == GL_INT || dataType == GL_UNSIGNED_INT || dataType == GL_FLOAT ? 4 :
            0;
    }

    unsigned getBytesPerElement(const osg::Array* data)
    {
        return data->getDataSize() * getBytesInDataType(data->getDataType());
    }

    unsigned getBytesPerElement(const osg::DrawElements* data)
    {
        return
            dynamic_cast<const osg::DrawElementsUByte*>(data)? 1 :
            dynamic_cast<const osg::DrawElementsUShort*>(data) ? 2 :
            4;
    }

    int getOrCreateBuffer(const osg::BufferData* data, GLenum type)
    {
        ArraySequenceMap::iterator a = _buffers.find(data);
        if (a != _buffers.end())
            return a->second;

        _model.buffers.push_back(gltf::Buffer());
        gltf::Buffer& buffer = _model.buffers.back();
        int id = _model.buffers.size() - 1;
        _buffers[data] = id;

        int bytes = getBytesInDataType(type);
        buffer.data.resize(data->getTotalDataSize());

        //TODO: account for endianess
        unsigned char* ptr = (unsigned char*)(data->getDataPointer());
        for(unsigned i=0; i<data->getTotalDataSize(); ++i)
            buffer.data[i] = *ptr++;

        return id;
    }

    int getOrCreateBufferView(const osg::BufferData* data, GLenum type, GLenum target)
    {
        ArraySequenceMap::iterator a = _bufferViews.find(data);
        if (a != _bufferViews.end())
            return a->second;

        int bufferId = -1;
        ArraySequenceMap::iterator buffersIter = _buffers.find(data);
        if (buffersIter != _buffers.end())
            bufferId = buffersIter->second;
        else
            bufferId = getOrCreateBuffer(data, type);

        _model.bufferViews.push_back(gltf::BufferView());
        gltf::BufferView& bv = _model.bufferViews.back();
        int id = _model.bufferViews.size()-1;
        _bufferViews[data] = id;

        bv.buffer = bufferId;
        bv.byteLength = data->getTotalDataSize();
        bv.byteOffset = 0;
        bv.target = target;

        //ONLY used for vertex attrbs, I guess:
        //unsigned bytesPerComponent = getBytesPerComponent(data->getDataType());
        //unsigned componentsPerElement = data->getDataSize();
        //bv.byteStride = bytesPerComponent * componentsPerElement;

        return id;
    }

    int getOrCreateAccessor(osg::Array* data, osg::PrimitiveSet* pset, gltf::Primitive& prim, const std::string& attr)
    {
        ArraySequenceMap::iterator a = _accessors.find(data);
        if ( a != _accessors.end())
            return a->second;

        ArraySequenceMap::iterator bv = _bufferViews.find(data);
        if (bv == _bufferViews.end())
            return -1;

        _model.accessors.push_back(gltf::Accessor());
        gltf::Accessor& accessor = _model.accessors.back();
        int accessorId = _model.accessors.size() - 1;
        prim.attributes[attr] = accessorId;

        accessor.type =
            data->getDataSize() == 1 ? TINYGLTF_TYPE_SCALAR :
            data->getDataSize() == 2 ? TINYGLTF_TYPE_VEC2 :
            data->getDataSize() == 3 ? TINYGLTF_TYPE_VEC3 :
            data->getDataSize() == 4 ? TINYGLTF_TYPE_VEC4 :
            TINYGLTF_TYPE_SCALAR;

        accessor.bufferView = bv->second;
        accessor.byteOffset = 0;
        accessor.componentType = data->getDataType();
        accessor.count = data->getNumElements();

        const osg::DrawArrays* da = dynamic_cast<const osg::DrawArrays*>(pset);
        if (da)
        {
            accessor.byteOffset = da->getFirst() * getBytesPerElement(data);            
        }

        //TODO: indexed elements
        osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(pset);
        if (de)
        {
            _model.accessors.push_back(gltf::Accessor());
            gltf::Accessor& idxAccessor = _model.accessors.back();
            prim.indices = _model.accessors.size() - 1;

            idxAccessor.type = TINYGLTF_TYPE_SCALAR;
            idxAccessor.byteOffset = 0;
            idxAccessor.componentType = de->getDataType();
            idxAccessor.count = de->getNumIndices();

            getOrCreateBuffer(de, idxAccessor.componentType);
            int idxBV = getOrCreateBufferView(de, idxAccessor.componentType, GL_ELEMENT_ARRAY_BUFFER);

            idxAccessor.bufferView = idxBV;
        }

        return accessorId;
    }

    void apply(osg::Drawable& drawable)
    {
        if (drawable.asGeometry())
        {
            apply(static_cast<osg::Node&>(drawable));

            osg::Geometry* geom = drawable.asGeometry();

            _model.meshes.push_back(gltf::Mesh());
            gltf::Mesh& mesh = _model.meshes.back();
            _model.nodes.back().mesh = _model.meshes.size() - 1;

            osg::Vec3f posMin(FLT_MAX, FLT_MAX, FLT_MAX);
            osg::Vec3f posMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
            osg::Vec3Array* positions = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
            if (positions)
            {
                getOrCreateBufferView(positions, GL_FLOAT, GL_ARRAY_BUFFER);
                for(unsigned i=0; i<positions->size(); ++i)
                {
                    const osg::Vec3f& v = (*positions)[i];
                    posMin.x() = osg::minimum(posMin.x(), v.x());
                    posMin.y() = osg::minimum(posMin.y(), v.y());
                    posMin.z() = osg::minimum(posMin.z(), v.z());
                    posMax.x() = osg::maximum(posMax.x(), v.x());
                    posMax.y() = osg::maximum(posMax.y(), v.y());
                    posMax.z() = osg::maximum(posMax.z(), v.z());
                }
            }

            osg::Vec3Array* normals = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
            if (normals)
            {
                getOrCreateBufferView(normals, GL_FLOAT, GL_ARRAY_BUFFER);
            }

            osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
            if (colors)
            {
                getOrCreateBufferView(colors, GL_FLOAT, GL_ARRAY_BUFFER);
            }

            for(unsigned i=0; i<geom->getNumPrimitiveSets(); ++i)
            {
                osg::PrimitiveSet* pset = geom->getPrimitiveSet(i);

                mesh.primitives.push_back(gltf::Primitive());
                gltf::Primitive& primitive = mesh.primitives.back();

                primitive.mode = pset->getMode();
                    
                int a = getOrCreateAccessor(positions, pset, primitive, "POSITION");

                // record min/max for position array (required):
                gltf::Accessor& posacc = _model.accessors[a];
                posacc.minValues.push_back(posMin.x());
                posacc.minValues.push_back(posMin.y());
                posacc.minValues.push_back(posMin.z());
                posacc.maxValues.push_back(posMax.x());
                posacc.maxValues.push_back(posMax.y());
                posacc.maxValues.push_back(posMax.z());

                getOrCreateAccessor(normals, pset, primitive, "NORMAL");

                getOrCreateAccessor(colors, pset, primitive, "COLOR_0");
            }
        }
    }
};


class GLTFReader : public osgDB::ReaderWriter
{
public:
    GLTFReader()
    {
        supportsExtension("gltf", "glTF ascii loader");
        supportsExtension("glb", "glTF binary loader");
        supportsExtension("b3dm", "b3dm loader");
    }

    virtual const char* className() const { return "glTF Loader"; }

    // Turn all of the accessors and turn them into arrays
    void extractArrays(const tinygltf::Model &model, std::vector<osg::ref_ptr<osg::Array>> &arrays) const
    {
        for (unsigned int i = 0; i < model.accessors.size(); i++)
        {
            const Accessor& accessor = model.accessors[i];
            const BufferView& bufferView = model.bufferViews[accessor.bufferView];
            const Buffer& buffer = model.buffers[bufferView.buffer];


            osg::ref_ptr< osg::Array > osgArray;

            if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)
            {
                if (accessor.type == TINYGLTF_TYPE_SCALAR)
                {
                    osg::FloatArray* floatArray = new osg::FloatArray;
                    const BufferView& bufferView = model.bufferViews[accessor.bufferView];
                    const Buffer& buffer = model.buffers[bufferView.buffer];

                    float* array = (float*)(&buffer.data.at(0) + accessor.byteOffset + bufferView.byteOffset);
                    unsigned int pos = 0;
                    for (unsigned int j = 0; j < accessor.count; j++)
                    {
                        float s = array[pos];
                        if (bufferView.byteStride > 0)
                        {
                            pos += (bufferView.byteStride / 4);
                        }
                        else
                        {
                            pos += 1;
                        }
                        floatArray->push_back(s);
                    }
                    osgArray = floatArray;
                }
                else if (accessor.type == TINYGLTF_TYPE_VEC2)
                {
                    osg::Vec2Array* vec2Array = new osg::Vec2Array;
                    const BufferView& bufferView = model.bufferViews[accessor.bufferView];
                    const Buffer& buffer = model.buffers[bufferView.buffer];

                    float* array = (float*)(&buffer.data.at(0) + accessor.byteOffset + bufferView.byteOffset);
                    unsigned int pos = 0;
                    for (unsigned int j = 0; j < accessor.count; j++)
                    {
                        float s = array[pos];
                        float t = array[pos + 1];
                        if (bufferView.byteStride > 0)
                        {
                            pos += (bufferView.byteStride / 4);
                        }
                        else
                        {
                            pos += 2;
                        }
                        vec2Array->push_back(osg::Vec2(s, t));
                    }
                    osgArray = vec2Array;
                }
                else if (accessor.type == TINYGLTF_TYPE_VEC3)
                {
                    osg::Vec3Array* vec3Array = new osg::Vec3Array;
                    float* array = (float*)(&buffer.data.at(0) + accessor.byteOffset + bufferView.byteOffset);
                    unsigned int pos = 0;
                    for (unsigned int j = 0; j < accessor.count; j++)
                    {
                        float x = array[pos];
                        float y = array[pos + 1];
                        float z = array[pos + 2];
                        if (bufferView.byteStride > 0)
                        {
                            pos += (bufferView.byteStride / 4);
                        }
                        else
                        {
                            pos += 3;
                        }
                        vec3Array->push_back(osg::Vec3(x, y, z));
                        osgArray = vec3Array;
                    }
                }
                else if (accessor.type == TINYGLTF_TYPE_VEC4)
                {
                    osg::Vec4Array* vec4Array = new osg::Vec4Array;
                    float* array = (float*)(&buffer.data.at(0) + accessor.byteOffset + bufferView.byteOffset);
                    unsigned int pos = 0;
                    for (unsigned int j = 0; j < accessor.count; j++)
                    {
                        float r = array[pos];
                        float g = array[pos + 1];
                        float b = array[pos + 2];
                        float a = array[pos + 3];
                        if (bufferView.byteStride > 0)
                        {
                            pos += (bufferView.byteStride / 4);
                        }
                        else
                        {
                            pos += 4;
                        }
                        vec4Array->push_back(osg::Vec4(r, g, b, a));
                        osgArray = vec4Array;
                    }
                }
            }

#if 0
            else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE && accessor.type == TINYGLTF_TYPE_SCALAR)
            {
                osg::UByteArray* output = new osg::UByteArray();
                unsigned char* array = (unsigned char*)(&buffer.data[0] + accessor.byteOffset + bufferView.byteOffset);
                for(unsigned j=0; j<accessor.count; ++j)
                {
                    output->push_back(array[j]);
                }
                osgArray = output;
            }
            else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT && accessor.type == TINYGLTF_TYPE_SCALAR)
            {
                osg::UShortArray* output = new osg::UShortArray();
                unsigned short* array = (unsigned short*)(&buffer.data[0] + accessor.byteOffset + bufferView.byteOffset);
                for (unsigned j = 0; j < accessor.count; ++j)
                {
                    output->push_back(array[j*2]);
                }
                osgArray = output;
            }
            else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT && accessor.type == TINYGLTF_TYPE_SCALAR)
            {
                osg::UIntArray* output = new osg::UIntArray();
                unsigned int* array = (unsigned int*)(&buffer.data[0] + accessor.byteOffset + bufferView.byteOffset);
                for (unsigned j = 0; j < accessor.count; ++j)
                {
                    output->push_back(array[j * 4]);
                }
                osgArray = output;
            }
#endif


            if (osgArray.valid())
            {
                osgArray->setBinding(osg::Array::BIND_PER_VERTEX);
            }
            else
            {
                OSG_DEBUG << "Adding null array for " << i << std::endl;
            }
            arrays.push_back(osgArray);
        }
    }


    osg::Node* makeMesh(const tinygltf::Model &model, const tinygltf::Mesh& mesh) const
    {
        osg::Group *group = new osg::Group;

        std::vector< osg::ref_ptr< osg::Array > > arrays;
        extractArrays(model, arrays);

        OSG_NOTICE << "Drawing " << mesh.primitives.size() << " primitives in mesh" << std::endl;

        for (size_t i = 0; i < mesh.primitives.size(); i++) {

            OSG_NOTICE << " Processing primitive " << i << std::endl;
            const tinygltf::Primitive &primitive = mesh.primitives[i];
            if (primitive.indices < 0)
            {
                return 0;
            }

            osg::ref_ptr< osg::Geometry > geom = new osg::Geometry;
            geom->setUseDisplayList(false);
            geom->setUseVertexBufferObjects(true);

            group->addChild(geom.get());

            // The base color factor of the material
            osg::Vec4 baseColorFactor(1.0f, 1.0f, 1.0f, 1.0f);

            if (primitive.material >= 0)
            {
                const tinygltf::Material& material = model.materials[primitive.material];

                /*
                OSG_NOTICE << "extCommonValues=" << material.extCommonValues.size() << std::endl;
                for (ParameterMap::iterator paramItr = material.extCommonValues.begin(); paramItr != material.extCommonValues.end(); ++paramItr)
                {
                    OSG_NOTICE << paramItr->first << "=" << paramItr->second.string_value << std::endl;
                }
                */

                OSG_NOTICE << "additionalValues=" << material.additionalValues.size() << std::endl;
                for (ParameterMap::const_iterator paramItr = material.additionalValues.begin(); paramItr != material.additionalValues.end(); ++paramItr)
                {
                    OSG_NOTICE << "    " << paramItr->first << "=" << paramItr->second.string_value << std::endl;
                }

                //OSG_NOTICE << "values=" << material.values.size() << std::endl;
                for (ParameterMap::const_iterator paramItr = material.values.begin(); paramItr != material.values.end(); ++paramItr)
                {
                    if (paramItr->first == "baseColorFactor")
                    {
                        ColorValue color = paramItr->second.ColorFactor();
                        baseColorFactor = osg::Vec4(color[0], color[1], color[2], color[3]);
                    }
                    else
                    {
                        OSG_NOTICE << "    " << paramItr->first << "=" << paramItr->second.string_value << std::endl;
                    }

                }
                /*
                OSG_NOTICE << "extPBRValues=" << material.extPBRValues.size() << std::endl;
                for (ParameterMap::iterator paramItr = material.extPBRValues.begin(); paramItr != material.extPBRValues.end(); ++paramItr)
                {
                    OSG_NOTICE << paramItr->first << "=" << paramItr->second.string_value << std::endl;
                }
                */

                for (ParameterMap::const_iterator paramItr = material.values.begin(); paramItr != material.values.end(); ++paramItr)
                {
                    if (paramItr->first == "baseColorTexture")
                    {
                        std::map< std::string, double>::const_iterator i = paramItr->second.json_double_value.find("index");
                        if (i != paramItr->second.json_double_value.end())
                        {
                            int index = i->second;

                            const tinygltf::Texture& texture = model.textures[index];

                            osg::Texture2D* tex = new osg::Texture2D;
                            if (texture.sampler >= 0 && texture.sampler < model.samplers.size())
                            {
                                const Sampler& sampler = model.samplers[texture.sampler];
                                tex->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)sampler.minFilter);
                                tex->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)sampler.magFilter);
                                tex->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)sampler.wrapS);
                                tex->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)sampler.wrapT);
                                tex->setWrap(osg::Texture::WRAP_R, (osg::Texture::WrapMode)sampler.wrapR);
                            }

                            const tinygltf::Image& image = model.images[texture.source];
                            osg::ref_ptr< osg::Image> img = new osg::Image;

                            GLenum format = GL_RGB;
                            if (image.component == 4) format = GL_RGBA;

                            if (image.image.size() > 0)
                            {
                                unsigned char *imgData = new unsigned char[image.image.size()];
                                memcpy(imgData, &image.image.at(0), image.image.size());
                                img->setImage(image.width, image.height, 1, format, format, GL_UNSIGNED_BYTE, imgData, osg::Image::AllocationMode::USE_NEW_DELETE);
                            }

                            tex->setImage(img);
                            geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
                        }
                    }
                }
            }

            // Always make a color array with the base color
            osg::Vec4Array* colors = new osg::Vec4Array();
            geom->setColorArray(colors, osg::Array::BIND_OVERALL);
            colors->push_back(baseColorFactor);

            std::map<std::string, int>::const_iterator it(primitive.attributes.begin());
            std::map<std::string, int>::const_iterator itEnd(
                primitive.attributes.end());

            for (; it != itEnd; it++)
            {
                const tinygltf::Accessor &accessor = model.accessors[it->second];

                if (it->first.compare("POSITION") == 0)
                {
                    geom->setVertexArray(arrays[it->second]);
                }

                else if (it->first.compare("NORMAL") == 0)
                {
                    geom->setNormalArray(arrays[it->second]);
                }
                else if (it->first.compare("TEXCOORD_0") == 0)
                {
                    geom->setTexCoordArray(0, arrays[it->second]);
                }
                else if (it->first.compare("TEXCOORD_1") == 0)
                {
                    geom->setTexCoordArray(1, arrays[it->second]);
                }
                else if (it->first.compare("COLOR_0") == 0)
                {
                    OSG_NOTICE << "Setting color array " << arrays[it->second].get() << std::endl;
                    geom->setColorArray(arrays[it->second]);
                }
                else
                {
                    OSG_NOTICE << "Skipping array " << it->first << std::endl;
                }
            }

            const tinygltf::Accessor &indexAccessor =
                model.accessors[primitive.indices];

            int mode = -1;
            if (primitive.mode == TINYGLTF_MODE_TRIANGLES) {
                mode = GL_TRIANGLES;
            }
            else if (primitive.mode == TINYGLTF_MODE_TRIANGLE_STRIP) {
                mode = GL_TRIANGLE_STRIP;
            }
            else if (primitive.mode == TINYGLTF_MODE_TRIANGLE_FAN) {
                mode = GL_TRIANGLE_FAN;
            }
            else if (primitive.mode == TINYGLTF_MODE_POINTS) {
                mode = GL_POINTS;
            }
            else if (primitive.mode == TINYGLTF_MODE_LINE) {
                mode = GL_LINES;
            }
            else if (primitive.mode == TINYGLTF_MODE_LINE_LOOP) {
                mode = GL_LINE_LOOP;
            }

            {
                const BufferView& bufferView = model.bufferViews[indexAccessor.bufferView];
                const Buffer& buffer = model.buffers[bufferView.buffer];

                if (indexAccessor.componentType == GL_UNSIGNED_SHORT)
                {
                    osg::DrawElementsUShort* drawElements = new osg::DrawElementsUShort(mode);
                    unsigned short* indices = (unsigned short*)(&buffer.data.at(0) + bufferView.byteOffset + indexAccessor.byteOffset);
                    for (unsigned int j = 0; j < indexAccessor.count; j++)
                    {
                        unsigned short index = indices[j];
                        drawElements->push_back(index);
                    }
                    geom->addPrimitiveSet(drawElements);
                }
                else if (indexAccessor.componentType == GL_UNSIGNED_INT)
                {
                    osg::DrawElementsUInt* drawElements = new osg::DrawElementsUInt(mode);
                    unsigned int* indices = (unsigned int*)(&buffer.data.at(0) + bufferView.byteOffset + indexAccessor.byteOffset);
                    for (unsigned int j = 0; j < indexAccessor.count; j++)
                    {
                        unsigned int index = indices[j];
                        drawElements->push_back(index);
                    }
                    geom->addPrimitiveSet(drawElements);
                }
                else if (indexAccessor.componentType == GL_UNSIGNED_BYTE)
                {
                    osg::DrawElementsUByte* drawElements = new osg::DrawElementsUByte(mode);
                    unsigned char* indices = (unsigned char*)(&buffer.data.at(0) + bufferView.byteOffset + indexAccessor.byteOffset);
                    for (unsigned int j = 0; j < indexAccessor.count; j++)
                    {
                        unsigned char index = indices[j];
                        drawElements->push_back(index);
                    }
                    geom->addPrimitiveSet(drawElements);
                }
            }
        }

        return group;
    }


    osg::Node* createNode(const tinygltf::Model &model, const tinygltf::Node& node) const
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setName(node.name);
        if (node.matrix.size() == 16)
        {
            osg::Matrixd mat;
            mat.set(node.matrix.data());
            mt->setMatrix(mat);
        }
        else
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

            mt->setMatrix(scale * rotation * translation);
        }


        // todo transformation
        if (node.mesh >= 0)
        {
            mt->addChild(makeMesh(model, model.meshes[node.mesh]));
        }

        // Load any children.
        for (unsigned int i = 0; i < node.children.size(); i++)
        {
            osg::Node* child = createNode(model, model.nodes[node.children[i]]);
            if (child)
            {
                mt->addChild(child);
            }
        }
        return mt;
    }

    osg::Node* makeNodeFromModel(const tinygltf::Model &model) const
    {
        // Rotate y-up to z-up
        osg::MatrixTransform* transform = new osg::MatrixTransform;
        transform->setMatrix(osg::Matrixd::rotate(osg::Vec3d(0.0, 1.0, 0.0), osg::Vec3d(0.0, 0.0, 1.0)));

        for (unsigned int i = 0; i < model.scenes.size(); i++)
        {         
            const tinygltf::Scene &scene = model.scenes[i];

            for (size_t j = 0; j < scene.nodes.size(); j++) {
                osg::Node* node = createNode(model, model.nodes[scene.nodes[j]]);
                if (node)
                {
                    transform->addChild(node);
                }
            }
        }

        return transform;
    }

    struct b3dmheader
    {
        char magic[4];
        unsigned int version;
        unsigned int byteLength;
        unsigned int featureTableJSONByteLength;
        unsigned int featureTableBinaryByteLength;
        unsigned int batchTableJSONByteLength;
        unsigned int batchTableBinaryByteLength;
    };

    osg::Node* loadb3dm(const std::string& location) const
    {
        // Load the whole thing into memory
        URIStream inputStream(location, std::ifstream::binary);

        std::istreambuf_iterator<char> eof;
        std::string data(std::istreambuf_iterator<char>(inputStream), eof);

        // Check the header's magic string. If it's not there, attempt
        // to run a decompressor on it

        std::string magic(data, 0, 4);
        if (magic != "b3dm")
        {
            osg::ref_ptr<osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
            if (compressor.valid())
            {
                std::stringstream in_data(data);
                std::string temp;
                if (!compressor->decompress(in_data, temp))
                {
                    OE_WARN << LC << "Invalid b3dm" << std::endl;
                    return NULL;
                }
                data = temp;
            }
        }

        b3dmheader header;
        unsigned bytesRead = 0;

        std::stringstream buf(data);
        buf.read(reinterpret_cast<char*>(&header), sizeof(b3dmheader));
        bytesRead += sizeof(b3dmheader);

#ifdef OE_IS_BIG_ENDIAN
        byteSwapInPlace(header.version);
        byteSwapInPlace(header.byteLength);
        byteSwapInPlace(header.featureTableJSONByteLength);
        byteSwapInPlace(header.featureTableBinaryByteLength);
        byteSwapInPlace(header.batchTableJSONByteLength);
        byteSwapInPlace(header.batchTableBinaryByteLength);
#endif
        
        size_t sz = header.byteLength;

        osg::Vec3d rtc_center;

        if (header.featureTableJSONByteLength > 0)
        {
            std::string featureTableJson;
            featureTableJson.resize(header.featureTableJSONByteLength);
            buf.read(reinterpret_cast<char*>(&featureTableJson[0]), header.featureTableJSONByteLength);
            OSG_NOTICE << "Read featureTableJson " << featureTableJson << std::endl;

            json ftJson = json::parse(featureTableJson);
            if (ftJson.find("RTC_CENTER") != ftJson.end()) {
                json RTC_CENTER = ftJson["RTC_CENTER"];
                rtc_center.x() = RTC_CENTER[0];
                rtc_center.y() = RTC_CENTER[1];
                rtc_center.z() = RTC_CENTER[2];
            }
            OE_INFO << LC << "Read rtc_center " << rtc_center.x() << ", " << rtc_center.y() << ", " << rtc_center.z() << std::endl;

            bytesRead += header.featureTableJSONByteLength;
        }

        if (header.featureTableBinaryByteLength > 0)
        {
            std::string featureTableBinary;
            featureTableBinary.resize(header.featureTableBinaryByteLength);
            buf.read(reinterpret_cast<char*>(&featureTableBinary[0]), header.featureTableBinaryByteLength);
            OSG_NOTICE << "Read featureTableJson " << featureTableBinary << std::endl;
            bytesRead += header.featureTableBinaryByteLength;
        }

        if (header.batchTableJSONByteLength > 0)
        {
            std::string batchTableJSON;
            batchTableJSON.resize(header.batchTableJSONByteLength);
            buf.read(reinterpret_cast<char*>(&batchTableJSON[0]), header.batchTableJSONByteLength);
            OSG_NOTICE << "Read batchTableJSON " << batchTableJSON << std::endl;
            bytesRead += header.batchTableJSONByteLength;
        }

        if (header.batchTableBinaryByteLength > 0)
        {
            std::string batchTableBinary;
            batchTableBinary.resize(header.batchTableBinaryByteLength);
            buf.read(reinterpret_cast<char*>(&batchTableBinary[0]), header.batchTableBinaryByteLength);
            OSG_NOTICE << "Read batchTableJSON " << batchTableBinary << std::endl;
            bytesRead += header.batchTableBinaryByteLength;
        }

        std::string gltfData;
        gltfData.resize(sz - bytesRead);
        buf.read(reinterpret_cast<char *>(&gltfData[0]), static_cast<std::streamsize>(gltfData.size()));

        Model model;
        TinyGLTF loader;
        std::string err;
        std::string warn;
        loader.LoadBinaryFromMemory(&model, &err, &warn, reinterpret_cast<unsigned char*>(&gltfData[0]), sz);


        osg::MatrixTransform *mt = new osg::MatrixTransform;

        osg::Node* modelNode = makeNodeFromModel(model);
        if (rtc_center.x() == 0.0 && rtc_center.y() == 0.0 && rtc_center.z() == 0.0)
        {
            return modelNode;
        }
        else
        {
            osg::MatrixTransform* mt = new osg::MatrixTransform;
            mt->setMatrix(osg::Matrix::translate(rtc_center));
            mt->addChild(modelNode);
            return mt;
        }
    }

    virtual ReadResult readObject(const std::string& location, const Options* opt) const
    {
        return readNode(location, opt);
    }

    virtual ReadResult readNode(const std::string& location, const Options* options) const
    {
        std::string ext = osgDB::getFileExtension(location);
        if (!acceptsExtension(ext))
            return ReadResult::FILE_NOT_HANDLED;

        Model model;
        TinyGLTF loader;
        std::string err;
        std::string warn;

        OE_INFO << LC << "Load: " << location << std::endl;

        if (ext == "glb")
        {
            loader.LoadBinaryFromFile(&model, &err, &warn, location);
        }
        else if (ext == "b3dm")
        {
            return loadb3dm(location);
        }
        else
        {
            loader.LoadASCIIFromFile(&model, &err, &warn, location);
        }
        if (!err.empty()) {
            OE_WARN << LC << "gltf Error loading " << location << std::endl;
            OE_WARN << LC << err << std::endl;
            return ReadResult::ERROR_IN_READING_FILE;
        }

        return makeNodeFromModel(model);
    }

    //! Writes a node to GLTF.
    WriteResult writeNode(const osg::Node& node, const std::string& location, const Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension(location);
        if (!acceptsExtension(ext))
            return WriteResult::FILE_NOT_HANDLED;

        if (ciEquals(ext, "gltf"))
        {
            return writeGLTF(node, location, false, options);
        }
        else if (ciEquals(ext, "glb"))
        {
            return writeGLTF(node, location, true, options);
        }
        else if (ciEquals(ext, "b3dm"))
        {
            return writeB3DM(node, location, true, options);
        }

        return WriteResult::ERROR_IN_WRITING_FILE;
    }

    void convertOSGtoGLTF(const osg::Node& node, gltf::Model& model) const
    {
        model.asset.version = "2.0";

        osg::Node& nc_node = const_cast<osg::Node&>(node); // won't change it, promise :)
        nc_node.ref();

        // GLTF uses a +X=right +y=up -z=forward coordinate system
        osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
        transform->setMatrix(osg::Matrixd::rotate(osg::Vec3d(0.0, 0.0, 1.0), osg::Vec3d(0.0, 1.0, 0.0)));
        transform->addChild(&nc_node);

        OSGtoGLTF converter(model);
        transform->accept(converter);

        transform->removeChild(&nc_node);
        nc_node.unref_nodelete();
    }

    WriteResult writeGLTF(const osg::Node& node, const std::string& location, bool isBinary, const Options* options) const
    {
        gltf::Model model;
        convertOSGtoGLTF(node, model);

        TinyGLTF writer;

        writer.WriteGltfSceneToFile(
            &model,
            location,
            true,           // embedImages
            true,           // embedBuffers
            true,           // prettyPrint
            isBinary);      // writeBinary

        return WriteResult::FILE_SAVED;
    }

    WriteResult writeB3DM(const osg::Node& node, const std::string& location, bool isBinary, const Options* options) const
    {
        std::fstream fout(location.c_str(), std::ios::out | std::ios::binary);
        if (!fout.is_open())
            return WriteResult::ERROR_IN_WRITING_FILE;

        std::string featureTableJSON;
        {
            Json::Value value(Json::objectValue);
            value["BATCH_LENGTH"] = 1;
            // no RTC_CENTER
            Json::FastWriter writer;
            featureTableJSON = writer.write(value);
        }
        int featureTablePadding = 4 - (featureTableJSON.length() % 4);
        if (featureTablePadding == 4) featureTablePadding = 0;

        // no binary feature table

        // no batch table, json or binary

        // convert OSG to GLTF and write to a buffer:
        gltf::Model model;
        convertOSGtoGLTF(node, model);

        TinyGLTF gltfWriter;
        std::ostringstream gltfBuf;
        gltfWriter.WriteGltfSceneToBinaryStream(&model, location, gltfBuf, true, true);        
        std::string gltfData = gltfBuf.str();
        int gltfDataPadding = 4 - (gltfData.length() % 4);
        if (gltfDataPadding == 4) gltfDataPadding = 0;
        
        // assemble the header:
        b3dmheader header;
        header.magic[0] = 'b', header.magic[1] = '3', header.magic[2] = 'd', header.magic[3] = 'm';
        header.version = 1;
        header.featureTableJSONByteLength = featureTableJSON.length() + featureTablePadding;
        header.featureTableBinaryByteLength = 0;
        header.batchTableJSONByteLength = 0;
        header.batchTableBinaryByteLength = 0;
        header.byteLength =
            sizeof(b3dmheader) +
            header.featureTableJSONByteLength +
            header.featureTableBinaryByteLength +
            header.batchTableJSONByteLength +
            header.batchTableBinaryByteLength +
            gltfData.length() +
            gltfDataPadding;

#ifdef OE_IS_BIG_ENDIAN
        byteSwapInPlace(header.version);
        byteSwapInPlace(header.byteLength);
        byteSwapInPlace(header.featureTableJSONByteLength);
        byteSwapInPlace(header.featureTableBinaryByteLength);
        byteSwapInPlace(header.batchTableJSONByteLength);
        byteSwapInPlace(header.batchTableBinaryByteLength);
#endif

        // B3DM header:
        fout.write((const char*)&header, sizeof(b3dmheader));

        // Tables:
        fout.write(featureTableJSON.c_str(), featureTableJSON.length());
        fout.write("   ", featureTablePadding);

        // If we want to write the other 3 tables, they go here

        // GLTF binary data
        fout.write(gltfData.c_str(), gltfData.length());
        fout.write("\0\0\0", gltfDataPadding);

        fout.close();

        return WriteResult::FILE_SAVED;
    }
};

REGISTER_OSGPLUGIN(gltf, GLTFReader)