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
#include "GeometryCloud"
#include "Metrics"

#include <osgUtil/Optimizer>
#include <osgUtil/MeshOptimizers>

#undef LC
#define LC "[GeometryCloud] "

using namespace osgEarth;

namespace
{
    struct CBCB : public osg::Drawable::ComputeBoundingBoxCallback
    {
        osg::BoundingBox computeBound(const osg::Drawable& drawable) const override
        {
            const osg::Geometry& g = *drawable.asGeometry();
            osg::BoundingBox bs;
            const osg::Vec3Array* verts = static_cast<const osg::Vec3Array*>(g.getVertexArray());
            for (auto& vert : *verts) bs.expandBy(vert);
            return bs;
        }
    };

    struct DrawElementsIndirectRenderer : public osg::Geometry::DrawCallback
    {
        GeometryCloud* _cloud;

        DrawElementsIndirectRenderer(GeometryCloud* cloud) : 
            _cloud(cloud) { }

        void drawImplementation(osg::RenderInfo& ri, const osg::Drawable* drawable) const override
        {
            OE_PROFILING_ZONE_NAMED("drawImplementation");

            osg::State& state = *ri.getState();

            osg::GLExtensions* ext = state.get<osg::GLExtensions>();

            const osg::Geometry* geom = static_cast<const osg::Geometry*>(drawable);

            // prepare the VAO, etc.
            osg::VertexArrayState* vas = state.getCurrentVertexArrayState();
            vas->setVertexBufferObjectSupported(true);
            geom->drawVertexArraysImplementation(ri);

            // bind the combined EBO to the VAO
            osg::GLBufferObject* ebo = geom->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
            if (vas->getRequiresSetArrays() ||
                vas->getCurrentElementBufferObject() != ebo)
            {
                OE_PROFILING_ZONE_NAMED("Bind EBO");
                //osg::GLBufferObject* ebo = geom->getPrimitiveSet(0)->getOrCreateGLBufferObject(state.getContextID());
                vas->bindElementBufferObject(ebo);
                //state.bindElementBufferObject(ebo);
            }

            // engage
            {
                OE_PROFILING_ZONE_NAMED("glMultiDrawElementsIndirect");

                ext->glMultiDrawElementsIndirect(
                    GL_TRIANGLES,
                    GL_UNSIGNED_SHORT,
                    nullptr,                       // in GL_DRAW_INDIRECT_BUFFER on GPU
                    _cloud->getNumDrawCommands(),  // number of commands to execute
                    0);                            // stride=0, commands are tightly packed
            }
        }
    };
}

GeometryCloud::GeometryCloud(TextureArena* texarena) :
    osg::NodeVisitor(),
    _texarena(texarena)
{
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);

    _geom = new osg::Geometry();
    _geom->setUseVertexBufferObjects(true);
    _geom->setUseDisplayList(false);

    _geom->setDrawCallback(new DrawElementsIndirectRenderer(this));
    _geom->setCullingActive(false);

    _verts = new osg::Vec3Array();
    _verts->setBinding(osg::Array::BIND_PER_VERTEX);
    _geom->setVertexArray(_verts);

    _colors = new osg::Vec4Array();
    _colors->setBinding(osg::Array::BIND_PER_VERTEX);
    _geom->setColorArray(_colors);

    _normals = new osg::Vec3Array();
    _normals->setBinding(osg::Array::BIND_PER_VERTEX);
    _geom->setNormalArray(_normals);

    _texcoords = new osg::Vec2Array();
    _texcoords->setBinding(osg::Array::BIND_PER_VERTEX);
    _geom->setTexCoordArray(7, _texcoords);

    // texture arena indices for albedo/diffuse
    _albedoArenaIndices = new osg::ShortArray();
    _albedoArenaIndices->setBinding(osg::Array::BIND_PER_VERTEX);
    _albedoArenaIndices->setPreserveDataType(true);
    _geom->setVertexAttribArray(6, _albedoArenaIndices);

    // texture arena indices for normal map
    _normalArenaIndices = new osg::ShortArray();
    _normalArenaIndices->setBinding(osg::Array::BIND_PER_VERTEX);
    _normalArenaIndices->setPreserveDataType(true);
    _geom->setVertexAttribArray(7, _normalArenaIndices);

    // N.B. UShort is sufficient b/c we are using the DrawElementsIndirect.baseVertex
    // technique to offset element indicies and thereby support a VBO with >65535 verts.
    _primset = new osg::DrawElementsUShort(GL_TRIANGLES);
    _geom->addPrimitiveSet(_primset);

    _geom->getOrCreateStateSet()->setMode(GL_BLEND, 1);

    // Need a custom BB computer since this is not a normal geometry
    _geom->setComputeBoundingBoxCallback(new CBCB());
}

GeometryCloud::AddResult
GeometryCloud::add(
    osg::Node* node,
    unsigned alignment,
    int normalMapTextureImageUnit)
{
    AddResult result;

    // convert all primitive sets to GL_TRIANGLES
    osgUtil::Optimizer o;
    o.optimize(node, o.INDEX_MESH);

    // Reorder indices for optimal cache usage.
    // DO NOT do this if alignment is set; using alignment
    // implies the verts are in a specific order for a good reason, which
    // is usually because the shader relies on gl_VertexID.
    if (alignment == 0u)
    {
        osgUtil::VertexCacheVisitor vcv;
        node->accept(vcv);
        vcv.optimizeVertices();
    }

    // pad the arrays to the alignment (to make gl_VertexID modulus work correctly)
    if (alignment > 0u)
    {
        unsigned padding = align((unsigned)_verts->size(), alignment) - _verts->size();
        OE_DEBUG << "vert size = " << _verts->size() << " so pad it with " << padding << " bytes" << std::endl;
        for(unsigned i=0; i<padding; ++i)
        {   
            _verts->push_back(osg::Vec3());
            _colors->push_back(osg::Vec4(1,0,0,1));
            _normals->push_back(osg::Vec3(0,0,1));
            _texcoords->push_back(osg::Vec2(0,0));
            _albedoArenaIndices->push_back(-1);
            _normalArenaIndices->push_back(-1);
        }
    }

    // track the starting vertex of this model:
    _vertexOffsets.push_back(_verts->size());

    // track the starting element index of this model:
    _elementOffsets.push_back(_primset->getNumIndices());

    // traverse the model, consolidating arrays and rewriting texture indices
    // and counting elements indices
    _numElements = 0u;

    _albedoArenaIndexStack = std::stack<int>();
    _albedoArenaIndexStack.push(-1);

    _normalArenaIndexStack = std::stack<int>();
    _normalArenaIndexStack.push(-1);

    _matrixStack = std::stack<osg::Matrix>();

    _normalMapTextureImageUnit = normalMapTextureImageUnit;

    // This is needed b/c we are using a RAW pointer in the LUT.
    // So it can (and will) be reused.
    // Consider using some other way to uniquely identify textures;
    // that way we can share them across models
    _arenaIndexLUT.clear();

    node->accept(*this);

    _elementCounts.push_back(_numElements);
    _geom->dirtyBound();

    // return the draw command number of the newly added geometry.
    result._commandIndex = getNumDrawCommands() - 1;
    return result;
}

namespace
{
    // template to append one array onto another
    template<typename T> void append(T* dest, const osg::Array* src, unsigned numVerts)
    {
        if (src == NULL)
        {
            dest->reserveArray(dest->size() + numVerts);
            for(unsigned i=0; i<numVerts; ++i)
                dest->push_back(typename T :: ElementDataType ());
        }
        else if (src->getBinding() == osg::Array::BIND_PER_VERTEX)
        {
            dest->reserveArray(dest->size() + numVerts); //src->getNumElements());
            const T* src_typed = static_cast<const T*>(src);
            std::copy(src_typed->begin(), src_typed->end(), std::back_inserter(*dest));

            // pad out to match
            if (src->getNumElements() < numVerts)
                for(unsigned i=0; i<numVerts-src->getNumElements(); ++i)
                    dest->push_back(typename T :: ElementDataType ());

        }
        else if (src->getBinding() == osg::Array::BIND_OVERALL)
        {
            dest->reserveArray(dest->size() + numVerts);
            const T* src_typed = static_cast<const T*>(src);
            for(unsigned i=0; i<numVerts; ++i)
                dest->push_back((*src_typed)[0]);
        }
    }
}

bool
GeometryCloud::pushStateSet(osg::Node& node)
{
    osg::StateSet* stateset = node.getStateSet();
    if (stateset)
    {
        int albedoArenaIndex = _albedoArenaIndexStack.top();

        // Albedo texture in slot 0 always:
        if (true) // might make this conditional someday
        {
            osg::Texture* tex = dynamic_cast<osg::Texture*>(
                stateset->getTextureAttribute(0, osg::StateAttribute::TEXTURE));

            if (tex && tex->getImage(0))
            {
                auto i = _arenaIndexLUT.find(tex);
                if (i == _arenaIndexLUT.end())
                {
                    Texture::Ptr t = Texture::create();
                    t->_image = tex->getImage(0);
                    t->_uri = t->_image->getFileName();

                    int nextIndex = _texarena->add(t);
                    if (nextIndex >= 0)
                    {
                        _arenaIndexLUT[tex] = nextIndex;
                        albedoArenaIndex = nextIndex;
                    }
                    else
                    {
                        OE_WARN << "Failed to add a texture. Contact support!" << std::endl;
                    }
                }
                else
                {
                    albedoArenaIndex = i->second;
                }
            }
        }
        _albedoArenaIndexStack.push(albedoArenaIndex);

        // check normal map texture, if available
        int normalArenaIndex = _normalArenaIndexStack.top();
        if (_normalMapTextureImageUnit > 0)
        {
            osg::Texture* tex = dynamic_cast<osg::Texture*>(
                stateset->getTextureAttribute(_normalMapTextureImageUnit, osg::StateAttribute::TEXTURE));

            if (tex && tex->getImage(0))
            {
                auto i = _arenaIndexLUT.find(tex);
                if (i == _arenaIndexLUT.end())
                {
                    Texture::Ptr t = Texture::create();
                    t->_image = tex->getImage(0);
                    t->_compress = false; // do not compress normal maps

                    int nextIndex = _texarena->add(t);
                    if (nextIndex >= 0)
                    {
                        _arenaIndexLUT[tex] = nextIndex;
                        normalArenaIndex = nextIndex;
                    }
                    else
                    {
                        OE_WARN << "Failed to add a normal map. Contact support!" << std::endl;
                    }
                }
                else
                {
                    normalArenaIndex = i->second;
                }
            }
        }
        _normalArenaIndexStack.push(normalArenaIndex);

        return true;
    }
    else
    {
        return false;
    }
}

void
GeometryCloud::popStateSet()
{
    _albedoArenaIndexStack.pop();
    _normalArenaIndexStack.pop();
}

void
GeometryCloud::apply(osg::Node& node)
{
    bool pushed = pushStateSet(node);
    traverse(node);
    if (pushed) popStateSet();
}

void
GeometryCloud::apply(osg::Transform& node)
{
    osg::Matrix m = _matrixStack.empty() ? osg::Matrix() : _matrixStack.top();
    node.computeLocalToWorldMatrix(m, this);
    _matrixStack.push(m);
    apply(static_cast<osg::Group&>(node));
    _matrixStack.pop();
}

void
GeometryCloud::apply(osg::Geometry& node)
{
    bool pushed = pushStateSet(node);

    // offset of verts within this model:
    unsigned globalOffset = _vertexOffsets.empty() ? 0 : _vertexOffsets.back();

    // offset of verts local to this geometry:
    unsigned localOffset = _verts->size() - globalOffset;

    unsigned size = node.getVertexArray()->getNumElements();

    if (!_matrixStack.empty())
    {
        osg::Vec3Array* nodeVerts = dynamic_cast<osg::Vec3Array*>(node.getVertexArray());
        if (nodeVerts)
        {
            for (auto& vert : *nodeVerts)
                vert = vert * _matrixStack.top();
        }
    }

    append(_verts, node.getVertexArray(), size);
    append(_normals, node.getNormalArray(), size);
    append(_colors, node.getColorArray(), size);
    append(_texcoords, node.getTexCoordArray(0), size);

    _albedoArenaIndices->reserve(_albedoArenaIndices->size() + size);
    for (unsigned i = 0; i < size; ++i)
    {
        _albedoArenaIndices->push_back(_albedoArenaIndexStack.top());
    }

    _normalArenaIndices->reserve(_normalArenaIndices->size() + size);
    for (unsigned i = 0; i < size; ++i)
    {
        _normalArenaIndices->push_back(_normalArenaIndexStack.top());
    }

    // assemble the elements set
    for(unsigned i=0; i < node.getNumPrimitiveSets(); ++i)
    {
        osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(node.getPrimitiveSet(i));
        if (de)
        {
            for(unsigned k=0; k<de->getNumIndices(); ++k)
            {
                int index = de->getElement(k);
                // by using a "model-local" offset here, we can use UShort even
                // if our vertex array size exceeds 65535 by storing the 
                // baseVertex in our DrawElements structure
                _primset->addElement(localOffset + index);
                ++_numElements;
            }
        }
    }

    if (pushed) popStateSet();
}

void
GeometryCloud::draw(osg::RenderInfo& ri)
{
    _geom->draw(ri);
}

bool
GeometryCloud::getDrawCommand(unsigned i, DrawElementsIndirectCommand& cmd) const
{
    if (i <= getNumDrawCommands())
    {
        cmd.count = _elementCounts[i];       // how many indices comprise this draw command
        cmd.instanceCount = 0;               // will be assigned by the Cull/Sort shader
        cmd.firstIndex = _elementOffsets[i]; // not used by us...or is it
        cmd.baseVertex = _vertexOffsets[i];  // offset to add to element indices (nice, lets us use USHORT)
        cmd.baseInstance = 0;                // will be assigned by the Cull/Sort shader
        return true;
    }
    return false;
}
