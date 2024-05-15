/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "TileGeometry"

using namespace osgEarth;
using namespace osgEarth::Corey;

TileGeometry*
TileGeometry::create(const TileMesh& mesh)
{
    OE_SOFT_ASSERT_AND_RETURN(mesh.verts.valid(), {});
    OE_SOFT_ASSERT_AND_RETURN(mesh.normals.valid(), {});
    OE_SOFT_ASSERT_AND_RETURN(mesh.uvs.valid(), {});
    OE_SOFT_ASSERT_AND_RETURN(mesh.indices.valid(), {});

    auto geom = new TileGeometry();
    geom->setVertexArray(mesh.verts);
    geom->setNormalArray(mesh.normals);
    geom->setTexCoordArray(0, mesh.uvs);
    if (mesh.vert_neighbors.valid())
        geom->setTexCoordArray(1, mesh.vert_neighbors);
    if (mesh.normal_neighbors.valid())
        geom->setTexCoordArray(2, mesh.normal_neighbors);

    if (mesh.indices.valid())
    {
        geom->addPrimitiveSet(mesh.indices);

        // later we will see about sharing these
        geom->_drawElementsData = new DrawElementsGL4Data();
    }

    geom->hasConstraints = mesh.hasConstraints;

    // if we are using GL4, create the GL4 tile model.
    if (/*using GL4*/true)
    {
        unsigned size = geom->getVertexArray()->getNumElements();
        geom->verts.reserve(size);

        for (unsigned i = 0; i < size; ++i)
        {
            GL4Vertex v;

            v.position = (*mesh.verts)[i];
            v.normal = (*mesh.normals)[i];
            v.uv = (*mesh.uvs)[i];
            if (mesh.vert_neighbors.valid())
                v.neighborPosition = (*mesh.vert_neighbors)[i];
            if (mesh.normal_neighbors.valid())
                v.neighborNormal = (*mesh.normal_neighbors)[i];

            geom->verts.emplace_back(std::move(v));
        }
    }

    return geom;
}

TileGeometry::TileGeometry() : osg::Geometry()
{
    _supportsVertexBufferObjects = true;
    setSupportsDisplayList(false);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

const DrawElementsIndirectBindlessCommandNV&
TileGeometry::getOrCreateNVGLCommand(osg::State& state)
{
    OE_SOFT_ASSERT(verts.size() > 0);
    OE_SOFT_ASSERT(getNumPrimitiveSets() == 1);

    bool dirty = false;

    // first the drawelements
    auto* prim = static_cast<osg::DrawElements*>(getPrimitiveSet(0));
    auto& de = DrawElementsGL4Data::GLObjects::get(_drawElementsData->_globjects, state);

    if (de._ebo == nullptr || !de._ebo->valid())
    {
        de._ebo = GLBuffer::create_shared(GL_ELEMENT_ARRAY_BUFFER_ARB, state);
        de._ebo->bind();
        de._ebo->debugLabel("Terrain geometry", "Shared EBO");
        de._ebo->bufferStorage(prim->getTotalDataSize(), prim->getDataPointer(), 0);
        de._ebo->unbind();

        dirty = true;
    }

    auto& gs = GLObjects::get(_globjects, state);

    if (gs._vbo == nullptr || !gs._vbo->valid())
    {
        // supply a "size hint" for unconstrained tiles to the GLBuffer so it can try to re-use
        GLsizei size = verts.size() * sizeof(GL4Vertex);
        if (hasConstraints)
            gs._vbo = GLBuffer::create_shared(GL_ARRAY_BUFFER_ARB, state);
        else
            gs._vbo = GLBuffer::create_shared(GL_ARRAY_BUFFER_ARB, state, size);

        gs._vbo->bind();
        gs._vbo->debugLabel("Terrain geometry", "Shared VBO");
        gs._vbo->bufferStorage(size, verts.data());
        gs._vbo->unbind();

        dirty = true;
    }

    // make them resident in each context separately
    de._ebo->makeResident(state);
    gs._vbo->makeResident(state);

    OE_SOFT_ASSERT(de._ebo->address() != 0);
    OE_SOFT_ASSERT(de._ebo->size() > 0);

    OE_SOFT_ASSERT(gs._vbo->address() != 0);
    OE_SOFT_ASSERT(gs._vbo->size() > 0);

    if (dirty)
    {
        gs._command.cmd.count = prim->getNumIndices(); // size();
        gs._command.cmd.instanceCount = 1;
        gs._command.cmd.firstIndex = 0;
        gs._command.cmd.baseVertex = 0;
        gs._command.cmd.baseInstance = 0;

        gs._command.reserved = 0;

        gs._command.indexBuffer.index = 0;
        gs._command.indexBuffer.reserved = 0;
        gs._command.indexBuffer.address = de._ebo->address();
        gs._command.indexBuffer.length = de._ebo->size();

        gs._command.vertexBuffer.index = 0;
        gs._command.vertexBuffer.reserved = 0;
        gs._command.vertexBuffer.address = gs._vbo->address();
        gs._command.vertexBuffer.length = gs._vbo->size();
    }

    return gs._command;
}

void TileGeometry::resizeGLObjectBuffers(unsigned int maxSize)
{
    osg::Geometry::resizeGLObjectBuffers(maxSize);

    if (maxSize > _globjects.size())
        _globjects.resize(maxSize);
}

void TileGeometry::releaseGLObjects(osg::State* state) const
{
    osg::Geometry::releaseGLObjects(state);

    if (state)
    {
        auto& gl = GLObjects::get(_globjects, *state);
        gl._vbo = nullptr;
    }

    // Do nothing if state is nullptr!
    // Let nature take its course and let the GLObjectPool deal with it
}
