/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "QuantizedMesh"
#include <osgEarth/Locators>
#include <osgEarth/GeoData>
#include <osgUtil/SmoothingVisitor>

using namespace osgEarth;

int
QuantizedMeshReader::zigZagDecode(int n)
{
    return (n >> 1) ^ (-(n & 1));
}

void
QuantizedMeshReader::parseHeader(std::stringstream& buf, Header& header)
{
    buf.read(reinterpret_cast<char*>(&header), sizeof(Header));
}

void
QuantizedMeshReader::parseVertexData(std::stringstream& buf, VertexData& vertexData)
{
    buf.read(reinterpret_cast<char*>(&vertexData.vertexCount), sizeof(unsigned int));
    vertexData.u.resize(vertexData.vertexCount);
    vertexData.v.resize(vertexData.vertexCount);
    vertexData.height.resize(vertexData.vertexCount);
    buf.read(reinterpret_cast<char*>(vertexData.u.data()), sizeof(unsigned short) * vertexData.vertexCount);
    buf.read(reinterpret_cast<char*>(vertexData.v.data()), sizeof(unsigned short) * vertexData.vertexCount);
    buf.read(reinterpret_cast<char*>(vertexData.height.data()), sizeof(unsigned short) * vertexData.vertexCount);

    // Decode delta-encoded values
    unsigned short u = 0;
    unsigned short v = 0;
    unsigned short height = 0;
    for (unsigned int i = 0; i < vertexData.vertexCount; ++i)
    {
        u += zigZagDecode(vertexData.u[i]);
        v += zigZagDecode(vertexData.v[i]);
        height += zigZagDecode(vertexData.height[i]);
        vertexData.u[i] = u;
        vertexData.v[i] = v;
        vertexData.height[i] = height;
    }
}

osg::ref_ptr<osg::DrawElements>
QuantizedMeshReader::parseTriangleIndices(std::stringstream& buf, unsigned int vertexCount)
{
    unsigned int triangleCount;
    buf.read(reinterpret_cast<char*>(&triangleCount), sizeof(unsigned int));

    osg::ref_ptr<osg::DrawElements> drawElements;

    if (vertexCount > 65536)
    {
        // Use 32-bit indices for large vertex counts
        std::vector<unsigned int> indices;
        indices.resize(triangleCount * 3u);
        buf.read(reinterpret_cast<char*>(indices.data()), sizeof(unsigned int) * indices.size());

        unsigned int highest = 0;
        for (unsigned int i = 0; i < indices.size(); ++i)
        {
            unsigned int code = indices[i];
            indices[i] = highest - code;
            if (code == 0) {
                ++highest;
            }
        }

        drawElements = new osg::DrawElementsUInt(GL_TRIANGLES);
        for (unsigned int i = 0; i < indices.size(); ++i)
        {
            drawElements->addElement(indices[i]);
        }
    }
    else
    {
        // Use 16-bit indices for smaller vertex counts
        std::vector<unsigned short> indices;
        indices.resize(triangleCount * 3u);
        buf.read(reinterpret_cast<char*>(indices.data()), sizeof(unsigned short) * indices.size());

        unsigned short highest = 0;
        for (unsigned int i = 0; i < indices.size(); ++i)
        {
            unsigned short code = indices[i];
            indices[i] = highest - code;
            if (code == 0) {
                ++highest;
            }
        }

        drawElements = new osg::DrawElementsUShort(GL_TRIANGLES);
        for (unsigned int i = 0; i < indices.size(); ++i)
        {
            drawElements->addElement(indices[i]);
        }
    }

    return drawElements;
}

TileMesh
QuantizedMeshReader::createTileMesh(
    const TileKey& key,
    const Header& header,
    const VertexData& vertexData,
    osg::ref_ptr<osg::DrawElements> indices)
{
    TileMesh mesh;
    osgEarth::GeoLocator locator(key.getExtent());
    osgEarth::GeoPoint centroid_world = key.getExtent().getCentroid();
    osg::Matrix world2local, local2world;
    centroid_world.createWorldToLocal(world2local);
    local2world.invert(world2local);

    osg::ref_ptr<osg::VertexBufferObject> vbo = new osg::VertexBufferObject();

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(vertexData.vertexCount);
    verts->setVertexBufferObject(vbo.get());
    verts->setBinding(osg::Array::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Vec3Array> texCoords = new osg::Vec3Array(vertexData.vertexCount);
    texCoords->setVertexBufferObject(vbo.get());
    texCoords->setBinding(osg::Array::BIND_PER_VERTEX);

    osg::Vec3d unit;
    osg::Vec3d model;
    osg::Vec3d modelLTP;

    for (unsigned int i = 0; i < vertexData.vertexCount; ++i)
    {
        float s = (float)vertexData.u[i] / 32767.0f;
        float t = (float)vertexData.v[i] / 32767.0f;
        float r = (float)vertexData.height[i] / 32767.0f;

        float height = header.MinimumHeight + r * (header.MaximumHeight - header.MinimumHeight);
        unit.set(s, t, height);
        locator.unitToWorld(unit, model);
        modelLTP = model * world2local;

        int default_marker = VERTEX_VISIBLE | VERTEX_CONSTRAINT | VERTEX_HAS_ELEVATION;
        (*texCoords)[i].set(osg::Vec3(s, t, (float)default_marker));
        (*verts)[i].set(modelLTP);
    }

    // Generate normals using smoothing visitor
    osg::ref_ptr<osg::Vec3Array> normals;
    if (!normals)
    {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setVertexArray(verts);
        geometry->addPrimitiveSet(indices);
        geode->addDrawable(geometry);
        osgUtil::SmoothingVisitor sv;
        geode->accept(sv);
        normals = static_cast<osg::Vec3Array*>(geometry->getNormalArray());
        if (normals)
        {
            normals->setVertexBufferObject(vbo.get());
        }
    }

    mesh.verts = verts;
    mesh.uvs = texCoords;
    mesh.normals = normals;
    mesh.indices = indices;
    mesh.localToWorld = local2world;

    return mesh;
}

TileMesh
QuantizedMeshReader::readFromStream(const TileKey& key, std::stringstream& buf)
{
    Header header;
    parseHeader(buf, header);

    VertexData vertexData;
    parseVertexData(buf, vertexData);

    osg::ref_ptr<osg::DrawElements> indices = parseTriangleIndices(buf, vertexData.vertexCount);

    return createTileMesh(key, header, vertexData, indices);
}

TileMesh
QuantizedMeshReader::readFromString(const TileKey& key, const std::string& data)
{
    std::stringstream buf(data);
    return readFromStream(key, buf);
}