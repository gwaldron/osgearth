#include <osgEarth/CompressedArray>

#ifdef OSGEARTH_HAVE_MESH_OPTIMIZER
#include <osgDB/InputStream>
#include <osgDB/OutputStream>
#include <osgDB/ObjectWrapper>
#include <osgDB/Registry>

#include <meshoptimizer.h>

using namespace osgEarth;

CompressedVec3Array::CompressedVec3Array() :
    osg::Vec3Array(),
    _quantization(CompressedVec3Array::QUANTIZE_NONE)
{
}

CompressedVec3Array::CompressedVec3Array(osg::Vec3Array& va, QuantizationType quantization) :
    osg::Vec3Array(va),
    _quantization(quantization)
{
}

CompressedVec3Array::CompressedVec3Array(const CompressedVec3Array& rhs, const osg::CopyOp& copyop) :
    osg::Vec3Array(rhs, copyop),
    _quantization(rhs._quantization)
{
}

CompressedVec2Array::CompressedVec2Array() :
    osg::Vec2Array()
{
}

CompressedVec2Array::CompressedVec2Array(osg::Vec2Array& va) :
    osg::Vec2Array(va)
{
}

CompressedVec2Array::CompressedVec2Array(const CompressedVec2Array& rhs, const osg::CopyOp& copyop) :
    osg::Vec2Array(rhs, copyop)
{
}

unsigned short rescaleToUShortMax(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return static_cast<unsigned short>(v * USHRT_MAX);
}


CompressedDrawElementsUShort::CompressedDrawElementsUShort():
    osg::DrawElementsUShort()
{
}

CompressedDrawElementsUShort::CompressedDrawElementsUShort(osg::DrawElementsUShort& de):
    osg::DrawElementsUShort(de)
{
}

CompressedDrawElementsUShort::CompressedDrawElementsUShort(const CompressedDrawElementsUShort& rhs, const osg::CopyOp& copyop):
    osg::DrawElementsUShort(rhs, copyop)
{
}


CompressedDrawElementsUByte::CompressedDrawElementsUByte() :
    osg::DrawElementsUByte()
{
}

CompressedDrawElementsUByte::CompressedDrawElementsUByte(osg::DrawElementsUByte& de) :
    osg::DrawElementsUByte(de)
{
}

CompressedDrawElementsUByte::CompressedDrawElementsUByte(const CompressedDrawElementsUByte& rhs, const osg::CopyOp& copyop) :
    osg::DrawElementsUByte(rhs, copyop)
{
}

CompressedDrawElementsUInt::CompressedDrawElementsUInt() :
    osg::DrawElementsUInt()
{
}

CompressedDrawElementsUInt::CompressedDrawElementsUInt(osg::DrawElementsUInt& de) :
    osg::DrawElementsUInt(de)
{
}

CompressedDrawElementsUInt::CompressedDrawElementsUInt(const CompressedDrawElementsUInt& rhs, const osg::CopyOp& copyop) :
    osg::DrawElementsUInt(rhs, copyop)
{
}


CompressedUIntArray::CompressedUIntArray() :
    osg::UIntArray()
{
}

CompressedUIntArray::CompressedUIntArray(osg::UIntArray& array) :
    osg::UIntArray(array)
{
}

CompressedUIntArray::CompressedUIntArray(const CompressedUIntArray& rhs, const osg::CopyOp& copyop) :
    osg::UIntArray(rhs, copyop)
{
}


namespace osgEarth {
    namespace Serializers {
        
        
        namespace CompressedVec3Array
        {
            static bool checkData(const osgEarth::CompressedVec3Array& g)
            {
                return g.size() > 0;
            }

            static bool readData(osgDB::InputStream& is, osgEarth::CompressedVec3Array& g)
            {
                unsigned int size = is.readSize();
                is >> is.BEGIN_BRACKET;

                int quantization = osgEarth::CompressedVec3Array::QUANTIZE_NONE;
                is >> is.PROPERTY("Quantization") >> quantization;
                g.setQuantization((osgEarth::CompressedVec3Array::QuantizationType)quantization);

                osg::Vec3 min, max;
                if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_VERTEX)
                {
                    is >> is.PROPERTY("Min") >> min;
                    is >> is.PROPERTY("Max") >> max;
                }

                g.reserve(size);

                // Read the compressed data array
                is >> is.BEGIN_BRACKET;
                unsigned int dataSize = is.readSize();
                std::vector<unsigned char> vbuf(dataSize);
                is.readCharArray((char*)vbuf.data(), dataSize);
                is >> is.END_BRACKET;

                if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_VERTEX)
                {
                    std::vector<osg::Vec4us> decoded(size);
                    meshopt_decodeVertexBuffer(&decoded[0], decoded.size(), sizeof(osg::Vec4us), &vbuf[0], vbuf.size());                    

                    for (unsigned int i = 0; i < size; ++i)
                    {
                        float x = min.x() + (max.x() - min.x()) * (float)decoded[i].x() / (float)USHRT_MAX;
                        float y = min.y() + (max.y() - min.y()) * (float)decoded[i].y() / (float)USHRT_MAX;
                        float z = min.z() + (max.z() - min.z()) * (float)decoded[i].z() / (float)USHRT_MAX;
                        g.push_back(osg::Vec3(x, y, z));
                    }
                }
                if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_NORMAL)
                {
                    std::vector<unsigned int> decoded(size);
                    meshopt_decodeVertexBuffer(&decoded[0], decoded.size(), sizeof(unsigned int), &vbuf[0], vbuf.size());

                    for (unsigned int i = 0; i < size; ++i)
                    {
                        unsigned int n = decoded[i];
                        // Extract each component
                        unsigned int nx = (n >> 20) & 0x3FF; // Highest 10 bits
                        unsigned int ny = (n >> 10) & 0x3FF; // Next 10 bits
                        unsigned int nz = n & 0x3FF;         // Lowest 10 bits

                        osg::Vec3 normal(
                            (float)nx / 1023.0f,
                            (float)ny / 1023.0f,
                            (float)nz / 1023.0f
                        );
                        normal.normalize();
                        g.push_back(normal);
                    }
                }
                else if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_HALF)
                {
                    std::vector<osg::Vec4us> decoded(size);
                    meshopt_decodeVertexBuffer(&decoded[0], decoded.size(), sizeof(osg::Vec4us), &vbuf[0], vbuf.size());                    
                    for (unsigned int i = 0; i < decoded.size(); ++i)
                    {
                        osg::Vec3 v(
                            meshopt_dequantizeHalf(decoded[i].x()),
                            meshopt_dequantizeHalf(decoded[i].y()),
                            meshopt_dequantizeHalf(decoded[i].z())
                        );
                        g.push_back(v);
                    }
                }
                else
                {
                    // Just load the data directly into the vec3 buffer.
                    g.resizeArray(size);
                    meshopt_decodeVertexBuffer(const_cast<void*>(g.getDataPointer()), g.size(), sizeof(osg::Vec3), &vbuf[0], vbuf.size());
                }

                is >> is.END_BRACKET;


                return true;
            }

            static bool writeData(osgDB::OutputStream& os, const osgEarth::CompressedVec3Array& g)
            {
                os.writeSize(g.getNumElements()); os << os.BEGIN_BRACKET << std::endl;

                os << os.PROPERTY("Quantization") << (int)g.getQuantization() << std::endl;

                std::vector<unsigned char> vbuf;
                if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_VERTEX)
                {
                    // Compute the min / max
                    osg::Vec3 min, max;
                    for (std::size_t i = 0; i < g.size(); ++i)
                    {
                        if (i == 0)
                        {
                            min = max = g[i];
                        }
                        else
                        {
                            min.x() = std::min(min.x(), g[i].x());
                            min.y() = std::min(min.y(), g[i].y());
                            min.z() = std::min(min.z(), g[i].z());
                            max.x() = std::max(max.x(), g[i].x());
                            max.y() = std::max(max.y(), g[i].y());
                            max.z() = std::max(max.z(), g[i].z());
                        }
                    }

                    os << os.PROPERTY("Min") << min << std::endl;
                    os << os.PROPERTY("Max") << max << std::endl;

                    std::vector<osg::Vec4us> packed(g.size());
                    for (unsigned int i = 0; i < g.size(); ++i)
                    {
                        packed[i].x() = rescaleToUShortMax((g[i].x() - min.x()) / (max.x() - min.x()));
                        packed[i].y() = rescaleToUShortMax((g[i].y() - min.y()) / (max.y() - min.y()));
                        packed[i].z() = rescaleToUShortMax((g[i].z() - min.z()) / (max.z() - min.z()));
                        packed[i].a() = 0;
                    }
                    vbuf.resize(meshopt_encodeVertexBufferBound(packed.size(), sizeof(osg::Vec4us)));
                    vbuf.resize(meshopt_encodeVertexBuffer(&vbuf[0], vbuf.size(), &packed[0], packed.size(), sizeof(osg::Vec4us)));
                }
                if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_NORMAL)
                {
                    std::vector<unsigned int> packed(g.size());
                    for (unsigned int i = 0; i < g.size(); ++i)
                    {
                        unsigned int normal =
                            (meshopt_quantizeSnorm(g[i].x(), 10) << 20) |
                            (meshopt_quantizeSnorm(g[i].y(), 10) << 10) |
                            meshopt_quantizeSnorm(g[i].z(), 10);
                        packed[i] = normal;
                    }
                    vbuf.resize(meshopt_encodeVertexBufferBound(packed.size(), sizeof(unsigned int)));
                    vbuf.resize(meshopt_encodeVertexBuffer(&vbuf[0], vbuf.size(), &packed[0], packed.size(), sizeof(unsigned int)));
                }
                else if (g.getQuantization() == osgEarth::CompressedVec3Array::QUANTIZE_HALF)
                {
                    std::vector<osg::Vec4us> packed(g.size());
                    for (unsigned int i = 0; i < g.size(); ++i)
                    {                        
                        packed[i].x() = meshopt_quantizeHalf(g[i].x());
                        packed[i].y() = meshopt_quantizeHalf(g[i].y());
                        packed[i].z() = meshopt_quantizeHalf(g[i].z());
                        packed[i].a() = 0; // Padding to 4 byte boundary
                    }
                    vbuf.resize(meshopt_encodeVertexBufferBound(packed.size(), sizeof(osg::Vec4us)));
                    vbuf.resize(meshopt_encodeVertexBuffer(&vbuf[0], vbuf.size(), &packed[0], packed.size(), sizeof(osg::Vec4us)));                    
                }
                else
                {
                    vbuf.resize(meshopt_encodeVertexBufferBound(g.size(), sizeof(osg::Vec3)));
                    vbuf.resize(meshopt_encodeVertexBuffer(&vbuf[0], vbuf.size(), g.getDataPointer(), g.size(), sizeof(osg::Vec3)));
                }


                os << os.BEGIN_BRACKET << std::endl;
                os.writeSize(vbuf.size());
                os.writeCharArray((char*)vbuf.data(), vbuf.size());
                os << os.END_BRACKET << std::endl; // End data

                os << os.END_BRACKET << std::endl;
                return true;
            }


            REGISTER_OBJECT_WRAPPER(
                CompressedVec3Array,
                new osgEarth::CompressedVec3Array,
                osgEarth::CompressedVec3Array,
                "osg::Object osg::BufferData osg::Array osgEarth::CompressedVec3Array")
            {
                ADD_USER_SERIALIZER(Data);
            }
        }

        // CompressedVec2Array
        namespace CompressedVec2Array
        {
            static bool checkData(const osgEarth::CompressedVec2Array& g)
            {
                return g.size() > 0;
            }

            static bool readData(osgDB::InputStream& is, osgEarth::CompressedVec2Array& g)
            {
                unsigned int size = is.readSize();
                is >> is.BEGIN_BRACKET;                

                g.reserve(size);

                // Read the compressed data array
                is >> is.BEGIN_BRACKET;
                unsigned int dataSize = is.readSize();
                std::vector<unsigned char> vbuf(dataSize);
                is.readCharArray((char*)vbuf.data(), dataSize);
                is >> is.END_BRACKET;

                
                std::vector<osg::Vec2us> decoded(size);
                meshopt_decodeVertexBuffer(&decoded[0], decoded.size(), sizeof(osg::Vec2us), &vbuf[0], vbuf.size());
                for (unsigned int i = 0; i < decoded.size(); ++i)
                {
                    osg::Vec2 v(
                        meshopt_dequantizeHalf(decoded[i].x()),
                        meshopt_dequantizeHalf(decoded[i].y())
                    );
                    g.push_back(v);
                }

                is >> is.END_BRACKET;

                return true;
            }

            static bool writeData(osgDB::OutputStream& os, const osgEarth::CompressedVec2Array& g)
            {
                os.writeSize(g.getNumElements()); os << os.BEGIN_BRACKET << std::endl;
                std::vector<unsigned char> vbuf;
                std::vector<osg::Vec2us> packed(g.size());
                for (unsigned int i = 0; i < g.size(); ++i)
                {
                    packed[i].x() = meshopt_quantizeHalf(g[i].x());
                    packed[i].y() = meshopt_quantizeHalf(g[i].y());
                }
                vbuf.resize(meshopt_encodeVertexBufferBound(packed.size(), sizeof(osg::Vec2us)));
                vbuf.resize(meshopt_encodeVertexBuffer(&vbuf[0], vbuf.size(), &packed[0], packed.size(), sizeof(osg::Vec2us)));
                

                os << os.BEGIN_BRACKET << std::endl;
                os.writeSize(vbuf.size());
                os.writeCharArray((char*)vbuf.data(), vbuf.size());
                os << os.END_BRACKET << std::endl; // End data

                os << os.END_BRACKET << std::endl;
                return true;
            }


            REGISTER_OBJECT_WRAPPER(
                CompressedVec2Array,
                new osgEarth::CompressedVec2Array,
                osgEarth::CompressedVec2Array,
                "osg::Object osg::BufferData osg::Array osgEarth::CompressedVec2Array")
            {
                ADD_USER_SERIALIZER(Data);
            }
        }


        // CompressedDrawElementsUShort
        namespace CompressedDrawElementsUShort
        {
            static bool checkData(const osgEarth::CompressedDrawElementsUShort& g)
            {
                return g.size() > 0;
            }

            static bool readData(osgDB::InputStream& is, osgEarth::CompressedDrawElementsUShort& g)
            {
                unsigned int size = is.readSize();
                is >> is.BEGIN_BRACKET;

                // Read the compressed data array
                is >> is.BEGIN_BRACKET;
                unsigned int dataSize = is.readSize();
                std::vector<unsigned char> ibuf(dataSize);
                is.readCharArray((char*)ibuf.data(), dataSize);
                is >> is.END_BRACKET;


                std::vector<unsigned int> indices(size);
                meshopt_decodeIndexBuffer(&indices[0], size, &ibuf[0], ibuf.size());
                g.resize(indices.size());
                for (unsigned int i = 0; i < indices.size(); ++i)
                {
                    g[i] = indices[i];
                }

                is >> is.END_BRACKET;

                return true;
            }

            static bool writeData(osgDB::OutputStream& os, const osgEarth::CompressedDrawElementsUShort& g)
            {
                os.writeSize(g.getNumIndices()); os << os.BEGIN_BRACKET << std::endl;
                std::vector<unsigned int> indices(g.size());
                for (unsigned int i = 0; i < g.size(); ++i)
                {
                    indices[i] = g[i];
                }

                std::vector<unsigned char> vbuf;                
                vbuf.resize(meshopt_encodeIndexBufferBound(g.size(), USHRT_MAX));
                vbuf.resize(meshopt_encodeIndexBuffer(&vbuf[0], vbuf.size(), &indices[0], indices.size()));

               
                os << os.BEGIN_BRACKET << std::endl;
                os.writeSize(vbuf.size());
                os.writeCharArray((char*)vbuf.data(), vbuf.size());
                os << os.END_BRACKET << std::endl; // End data

                os << os.END_BRACKET << std::endl;
                return true;
            }                   

            REGISTER_OBJECT_WRAPPER(
                CompressedDrawElementsUShort,
                new osgEarth::CompressedDrawElementsUShort,
                osgEarth::CompressedDrawElementsUShort,
                "osg::Object osg::BufferData osg::PrimitiveSet osgEarth::CompressedDrawElementsUShort")
            {
                ADD_USER_SERIALIZER(Data);
            }
        }

        // CompressedDrawElementsUByte
        namespace CompressedDrawElementsUByte
        {
            static bool checkData(const osgEarth::CompressedDrawElementsUByte& g)
            {
                return g.size() > 0;
            }

            static bool readData(osgDB::InputStream& is, osgEarth::CompressedDrawElementsUByte& g)
            {
                unsigned int size = is.readSize();
                is >> is.BEGIN_BRACKET;

                // Read the compressed data array
                is >> is.BEGIN_BRACKET;
                unsigned int dataSize = is.readSize();
                std::vector<unsigned char> ibuf(dataSize);
                is.readCharArray((char*)ibuf.data(), dataSize);
                is >> is.END_BRACKET;


                std::vector<unsigned int> indices(size);
                meshopt_decodeIndexBuffer(&indices[0], size, &ibuf[0], ibuf.size());
                g.resize(indices.size());
                for (unsigned int i = 0; i < indices.size(); ++i)
                {
                    g[i] = indices[i];
                }

                is >> is.END_BRACKET;

                return true;
            }

            static bool writeData(osgDB::OutputStream& os, const osgEarth::CompressedDrawElementsUByte& g)
            {
                os.writeSize(g.getNumIndices()); os << os.BEGIN_BRACKET << std::endl;
                std::vector<unsigned int> indices(g.size());
                for (unsigned int i = 0; i < g.size(); ++i)
                {
                    indices[i] = g[i];
                }

                std::vector<unsigned char> vbuf;
                vbuf.resize(meshopt_encodeIndexBufferBound(g.size(), 255));
                vbuf.resize(meshopt_encodeIndexBuffer(&vbuf[0], vbuf.size(), &indices[0], indices.size()));

                os << os.BEGIN_BRACKET << std::endl;
                os.writeSize(vbuf.size());
                os.writeCharArray((char*)vbuf.data(), vbuf.size());
                os << os.END_BRACKET << std::endl; // End data

                os << os.END_BRACKET << std::endl;
                return true;
            }

            REGISTER_OBJECT_WRAPPER(
                CompressedDrawElementsUByte,
                new osgEarth::CompressedDrawElementsUByte,
                osgEarth::CompressedDrawElementsUByte,
                "osg::Object osg::BufferData osg::PrimitiveSet osgEarth::CompressedDrawElementsUByte")
            {
                ADD_USER_SERIALIZER(Data);
            }
        }

        // CompressedDrawElementsUInt
        namespace CompressedDrawElementsUInt
        {
            static bool checkData(const osgEarth::CompressedDrawElementsUInt& g)
            {
                return g.size() > 0;
            }

            static bool readData(osgDB::InputStream& is, osgEarth::CompressedDrawElementsUInt& g)
            {
                unsigned int size = is.readSize();
                is >> is.BEGIN_BRACKET;

                // Read the compressed data array
                is >> is.BEGIN_BRACKET;
                unsigned int dataSize = is.readSize();
                std::vector<unsigned char> ibuf(dataSize);
                is.readCharArray((char*)ibuf.data(), dataSize);
                is >> is.END_BRACKET;


                std::vector<unsigned int> indices(size);
                meshopt_decodeIndexBuffer(&indices[0], size, &ibuf[0], ibuf.size());
                g.resize(indices.size());
                for (unsigned int i = 0; i < indices.size(); ++i)
                {
                    g[i] = indices[i];
                }

                is >> is.END_BRACKET;

                return true;
            }

            static bool writeData(osgDB::OutputStream& os, const osgEarth::CompressedDrawElementsUInt& g)
            {
                os.writeSize(g.getNumIndices()); os << os.BEGIN_BRACKET << std::endl;
                std::vector<unsigned int> indices(g.size());
                for (unsigned int i = 0; i < g.size(); ++i)
                {
                    indices[i] = g[i];
                }

                std::vector<unsigned char> vbuf;
                vbuf.resize(meshopt_encodeIndexBufferBound(g.size(), UINT_MAX));
                vbuf.resize(meshopt_encodeIndexBuffer(&vbuf[0], vbuf.size(), &indices[0], indices.size()));

                os << os.BEGIN_BRACKET << std::endl;
                os.writeSize(vbuf.size());
                os.writeCharArray((char*)vbuf.data(), vbuf.size());
                os << os.END_BRACKET << std::endl; // End data

                os << os.END_BRACKET << std::endl;
                return true;
            }

            REGISTER_OBJECT_WRAPPER(
                CompressedDrawElementsUInt,
                new osgEarth::CompressedDrawElementsUInt,
                osgEarth::CompressedDrawElementsUInt,
                "osg::Object osg::BufferData osg::PrimitiveSet osgEarth::CompressedDrawElementsUInt")
            {
                ADD_USER_SERIALIZER(Data);
            }
        }


        // CompressedUIntArray
        namespace CompressedUIntArray
        {
            static bool checkData(const osgEarth::CompressedUIntArray& g)
            {
                return g.size() > 0;
            }

            static bool readData(osgDB::InputStream& is, osgEarth::CompressedUIntArray& g)
            {                
                unsigned int size = is.readSize();
                is >> is.BEGIN_BRACKET;

                // Read the compressed data array
                is >> is.BEGIN_BRACKET;
                unsigned int dataSize = is.readSize();
                std::vector<unsigned char> vbuf(dataSize);
                is.readCharArray((char*)vbuf.data(), dataSize);
                is >> is.END_BRACKET;


                // Just load the data directly into the vec3 buffer.
                g.resizeArray(size);
                meshopt_decodeVertexBuffer(const_cast<void*>(g.getDataPointer()), g.size(), g.getElementSize(), &vbuf[0], vbuf.size());

                is >> is.END_BRACKET;

                return true;
            }

            static bool writeData(osgDB::OutputStream& os, const osgEarth::CompressedUIntArray& g)
            {
                os.writeSize(g.size()); os << os.BEGIN_BRACKET << std::endl;

                std::vector<unsigned char> vbuf;
                vbuf.resize(meshopt_encodeVertexBufferBound(g.size(), g.getElementSize()));
                vbuf.resize(meshopt_encodeVertexBuffer(&vbuf[0], vbuf.size(), g.getDataPointer(), g.size(), g.getElementSize()));

                os << os.BEGIN_BRACKET << std::endl;
                os.writeSize(vbuf.size());
                os.writeCharArray((char*)vbuf.data(), vbuf.size());

                os << os.END_BRACKET << std::endl; // End data

                os << os.END_BRACKET << std::endl;
                return true;
            }

            REGISTER_OBJECT_WRAPPER(
                CompressedUIntArray,
                new osgEarth::CompressedUIntArray,
                osgEarth::CompressedUIntArray,
                "osg::Object osg::BufferData osg::Array osgEarth::CompressedUIntArray")
            {
                ADD_USER_SERIALIZER(Data);
            }
        }
    }
}

#endif // OSGEARTH_HAVE_MESH_OPTIMIZER