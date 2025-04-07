/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgDB/ObjectWrapper>
#include <osgDB/DataTypes>

#ifdef OSGEARTH_HAVE_BLOSC
#include <blosc.h>

class BloscCompressor : public osgDB::BaseCompressor
{
public:
    BloscCompressor()
    {
    }

    virtual bool compress(std::ostream& fout, const std::string& src)
    {
        unsigned int destSize = src.size() + BLOSC_MAX_OVERHEAD;
        char* dest = new char[destSize];
        std::size_t typeSize = 1;
        bool compressed = false;
        unsigned int numThreads = 1;
        //int numBytes = blosc_compress(_compressionLevel, BLOSC_NOSHUFFLE, typeSize, src.size(), src.c_str(), dest, destSize);
        int numBytes = blosc_compress_ctx(_compressionLevel, BLOSC_NOSHUFFLE, typeSize, src.size(), src.c_str(), dest, destSize, "blosclz", 0, numThreads);
        if (numBytes > 0)
        {
            compressed = true;
            fout.write((char*)&numBytes, osgDB::INT_SIZE);
            fout.write(dest, numBytes);
        }
        delete[]dest;
        return compressed;
    }

    virtual bool decompress(std::istream& fin, std::string& target)
    {
        // Read the size of the buffer
        int size = 0; fin.read((char*)&size, osgDB::INT_SIZE);

        // Read the buffer into memory
        char* buffer = new char[size];
        fin.read(buffer, size);

        bool decompressed = false;
        // Compute the number of bytes to allocate for the destination buffer
        size_t numBytes;
        int validated = blosc_cbuffer_validate(buffer, size, &numBytes);
        if (validated == 0)
        {
            char* dest = new char[numBytes];
            //int decompressedSize = blosc_decompress(buffer, dest, numBytes);
            unsigned int numThreads = 1;
            int decompressedSize = blosc_decompress_ctx(buffer, dest, numBytes, numThreads);
            if (decompressedSize > 0)
            {
                target.resize(decompressedSize);
                target.assign(dest, decompressedSize);
            }
            delete[] dest;
            decompressed = true;
        }
        delete[]buffer;
        return decompressed;
    }

    unsigned int _compressionLevel = 5;
};

REGISTER_COMPRESSOR("blosc", BloscCompressor)

#endif

