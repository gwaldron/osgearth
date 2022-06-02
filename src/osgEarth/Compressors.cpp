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

#include <osgDB/ObjectWrapper>
#include <osgDB/DataTypes>

#if OSGEARTH_HAVE_BLOSC
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

