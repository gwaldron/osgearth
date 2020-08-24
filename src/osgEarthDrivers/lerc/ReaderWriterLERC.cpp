/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Notify>

#include <Lerc_c_api.h>
#include <Lerc_types.h>

#define LC "[lerc] "

typedef unsigned char Byte;    // convenience
typedef unsigned int uint32;

class ReaderWriterLERC : public osgDB::ReaderWriter
{
public:
    ReaderWriterLERC()
    {
        supportsExtension("lerc", "ESRI Lerc");

    }

    virtual const char* className() const { return "ESRI Lerc"; }

    virtual ReadResult readObject(std::istream& fin, const osgDB::ReaderWriter::Options* options = NULL) const
    {
        return readImage(fin, options);
    }

    virtual ReadResult readObject(const std::string& file, const osgDB::ReaderWriter::Options* options = NULL) const
    {
        return readImage(file, options);
    }

    virtual ReadResult readImage(std::istream& fin, const osgDB::ReaderWriter::Options* = NULL) const
    {
        // get length of file:
        fin.seekg(0, fin.end);
        int length = fin.tellg();
        fin.seekg(0, fin.beg);
        char* data = new char[length];
        fin.read(data, length);

        uint32 infoArr[8];

        lerc_status hr(0);

        hr = lerc_getBlobInfo((const unsigned char*)(data), length, infoArr, NULL, 8, 0);
        if (hr)
        {
            OE_WARN << LC << "Failed to get blob info error = " << hr << std::endl;
            return ReadResult::ERROR_IN_READING_FILE;
        }

        unsigned int dataType = infoArr[1];
        unsigned int numDims = infoArr[2];
        unsigned int width = infoArr[3];
        unsigned int height = infoArr[4];
        unsigned int numBands = infoArr[5];
        unsigned int nValidPixels = infoArr[6];
        unsigned int blobSize = infoArr[7];

        GLenum glDataType;
        int    sampleSize;
        GLint  internalFormat;

        switch (dataType)
        {
            case (uint32)LercNS::DataType::dt_char:
            {
                sampleSize = sizeof(char);
                glDataType = GL_UNSIGNED_BYTE;
                internalFormat =
                    numBands == 1 ? GL_R8 :
                    numBands == 2 ? GL_RG8 :
                    numBands == 3 ? GL_RGB8 :
                    GL_RGBA8;
                break;
            }
            case (uint32)LercNS::DataType::dt_uchar:
            {
                sampleSize = sizeof(unsigned char);
                glDataType = GL_UNSIGNED_BYTE;
                internalFormat =
                    numBands == 1 ? GL_R8 :
                    numBands == 2 ? GL_RG8 :
                    numBands == 3 ? GL_RGB8 :
                    GL_RGBA8;
                break;
            }
            case (uint32)LercNS::DataType::dt_short:
            {
                sampleSize = sizeof(short);
                glDataType = GL_SHORT;
                internalFormat =
                    numBands == 1 ? GL_R16 :
                    numBands == 2 ? GL_RG16 :
                    numBands == 3 ? GL_RGB16 :
                    GL_RGBA16;
                break;
            }
            case (uint32)LercNS::DataType::dt_ushort:
            {
                sampleSize = sizeof(unsigned short);
                glDataType = GL_UNSIGNED_SHORT;
                internalFormat =
                    numBands == 1 ? GL_R16 :
                    numBands == 2 ? GL_RG16 :
                    numBands == 3 ? GL_RGB16 :
                    GL_RGBA16;
                break;
            }
            case (uint32)LercNS::DataType::dt_double:
            {
                sampleSize = sizeof(double);
                glDataType = GL_DOUBLE;
                internalFormat = GL_R32F;
                internalFormat =
                    numBands == 1 ? GL_R32F :
                    numBands == 2 ? GL_RG32F :
                    numBands == 3 ? GL_RGB32F :
                    GL_RGBA32F;
                break;
            }
            default:
            {
                sampleSize = sizeof(float);
                glDataType = GL_FLOAT;
                internalFormat = GL_R32F;
                internalFormat =
                    numBands == 1 ? GL_R32F :
                    numBands == 2 ? GL_RG32F :
                    numBands == 3 ? GL_RGB32F :
                    GL_RGBA32F;
                break;
            }

        }

        GLenum pixelFormat =
            numBands == 1 ? GL_RED :
            numBands == 2 ? GL_RG :
            numBands == 3 ? GL_RGB :
            GL_RGBA;

        // Allocate enough memory to hold the output image.
        unsigned int totalOutputSize = width * height * sampleSize * numDims * numBands;
        Byte* output = new Byte[totalOutputSize];
        memset(output, 0, totalOutputSize);

        // Decode the image
        unsigned int bandOffset = 0;
        hr = lerc_decode((const unsigned char*)(data), length, 0, numDims, width, height, numBands, dataType, (void*)output);
        if (hr)
        {
            delete[]output;
            OE_WARN << LC << "Failed to decode lerc blob error=" << hr << std::endl;
            return ReadResult::ERROR_IN_READING_FILE;
        }

        // Allocate the final output image
        osg::ref_ptr< osg::Image > image = new osg::Image;

        if (numBands > 1)
        {
            // Interleave the bands properly if we have more than one.
            image->allocateImage(width, height, 1, pixelFormat, glDataType);
            memset(image->data(), 0, image->getImageSizeInBytes());

            unsigned int bandSize = width * height * sampleSize * numDims;

            for (unsigned int r = 0; r < height; ++r)
            {
                for (unsigned int c = 0; c < width; ++c)
                {
                    for (unsigned int band = 0; band < numBands; ++band)
                    {
                        Byte* bandPtr = output + (band * bandSize);
                        *(image->data(c, r) + band) = bandPtr[r * width + c];
                    }
                }
            }
            // Delete the original output array, no longer needed.
            delete[]output;
        }
        else
        {
            // Just set the output array as the image, image takes ownership of the array.
            image->setImage(width, height, 1, internalFormat, pixelFormat, glDataType, (unsigned char*)output, osg::Image::USE_NEW_DELETE);
        }

        image->flipVertical();
        image->setInternalTextureFormat(internalFormat);

        return image;
    }

    virtual ReadResult readImage(const std::string& file, const osgDB::ReaderWriter::Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension(file);
        if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

        std::string fileName = osgDB::findDataFile(file, options);
        if (fileName.empty()) return ReadResult::FILE_NOT_FOUND;

        osgDB::ifstream istream(fileName.c_str(), std::ios::in | std::ios::binary);
        if (!istream) return ReadResult::ERROR_IN_READING_FILE;

        return readImage(istream, options);
    }
};

REGISTER_OSGPLUGIN(lerc, ReaderWriterLERC)