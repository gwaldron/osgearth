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
#include <osg/Notify>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <basisu/transcoder/basisu_transcoder.h>

using namespace basisu;

class ReaderWriterBasis : public osgDB::ReaderWriter
{   
public:
    ReaderWriterBasis()
    {
        supportsExtension("basis", "Basis image format");

        // one-time initialization at startup
        basist::basisu_transcoder_init();
        sel_codebook = basist::etc1_global_selector_codebook(basist::g_global_selector_cb_size, basist::g_global_selector_cb);        
    }
    
    virtual const char* className() const { return "Basis Universal Image Reader/Writer"; }
   
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
        char * data = new char[length];
        fin.read(data, length);        

        basist::basisu_transcoder transcoder(&sel_codebook);

        unsigned int numImages = transcoder.get_total_images(data, length);        

        // TODO:  Pass in an option or automatically determine the desired output format

        /*
                basist::transcoder_texture_format transcoder_texture_format = basist::transcoder_texture_format::cTFETC1;
                GLenum internalTextureFormat = GL_COMPRESSED_RGB8_ETC2;
                GLenum pixelFormat = GL_COMPRESSED_RGB8_ETC2;
                */

        basist::transcoder_texture_format transcoder_texture_format = basist::transcoder_texture_format::cTFBC1;
        GLenum internalTextureFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
        GLenum pixelFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;

        /*
        basist::transcoder_texture_format transcoder_texture_format = basist::transcoder_texture_format::cTFBC3;
        GLenum internalTextureFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
        GLenum pixelFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
        */

        basist::basisu_image_info image_info;
        unsigned int imageIndex = 0;
        if (transcoder.get_image_info(data, length, image_info, imageIndex))
        {            
            OSG_INFO << "Got image info " << std::endl
                << "Dimensions " << image_info.m_width << "x" << image_info.m_height << std::endl
                << "Total levels " << image_info.m_total_levels << std::endl
                << "Alpha " << image_info.m_alpha_flag << std::endl
                << "Total levels " << image_info.m_total_levels << std::endl;

            transcoder.start_transcoding(data, length);

            unsigned int totalSize = 0;

            std::vector< unsigned int > mipmapDataOffsets;
            mipmapDataOffsets.reserve( image_info.m_total_levels - 1);

            // Compute the total size of the output buffer
            for (unsigned int levelIndex = 0; levelIndex < image_info.m_total_levels; levelIndex++)
            {
                if (levelIndex > 0)
                {
                    mipmapDataOffsets.push_back(totalSize);
                }

                basist::basisu_image_level_info level_info;
                transcoder.get_image_level_info(data, length, level_info, 0, levelIndex);

                unsigned int bytesPerBlock = basist::basis_get_bytes_per_block(transcoder_texture_format);
                unsigned int levelSize = bytesPerBlock * level_info.m_total_blocks;
                totalSize += levelSize;
            }

            // Allocate memory for the total image including mipmaps
            unsigned char *decoded = new unsigned char[totalSize];
            memset(decoded, 0, totalSize);            

            for (unsigned int levelIndex = 0; levelIndex < image_info.m_total_levels; levelIndex++)
            {
                basist::basisu_image_level_info level_info;            
                transcoder.get_image_level_info(data, length, level_info, 0, levelIndex);                

                unsigned int offset = 0;
                if (levelIndex > 0)
                {
                    offset = mipmapDataOffsets[levelIndex - 1];
                }

                transcoder.transcode_image_level(data, length, imageIndex, levelIndex, &decoded[offset], level_info.m_total_blocks, transcoder_texture_format, 0);                
            }

            osg::Image* image = new osg::Image;
            image->setImage(image_info.m_width, image_info.m_height, 1, internalTextureFormat, pixelFormat, GL_UNSIGNED_BYTE, decoded, osg::Image::USE_NEW_DELETE);
            if (!mipmapDataOffsets.empty())
            {
                image->setMipmapLevels(mipmapDataOffsets);
            }
            return image;
        }        

        return ReadResult::ERROR_IN_READING_FILE;
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

private:
    basist::etc1_global_selector_codebook sel_codebook;
};

REGISTER_OSGPLUGIN(basis, ReaderWriterBasis)