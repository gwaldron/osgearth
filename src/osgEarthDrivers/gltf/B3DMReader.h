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
#ifndef OSGEARTH_B3DM_READER_H
#define OSGEARTH_B3DM_READER_H

#include <osgEarth/Endian>
#include <osgEarth/URI>
#include <osgEarth/JsonUtils>
#include <osgDB/ObjectWrapper>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include "GLTFReader.h"

using namespace osgEarth;

#undef LC
#define LC "[B3DMReader] "

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

class B3DMReader
{
public:
    mutable GLTFReader::TextureCache* _texCache;

    B3DMReader() : _texCache(NULL)
    {
    }

    void setTextureCache(GLTFReader::TextureCache* cache) const
    {
        _texCache = cache;
    }

    static std::string ExpandFilePath(const std::string &filepath, void * userData)
    {
        const std::string& referrer = *(const std::string*)userData;
        std::string path = osgDB::getRealPath(osgDB::isAbsolutePath(filepath) ? filepath : osgDB::concatPaths(osgDB::getFilePath(referrer), filepath));
        //OSG_NOTICE << "ExpandFilePath: expanded " << filepath << " to " << path << std::endl;
        return tinygltf::ExpandFilePath(path, userData);
    }

    //! Read a B3DM file and return a node
    //osg::Node* read(const std::string& location, const osgDB::Options* readOptions) const
    //{
    //    // Load the whole thing into memory
    //    URIStream inputStream(location, std::ifstream::binary);
    //    return read(location, inputStream, readOptions);
    //}

    //! Read a B3DM data package and return a node.
    osg::Node* read(const std::string& location, const std::string& inputStream, const osgDB::Options* readOptions) const
    {
        // Check the header's magic string. If it's not there, attempt
        // to run a decompressor on it

        std::string decompressedData;
        const std::string* data = &inputStream;

        std::string magic(*data, 0, 4);
        if (magic != "b3dm")
        {
            osg::ref_ptr<osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
            if (compressor.valid())
            {
                std::stringstream in_data(inputStream);
                if (!compressor->decompress(in_data, decompressedData))
                {
                    OE_WARN << LC << "Invalid b3dm" << std::endl;
                    return NULL;
                }
                data = &decompressedData;
            }
        }

        b3dmheader header;
        unsigned bytesRead = 0;

        std::stringstream buf(*data);
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
            OE_DEBUG << "Read featureTableJson " << featureTableJson << std::endl;

            osgEarth::Json::Reader reader;
            osgEarth::Json::Value doc;
            if (reader.parse(featureTableJson, doc))
            {
                Json::Value RTC_CENTER = doc["RTC_CENTER"];
                if (!RTC_CENTER.empty())
                {
                    Json::Value::iterator i = RTC_CENTER.begin();
                    rtc_center.x() = (*i++).asDouble();
                    rtc_center.y() = (*i++).asDouble();
                    rtc_center.z() = (*i++).asDouble();
                }
            }          

            /*
            json ftJson = json::parse(featureTableJson);
            if (ftJson.find("RTC_CENTER") != ftJson.end()) {
                json RTC_CENTER = ftJson["RTC_CENTER"];
                rtc_center.x() = RTC_CENTER[0];
                rtc_center.y() = RTC_CENTER[1];
                rtc_center.z() = RTC_CENTER[2];
            }
            OE_DEBUG << LC << "Read rtc_center " << rtc_center.x() << ", " << rtc_center.y() << ", " << rtc_center.z() << std::endl;
            */

            bytesRead += header.featureTableJSONByteLength;
        }

        if (header.featureTableBinaryByteLength > 0)
        {
            std::string featureTableBinary;
            featureTableBinary.resize(header.featureTableBinaryByteLength);
            buf.read(reinterpret_cast<char*>(&featureTableBinary[0]), header.featureTableBinaryByteLength);
            OE_DEBUG << "Read featureTableJson " << featureTableBinary << std::endl;
            bytesRead += header.featureTableBinaryByteLength;
        }

        if (header.batchTableJSONByteLength > 0)
        {
            std::string batchTableJSON;
            batchTableJSON.resize(header.batchTableJSONByteLength);
            buf.read(reinterpret_cast<char*>(&batchTableJSON[0]), header.batchTableJSONByteLength);
            OE_DEBUG << "Read batchTableJSON " << batchTableJSON << std::endl;
            bytesRead += header.batchTableJSONByteLength;
        }

        if (header.batchTableBinaryByteLength > 0)
        {
            std::string batchTableBinary;
            batchTableBinary.resize(header.batchTableBinaryByteLength);
            buf.read(reinterpret_cast<char*>(&batchTableBinary[0]), header.batchTableBinaryByteLength);
            OE_DEBUG << "Read batchTableJSON " << batchTableBinary << std::endl;
            bytesRead += header.batchTableBinaryByteLength;
        }

        std::string gltfData;
        gltfData.resize(sz - bytesRead);
        buf.read(reinterpret_cast<char *>(&gltfData[0]), static_cast<std::streamsize>(gltfData.size()));

        tinygltf::Model model;
        tinygltf::TinyGLTF loader;
        std::string err;
        std::string warn;

        FsCallbacks fs;
        fs.FileExists = &tinygltf::FileExists;
        fs.ExpandFilePath = &B3DMReader::ExpandFilePath;
        fs.ReadWholeFile = &tinygltf::ReadWholeFile;
        fs.WriteWholeFile = &tinygltf::WriteWholeFile;
        fs.user_data = (void*)&location;
        loader.SetFsCallbacks(fs);

        tinygltf::Options opt;
        opt.skip_imagery = readOptions && readOptions->getOptionString().find("gltfSkipImagery") != std::string::npos;        

        loader.LoadBinaryFromMemory(&model, &err, &warn, reinterpret_cast<unsigned char*>(&gltfData[0]), sz, "", REQUIRE_VERSION, &opt);

        if (!err.empty())
            OE_WARN << LC << "GLTF ERROR: " << err << std::endl;
        if (!warn.empty())
            OE_WARN << LC << "GLTF WARNING: " << warn << std::endl;

        GLTFReader gltfReader;
        gltfReader.setTextureCache(_texCache);
        GLTFReader::Env env(location, readOptions);
        osg::Node* modelNode = gltfReader.makeNodeFromModel(model, env);
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
};

#endif // OSGEARTH_B3DM_READER_H