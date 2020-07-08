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
#ifndef OSGEARTH_B3DM_WRITER_H
#define OSGEARTH_B3DM_WRITER_H

#include "B3DMReader.h"
#include "GLTFWriter.h"
#include <osgEarth/JsonUtils>
using namespace osgEarth;

#undef LC
#define LC "[B3DMWriter] "

static void WriteBinaryGltfStream(std::ostream& gltfFile,
                                  const std::string &content) {
    const std::string header = "glTF";
    const int version = 2;
    const int padding_size = content.size() % 4;

    // 12 bytes for header, JSON content length, 8 bytes for JSON chunk info, padding
    const int length = 12 + 8 + int(content.size()) + padding_size;

    gltfFile.write(header.c_str(), header.size());
    gltfFile.write(reinterpret_cast<const char *>(&version), sizeof(version));
    gltfFile.write(reinterpret_cast<const char *>(&length), sizeof(length));

    // JSON chunk info, then JSON data
    const int model_length = int(content.size()) + padding_size;
    const int model_format = 0x4E4F534A;
    gltfFile.write(reinterpret_cast<const char *>(&model_length), sizeof(model_length));
    gltfFile.write(reinterpret_cast<const char *>(&model_format), sizeof(model_format));
    gltfFile.write(content.c_str(), content.size());

    // Chunk must be multiplies of 4, so pad with spaces
    if (padding_size > 0) {
        const std::string padding = std::string(padding_size, ' ');
        gltfFile.write(padding.c_str(), padding.size());
    }
}

class OETinyGLTF : public TinyGLTF
{
public:
    bool WriteGltfSceneToBinaryStream(Model *model,
                                      const std::string& filename,
                                      std::ostream& stream,
                                      bool embedImages,
                                      bool embedBuffers);
protected:
    // Private tiny_gltf stuff that we need
    WriteImageDataFunction WriteImageData = &tinygltf::WriteImageData;
      FsCallbacks fs = {
          &tinygltf::FileExists, &tinygltf::ExpandFilePath,
          &tinygltf::ReadWholeFile, &tinygltf::WriteWholeFile,
          nullptr  // Fs callback user data
      };
};

bool OETinyGLTF::WriteGltfSceneToBinaryStream(Model *model,
                                              const std::string& filename,
                                              std::ostream& stream,
                                              bool embedImages = false,
                                              bool embedBuffers = false)
{
    JsonDocument output;
    bool prettyPrint = false;

    /// Serialize all properties except buffers and images.
    SerializeGltfModel(model, output);

    std::string defaultBinFilename = GetBaseFilename(filename);
    std::string defaultBinFileExt = ".bin";
    std::string::size_type pos = defaultBinFilename.rfind('.', defaultBinFilename.length());

    if (pos != std::string::npos) {
        defaultBinFilename = defaultBinFilename.substr(0, pos);
    }
    std::string baseDir = GetBaseDir(filename);
    if (baseDir.empty()) {
        baseDir = "./";
    }

        // BUFFERS
    std::vector<std::string> usedUris;
    json buffers;
    JsonReserveArray(buffers, model->buffers.size());
    for (unsigned int i = 0; i < model->buffers.size(); ++i) {
        json buffer;
        if (embedBuffers) {
            SerializeGltfBuffer(model->buffers[i], buffer);
        }
        else {
            std::string binSavePath;
            std::string binUri;
            if (!model->buffers[i].uri.empty() && !IsDataURI(model->buffers[i].uri)) {
                binUri = model->buffers[i].uri;
            }
            else {
                binUri = defaultBinFilename + defaultBinFileExt;
                bool inUse = true;
                int numUsed = 0;
                while (inUse) {
                    inUse = false;
                    for (const std::string& usedName : usedUris) {
                        if (binUri.compare(usedName) != 0) continue;
                        inUse = true;
                        binUri = defaultBinFilename + std::to_string(numUsed++) + defaultBinFileExt;
                        break;
                    }
                }
            }
            usedUris.push_back(binUri);
            binSavePath = JoinPath(baseDir, binUri);
            if (!SerializeGltfBuffer(model->buffers[i], buffer, binSavePath,
                binUri)) {
                return false;
            }
        }
        JsonPushBack(buffers, std::move(buffer));
    }
    JsonAddMember(output, "buffers", std::move(buffers));

    // IMAGES
    if (model->images.size()) {
        json images;
        JsonReserveArray(images, model->images.size());
        for (unsigned int i = 0; i < model->images.size(); ++i) {
            json image;

            UpdateImageObject(model->images[i], baseDir, int(i), embedImages,
                              &this->WriteImageData, reinterpret_cast<void *>(&fs));
            SerializeGltfImage(model->images[i], image);
            JsonPushBack(images, std::move(image));
        }
        JsonAddMember(output, "images", std::move(images));
    }
    WriteBinaryGltfStream(stream, JsonToString(output, prettyPrint ? 2 : -1));
    return true;
}


class B3DMWriter
{
public:
    osgDB::ReaderWriter::WriteResult write(const osg::Node& node, 
                                           const std::string& location, 
                                           bool isBinary,
                                           const osgDB::Options* options) const
    {
        std::fstream fout(location.c_str(), std::ios::out | std::ios::binary);
        if (!fout.is_open())
            return osgDB::ReaderWriter::WriteResult::ERROR_IN_WRITING_FILE;

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
        GLTFWriter gltfWriter;
        tinygltf::Model model;
        gltfWriter.convertOSGtoGLTF(node, model);

        OETinyGLTF gltfOut;
        std::ostringstream gltfBuf;
        gltfOut.WriteGltfSceneToBinaryStream(&model, location, gltfBuf, true, true);
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

        std::stringstream compressionInput;
        std::ostream* output = &fout;

        osg::ref_ptr<osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
        if (compressor.valid())
        {
            output = &compressionInput;
        }

        // B3DM header:
        output->write((const char*)&header, sizeof(b3dmheader));

        // Tables:
        output->write(featureTableJSON.c_str(), featureTableJSON.length());
        output->write("   ", featureTablePadding);

        // If we want to write the other 3 tables, they go here

        // GLTF binary data
        output->write(gltfData.c_str(), gltfData.length());
        output->write("\0\0\0", gltfDataPadding);

        if (compressor.valid())
        {
            compressor->compress(fout, compressionInput.str());
        }

        fout.close();

        return osgDB::ReaderWriter::WriteResult::FILE_SAVED;
    }
};

#endif // OSGEARTH_B3DM_WRITER_H
