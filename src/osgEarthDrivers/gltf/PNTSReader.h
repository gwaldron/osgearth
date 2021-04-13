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
#ifndef OSGEARTH_PNTS_READER_H
#define OSGEARTH_PNTS_READER_H

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
#define LC "[PNTSReader] "

struct pntsheader
{
    char magic[4];
    unsigned int version;
    unsigned int byteLength;
    unsigned int featureTableJSONByteLength;
    unsigned int featureTableBinaryByteLength;
    unsigned int batchTableJSONByteLength;
    unsigned int batchTableBinaryByteLength;
};

class PNTSReader
{
public:
    mutable GLTFReader::TextureCache* _texCache;

    PNTSReader() : _texCache(NULL)
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
        if (magic != "pnts")
        {
            osg::ref_ptr<osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
            if (compressor.valid())
            {
                std::stringstream in_data(inputStream);
                if (!compressor->decompress(in_data, decompressedData))
                {
                    OE_WARN << LC << "Invalid pnts" << std::endl;
                    return NULL;
                }
                data = &decompressedData;
            }
        }

        pntsheader header;
        unsigned bytesRead = 0;

        std::stringstream buf(*data);
        buf.read(reinterpret_cast<char*>(&header), sizeof(pntsheader));
        bytesRead += sizeof(pntsheader);

#ifdef OE_IS_BIG_ENDIAN
        byteSwapInPlace(header.version);
        byteSwapInPlace(header.byteLength);
        byteSwapInPlace(header.featureTableJSONByteLength);
        byteSwapInPlace(header.featureTableBinaryByteLength);
        byteSwapInPlace(header.batchTableJSONByteLength);
        byteSwapInPlace(header.batchTableBinaryByteLength);
#endif

        size_t sz = header.byteLength;

        unsigned int points_length = 0;

		osg::Vec3d rtc_center;

        bool bQuantized_volume_offset = false;
		osg::Vec3d quantized_volume_offset;

		bool bQuantized_volume_scale = false;
		osg::Vec3d quantized_volume_scale;

		bool bConstant_rgba = false;
		osg::Vec4b constant_rgba;

        unsigned int batch_length = 0;

		bool bPositions = false;
		unsigned int positionByteOffset = 0;

		bool bPosition_quantizeds = false;
		unsigned int positionQuantizedByteOffset = 0;

		bool bRgbas = false;
		unsigned int rgbaByteOffset = 0;

		bool bRgbs = false;
		unsigned int rgbByteOffset = 0;

		bool bRgb565s = false;
		unsigned int rgb565ByteOffset = 0;

		bool bNormals = false;
		unsigned int normalByteOffset = 0;

		bool bDracoCompression = false;
        unsigned int dracoByteOffset = 0;
        unsigned int dracoByteLength = 0;

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
                Json::Value POINTS_LENGTH = doc["POINTS_LENGTH"];
                if (!POINTS_LENGTH.empty())
                {
                    points_length = POINTS_LENGTH.asUInt();
				}

				Json::Value RTC_CENTER = doc["RTC_CENTER"];
				if (!RTC_CENTER.empty())
				{
					Json::Value::iterator i = RTC_CENTER.begin();
					rtc_center.x() = (*i++).asDouble();
					rtc_center.y() = (*i++).asDouble();
					rtc_center.z() = (*i++).asDouble();
				}

				Json::Value QUANTIZED_VOLUME_OFFSET = doc["QUANTIZED_VOLUME_OFFSET"];
                if (!QUANTIZED_VOLUME_OFFSET.empty())
				{
					Json::Value::iterator i = QUANTIZED_VOLUME_OFFSET.begin();
                    quantized_volume_offset.x() = (*i++).asDouble();
                    quantized_volume_offset.y() = (*i++).asDouble();
                    quantized_volume_offset.z() = (*i++).asDouble();
                    bQuantized_volume_offset = true;
				}

				Json::Value QUANTIZED_VOLUME_SCALE = doc["QUANTIZED_VOLUME_SCALE"];
				if (!QUANTIZED_VOLUME_SCALE.empty())
				{
					Json::Value::iterator i = QUANTIZED_VOLUME_SCALE.begin();
                    quantized_volume_scale.x() = (*i++).asDouble();
                    quantized_volume_scale.y() = (*i++).asDouble();
					quantized_volume_scale.z() = (*i++).asDouble();
                    bQuantized_volume_scale = true;
				}

				Json::Value CONSTANT_RGBA = doc["CONSTANT_RGBA"];
				if (!CONSTANT_RGBA.empty())
				{
					Json::Value::iterator i = CONSTANT_RGBA.begin();
                    constant_rgba.r() = (*i++).asInt();
                    constant_rgba.g() = (*i++).asInt();
					constant_rgba.b() = (*i++).asInt();
					constant_rgba.a() = (*i++).asInt();
                    bConstant_rgba = true;
				}

				Json::Value BATCH_LENGTH = doc["BATCH_LENGTH"];
                if (!BATCH_LENGTH.empty())
                {
                    batch_length = BATCH_LENGTH.asUInt();
				}

				Json::Value POSITION = doc["POSITION"];
				if (!POSITION.empty())
				{
					Json::Value BYTEOFFSET = doc["byteOffset"];
                    positionByteOffset = BYTEOFFSET.asUInt();
                    bPositions = true;
				}

				Json::Value POSITION_QUANTIZED = doc["POSITION_QUANTIZED"];
				if (!POSITION_QUANTIZED.empty())
				{
					Json::Value BYTEOFFSET = doc["byteOffset"];
					positionQuantizedByteOffset = BYTEOFFSET.asUInt();
					bPosition_quantizeds = true;
				}

				Json::Value RGBA = doc["RGBA"];
				if (!RGBA.empty())
				{
					Json::Value BYTEOFFSET = doc["byteOffset"];
					rgbaByteOffset = BYTEOFFSET.asUInt();
					bRgbas = true;
				}

				Json::Value RGB = doc["RGB"];
				if (!RGB.empty())
				{
					Json::Value BYTEOFFSET = doc["byteOffset"];
					rgbByteOffset = BYTEOFFSET.asUInt();
					bRgbs = true;
				}

				Json::Value RGB565 = doc["RGB565"];
				if (!RGB565.empty())
				{
					Json::Value BYTEOFFSET = doc["byteOffset"];
					rgb565ByteOffset = BYTEOFFSET.asUInt();
					bRgb565s = true;
				}

				Json::Value NORMAL = doc["NORMAL"];
				if (!NORMAL.empty())
				{
					Json::Value BYTEOFFSET = doc["byteOffset"];
					normalByteOffset = BYTEOFFSET.asUInt();
					bNormals = true;
				}

				Json::Value extensions = doc["extensions"];
                if (!extensions.empty())
                {
					Json::Value draco_point_compression = extensions["3DTILES_draco_point_compression"];
					if (!draco_point_compression.empty())
                    {
						bDracoCompression = true;
						unsigned int byteOffset = draco_point_compression["byteOffset"].asUInt();
                        unsigned int byteLength = draco_point_compression["byteLength"].asUInt();

                        dracoByteOffset = byteOffset;
                        dracoByteLength = byteLength;
                    }
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

		//创建顶点数组
		osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
		osg::ref_ptr<osg::Vec4bArray> colors = new osg::Vec4bArray();

        if (header.featureTableBinaryByteLength > 0)
        {
            std::string featureTableBinary;
            featureTableBinary.resize(header.featureTableBinaryByteLength);
            buf.read(reinterpret_cast<char*>(&featureTableBinary[0]), header.featureTableBinaryByteLength);
            OE_DEBUG << "Read featureTableJson " << featureTableBinary << std::endl;
			bytesRead += header.featureTableBinaryByteLength;

            if (bDracoCompression)
			{
#ifdef OSGEARTH_HAVE_DRACO
				char* buffer = reinterpret_cast<char*>(&featureTableBinary[0]);

				draco::DecoderBuffer decbuffer;
                decbuffer.Init((const char*)buffer + dracoByteOffset, dracoByteLength);

				auto type_statusor = draco::Decoder::GetEncodedGeometryType(&decbuffer);
				const draco::EncodedGeometryType geom_type = type_statusor.value();
				draco::Decoder decoder;
				if (geom_type == draco::TRIANGULAR_MESH) {
					auto statusor = decoder.DecodeMeshFromBuffer(&decbuffer);
					std::unique_ptr<draco::Mesh> mesh = std::move(statusor).value();
					draco::Mesh* pMesh = mesh.get();

					//xDracoMesh::ToMesh(pMesh, mdlMesh, mdlPrim);
				}
                else if (geom_type == draco::POINT_CLOUD) {
                    auto statusor = decoder.DecodePointCloudFromBuffer(&decbuffer);
                    std::unique_ptr<draco::PointCloud> pc = std::move(statusor).value();
					draco::PointCloud* pPC = pc.get(); const int pos_att_id = pPC->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
					const int color_att_id = pPC->GetNamedAttributeId(draco::GeometryAttribute::COLOR);
					const int tex_coord_att_id = pPC->GetNamedAttributeId(draco::GeometryAttribute::TEX_COORD);
					const int normal_att_id = pPC->GetNamedAttributeId(draco::GeometryAttribute::NORMAL);

					if (pos_att_id >= 0)
					{
						const draco::PointAttribute* pos_att = pPC->attribute(pos_att_id);
						draco::DataType dataType = pos_att->data_type();
						int8_t components = pos_att->num_components();

						for (draco::PointIndex v(0); v < pPC->num_points(); ++v) {
							const uint8_t* address = pos_att->GetAddress(pos_att->mapped_index(v));
							if (dataType == draco::DT_INT8)
							{
								const int8_t* vals = (const int8_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_UINT8)
							{
								const uint8_t* vals = (const uint8_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_INT16)
							{
								const int16_t* vals = (const int16_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_UINT16)
							{
								const uint16_t* vals = (const uint16_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_INT32)
							{
								const int32_t* vals = (const int32_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_UINT32)
							{
								const uint32_t* vals = (const uint32_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_INT64)
							{
								const int64_t* vals = (const int64_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_UINT64)
							{
								const uint64_t* vals = (const uint64_t*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_FLOAT32)
							{
								const float* vals = (const float*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
							else if (dataType == draco::DT_FLOAT64)
							{
								const double* vals = (const double*)address;
								osg::Vec3f pt;
								pt.x() = vals[0];
								pt.y() = vals[1];
								pt.z() = vals[2];
								//pt.state.classification = _CLASS_CREATED_;
								coords->push_back(pt);
							}
						}
					}

					if (color_att_id >= 0)
					{
						const draco::PointAttribute* color_att = pPC->attribute(color_att_id);
						draco::DataType dataType = color_att->data_type();
						int8_t components = color_att->num_components();

						for (draco::PointIndex v(0); v < pPC->num_points(); ++v) {
							const uint8_t* address = color_att->GetAddress(color_att->mapped_index(v));
							if (dataType == draco::DT_INT8)
							{
								const int8_t* vals = (const int8_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_UINT8)
							{
								const uint8_t* vals = (const uint8_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_INT16)
							{
								const int16_t* vals = (const int16_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_UINT16)
							{
								const uint16_t* vals = (const uint16_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_INT32)
							{
								const int32_t* vals = (const int32_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_UINT32)
							{
								const uint32_t* vals = (const uint32_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_INT64)
							{
								const int64_t* vals = (const int64_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_UINT64)
							{
								const uint64_t* vals = (const uint64_t*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_FLOAT32)
							{
								const float* vals = (const float*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
							else if (dataType == draco::DT_FLOAT64)
							{
								const double* vals = (const double*)address;
								osg::Vec4b clr;
								if (components > 0)
									clr.r() = vals[0];
								if (components > 1)
									clr.g() = vals[1];
								if (components > 2)
									clr.b() = vals[2];
								if (components > 3)
									clr.a() = vals[3];
								colors->push_back(clr);
							}
						}
					}

					if (normal_att_id >= 0)
					{
						const draco::PointAttribute* normal_att = pPC->attribute(normal_att_id);
						draco::DataType dataType = normal_att->data_type();
						int8_t components = normal_att->num_components();

						for (draco::PointIndex v(0); v < pPC->num_points(); ++v) {
							const uint8_t* address = normal_att->GetAddress(normal_att->mapped_index(v));
							if (dataType == draco::DT_INT8)
							{
								const int8_t* vals = (const int8_t*)address;
							}
							else if (dataType == draco::DT_UINT8)
							{
								const uint8_t* vals = (const uint8_t*)address;
							}
							else if (dataType == draco::DT_INT16)
							{
								const int16_t* vals = (const int16_t*)address;
							}
							else if (dataType == draco::DT_UINT16)
							{
								const uint16_t* vals = (const uint16_t*)address;
							}
							else if (dataType == draco::DT_INT32)
							{
								const int32_t* vals = (const int32_t*)address;
							}
							else if (dataType == draco::DT_UINT32)
							{
								const uint32_t* vals = (const uint32_t*)address;
							}
							else if (dataType == draco::DT_INT64)
							{
								const int64_t* vals = (const int64_t*)address;
							}
							else if (dataType == draco::DT_UINT64)
							{
								const uint64_t* vals = (const uint64_t*)address;
							}
							else if (dataType == draco::DT_FLOAT32)
							{
								const float* vals = (const float*)address;
							}
							else if (dataType == draco::DT_FLOAT64)
							{
								const double* vals = (const double*)address;
							}
						}
					}
				}
#endif
            }
            else
            {
                char* buffer = reinterpret_cast<char*>(&featureTableBinary[0]);
                if (bPositions)
                {                 
                    float* positions = (float*)(buffer + positionByteOffset);
                    for (int i = 0; i < points_length; i++)
                    {
                        int idx = i * 3;
                        osg::Vec3f pt;
						pt.x() = positions[idx] + rtc_center[0];
						pt.y() = positions[idx + 1] + rtc_center[1];
						pt.z() = positions[idx + 2] + rtc_center[2];
						//pt.state.classification = _CLASS_CREATED_;
                        coords->push_back(pt);
                    }
                }
                else if (bPosition_quantizeds)
				{
					unsigned short* position_quantizeds = (unsigned short*)(buffer + positionQuantizedByteOffset);
                    //POSITION = POSITION_QUANTIZED * QUANTIZED_VOLUME_SCALE / 65535.0 + QUANTIZED_VOLUME_OFFSET
                    for (int i = 0; i < points_length; i++)
                    {
						int idx = i * 3;
						osg::Vec3f pt;
                        pt.x() = position_quantizeds[idx] * quantized_volume_scale[0] / 65535.0 + quantized_volume_offset[0] + rtc_center[0];
                        pt.y() = position_quantizeds[idx + 1] * quantized_volume_scale[1] / 65535.0 + quantized_volume_offset[1] + rtc_center[1];
                        pt.z() = position_quantizeds[idx + 2] * quantized_volume_scale[2] / 65535.0 + quantized_volume_offset[2] + rtc_center[2];
						//pt.state.classification = _CLASS_CREATED_;
						coords->push_back(pt);
                    }
				}

				if (bRgbs)
				{
					unsigned char* rgbs = (unsigned char*)(buffer + rgbByteOffset);
					for (int i = 0; i < points_length; i++)
					{
                        int idx = i * 3;
                        osg::Vec4b clr;
						clr.r() = rgbs[idx];
                        clr.g() = rgbs[idx + 1];
                        clr.b() = rgbs[idx + 2];
                        colors->push_back(clr);
					}
				}

				if (bRgbas)
				{
					unsigned char* rgbas = (unsigned char*)(buffer + rgbaByteOffset);
					for (int i = 0; i < points_length; i++)
					{
						int idx = i * 4;
						osg::Vec4b clr;
                        clr.r() = rgbas[idx];
                        clr.g() = rgbas[idx + 1];
                        clr.b() = rgbas[idx + 2];
						clr.a() = rgbas[idx + 3];
						colors->push_back(clr);
					}
				}
            }
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

		//create geode
		osg::Geode* geode = new osg::Geode();

		//create geometry
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

		//create array
		geometry->setVertexArray(coords.get());
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, coords->size()));

		geometry->setUseDisplayList(true);
		geometry->setUseVertexBufferObjects(true);

		geode->addDrawable(geometry.get());

		osg::Node* modelNode = geode;

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

#endif // OSGEARTH_PNTS_READER_H