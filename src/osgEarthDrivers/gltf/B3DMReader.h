/* osgEarth
* Copyright 2026 Pelican Mapping
* MIT License
*/
#pragma once

/**
 * 3D Tiles 1.0 Batched 3D Model (b3dm) reader.
 *
 * A b3dm is a 28-byte header, a feature table, a batch table and an embedded
 * GLB. The GLB is handed to GLTFReader in place (nothing is copied). The
 * feature table's RTC_CENTER, when present, becomes a translation above the
 * glTF root, which already carries the Y-up to Z-up rotation. Batch tables
 * are skipped.
 *
 * The two legacy header layouts that Cesium still accepts (20 and 24 bytes,
 * written by early tilers) are recognized the same way Cesium detects them.
 */

#include "GLTFReader.h"

#include <osgEarth/JsonUtils>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#undef LC
#define LC "[gltf] "

class B3DMReader
{
public:
    //! True when the buffer starts with the b3dm magic.
    static bool isB3DM(const void* bytes, std::size_t size)
    {
        return size >= 4 && std::memcmp(bytes, "b3dm", 4) == 0;
    }

    void setTextureCache(GLTFReader::TextureCache* cache) { _textureCache = cache; }

    /**
     * Reads a b3dm that is already in memory. "location" is the tile's own
     * URI, used to resolve the embedded GLB's relative references. The memory
     * must stay valid for the duration of the call.
     */
    osgDB::ReaderWriter::ReadResult read(
        const std::string& location, const void* bytes, std::size_t size, const osgDB::Options* options) const
    {
        const unsigned char* data = static_cast<const unsigned char*>(bytes);
        if (!isB3DM(bytes, size) || size < 20)
            return fail(location, "missing or truncated b3dm header");

        const std::uint32_t version = readU32(data + 4);
        if (version != 1)
            return fail(location, "unsupported b3dm version " + std::to_string(version));

        // byteLength covers the whole tile; tolerate trailing bytes in the buffer.
        const std::size_t byteLength = std::min<std::size_t>(readU32(data + 8), size);

        std::size_t headerLength = 28;
        std::size_t featureTableJSONLength = 0, featureTableBinaryLength = 0;
        std::size_t batchTableJSONLength = 0, batchTableBinaryLength = 0;

        // Cesium's legacy detection: a GLB magic ("glTF" read as a uint32 is
        // at least 570425344) shows up where a modern header has its batch
        // table lengths, because the legacy header is shorter.
        const std::uint32_t legacyMarker = 570425344u;
        if (size >= 24 && readU32(data + 20) >= legacyMarker)
        {
            // magic, version, byteLength, batchLength, batchTableByteLength
            headerLength = 20;
            batchTableJSONLength = readU32(data + 16);
        }
        else if (size >= 28 && readU32(data + 24) >= legacyMarker)
        {
            // magic, version, byteLength, batchTableJsonByteLength,
            // batchTableBinaryByteLength, batchLength
            headerLength = 24;
            batchTableJSONLength = readU32(data + 12);
            batchTableBinaryLength = readU32(data + 16);
        }
        else if (size >= 28)
        {
            featureTableJSONLength = readU32(data + 12);
            featureTableBinaryLength = readU32(data + 16);
            batchTableJSONLength = readU32(data + 20);
            batchTableBinaryLength = readU32(data + 24);
        }
        else return fail(location, "truncated b3dm header");

        const std::size_t glbOffset = headerLength + featureTableJSONLength + featureTableBinaryLength +
            batchTableJSONLength + batchTableBinaryLength;
        if (glbOffset >= byteLength)
            return fail(location, "b3dm contains no glTF payload");

        osg::Vec3d rtcCenter;
        bool hasRTC = false;
        if (featureTableJSONLength > 0)
        {
            const std::string json(reinterpret_cast<const char*>(data + headerLength), featureTableJSONLength);
            osgEarth::Json::Reader reader;
            osgEarth::Json::Value doc;
            if (reader.parse(json, doc))
            {
                const osgEarth::Json::Value& rtc = doc["RTC_CENTER"];
                if (rtc.isArray() && rtc.size() == 3)
                {
                    rtcCenter.set(rtc[0u].asDouble(), rtc[1u].asDouble(), rtc[2u].asDouble());
                    hasRTC = rtcCenter != osg::Vec3d();
                }
            }
        }

        GLTFReader reader;
        reader.setTextureCache(_textureCache);
        osgDB::ReaderWriter::ReadResult rr = reader.read(location, data + glbOffset, byteLength - glbOffset, options);
        if (!rr.validNode() || !hasRTC)
            return rr;

        osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform(osg::Matrixd::translate(rtcCenter));
        xform->setName(osgDB::getSimpleFileName(location));
        xform->addChild(rr.getNode());
        return osgDB::ReaderWriter::ReadResult(xform.release());
    }

private:
    GLTFReader::TextureCache* _textureCache = nullptr;

    //! Reads a little-endian uint32 regardless of host byte order.
    static std::uint32_t readU32(const unsigned char* p)
    {
        return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) | (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
    }

    //! Logs the failure and returns a read error.
    static osgDB::ReaderWriter::ReadResult fail(const std::string& location, const std::string& message)
    {
        OE_WARN << LC << "Error loading " << location << ": " << message << std::endl;
        return osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE;
    }
};
