/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthFeatures/MVT>

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/GeoData>
#include <osgEarthFeatures/FeatureSource>
#include <osgDB/Registry>
#include <list>
#include <stdio.h>
#include <stdlib.h>

#ifdef OSGEARTH_HAVE_MVT
#include "vector_tile.pb.h"
#endif

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[MVT] "

#define CMD_BITS 3
#define CMD_MOVETO 1
#define CMD_LINETO 2
#define CMD_CLOSEPATH 7

// https://github.com/mapbox/mapnik-vector-tile/blob/master/examples/c%2B%2B/tileinfo.cpp
enum CommandType {
    SEG_END    = 0,
    SEG_MOVETO = 1,
    SEG_LINETO = 2,
    SEG_CLOSE = (0x40 | 0x0f)
};

enum eGeomType {
    Unknown = 0,
    Point = 1,
    LineString = 2,
    Polygon = 3
};

int zig_zag_decode(int n)
{
    return (n >> 1) ^ (-(n & 1));
}

#ifdef OSGEARTH_HAVE_MVT

Geometry* decodeLine(const mapnik::vector::tile_feature& feature, const TileKey& key, unsigned int tileres)
{
    unsigned int length = 0;
    int cmd = -1;
    const int cmd_bits = 3;

    int x = 0;
    int y = 0;

    std::vector< osg::ref_ptr< osgEarth::Symbology::LineString > > lines;
    osg::ref_ptr< osgEarth::Symbology::LineString > currentLine;

    for (int k = 0; k < feature.geometry_size();)
    {
        if (!length)
        {
            unsigned int cmd_length = feature.geometry(k++);
            cmd = cmd_length & ((1 << cmd_bits) - 1);
            length = cmd_length >> cmd_bits;
        }
        if (length > 0)
        {
            length--;

            if (cmd == SEG_MOVETO || cmd == SEG_LINETO)
            {
                if (cmd == SEG_MOVETO)
                {
                    currentLine = new osgEarth::Symbology::LineString;
                    lines.push_back( currentLine.get() );
                }
                int px = feature.geometry(k++);
                int py = feature.geometry(k++);
                px = zig_zag_decode(px);
                py = zig_zag_decode(py);

                x += px;
                y += py;

                double width = key.getExtent().width();
                double height = key.getExtent().height();

                double geoX = key.getExtent().xMin() + (width/(double)tileres) * (double)x;
                double geoY = key.getExtent().yMax() - (height/(double)tileres) * (double)y;

                if (currentLine.valid())
                {
                    currentLine->push_back(geoX, geoY, 0);
                }
            }
        }
    }

    currentLine = 0;

    if (lines.size() == 0)
    {
        return 0;
    }
    else if (lines.size() == 1)
    {
        // Just return a simple LineString
        return lines[0].release();
    }
    else
    {
        // Return a multilinestring
        MultiGeometry* multi = new MultiGeometry;
        for (unsigned int i = 0; i < lines.size(); i++)
        {
            multi->add(lines[i].get());
        }
        return multi;
    }
}

Geometry* decodePoint(const mapnik::vector::tile_feature& feature, const TileKey& key, unsigned int tileres)
{
    unsigned int length = 0;
    int cmd = -1;
    const int cmd_bits = 3;

    int x = 0;
    int y = 0;

    osgEarth::Symbology::PointSet *geometry = new osgEarth::Symbology::PointSet();

    for (int k = 0; k < feature.geometry_size();)
    {
        if (!length)
        {
            unsigned int cmd_length = feature.geometry(k++);
            cmd = cmd_length & ((1 << cmd_bits) - 1);
            length = cmd_length >> cmd_bits;
        }
        if (length > 0)
        {
            length--;
            if (cmd == SEG_MOVETO || cmd == SEG_LINETO)
            {
                int px = feature.geometry(k++);
                int py = feature.geometry(k++);
                px = zig_zag_decode(px);
                py = zig_zag_decode(py);

                x += px;
                y += py;

                double width = key.getExtent().width();
                double height = key.getExtent().height();

                double geoX = key.getExtent().xMin() + (width/(double)tileres) * (double)x;
                double geoY = key.getExtent().yMax() - (height/(double)tileres) * (double)y;
                geometry->push_back(geoX, geoY, 0);
            }
        }
    }

    return geometry;
}

Geometry* decodePolygon(const mapnik::vector::tile_feature& feature, const  TileKey& key, unsigned int tileres)
{
    /*
     https://github.com/mapbox/vector-tile-spec/tree/master/2.1
     Decoding polygons is a bit more difficult than lines or points.
     A Polygon geometry is either a single polygon or a multipolygon.  Each polygon has one exterior ring and zero or more interior rings.
     The rings are in sequence and you must check the orientation of the ring to know if it's an exterior ring (new polygon) or an
     interior ring (inner polygon of the current polygon).
     */


    unsigned int length = 0;
    int cmd = -1;
    const int cmd_bits = 3;

    int x = 0;
    int y = 0;

    // The list of polygons we've collected
    std::vector< osg::ref_ptr< osgEarth::Symbology::Polygon > > polygons;

    osg::ref_ptr< osgEarth::Symbology::Polygon > currentPolygon;

    osg::ref_ptr< osgEarth::Symbology::Ring > currentRing;

    for (int k = 0; k < feature.geometry_size();)
    {
        if (!length)
        {
            unsigned int cmd_length = feature.geometry(k++);
            cmd = cmd_length & ((1 << cmd_bits) - 1);
            length = cmd_length >> cmd_bits;
        }
        if (length > 0)
        {
            length--;
            if (cmd == SEG_MOVETO || cmd == SEG_LINETO)
            {
                if (!currentRing)
                {
                    currentRing = new osgEarth::Symbology::Ring();
                }

                int px = feature.geometry(k++);
                int py = feature.geometry(k++);
                px = zig_zag_decode(px);
                py = zig_zag_decode(py);

                x += px;
                y += py;

                double width = key.getExtent().width();
                double height = key.getExtent().height();

                double geoX = key.getExtent().xMin() + (width/(double)tileres) * (double)x;
                double geoY = key.getExtent().yMax() - (height/(double)tileres) * (double)y;
                currentRing->push_back(geoX, geoY, 0);
            }
            else if (cmd == (SEG_CLOSE & ((1 << cmd_bits) - 1)))
            {
                // The orientation is the opposite of what we want for features.  clockwise means exterior ring, counter clockwise means interior

                // Figure out what to do with the ring based on the orientation of the ring
                Geometry::Orientation orientation = currentRing->getOrientation();
                // Close the ring.
                currentRing->close();

                // Clockwise means exterior ring.  Start a new polygon and add the ring.
                if (orientation == Geometry::ORIENTATION_CW)
                {
                    // osgearth orientations are reversed from mvt
                    currentRing->rewind(Geometry::ORIENTATION_CCW);

                    currentPolygon = new osgEarth::Symbology::Polygon(&currentRing->asVector());
                    polygons.push_back(currentPolygon.get());
                }
                else if (orientation == Geometry::ORIENTATION_CCW)
                // Counter clockwise means a hole, add it to the existing polygon.
                {
                    if (currentPolygon.valid())
                    {
                        // osgearth orientations are reversed from mvt
                        currentRing->rewind(Geometry::ORIENTATION_CW);
                        currentPolygon->getHoles().push_back( currentRing );
                    }
                    else
                    {
                        // this means we encountered a "hole" without a parent outer ring,
                        // discard for now -gw
                        OE_INFO << LC << "Discarding improperly wound polygon (hole without an outer ring)\n";
                    }
                }

                // Start a new ring
                currentRing = 0;
            }
        }
    }

    currentRing = 0;
    currentPolygon = 0;

    if (polygons.size() == 0)
    {
        return 0;
    }
    else if (polygons.size() == 1)
    {
        // Just return a simple polygon
        return polygons[0].release();
    }
    else
    {
        // Return a multipolygon
        MultiGeometry* multi = new MultiGeometry;
        for (unsigned int i = 0; i < polygons.size(); i++)
        {
            multi->add(polygons[i].get());
        }
        return multi;
    }
}

#endif


bool
    MVT::read(std::istream& in, const TileKey& key, FeatureList& features)
{
    features.clear();

#ifdef OSGEARTH_HAVE_MVT

    // Get the compressor
    osg::ref_ptr< osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
    if (!compressor.valid())
    {
        return false;
    }

    // Decompress the tile
    std::string original((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.seekg (0, std::ios::beg);
    std::string value;
    if (!compressor->decompress(in, value))
    {
        value = original;
    }


    mapnik::vector::tile tile;

    if (tile.ParseFromString(value))
    {
        for (unsigned int i = 0; i < tile.layers().size(); i++)
        {
            const mapnik::vector::tile_layer &layer = tile.layers().Get(i);


            for (unsigned int j = 0; j < layer.features().size(); j++)
            {
                const mapnik::vector::tile_feature &feature = layer.features().Get(j);


                osg::ref_ptr< Feature > oeFeature = new Feature(0, key.getProfile()->getSRS());

                // Set the layer name as "mvt_layer" so we can filter it later
                oeFeature->set("mvt_layer", layer.name());

                // Read attributes
                for (unsigned int k = 0; k < feature.tags().size(); k+=2)
                {
                    std::string key = layer.keys().Get(feature.tags().Get(k));
                    mapnik::vector::tile_value value = layer.values().Get(feature.tags().Get(k+1));

                    if (value.has_bool_value())
                    {
                        oeFeature->set(key, value.bool_value());
                    }
                    else if (value.has_double_value())
                    {
                        oeFeature->set(key, value.double_value());
                    }
                    else if (value.has_float_value())
                    {
                        oeFeature->set(key, value.float_value());
                    }
                    else if (value.has_int_value())
                    {
                        oeFeature->set(key, (int)value.int_value());
                    }
                    else if (value.has_sint_value())
                    {
                        oeFeature->set(key, (int)value.sint_value());
                    }
                    else if (value.has_string_value())
                    {
                        oeFeature->set(key, value.string_value());
                    }
                    else if (value.has_uint_value())
                    {
                        oeFeature->set(key, (int)value.uint_value());
                    }

                    // Special path for getting heights from our test dataset.
                    if (key == "other_tags")
                    {
                        std::string other_tags = value.string_value();

                        StringTokenizer tok("=>");
                        StringVector tized;
                        tok.tokenize(other_tags, tized);
                        if (tized.size() == 3)
                        {
                            if (tized[0] == "height")
                            {
                                std::string value = tized[2];
                                // Remove quotes from the height
                                float height = as<float>(value, FLT_MAX);
                                if (height != FLT_MAX)
                                {
                                    oeFeature->set("height", height);
                                }
                            }
                        }
                    }
                }



                osg::ref_ptr< osgEarth::Symbology::Geometry > geometry;

                eGeomType geomType = static_cast<eGeomType>(feature.type());
                if (geomType == ::Polygon)
                {
                    geometry = decodePolygon(feature, key, layer.extent());
                }
                else if (geomType == ::LineString)
                {
                    geometry = decodeLine(feature, key, layer.extent());
                }
                else if (geomType == ::Point)
                {
                    geometry = decodePoint(feature, key, layer.extent());
                }
                else
                {
                    geometry = decodeLine(feature, key, layer.extent());
                }

                if (geometry)
                {
                    oeFeature->setGeometry( geometry.get() );
                    features.push_back(oeFeature.get());
                }
            }
        }
    }
    else
    {
        OE_WARN << "Failed to parse mvt" << key.str() << std::endl;
        return false;
    }

    return true;
#else
    OE_NOTICE << "Mapnik Vector Tiles NOT SUPPORTED - please compile osgEarth with protobuf to enable." << std::endl;
    return false;
#endif
}
