/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <list>
#include <stdio.h>
#include <stdlib.h>

#ifdef OSGEARTH_HAVE_MVT
#include "vector_tile.pb.h"
#endif

using namespace osgEarth;
using namespace osgEarth::Features;

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
    std::string value;
    if ( !compressor->decompress(in, value) )
    {
        OE_WARN << "Decompression failed" << std::endl;
        return false;
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

                osg::ref_ptr< osgEarth::Symbology::Geometry > geometry;

                eGeomType geomType = static_cast<eGeomType>(feature.type());
                if (geomType == ::Polygon)
                {
                    geometry = new osgEarth::Symbology::Polygon();
                }
                else if (geomType == ::LineString)
                {
                    geometry = new osgEarth::Symbology::LineString();
                }
                else if (geomType == ::Point)
                {
                    geometry = new osgEarth::Symbology::PointSet();
                }
                else
                {
                    geometry = new osgEarth::Symbology::LineString();
                }

                osg::ref_ptr< Feature > oeFeature = new Feature(geometry, key.getProfile()->getSRS());
                features.push_back(oeFeature.get());

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


                unsigned int length = 0;
                int cmd = -1;
                const int cmd_bits = 3;

                unsigned int tileres = layer.extent();

                int x = 0;
                int y = 0;

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
                        else if (cmd == (SEG_CLOSE & ((1 << cmd_bits) - 1)))
                        {
                            geometry->push_back(geometry->front());
                        }
                    }
                }

                if (geometry->getType() == Geometry::TYPE_POLYGON)
                {
                    geometry->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CCW);
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
