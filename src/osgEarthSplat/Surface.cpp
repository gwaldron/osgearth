/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include "Surface"
#include "SplatCatalog"
#include "SplatShaders"
#include <osgEarth/Map>
#include <osgDB/Options>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[Surface] "

Surface::Surface()
{
    //nop
}

bool
Surface::configure(const ConfigOptions& conf, const Map* map, const osgDB::Options* dbo)
{
    SurfaceOptions in(conf);

    // Read in the catalog.
    _catalog = SplatCatalog::read( in.catalogURI().get(), dbo );
    if ( !_catalog.valid() )
    {
        OE_WARN << LC << "Failed to read catalog for surface\n";
        return false;
    }
    
    return true;
}

bool
Surface::loadTextures(const Coverage* coverage, const osgDB::Options* dbo)
{
    int numValidTextures = 0;

    if ( coverage == 0L || !_catalog.valid() )
        return false;

    if ( _catalog->createSplatTextureDef(dbo, _textureDef) )
    {
        // loaded, now create a sampling function.
        std::string code;
        if ( !createGLSLSamplingCode(coverage, code) )
        {
            OE_WARN << LC << "Failed to generate sampling code\n";
            return false;
        }

        _textureDef._samplingFunction = code;
    }
    else
    {
        OE_WARN << LC << "Failed to create a texture for a catalog (" << _catalog->name().get() << ")\n";
        return false;
    }

    return true;
}

namespace
{
#define INDENTATION 4
    struct indent {
        indent(int level) :_level(level){}
        int _level;
        friend std::ostream& operator<<(std::ostream& os, const indent& val) {
            for (int i=0; i<val._level * INDENTATION; ++i) 
                os << ' ';
            return os;
        }
    };

    void write(std::ostream& buf, SplatRangeDataVector::const_iterator& rangeData, int I)
    {
        buf << indent(I) << "primary = " << (rangeData->_textureIndex) << ".0;\n";
        if (rangeData->_detail.isSet()) {
            buf << indent(I) << "detail = " << (rangeData->_detail->_textureIndex) << ".0;\n";
            if (rangeData->_detail->_brightness.isSet())
                buf << indent(I) << "brightness = " << rangeData->_detail->_brightness.get() << ";\n";
            if (rangeData->_detail->_contrast.isSet())
                buf << indent(I) << "contrast = " << rangeData->_detail->_contrast.get() << ";\n";
            if (rangeData->_detail->_threshold.isSet())
                buf << indent(I) << "threshold = " << rangeData->_detail->_threshold.get() << ";\n";
            if (rangeData->_detail->_slope.isSet())
                buf << indent(I) << "slope = " << rangeData->_detail->_slope.get() << ";\n";
        }
    }
}

bool
Surface::createGLSLSamplingCode(const Coverage* coverage, std::string& output) const
{
    if ( !coverage )
    {
        OE_WARN << LC << "Sampling function: illegal state (no coverage or legend); \n";
        return false;
    }

    if ( !_textureDef._texture.valid() )
    {
        OE_WARN << LC << "Internal: texture is not set; cannot create a sampling function\n";
        return false;
    }

    std::string loside, hiside;

    // loside.
    {
        int I = 2;
        std::stringstream buf;

        unsigned pindex = 0;
        const SplatCoverageLegend::Predicates& preds = coverage->getLegend()->getPredicates();
        for(SplatCoverageLegend::Predicates::const_iterator p = preds.begin(); p != preds.end(); ++p, ++pindex)
        {
            const CoverageValuePredicate* pred = p->get();

            if ( pred->_exactValue.isSet() )
            {
                // Look up by class name:
                const std::string& className = pred->_mappedClassName.get();
                const SplatLUT::const_iterator i = _textureDef._splatLUT.find(className);
                if ( i != _textureDef._splatLUT.end() )
                {
                    if (pindex > 0)
                        buf << indent(I) << "else\n";

                    buf << indent(I) << "if (" << pred->_exactValue.get() << ".0 == value) {\n"; ++I;

                    const SplatRangeDataVector& rangeDataVector = i->second;

                    // single range data:
                    if (rangeDataVector.size() == 1)
                    {
                        write(buf, i->second.begin(), I);
                    }

                    // multiple ranges:
                    else
                    {
                        unsigned rangeExpressionIndex = 0;

                        // if the final range isn't 0, we need to plug it in as the default.
                        if (rangeDataVector.back()._minRange.get() > 0.0f)
                        {
                            write(buf, rangeDataVector.end()-1, I);
                        }

                        for (SplatRangeDataVector::const_iterator rangeData = rangeDataVector.begin();
                            rangeData !=rangeDataVector.end();
                            ++rangeData, ++rangeExpressionIndex)
                        {
                            if (rangeExpressionIndex > 0)
                                buf << indent(I) << "else\n";

                            buf << indent(I) << "if (env.range >= float(" << rangeData->_minRange.get() << ")) {\n"; ++I;
                            buf << indent(I) << "env.rangeLo = " << rangeData->_minRange.get() << ";\n";
                            
                            write(buf, rangeData, I);

                            --I; buf << indent(I) << "}\n"; 
                        }
                    }

                   --I; buf << indent(I) << "}\n";
                }
            }
        }

        loside = buf.str();
    }

    // hiside.
    {
        std::stringstream buf;

        int I = 2;
        unsigned pindex = 0;
        const SplatCoverageLegend::Predicates& preds = coverage->getLegend()->getPredicates();
        for(SplatCoverageLegend::Predicates::const_iterator p = preds.begin(); p != preds.end(); ++p, ++pindex)
        {
            const CoverageValuePredicate* pred = p->get();

            if ( pred->_exactValue.isSet() )
            {
                // Look up by class name:
                const std::string& className = pred->_mappedClassName.get();
                const SplatLUT::const_iterator i = _textureDef._splatLUT.find(className);
                if ( i != _textureDef._splatLUT.end() )
                {
                    if (pindex > 0)
                        buf << indent(I) << "else\n";

                    buf << indent(I) << "if (" << pred->_exactValue.get() << ".0 == value) {\n"; ++I;
                    
                    const SplatRangeDataVector& rangeDataVector = i->second;

                    // single range data:
                    if (rangeDataVector.size() == 1)
                    {
                        write(buf, i->second.begin(), I);
                    }

                    // multiple ranges:
                    else
                    {
                        unsigned rangeIndex = 0;
                        int nest = 0;
                        unsigned numRanges = i->second.size();

                        for (SplatRangeDataVector::const_iterator rangeData = i->second.begin();
                            rangeData != i->second.end();
                            ++rangeData, ++rangeIndex)
                        {
                            if (rangeIndex == 0)
                            {
                                // first one defines the high range.
                                buf << indent(I) << "env.rangeHi = " << rangeData->_minRange.get() << ";\n";
                            }
                            else
                            {
                                // not the first one
                                if (rangeIndex-1 < numRanges)
                                {
                                    // not the last one:
                                    buf << indent(I) << "if (env.range < float(" << rangeData->_minRange.get() << ")) {\n"; ++I;
                                    buf << indent(I) << "env.rangeHi = " << rangeData->_minRange.get() << ";\n";
                                    ++nest;
                                }
                                else break; // last one, we're done
                            }

                            write(buf, rangeData, I);
                        }

                        while (nest--) {
                            --I; buf << indent(I) << "}\n";
                        }
                    }

                   --I; buf << indent(I) << "}\n";
                }
            }
        }

        hiside = buf.str();
    }


    SplattingShaders splatting;
    std::string code = ShaderLoader::load(
        splatting.FragGetRenderInfo,
        splatting);


    osgEarth::replaceIn(code, "%LOSIDE_SAMPLING_FUNCTION%", loside);
    osgEarth::replaceIn(code, "%HISIDE_SAMPLING_FUNCTION%", hiside);

    output = code;

    OE_DEBUG << LC << "Sampling function = \n" << code << "\n\n";

    return true;
}
