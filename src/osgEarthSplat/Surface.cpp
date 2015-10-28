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

#undef  IND
#define IND "    "

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

    std::stringstream
        weightBuf,
        primaryBuf,
        detailBuf,
        brightnessBuf,
        contrastBuf,
        thresholdBuf,
        slopeBuf;

    unsigned
        primaryCount    = 0,
        detailCount     = 0,
        brightnessCount = 0,
        contrastCount   = 0,
        thresholdCount  = 0,
        slopeCount      = 0;

    const SplatCoverageLegend::Predicates& preds = coverage->getLegend()->getPredicates();
    for(SplatCoverageLegend::Predicates::const_iterator p = preds.begin(); p != preds.end(); ++p)
    {
        const CoverageValuePredicate* pred = p->get();

        if ( pred->_exactValue.isSet() )
        {
            // Look up by class name:
            const std::string& className = pred->_mappedClassName.get();
            const SplatLUT::const_iterator i = _textureDef._splatLUT.find(className);
            if ( i != _textureDef._splatLUT.end() )
            {
                // found it; loop over the range selectors:
                int selectorCount = 0;
                const SplatSelectorVector& selectors = i->second;

                OE_DEBUG << LC << "Class " << className << " has " << selectors.size() << " selectors.\n";

                for(SplatSelectorVector::const_iterator selector = selectors.begin();
                    selector != selectors.end();
                    ++selector)
                {
                    const std::string&    expression = selector->first;
                    const SplatRangeData& rangeData  = selector->second;

                    std::string val = pred->_exactValue.get();

                    weightBuf
                        << IND "float w" << val
                        << " = (1.0-clamp(abs(value-" << val << ".0),0.0,1.0));\n";

                    // Primary texture index:
                    if ( primaryCount == 0 )
                        primaryBuf << IND "primary += ";
                    else
                        primaryBuf << " + ";

                    // the "+1" is because "primary" starts out at -1.
                    primaryBuf << "w"<<val << "*" << (rangeData._textureIndex + 1) << ".0";
                    primaryCount++;

                    // Detail texture index:
                    if ( rangeData._detail.isSet() )
                    {
                        if ( detailCount == 0 )
                            detailBuf << IND "detail += ";
                        else
                            detailBuf << " + ";
                        // the "+1" is because "detail" starts out at -1.
                        detailBuf << "w"<<val << "*" << (rangeData._detail->_textureIndex + 1) << ".0";
                        detailCount++;

                        if ( rangeData._detail->_brightness.isSet() )
                        {
                            if ( brightnessCount == 0 )
                                brightnessBuf << IND "brightness += ";
                            else
                                brightnessBuf << " + ";
                            brightnessBuf << "w"<<val << "*" << rangeData._detail->_brightness.get();
                            brightnessCount++;
                        }

                        if ( rangeData._detail->_contrast.isSet() )
                        {
                            if ( contrastCount == 0 )
                                contrastBuf << IND "contrast += ";
                            else
                                contrastBuf << " + ";
                            contrastBuf << "w"<<val << "*" << rangeData._detail->_contrast.get();
                            contrastCount++;
                        }

                        if ( rangeData._detail->_threshold.isSet() )
                        {
                            if ( thresholdCount == 0 )
                                thresholdBuf << IND "threshold += ";
                            else
                                thresholdBuf << " + ";
                            thresholdBuf << "w"<<val << "*" << rangeData._detail->_threshold.get();
                            thresholdCount++;
                        }

                        if ( rangeData._detail->_slope.isSet() )
                        {
                            if ( slopeCount == 0 )
                                slopeBuf << IND "slope += ";
                            else
                                slopeBuf << " + ";
                            slopeBuf << "w"<<val << "*" << rangeData._detail->_slope.get();
                            slopeCount++;
                        }
                    }                    
                }
            }
        }
    }

    if ( primaryCount > 0 )
        primaryBuf << ";\n";

    if ( detailCount > 0 )
        detailBuf << ";\n";

    if ( brightnessCount > 0 )
        brightnessBuf << ";\n";

    if ( contrastCount > 0 )
        contrastBuf << ";\n";

    if ( thresholdCount > 0 )
        thresholdBuf << ";\n";

    if ( slopeCount > 0 )
        slopeBuf << ";\n";

    SplattingShaders splatting;
    std::string code = ShaderLoader::load(
        splatting.FragGetRenderInfo,
        splatting);

    std::string codeToInject = Stringify()
        << IND
        << weightBuf.str()
        << primaryBuf.str()
        << detailBuf.str()
        << brightnessBuf.str()
        << contrastBuf.str()
        << thresholdBuf.str()
        << slopeBuf.str();

    osgEarth::replaceIn(code, "$COVERAGE_SAMPLING_FUNCTION", codeToInject);

    output = code;

    OE_DEBUG << LC << "Sampling function = \n" << code << "\n\n";

    return true;
}
