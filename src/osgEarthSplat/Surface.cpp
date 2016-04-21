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
                // found it; loop over the range selectors:
                int selectorCount = 0;
                const SplatSelectorVector& selectors = i->second;

                OE_DEBUG << LC << "Class " << className << " has " << selectors.size() << " selectors.\n";

                if ( pindex > 0 )
                    buf << IND << "else\n";

                buf << IND << "if (" << pred->_exactValue.get() << ".0 == value) {\n";

                unsigned selectorIndex = 0;
                for(SplatSelectorVector::const_iterator selector = selectors.begin();
                    selector != selectors.end();
                    ++selector, ++selectorIndex)
                {
                    const std::string&    expression = selector->first;
                    const SplatRangeData& rangeData  = selector->second;

                    bool closeBracket = false;

                    if ( selectorIndex > 0 ) {
                        buf << IND IND << "else";
                        if ( expression.empty() ) {
                            buf << " {\n";
                            closeBracket = true;
                        }
                        else
                            buf << "\n";
                    }

                    if ( !expression.empty() )
                    {
                        buf << IND IND << "if (" << expression << ") {\n";
                        closeBracket = true;
                    }

                    std::string val = pred->_exactValue.get();

                    buf << IND IND IND << "primary    = " << (rangeData._textureIndex) << ".0;\n";
                    if ( rangeData._detail.isSet() ) {
                        buf << IND IND IND << "detail     = " << (rangeData._detail->_textureIndex) << ".0;\n";
                        if ( rangeData._detail->_brightness.isSet() )
                            buf << IND IND IND << "brightness = " << rangeData._detail->_brightness.get() << ";\n";
                        if ( rangeData._detail->_contrast.isSet() )
                            buf << IND IND IND << "contrast   = " << rangeData._detail->_contrast.get() << ";\n";
                        if ( rangeData._detail->_threshold.isSet() )
                            buf << IND IND IND << "threshold  = " << rangeData._detail->_threshold.get() << ";\n";
                        if ( rangeData._detail->_slope.isSet() )
                            buf << IND IND IND << "slope = " << rangeData._detail->_slope.get() << ";\n";
                    }

                    if ( closeBracket )
                    {
                        buf << IND IND << "}\n";
                    }
                }

                buf << IND << "}\n";
            }
        }
    }

    SplattingShaders splatting;
    std::string code = ShaderLoader::load(
        splatting.FragGetRenderInfo,
        splatting);

    std::string codeToInject = buf.str();

    osgEarth::replaceIn(code, "$COVERAGE_SAMPLING_FUNCTION", codeToInject);

    output = code;

    OE_DEBUG << LC << "Sampling function = \n" << code << "\n\n";

    return true;
}
