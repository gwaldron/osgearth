/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#ifndef OSGEARTHFEATURES_TEXT_SYMBOLIZER_H
#define OSGEARTHFEATURES_TEXT_SYMBOLIZER_H 1

#include <osgEarthFeatures/Common>
#include <osgEarthSymbology/TextSymbol>
#include <osgText/Text>
#include <osg/BoundingBox>

namespace osgEarth { namespace Features
{
    class Feature;
    class FilterContext;

    using namespace osgEarth::Symbology;

    /**
     * Applies the TextSymbol to a Text object.
     */
    class OSGEARTHFEATURES_EXPORT TextSymbolizer
    {
    public:
        //! Construct a new symbolizer
        TextSymbolizer(const TextSymbol* symbol);

        //! Translate the symbol's encoding to an OSG enum
        osgText::String::Encoding getEncoding() const;

        //! Apply this symbol to a text object
        void apply(osgText::Text* text) const;

        //! Apply this symbol to a text object, using the optional
        //! feature/context to evaluate expression fields. The optional
        //! bounding box can be used for alignment support.
        void apply(
            osgText::Text* text,
            Feature* feature,
            const FilterContext* context,
            const osg::BoundingBox* box) const;

    protected:
        osg::ref_ptr<const TextSymbol> _symbol;
    };

} } // namespace osgEarth::Features

#endif // OSGEARTHFEATURES_TEXT_SYMBOLIZER_H
