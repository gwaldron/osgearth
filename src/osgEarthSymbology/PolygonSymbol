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

#ifndef OSGEARTHSYMBOLOGY_GEOMETRY_SYMBOL_H
#define OSGEARTHSYMBOLOGY_GEOMETRY_SYMBOL_H 1

#include <osgEarthSymbology/Symbol>
#include <osgEarthSymbology/Fill>

namespace osgEarth { namespace Symbology
{
    /**
     * Symbol that describes how to render a polygonal geometry.
     */
    class OSGEARTHSYMBOLOGY_EXPORT PolygonSymbol : public Symbol
    {
    public:
        META_Object(osgEarthSymbology, PolygonSymbol);

        PolygonSymbol(const PolygonSymbol& rhs,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);
        PolygonSymbol( const Config& conf =Config() );

        /** dtor */
        virtual ~PolygonSymbol() { }

        /** Polygon fill properties. */
        optional<Fill>& fill() { return _fill; }
        const optional<Fill>& fill() const { return _fill; }

        /** Whether to use a LineStyle (if one exists in the Style) as an outline.
         * This defaults to true, but you set this to false to suppress outlining
         * even when a LineStyle exists. */
        optional<bool>& outline() { return _outline; }
        const optional<bool>& outline() const { return _outline; }

    public:
        virtual Config getConfig() const;
        virtual void mergeConfig(const Config& conf);
        static void parseSLD(const Config& c, class Style& style);

    protected:
        optional<Fill> _fill;
        optional<bool> _outline;
    };


} } // namespace osgEarth::Symbology

#endif // OSGEARTH_SYMBOLOGY_SYMBOL_H
