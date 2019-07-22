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

#ifndef OSGEARTHSYMBOLOGY_QUERY_H
#define OSGEARTHSYMBOLOGY_QUERY_H 1

#include <osgEarthSymbology/Common>
#include <osgEarth/GeoData>
#include <osgEarth/TileKey>

namespace osgEarth { namespace Symbology
{
    using namespace osgEarth;

    /**
     * A query filter that you can use to limit a set of symbology to process.
     */
    class OSGEARTHSYMBOLOGY_EXPORT Query
    {
    public:
        Query( const Config& conf =Config() );

        Query(const Query& rhs);

        virtual ~Query() { }

    public: // properties

        /** Sets the geospatial extent bounding this query. */
        optional<Bounds>& bounds() { return _bounds; }
        const optional<Bounds>& bounds() const { return _bounds; }

        /** Sets a driver-specific query expression. */
        optional<std::string>& expression() { return _expression; }
        const optional<std::string>& expression() const { return _expression; }

        /** Sets a driver-specific orderby expression. */
        optional<std::string>& orderby() { return _orderby; }
        const optional<std::string>& orderby() const { return _orderby; }

        /** Sets a driver-specific query expression. */
        optional<osgEarth::TileKey>& tileKey() { return _tileKey; }
        const optional<osgEarth::TileKey>& tileKey() const { return _tileKey; }

        /** The maximum number of features to be returned by this Query.  This is driver specific . */
        optional<int>& limit() { return _limit; }
        const optional<int>& limit() const { return _limit; }        

        /** Merges this query with another query, and returns the result */
        Query combineWith( const Query& other ) const;

    public: // configurable

        virtual Config getConfig() const;
        void mergeConfig( const Config& conf );

    protected:
        optional<Bounds> _bounds;
        optional<std::string> _expression;
        optional<std::string> _orderby;
        optional<osgEarth::TileKey> _tileKey;
        optional<int> _limit;
    };

} } // namespace osgEarth::Symbology

OSGEARTH_SPECIALIZE_CONFIG(osgEarth::Symbology::Query);

#endif // OSGEARTHSYMBOLOGY_QUERY_H
