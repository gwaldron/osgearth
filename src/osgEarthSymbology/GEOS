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

#ifndef OSGEARTHSYMBOLOGY_GEOS_H
#define OSGEARTHSYMBOLOGY_GEOS_H 1

#ifdef OSGEARTH_HAVE_GEOS

#include <osgEarthFeatures/Common>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/Geometry>
#include <geos/version.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/GeometryFactory.h>

namespace osgEarth { namespace Symbology
{
    using namespace osgEarth;

    class GEOSContext
    {
    public:
        GEOSContext();
        ~GEOSContext();

    public:
        Symbology::Geometry* exportGeometry(const geos::geom::Geometry* input);

        geos::geom::Geometry* importGeometry( const Symbology::Geometry* input );

        void disposeGeometry(geos::geom::Geometry* input);

    protected:
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 7
        geos::geom::GeometryFactory::Ptr _factory;
#elif GEOS_VERSION_MAJOR == 3 && GEOS_VERSION_MINOR == 6
        geos::geom::GeometryFactory::unique_ptr _factory;
#else
        geos::geom::GeometryFactory* _factory;
#endif
    };

} } // namespace osgEarth::Features

#endif // OSGEARTH_HAVE_GEOS

#endif // OSGEARTHSYMBOLOGY_GEOS_H

