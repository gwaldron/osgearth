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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#ifndef OSGEARTHUTIL_EPHEMERIS
#define OSGEARTHUTIL_EPHEMERIS

#include <osgEarthUtil/Common>
#include <osgEarth/DateTime>
#include <osgEarth/Units>
#include <osgEarth/GeoData>
#include <osg/Vec3d>

namespace osgEarth { namespace Util 
{
    using namespace osgEarth;

    /**
     * Location of a celestial body relative to the Earth.
     */
    struct CelestialBody
    {
        Angle rightAscension;
        Angle declination;
        Angle latitude;
        Angle longitude;
        Distance altitude;
        osg::Vec3d geocentric;
        osg::Vec3d eci;
    };

    /**
     * An Ephemeris gives the positions of naturally occurring astronimical
     * objects; in that case, the sun and the moon. Also includes some
     * related utility functions.
     */
    class OSGEARTHUTIL_EXPORT Ephemeris : public osg::Referenced
    {
    public:

        //! Return the sun's position for a given date and time.
        virtual CelestialBody getSunPosition(const DateTime& dt) const;

        //! Return the moon's position for a given date and time.
        virtual CelestialBody getMoonPosition(const DateTime& dt) const;

        /**
         * Gets an ECEF position from the right ascension, declination and range
         *
         * @param ra    Right ascension in radians
         * @param decl  Declination in radians
         * @param range Range in meters
         */
        osg::Vec3d getECEFfromRADecl(double ra, double decl, double range) const;
    };
    
} } // namespace osgEarth::Util

#endif // OSGEARTHUTIL_EPHEMERIS
