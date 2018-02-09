/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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
#include <osgEarthUtil/Ephemeris>
#include <osg/CoordinateSystemNode>
#include <sstream>

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

#undef  LC
#define LC "[Ephemeris] "

namespace
{
    // Astronomical Math
    // http://www.stjarnhimlen.se/comp/ppcomp.html

#define d2r(X) osg::DegreesToRadians(X)
#define r2d(X) osg::RadiansToDegrees(X)
#define nrad(X) { while( X >= TWO_PI ) X -= TWO_PI; while( X < 0.0 ) X += TWO_PI; }
#define nrad2(X) { while( X <= -osg::PI ) X += TWO_PI; while( X > osg::PI ) X -= TWO_PI; }

    static const double TWO_PI = (2.0*osg::PI);
    static const double JD2000 = 2451545.0;
    
    osg::Vec3d getPositionFromRADecl(double ra, double decl, double range)
    {
        // TODO: this should use EllipsoidMoidel instead
        return osg::Vec3d(
                range * cos(decl) * cos(ra),
                range * cos(decl) * sin(ra),
                range * sin(decl) );
    }

    // https://en.wikipedia.org/wiki/Julian_day#Converting_Gregorian_calendar_date_to_Julian_Day_Number
    double getJulianDate(int Y, int M, int D)
    {
        return (double)( (1461 * (Y + 4800 + (M - 14) / 12)) / 4 + (367 * (M - 2 - 12 * ((M - 14) / 12))) / 12 - (3 * ((Y + 4900 + (M-14)/12)/100))/4 + D - 32075 );
    }

    // http://www.stjarnhimlen.se/comp/tutorial.html#4
    double dayNumber(int Y, int M, int D, double hoursUTC)
    {
        int d = 367*Y - (7*(Y+(((M+9)/12))))/4 + ((275*M)/9) + D - 730530;
        return (double)d + hoursUTC/24.0;
    }

    double rev(double a)
    {
        return a - floor(a/360.0)*360.0;
    }

    struct Sun
    {
        // http://www.stjarnhimlen.se/comp/tutorial.html#5
        void getLatLon(int year, int month, int date, double hoursUTC,
                       double& out_lat,
                       double& out_lon) const
        {
            double d = dayNumber(year, month, date, hoursUTC);

            double w = 282.9404 + 4.70935E-5 * d;
            const double a = 1.0;

            double e = 0.016709 - 1.151E-9 *d;
            double M = 356.0470 + 0.9856002585 * d;
            double oblecl = 23.4393 - 3.563E-7 * d;
            double L = rev(w + rev(M));

            double E = rev(M + r2d(e*sin(d2r(M))*(1.0 + e * cos(d2r(M)))));
            double x = a*cos(d2r(E)) - e;
            double y = a*sin(d2r(rev(E)))*sqrt(1.0 - e*e);
            double r = sqrt(x*x + y*y);
            double v = r2d(atan2(y, x));
            double sunlon = rev(v + w);

            x = r*cos(d2r(sunlon));
            y = r*sin(d2r(sunlon));
            double z = 0;

            double xequat = x;
            double yequat = y*cos(d2r(oblecl)) + z*sin(d2r(oblecl));
            double zequat = y*sin(d2r(oblecl)) + z*cos(d2r(oblecl));

            double RA = rev(r2d(atan2(yequat, xequat)));
            double DECL = r2d(atan2(zequat, sqrt(xequat*xequat + yequat*yequat)));

            double GMST0 = (L + 180);
            double UT = d - floor(d);

            out_lat = d2r(DECL);
            out_lon = d2r(rev(0*180+RA-GMST0-UT*360));

            OE_DEBUG << "RA = " << RA << ", DECL = " << DECL << ", LAT = " << r2d(out_lat) << ", LON = " << r2d(out_lon) << std::endl;
        }

        void getECEF(int year, int month, int date, double hoursUTC, osg::Vec3d& out_ecef)
        {
            double lat, applon;
            getLatLon(year, month, date, hoursUTC, lat, applon);
            
            //TODO replace with a passed-in ellipsoid model and programmable distance-from-sun
            osg::EllipsoidModel em;           
            double distanceToSun = 146000000.0 - em.getRadiusEquator();
            double x, y, z;
            em.convertLatLongHeightToXYZ(lat, applon, distanceToSun, x, y, z);

            out_ecef.set(x, y, z);
        }
    };

    struct Moon
    {
        static std::string radiansToHoursMinutesSeconds(double ra)
        {
            while (ra < 0) ra += (osg::PI * 2.0);
            //Get the total number of hours
            double hours = (ra / (osg::PI * 2.0) ) * 24.0;
            double minutes = hours - (int)hours;
            hours -= minutes;
            minutes *= 60.0;
            double seconds = minutes - (int)minutes;
            seconds *= 60.0;
            std::stringstream buf;
            buf << (int)hours << ":" << (int)minutes << ":" << (int)seconds;
            return buf.str();
        }

        // Math: http://www.stjarnhimlen.se/comp/ppcomp.html
        // More: http://www.stjarnhimlen.se/comp/tutorial.html#7
        // Test: http://www.satellite-calculations.com/Satellite/suncalc.htm
        // Test: http://www.timeanddate.com/astronomy/moon/light.html
        osg::Vec3d getEarthLonLatRange(int year, int month, int date, double hoursUTC ) const
        {
            double d = dayNumber(year, month, date, hoursUTC);

            double N = d2r(125.1228 - 0.0529538083 * d);  nrad(N);
            double i = d2r(5.1454);                       
            double w = d2r(318.0634 + 0.1643573223 * d);  nrad(w);
            double a = 60.2666;//  (Earth radii)
            double e = 0.054900;
            double M = d2r(115.3654 + 13.0649929509 * d); nrad(M);

            double E = M + e * sin(M) * ( 1.0 + e * cos(M) );
            double E0 = E, E1 = 0.0;
            double epsilon = d2r(0.001);
            int count = 0;
            do {
                E1 = E0 - (E0 - e*sin(E0) - M) / (1.0 - e*cos(E0) );
                E = E1;
                std::swap(E0, E1);
                ++count;
            }
            while( fabs(E1-E0) > epsilon && count < 10 );
            
            //E = E - (E - e*sin(E) - M) / (1.0 - e*cos(E) );
            
            double xv = a * ( cos(E) - e );
            double yv = a * ( sqrt(1.0 - e*e) * sin(E) );

            double v = atan2( yv, xv );
            double r = sqrt( xv*xv + yv*yv );

            //Compute the geocentric (Earth-centered) position of the moon in the ecliptic coordinate system
            double xh = r * ( cos(N) * cos(v+w) - sin(N) * sin(v+w) * cos(i) );
            double yh = r * ( sin(N) * cos(v+w) + cos(N) * sin(v+w) * cos(i) );
            double zh = r * ( sin(v+w) * sin(i) );

            // calculate the ecliptic latitude and longitude here
            double lonEcl = atan2(yh, xh);
            double latEcl = atan2(zh, sqrt(xh*xh + yh*yh));

            //Just use the average distance from the earth
            double rg = 6378137.0 * a;

            // add in the perturbations.
            double Mm = M;
            double Ms = d2r(356.0470 + 0.9856002585 * d); //nrad(Ms); // mean anomaly of the sun
            double ws = d2r(282.9404 + 4.70935E-5   * d); //nrad(ws); // sun's longitude of perihelion
            double Ls = ws + Ms;    //nrad(Ls);
            double Lm = N + w + Mm; //nrad(Lm);
            double D = Lm - Ls;     //nrad(D);
            double F = Lm - N;      //nrad(F);

            lonEcl = lonEcl
                + d2r(-1.274) * sin(Mm - 2*D)    // (Evection)
                + d2r(+0.658) * sin(2*D)         // (Variation)
                + d2r(-0.186) * sin(Ms)          // (Yearly equation)
                + d2r(-0.059) * sin(2*Mm - 2*D)
                + d2r(-0.057) * sin(Mm - 2*D + Ms)
                + d2r(+0.053) * sin(Mm + 2*D)
                + d2r(+0.046) * sin(2*D - Ms)
                + d2r(+0.041) * sin(Mm - Ms)
                + d2r(-0.035) * sin(D)           // (Parallactic equation)
                + d2r(-0.031) * sin(Mm + Ms)
                + d2r(-0.015) * sin(2*F - 2*D)
                + d2r(+0.011) * sin(Mm - 4*D);

            latEcl = latEcl
                + d2r(-0.173) * sin(F - 2*D)
                + d2r(-0.055) * sin(Mm - F - 2*D)
                + d2r(-0.046) * sin(Mm + F - 2*D)
                + d2r(+0.033) * sin(F + 2*D)
                + d2r(+0.017) * sin(2*Mm + F);

            r = r +
                -0.58 * cos(Mm - 2*D)
                -0.46 * cos(2*D);

            // R is in "earth radii", so resolve to meters:
            //r *= 6378137.0;

            // convert to elliptic geocentric (unit)
            double xg = cos(lonEcl) * cos(latEcl);
            double yg = sin(lonEcl) * cos(latEcl);
            double zg = sin(latEcl);

            // and then to rectangular equatorial (unit)
            double ecl = d2r(23.4393 - 3.563E-7 * d); // obliquity of elliptic (tilt of earth)
            double xe = xg;
            double ye = yg*cos(ecl) - zg*sin(ecl);
            double ze = yg*sin(ecl) + zg*cos(ecl);

            // get the ra/decl:
            double RA   = atan2(ye, xe);
            double Decl = atan2(ze, sqrt(xe*xe + ye*ye));

            nrad(RA);
            double RAdeg  = r2d(RA);
            double Decdeg = r2d(Decl);
          
            // finally, adjust for the time of day (rotation of the earth).
            double UT = d - floor(d);
            double GMST0 = Ls + d2r(180.0);
            
            double earthLat = Decl;
            double earthLon = RA - GMST0 - d2r(UT*360);

            //OE_DEBUG << "RA = " << RA << ", DECL = " << DECL << ", LAT = " << r2d(out_lat) << ", LON = " << r2d(out_lon) << std::endl;

            return osg::Vec3d(earthLon, earthLat, r);
        }

        osg::Vec3d getECEF(int year, int month, int date, double hoursUTC) const
        {            
            osg::Vec3d LLA = getEarthLonLatRange(year, month, date, hoursUTC);
            
            osg::EllipsoidModel em;           
            double distanceToMoon = LLA[2] * em.getRadiusEquator() - em.getRadiusEquator();
            double x, y, z;
            em.convertLatLongHeightToXYZ(LLA[1], LLA[0], distanceToMoon, x, y, z);

            return osg::Vec3d(x, y, z);
        }
    };
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[Ephemeris] "

osg::Vec3d
Ephemeris::getSunPositionECEF(const DateTime& date) const
{
    Sun sun;
    osg::Vec3d ecef;
    sun.getECEF( date.year(), date.month(), date.day(), date.hours(), ecef );
    return ecef;
}

osg::Vec3d
Ephemeris::getMoonPositionECEF(const DateTime& date) const
{
    Moon moon;
    //osg::Vec3d rdr = moon.getRaDeclRange(date.year(), date.month(), date.day(), date.hours());
    //OE_NOTICE << "Moon: Y=" << date.year() << ", M=" << date.month() << ", D=" << date.day() << ", H=" << date.hours() << ": RA=" << osg::RadiansToDegrees(rdr.x()) << "; Decl=" << osg::RadiansToDegrees(rdr.y()) << "; Range=" << rdr.z() << std::endl;
    return moon.getECEF( date.year(), date.month(), date.day(), date.hours() );
}

osg::Vec3d
Ephemeris::getECEFfromRADecl( double ra, double decl, double range ) const
{
    return getPositionFromRADecl(ra, decl, range);
}
