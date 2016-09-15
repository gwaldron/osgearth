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
        return osg::Vec3d(
                range * cos(decl) * cos(ra),
                range * cos(decl) * sin(ra),
                range * sin(decl) );

        //return osg::Vec3(0,range,0) * 
        //    osg::Matrix::rotate( decl, 1, 0, 0 ) * 
        //    osg::Matrix::rotate( ra, 0, 0, 1 ); // - osg::PI_2, 0, 0, 1 );
    }


    double sgCalcEccAnom(double M, double e)
    {
        double eccAnom, E0, E1, diff;

        double epsilon = osg::DegreesToRadians(0.001);
        
        eccAnom = M + e * sin(M) * (1.0 + e * cos (M));
        // iterate to achieve a greater precision for larger eccentricities 
        if (e > 0.05)
        {
            E0 = eccAnom;
            do
            {
                 E1 = E0 - (E0 - e * sin(E0) - M) / (1 - e *cos(E0));
                 diff = fabs(E0 - E1);
                 E0 = E1;
            } while (diff > epsilon );
            return E0;
        }
        return eccAnom;
    }

    double getJulianDate( int year, int month, int date )
    {
        if ( month <= 2 )
        {
            month += 12;
            year -= 1;
        }

        int A = int(year/100);
        int B = 2-A+(A/4);
        int C = int(365.25*(year+4716));
        int D = int(30.6001*(month+1));
        return B + C + D + date - 1524.5;
    }

    struct Sun
    {
        // https://www.cfa.harvard.edu/~wsoon/JuanRamirez09-d/Chang09-OptimalTiltAngleforSolarCollector.pdf
        void getLatLonRaDecl(int year, int month, int date, double hoursUTC,
                             double& out_lat,
                             double& out_lon,
                             double& out_ra,
                             double& out_decl,
                             double& out_almanacTime) const
        {
            double JD = getJulianDate(year, month, date);
            double JD1 = (JD - JD2000);                         // julian time since JD2000 epoch
            double JC = JD1/36525.0;                            // julian century

            double mu = 282.937348 + 0.00004707624*JD1 + 0.0004569*(JC*JC);

            double epsilon = 280.466457 + 0.985647358*JD1 + 0.000304*(JC*JC);

            // orbit eccentricity:
            double E = 0.01670862 - 0.00004204 * JC;

            // mean anomaly of the perihelion
            double M = epsilon - mu;

            // perihelion anomaly:
            double v =
                M + 
                360.0*E*sin(d2r(M))/osg::PI + 
                900.0*(E*E)*sin(d2r(2*M))/4*osg::PI - 
                180.0*(E*E*E)*sin(d2r(M))/4.0*osg::PI;

            // longitude of the sun in ecliptic coordinates:
            double sun_lon = d2r(v - 360.0 + mu); // lambda
            nrad2(sun_lon);

            // angle between the ecliptic plane and the equatorial plane
            double zeta_deg = 23.4392;
            double zeta = d2r(zeta_deg);

            // latitude of the sun on the ecliptic plane:
            double omega = d2r(0.0);

            // latitude of the sun with respect to the equatorial plane (solar declination):
            double sun_lat = asin( sin(sun_lon)*sin(zeta) );
            nrad2(sun_lat);

            // finally, adjust for the time of day (rotation of the earth)
            double time_r = hoursUTC/24.0; // 0..1
            nrad(sun_lon); // clamp to 0..TWO_PI
            double sun_r = sun_lon/TWO_PI; // convert to 0..1

            // rotational difference between UTC and current time
            double diff_r = sun_r - time_r;
            double diff_lon = TWO_PI * diff_r;

            // apparent sun longitude.
            double app_sun_lon = sun_lon - diff_lon + osg::PI;
            nrad2(app_sun_lon);

            out_lat = sun_lat;
            out_lon = app_sun_lon;

            // right ascension and declination.
            double eclong = sun_lon;
            double oblqec = d2r(zeta_deg - 0.0000004*JD1);
            double num = cos(oblqec) * sin(eclong);
            double den = cos(eclong);
            out_ra = atan(num/den);
            if ( den < 0.0 ) out_ra += osg::PI;
            if ( den >= 0 && num < 0 ) out_ra += TWO_PI;
            out_decl = asin(sin(oblqec)*sin(eclong));

            // almanac time is the difference between the Julian Date and JD2000 epoch
            out_almanacTime = JD1;
        }

        void getECEF(int year, int month, int date, double hoursUTC, osg::Vec3d& out_ecef)
        {
            double lat, applon, ra, decl, almanacTime;
            getLatLonRaDecl(year, month, date, hoursUTC, lat, applon, ra, decl, almanacTime);
            out_ecef.set(
                cos(lat) * cos(-applon),
                cos(lat) * sin(-applon),
                sin(lat) );

            out_ecef *= 149600000;
        }

        void getLocalAzEl(int year, int month, int date, double hoursUTC, double lat, double lon, double& out_az, double out_el)
        {
            // UNTESTED!
            // http://stackoverflow.com/questions/257717/position-of-the-sun-given-time-of-day-and-lat-long 
            double sunLat, sunAppLon, ra, decl, almanacTime;
            getLatLonRaDecl(year, month, date, hoursUTC, sunLat, sunAppLon, ra, decl, almanacTime);
            // UTC sidereal time:
            double gmst = 6.697375 + .0657098242 * almanacTime + hoursUTC;
            gmst = fmod(gmst, 24.0);
            if ( gmst < 0.0 ) gmst += 24.0;
            // Local mean sidereal time:
            double lmst = gmst + r2d(lon)/15.0;
            lmst = fmod(lmst, 24.0);
            if ( lmst < 0.0 ) lmst += 24.0;
            lmst = d2r(lmst*15.0);
            // Hour angle:
            double ha = lmst - ra;
            nrad2(ha);
            // Az/el:
            out_el = asin(sin(decl)*sin(lat)+cos(decl)*cos(lat)*cos(ha));
            out_az = asin(-cos(decl)*sin(ha)/cos(out_el));
            double elc = asin(sin(decl)/sin(lat));
            if ( out_el >= elc ) out_az = osg::PI - out_az;
            if ( out_el <= elc && ha > 0.0 ) out_az += TWO_PI;
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
            int di = 367*year - 7 * ( year + (month+9)/12 ) / 4 + 275*month/9 + date - 730530;
            double d = (double)di;
            double time_r = hoursUTC/24.0; // 0..1            
            d += time_r;

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
                + d2r(+0.658) * sin(2*D)         //(Variation)
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
            r *= 6378137.0;

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
            double siteLon = 0.0, siteLat = 0.0;
            double siderealTime = GMST0 + d2r(UT*360.0) + siteLon;
            double hourAngle = siderealTime - RA;
            nrad2(hourAngle);

            double x = cos(hourAngle) * cos(Decl);
            double y = sin(hourAngle) * cos(Decl);
            double z = sin(Decl);
            
            double earthLat = atan2(z, sqrt(x*x + y*y) );
            
            // calculate the topographic right ascension and declination:
            double mpar  = asin(1.0/r);
            double gclat = siteLat - d2r(0.1924)*sin(2.0*siteLat);
            double rho   = 0.99833 + 0.00167*cos(2.0*siteLat);
            double g     = atan( tan(gclat) / cos(hourAngle) );
            double topRA   = RA - (mpar * rho * cos(gclat) * sin(hourAngle) / cos(Decl));
            double topDecl = Decl - (mpar * rho * sin(gclat) * sin(g - Decl) / sin(g));

            // use that to calculate the 
            double earthLon = topRA - GMST0 - d2r(UT*360.0);

            return osg::Vec3d(earthLon, earthLat, r);
        }

        osg::Vec3d getECEF(int year, int month, int date, double hoursUTC) const
        {            
            osg::Vec3d LLA = getEarthLonLatRange(year, month, date, hoursUTC);
            
            return osg::Vec3d(
                    LLA.z() * cos(LLA.y()) * cos(LLA.x()),
                    LLA.z() * cos(LLA.y()) * sin(LLA.x()),
                    LLA.z() * sin(LLA.y()) ); 
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
