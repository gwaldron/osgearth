/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthUtil/Environment>
#include <osgEarth/Registry>
#include <osgDB/ReadFile>
#include <osgEarthUtil/SkyNode>

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

#undef  LC
#define LC "[Environment] "

namespace
{
    // Astronomical Math
    // http://www.stjarnhimlen.se/comp/ppcomp.html

#define d2r(X) osg::DegreesToRadians(X)
#define r2d(X) osg::RadiansToDegrees(X)
#define nrad(X) { while( X > TWO_PI ) X -= TWO_PI; while( X < 0.0 ) X += TWO_PI; }
#define nrad2(X) { while( X <= -osg::PI ) X += TWO_PI; while( X > osg::PI ) X -= TWO_PI; }

    static const double TWO_PI = (2.0*osg::PI);
    static const double JD2000 = 2451545.0;

    
    osg::Vec3d getPositionFromRADecl(double ra, double decl, double range)
    {
        return osg::Vec3(0,range,0) * 
            osg::Matrix::rotate( decl, 1, 0, 0 ) * 
            osg::Matrix::rotate( ra - osg::PI_2, 0, 0, 1 );
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

    //double getTimeScale( int year, int month, int date, double hoursUT )
    //{
    //    int a = 367*year - 7 * ( year + (month+9)/12 ) / 4 + 275*month/9 + date - 730530;
    //    return (double)a + hoursUT/24.0;
    //}

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
        osg::Vec3d getPosition(int year, int month, int date, double hoursUTC ) const
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
            double zeta = d2r(23.4392); // zeta

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

#if 0
            OE_INFO
                << "sun lat = " << r2d(sun_lat) 
                << ", sun lon = " << r2d(sun_lon)
                << ", time delta_lon = " << r2d(diff_lon)
                << ", app sun lon = " << r2d(app_sun_lon)
                << std::endl;
#endif

            return osg::Vec3d(
                cos(sun_lat) * cos(-app_sun_lon),
                cos(sun_lat) * sin(-app_sun_lon),
                sin(sun_lat) );
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

        // From http://www.stjarnhimlen.se/comp/ppcomp.html
        osg::Vec3d getPosition(int year, int month, int date, double hoursUTC ) const
        {
            //double julianDate = getJulianDate( year, month, date );
            //julianDate += hoursUTC /24.0;
            double d = 367*year - 7 * ( year + (month+9)/12 ) / 4 + 275*month/9 + date - 730530;
            d += (hoursUTC / 24.0);                     

            double ecl = osg::DegreesToRadians(23.4393 - 3.563E-7 * d);

            double N = osg::DegreesToRadians(125.1228 - 0.0529538083 * d);
            double i = osg::DegreesToRadians(5.1454);
            double w = osg::DegreesToRadians(318.0634 + 0.1643573223 * d);
            double a = 60.2666;//  (Earth radii)
            double e = 0.054900;
            double M = osg::DegreesToRadians(115.3654 + 13.0649929509 * d);

            double E = M + e*(180.0/osg::PI) * sin(M) * ( 1.0 + e * cos(M) );
            
            double xv = a * ( cos(E) - e );
            double yv = a * ( sqrt(1.0 - e*e) * sin(E) );

            double v = atan2( yv, xv );
            double r = sqrt( xv*xv + yv*yv );

            //Compute the geocentric (Earth-centered) position of the moon in the ecliptic coordinate system
            double xh = r * ( cos(N) * cos(v+w) - sin(N) * sin(v+w) * cos(i) );
            double yh = r * ( sin(N) * cos(v+w) + cos(N) * sin(v+w) * cos(i) );
            double zh = r * ( sin(v+w) * sin(i) );

            // calculate the ecliptic latitude and longitude here
            double lonEcl = atan2 (yh, xh);
            double latEcl = atan2(zh, sqrt(xh*xh + yh*yh));

            double xg = r * cos(lonEcl) * cos(latEcl);
            double yg = r * sin(lonEcl) * cos(latEcl);
            double zg = r * sin(latEcl);

            double xe = xg;
            double ye = yg * cos(ecl) -zg * sin(ecl);
            double ze = yg * sin(ecl) +zg * cos(ecl);

            double RA    = atan2(ye, xe);
            double Dec = atan2(ze, sqrt(xe*xe + ye*ye));

            //Just use the average distance from the earth            
            double rg = 6378137.0 + 384400000.0;
            
            // finally, adjust for the time of day (rotation of the earth)
            double time_r = hoursUTC/24.0; // 0..1            
            double moon_r = RA/TWO_PI; // convert to 0..1

            // rotational difference between UTC and current time
            double diff_r = moon_r - time_r;
            double diff_lon = TWO_PI * diff_r;

            RA -= diff_lon;

            nrad2(RA);

            return getPositionFromRADecl( RA, Dec, rg );
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
    return sun.getPosition( date.year(), date.month(), date.day(), date.hours() );
}


osg::Vec3d
Ephemeris::getMoonPositionECEF(const DateTime& date) const
{
    Moon moon;
    return moon.getPosition( date.year(), date.month(), date.day(), date.hours() );
}


osg::Vec3d
Ephemeris::getECEFfromRADecl( double ra, double decl, double range )
{
    return getPositionFromRADecl(ra, decl, range);
}


//------------------------------------------------------------------------

EnvironmentOptions::EnvironmentOptions(const ConfigOptions& options) :
DriverConfigOptions( options )
{
    fromConfig(_conf);
}

void
EnvironmentOptions::fromConfig( const Config& conf )
{
    //nop
}

void
EnvironmentOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
EnvironmentOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    return conf;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[EnvironmentNode] "

EnvironmentNode::EnvironmentNode()
{
    _ephemeris = new Ephemeris();
    _dateTime = DateTime(2011, 06, 01, 0.0);
}

EnvironmentNode::~EnvironmentNode()
{
    //nop
}

void
EnvironmentNode::setEphemeris(Ephemeris* ephemeris)
{
    // cannot be null.
    _ephemeris = ephemeris ? ephemeris : new Ephemeris();
    onSetEphemeris();
}

Ephemeris*
EnvironmentNode::getEphemeris() const
{
    return _ephemeris.get();
}

void
EnvironmentNode::setDateTime(const DateTime& dt)
{
    _dateTime = dt;
    //OE_INFO << LC << "Time = " << dt.asRFC1123() << std::endl;
    onSetDateTime();
}

const DateTime&
EnvironmentNode::getDateTime() const
{
    return _dateTime;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[EnvironmentFactory] "
#define MAP_TAG                 "__osgEarth::Map"
#define ENVIRONMENT_OPTIONS_TAG "__osgEarth::Util::EnvironmentOptions"

EnvironmentNode*
EnvironmentFactory::create(const EnvironmentOptions& options,
                           const Map*                map)
{
    EnvironmentNode* result = 0L;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_environment_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( MAP_TAG, (void*)map );
        rwopts->setPluginData( ENVIRONMENT_OPTIONS_TAG, (void*)&options );

        result = dynamic_cast<EnvironmentNode*>( osgDB::readNodeFile( driverExt, rwopts.get() ) );
        if ( result )
        {
            OE_INFO << "Loaded environment driver: \"" << options.getDriver() << "\" OK." << std::endl;
        }
        else
        {
            OE_WARN << "FAIL, unable to load environment driver for \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    else
    {
        return new SkyNode(map);
        //OE_WARN << LC << "FAIL, illegal null driver specification" << std::endl;
    }

    return result;
}

//------------------------------------------------------------------------

const EnvironmentOptions&
EnvironmentDriver::getEnvironmentOptions( const osgDB::Options* options ) const
{
    return *static_cast<const EnvironmentOptions*>( options->getPluginData( ENVIRONMENT_OPTIONS_TAG ) );
}


const Map*
EnvironmentDriver::getMap( const osgDB::Options* options ) const
{
    return static_cast<const Map*>( options->getPluginData( MAP_TAG ) );
}
