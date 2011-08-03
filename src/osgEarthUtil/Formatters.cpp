/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthUtil/Formatters>
#include <iomanip>
#include <sstream>
#include <cstdio>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[LatLongFormatter] "

LatLongFormatter::LatLongFormatter(const AngularFormat& defaultFormat,
                                   unsigned             options ) :
_defaultFormat( defaultFormat ),
_options      ( options ),
_prec         ( 4 )
{
    if ( _defaultFormat == FORMAT_DEFAULT )
    {
        _defaultFormat = FORMAT_DEGREES_MINUTES_SECONDS;
    }
}

std::string
LatLongFormatter::format( const Angular& angle, const AngularFormat& format )
{
    std::stringstream buf;
    std::string result;
    std::string space = _options & USE_SPACES ? " " : "";

    AngularFormat f =
        format == FORMAT_DEFAULT ? _defaultFormat :
        format;

    if ( _prec > 0 )
        buf << std::setprecision(_prec);

    switch( f )
    {
    case FORMAT_DECIMAL_DEGREES:
        {
            if ( _options & USE_SYMBOLS )
                buf << angle.as(Units::DEGREES) << "°";
            else
                buf << angle.as(Units::DEGREES);
        }
        break;

    case FORMAT_DEGREES_DECIMAL_MINUTES:
        {
            double df = angle.as(Units::DEGREES);
            int    d  = (int)floor(df);
            double mf = 60.0*(df-(double)d);
            if ( mf == 60.0 ) {
                d += 1;
                mf = 0.0;
            }
            if ( _options & USE_SYMBOLS )
                buf << d << "\xb0" << space << mf << "'";
            else if ( _options & USE_COLONS )
                buf << d << ":" << mf;
            else
                buf << d << " " << mf;
        }
        break;

    case FORMAT_DEGREES_MINUTES_SECONDS:
        {
            double df = angle.as(Units::DEGREES);
            int    d  = (int)floor(df);
            double mf = 60.0*(df-(double)d);
            int    m  = (int)floor(mf);
            double sf = 60.0*(mf-(double)m);
            if ( sf == 60.0 ) {
                m += 1;
                sf = 0.0;
                if ( m == 60 ) {
                    d += 1;
                    m = 0;
                }
            }
            if ( _options & USE_SYMBOLS )
                buf << d << "\xb0" << space << m << "'" << space << sf << "\"";
            else if ( _options & USE_COLONS )
                buf << d << ":" << m << ":" << sf;
            else
                buf << d << " " << m << " " << sf;
        }
        break;
    }

    result = buf.str();
    return result;
}

bool
LatLongFormatter::parseAngle( const std::string& input, Angular& out_value )
{
    const char* c = input.c_str();

    double d=0.0, m=0.0, s=0.0;

    if (sscanf(c, "%lf:%lf:%lf",     &d, &m, &s) == 3 ||
        sscanf(c, "%lf\xb0%lf'%lf\"",   &d, &m, &s) == 3 ||
        sscanf(c, "%lf\xb0 %lf' %lf\"", &d, &m ,&s) == 3 ||
        sscanf(c, "%lfd %lf' %lf\"", &d, &m, &s) == 3 ||
        sscanf(c, "%lfd %lfm %lfs",  &d, &m, &s) == 3 ||
        sscanf(c, "%lf %lf' %lf\"",  &d, &m, &s) == 3 )
    {
        out_value.set( d + m/60.0 + s/3600.0, Units::DEGREES );
        return true;
    }
    else if (
        sscanf(c, "%lf:%lf",   &d, &m) == 2 ||
        sscanf(c, "%lf\xb0 %lf'", &d, &m) == 2 ||
        sscanf(c, "%lf\xb0%lf'",  &d, &m) == 2 ||
        sscanf(c, "%lfd %lf'", &d, &m) == 2 ||
        sscanf(c, "%lfd %lfm", &d, &m) == 2 ||
        sscanf(c, "%lfd%lf'",  &d, &m) == 2 ||
        sscanf(c, "%lf %lf'",  &d, &m) == 2 )
    {
        out_value.set( d + m/60.0, Units::DEGREES );
        return true;
    }
    else if (
        sscanf(c, "%lf\xb0", &d) == 1 ||
        sscanf(c, "%lfd", &d) == 1 ||
        sscanf(c, "%lf",  &d) == 1 )
    {
        out_value.set( d, Units::DEGREES );
        return true;
    }

    return false;
}

//------------------------------------------------------------------------

#undef LC
#define LC "[MGRSFormatter] "

namespace
{
    static char*    GZD_ALPHABET     = "CDEFGHJKLMNPQRSTUVWXX";    // 2 X's because X is a 12 degree high grid zone
    static char*    UTM_COL_ALPHABET = "ABCDEFGHJKLMNPQRSTUVWXYZ";
    static char*    UTM_ROW_ALPHABET = "ABCDEFGHJKLMNPQRSTUV";
    static unsigned UTM_ROW_ALPHABET_SIZE = 20;

    static char*    UPS_COL_ALPHABET = "ABCFGHJKLPQRSTUXYZ";        // omit I, O, D, E, M, N, V, W
    static char*    UPS_ROW_ALPHABET = "ABCDEFGHJKLMNPQRSTUVWXYZ";  // omit I, O
    static unsigned UPS_ROW_ALPHABET_SIZE = 24;
}

MGRSFormatter::MGRSFormatter( const SpatialReference* referenceSRS )
{
    if ( referenceSRS )
    {
        _refSRS = referenceSRS->getGeographicSRS();
        
        // use the "AA" lettering scheme for these older datum ellipsoids.
        std::string eName = _refSRS->getEllipsoid()->getName();
        _useAA = 
            eName.find("bessel") != std::string::npos ||
            eName.find("clark")  != std::string::npos ||
            eName.find("clrk")   != std::string::npos;
    }
    else
    {
        _refSRS = SpatialReference::create( "wgs84" );
        _useAA = false;
    }
}

std::string
MGRSFormatter::format( double latDeg, double lonDeg ) const
{
    unsigned    zone;
    char        gzd;
    unsigned    x=0, y=0;
    char        sqid[3];

    sqid[0] = '?';
    sqid[1] = '?';
    sqid[2] = 0;

    if ( latDeg >= 84.0 ) // north polar
    {
        zone = 0;
        gzd = lonDeg < 0.0 ? 'Y' : 'Z';

        osg::ref_ptr<const SpatialReference> ups = SpatialReference::create("+proj=ups +north");
        if ( !ups.valid() )
        {
            OE_WARN << LC << "Failed to create UPS-NORTH SRS" << std::endl;
            return "";
        }

        double upsX, upsY;
        if ( _refSRS->transform2D( lonDeg, latDeg, ups.get(), upsX, upsY ) == false )
        {
            OE_WARN << LC << "Failed to transform lat/long to UPS-NORTH" << std::endl;
            return "";
        }


    }

    else if ( latDeg <= -80.0 ) // south polar
    {
        zone = 0;
        gzd = lonDeg < 0.0 ? 'A' : 'B';
    }

    else // UTM
    {
        // figure out the grid zone designator
        unsigned gzdIndex = ((unsigned)(latDeg+80.0))/8;
        gzd = GZD_ALPHABET[gzdIndex];

        // figure out the UTM zone:
        zone = (unsigned)floor((lonDeg+180.0)/6.0);   // [0..59]
        bool north = latDeg >= 0.0;

        // convert the input coordinates to UTM:
        // yes, always use +north so we get Y relative to equator
        std::stringstream buf;
        buf << "+proj=utm +zone=" << (zone+1) << " +north +units=m";
        osg::ref_ptr<SpatialReference> utm = SpatialReference::create( buf.str() );

        double utmX, utmY;
        if ( _refSRS->transform2D( lonDeg, latDeg, utm.get(), utmX, utmY ) == false )
        {
            OE_WARN << LC << "Error transforming lat/long into UTM" << std::endl;
            return "";
        }

        // the alphabet set:
        unsigned set = zone % 6; // [0..5]

        // find the horizontal SQID offset (100KM increments) from the central meridian:
        unsigned xSetOffset = 8 * (set % 3);
        double xMeridianOffset = utmX - 500000.0;
        int sqMeridianOffset = xMeridianOffset >= 0.0 ? (int)floor(xMeridianOffset/100000.0) : -(int)floor(1.0-(xMeridianOffset/100000.0));
        unsigned indexOffset = (4 + sqMeridianOffset);
        sqid[0] = UTM_COL_ALPHABET[xSetOffset + indexOffset];
        double xWest = 500000.0 + (100000.0*(double)sqMeridianOffset);
        x = utmX - xWest;

        // find the vertical SQID offset (100KM increments) from the equator:
        unsigned ySetOffset = 5 * (set % 2);
        int sqEquatorOffset = (int)floor(utmY/100000.0);
        int absOffset = sqEquatorOffset + (10 * UTM_ROW_ALPHABET_SIZE);
        if ( !_useAA )
            absOffset += 10;
        sqid[1] = UTM_ROW_ALPHABET[absOffset % UTM_ROW_ALPHABET_SIZE];
        y = utmY - (100000.0*(double)sqEquatorOffset);
    }

    std::stringstream buf;
    buf << (zone+1) << gzd << " " << sqid << " " << x << " " << y;
    std::string result;
    result = buf.str();
    return result;
}
