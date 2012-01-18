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
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[LatLongFormatter] "

LatLongFormatter::LatLongFormatter(const AngularFormat& defaultFormat,
                                   unsigned             options ) :
_defaultFormat( defaultFormat ),
_options      ( options ),
_prec         ( 5 )
{
    if ( _defaultFormat == FORMAT_DEFAULT )
    {
        _defaultFormat = FORMAT_DEGREES_MINUTES_SECONDS;
    }
}

std::string
LatLongFormatter::format( const osg::Vec3d& coords, const SpatialReference* srs ) const
{
    osg::Vec3d geo = coords;
    if ( srs && !srs->isGeographic() )
    {
        srs->transform( coords, srs->getGeographicSRS(), geo );
    }
    return Stringify()
        << format( Angular(geo.y()) )
        << ", "
        << format( Angular(geo.x()) );
}

std::string
LatLongFormatter::format( const Angular& angle, int precision, const AngularFormat& format ) const
{
    std::stringstream buf;
    std::string result;
    std::string space = _options & USE_SPACES ? " " : "";

    AngularFormat f =
        format == FORMAT_DEFAULT ? _defaultFormat :
        format;

    if ( precision < 0 )
        precision = _prec;

    if ( precision > 0 )
        buf << std::setprecision(precision);

    switch( f )
    {
    case FORMAT_DECIMAL_DEGREES:
        {
            if ( _options & USE_SYMBOLS )
                buf << angle.as(Units::DEGREES) << "\xb0";
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
    static unsigned UPS_COL_ALPHABET_SIZE = 18;
    static char*    UPS_ROW_ALPHABET = "ABCDEFGHJKLMNPQRSTUVWXYZ";  // omit I, O
    static unsigned UPS_ROW_ALPHABET_SIZE = 24;

    static std::string s_lateralZoneSpecs[] = {
        "+proj=utm +zone=1 +north +units=m",  "+proj=utm +zone=2 +north +units=m",
        "+proj=utm +zone=3 +north +units=m",  "+proj=utm +zone=4 +north +units=m",
        "+proj=utm +zone=5 +north +units=m",  "+proj=utm +zone=6 +north +units=m",
        "+proj=utm +zone=7 +north +units=m",  "+proj=utm +zone=8 +north +units=m",
        "+proj=utm +zone=9 +north +units=m",  "+proj=utm +zone=10 +north +units=m",
        "+proj=utm +zone=11 +north +units=m", "+proj=utm +zone=12 +north +units=m",
        "+proj=utm +zone=13 +north +units=m", "+proj=utm +zone=14 +north +units=m",
        "+proj=utm +zone=15 +north +units=m", "+proj=utm +zone=16 +north +units=m",
        "+proj=utm +zone=17 +north +units=m", "+proj=utm +zone=18 +north +units=m",
        "+proj=utm +zone=19 +north +units=m", "+proj=utm +zone=20 +north +units=m",
        "+proj=utm +zone=21 +north +units=m", "+proj=utm +zone=22 +north +units=m",
        "+proj=utm +zone=23 +north +units=m", "+proj=utm +zone=24 +north +units=m",
        "+proj=utm +zone=25 +north +units=m", "+proj=utm +zone=26 +north +units=m",
        "+proj=utm +zone=27 +north +units=m", "+proj=utm +zone=28 +north +units=m",
        "+proj=utm +zone=29 +north +units=m", "+proj=utm +zone=30 +north +units=m",
        "+proj=utm +zone=31 +north +units=m", "+proj=utm +zone=32 +north +units=m",
        "+proj=utm +zone=33 +north +units=m", "+proj=utm +zone=34 +north +units=m",
        "+proj=utm +zone=35 +north +units=m", "+proj=utm +zone=36 +north +units=m",
        "+proj=utm +zone=37 +north +units=m", "+proj=utm +zone=38 +north +units=m",
        "+proj=utm +zone=39 +north +units=m", "+proj=utm +zone=40 +north +units=m",
        "+proj=utm +zone=41 +north +units=m", "+proj=utm +zone=42 +north +units=m",
        "+proj=utm +zone=43 +north +units=m", "+proj=utm +zone=44 +north +units=m",
        "+proj=utm +zone=45 +north +units=m", "+proj=utm +zone=46 +north +units=m",
        "+proj=utm +zone=47 +north +units=m", "+proj=utm +zone=48 +north +units=m",
        "+proj=utm +zone=49 +north +units=m", "+proj=utm +zone=50 +north +units=m",
        "+proj=utm +zone=51 +north +units=m", "+proj=utm +zone=52 +north +units=m",
        "+proj=utm +zone=53 +north +units=m", "+proj=utm +zone=54 +north +units=m",
        "+proj=utm +zone=55 +north +units=m", "+proj=utm +zone=56 +north +units=m",
        "+proj=utm +zone=57 +north +units=m", "+proj=utm +zone=58 +north +units=m",
        "+proj=utm +zone=59 +north +units=m", "+proj=utm +zone=60 +north +units=m"
    };

    static std::string s_polarZoneSpecs[] = {
        "+proj=stere +lat_ts=90 +lat_0=90 +lon_0=0 +k_0=1 +x_0=0 +y_0=0",   // north
        "+proj=stere +lat_ts=-90 +lat_0=-90 +lon_0=0 +k_0=1 +x_0=0 +y_0=0"  // south
    };
}

MGRSFormatter::MGRSFormatter(Precision               precision,
                             const SpatialReference* referenceSRS,
                             unsigned                options ) :
_precision( precision ),
_options  ( options )
{
    if ( referenceSRS )
    {
        _refSRS = referenceSRS->getGeographicSRS();
    }
    else
    {
        _refSRS = SpatialReference::create( "wgs84" );
    }

    if ( options & FORCE_AA_SCHEME )
    {
        _useAL = false;
    }
    else if ( options & FORCE_AL_SCHEME )
    {
        _useAL = true;
    }
    else
    {
        // use the "AL" lettering scheme for these older datum ellipsoids.
        std::string eName = _refSRS->getEllipsoid()->getName();
        _useAL = 
            eName.find("bessel") != std::string::npos ||
            eName.find("clark")  != std::string::npos ||
            eName.find("clrk")   != std::string::npos;
    }
}

std::string
MGRSFormatter::format( const osg::Vec3d& coords, const SpatialReference* srs ) const
{
    osg::Vec3d geo = coords;
    if ( srs && !srs->isGeographic() )
    {
        srs->transform( coords, srs->getGeographicSRS(), geo );
    }
    return format( geo.y(), geo.x() );
}

bool
MGRSFormatter::transform( const osg::Vec3d& input, const SpatialReference* inputSRS, MGRSCoord& out ) const
{
    // convert to lat/long if necessary:
    osg::Vec3d inputGeo = input;
    if ( inputSRS && !inputSRS->isGeographic() )
        inputSRS->transform(input, inputSRS->getGeographicSRS(), inputGeo);

    unsigned    zone;
    char        gzd;
    unsigned    x=0, y=0;
    char        sqid[3];
    std::string space;

    if ( _options & USE_SPACES )
        space = " ";

    sqid[0] = '?';
    sqid[1] = '?';
    sqid[2] = 0;

    double latDeg = inputGeo.y();
    double lonDeg = inputGeo.x();

    if ( latDeg >= 84.0 || latDeg <= -80.0 ) // polar projection
    {
        bool isNorth = latDeg > 0.0;
        zone = 0;
        gzd = isNorth ? (lonDeg < 0.0 ? 'Y' : 'Z') : (lonDeg < 0.0? 'A' : 'B');

        osg::ref_ptr<const SpatialReference> ups =
            const_cast<MGRSFormatter*>(this)->_srsCache[ s_polarZoneSpecs[isNorth?0:1] ];
        if (!ups.valid())
            ups = SpatialReference::create( s_polarZoneSpecs[isNorth?0:1] );

        if ( !ups.valid() )
        {
            OE_WARN << LC << "Failed to create UPS SRS" << std::endl;
            return false;
        }

        double upsX, upsY;
        if ( _refSRS->transform2D( lonDeg, latDeg, ups.get(), upsX, upsY ) == false )
        {
            OE_WARN << LC << "Failed to transform lat/long to UPS" << std::endl;
            return false;
        }

        int sqXOffset = upsX >= 0.0 ? (int)floor(upsX/100000.0) : -(int)floor(1.0-(upsX/100000.0));
        int sqYOffset = upsY >= 0.0 ? (int)floor(upsY/100000.0) : -(int)floor(1.0-(upsY/100000.0));

        int alphaOffset = isNorth ? 7 : 12;

        sqid[0] = UPS_COL_ALPHABET[ (UPS_COL_ALPHABET_SIZE+sqXOffset) % UPS_COL_ALPHABET_SIZE ];
        sqid[1] = UPS_ROW_ALPHABET[alphaOffset + sqYOffset];

        x = upsX - (100000.0*(double)sqXOffset);
        y = upsY - (100000.0*(double)sqYOffset);
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

        // using an SRS cache speed things up a lot..
        osg::ref_ptr<const SpatialReference>& utm = 
            const_cast<MGRSFormatter*>(this)->_srsCache[s_lateralZoneSpecs[zone]];
        if ( !utm.valid() )
            utm = SpatialReference::create( s_lateralZoneSpecs[zone] );

        double utmX, utmY;
        if ( _refSRS->transform2D( lonDeg, latDeg, utm.get(), utmX, utmY ) == false )
        {
            OE_WARN << LC << "Error transforming lat/long into UTM" << std::endl;
            return false;
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
        unsigned ySetOffset = 5 * (zone % 2); //(set % 2);
        int sqEquatorOffset = (int)floor(utmY/100000.0);
        int absOffset = ySetOffset + sqEquatorOffset + (10 * UTM_ROW_ALPHABET_SIZE);
        if ( _useAL )
            absOffset += 10;
        sqid[1] = UTM_ROW_ALPHABET[absOffset % UTM_ROW_ALPHABET_SIZE];
        y = utmY - (100000.0*(double)sqEquatorOffset);
    }

    if ( (unsigned)_precision > PRECISION_1M )
    {
        x /= (unsigned)_precision;
        y /= (unsigned)_precision;
    }

    out.gzd  = Stringify() << (zone+1) << gzd;
    out.sqid = sqid;
    out.x    = x;
    out.y    = y;

    return true;
}

std::string
MGRSFormatter::format( double latDeg, double lonDeg ) const
{
    std::string space;

    if ( _options & USE_SPACES )
        space = " ";

    std::string result;

    MGRSCoord mgrs;
    if ( transform( osg::Vec3d(lonDeg, latDeg, 0), 0L, mgrs ) )
    {
        std::stringstream buf;
        buf << mgrs.gzd << space << mgrs.sqid;

        if ( (unsigned)_precision < PRECISION_100000M )
        {
            int sigdigs =
                _precision == PRECISION_10000M ? 1 :
                _precision == PRECISION_1000M  ? 2 :
                _precision == PRECISION_100M   ? 3 :
                _precision == PRECISION_10M    ? 4 :
                5;

            buf << space
                << std::setfill('0')
                << std::setw(sigdigs) << mgrs.x
                << space
                << std::setw(sigdigs) << mgrs.y;
        }

        result = buf.str();
    }

    return result;
}
