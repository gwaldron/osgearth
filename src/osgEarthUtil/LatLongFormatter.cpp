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
#include <osgEarthUtil/LatLongFormatter>

#include <stdio.h>

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
LatLongFormatter::format( const GeoPoint& p ) const
{
    GeoPoint geo = p;
    if ( !geo.makeGeographic() )
        return "";

    return Stringify()
        << format( Angular(geo.y()), true )
        << ", "
        << format( Angular(geo.x()), false );
}

std::string
LatLongFormatter::format( const Angular& angle, bool latitude, int precision, const AngularFormat& format ) const
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

   
       

    double df = angle.as(Units::DEGREES);
    while( df < -180. ) df += 360.;
    while( df >  180. ) df -= 360.;

    // Determine the label to use for suffixes or prefixes
    std::string label;
    if (latitude)
    {
        label = df < 0 ? "S":"N";
    }
    else
    {
        label = df < 0 ? "W":"E";
    }
  
    std::string prefix = "";
    std::string suffix = "";
    bool makePositive = false;
    if (_options & USE_PREFIXES)
    {
        prefix = label;
        makePositive = true;
    }
    else if (_options & USE_SUFFIXES)
    {
        suffix = label;
        makePositive = true;
    }

    // If we are using suffixes or prefixes then make the value positive.
    if (makePositive)
    {
        df = osg::absolute(df);
    }

    switch( f )
    {
    case FORMAT_DECIMAL_DEGREES:
        {
            if ( _options & USE_SYMBOLS )
                buf << prefix << df << "\xb0" << suffix;
            else
                buf << prefix << df << suffix;
        }
        break;

    case FORMAT_DEGREES_DECIMAL_MINUTES:
        {
            int    d  = (int)floor(df);
            double mf = 60.0*(df-(double)d);
            if ( mf == 60.0 ) {
                d += 1;
                mf = 0.0;
            }
            if ( _options & USE_SYMBOLS )
                buf << prefix << d << "\xb0" << space << mf << "'" << suffix;
            else if ( _options & USE_COLONS )
                buf << prefix << d << ":" << mf << suffix;
            else
                buf << prefix << d << " " << mf << suffix;
        }
        break;

    case FORMAT_DEGREES_MINUTES_SECONDS:
        {
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
            {
                buf << prefix << d << "\xb0" << space << m << "'" << space << sf << "\"" << suffix;
            }
            else if ( _options & USE_COLONS )
                buf << prefix << d << ":" << m << ":" << sf << suffix;
            else
                buf << prefix << d << " " << m << " " << sf << suffix;
        }
        break;
    case FORMAT_DEGREES_MINUTES_SECONDS_TERSE:
        {
            int    d  = (int)floor(df);
            double mf = 60.0*(df-(double)d);
            int    m  = (int)floor(mf);
            double sf = 60.0*(mf-(double)m);
            int s = osg::round(sf);

            if ( s == 60 ) {
                m += 1;
                s = 0;
                if ( m == 60 ) {
                    d += 1;
                    m = 0;
                }
            }

            

            if ( _options & USE_SYMBOLS )
            {
                buf << prefix << d << "\xb0";
                if (m != 0 || s != 0)
                {
                    buf << space << m << "'";
                    if (s != 0)
                    {
                         buf << space << s << "\"" << suffix;
                    }
                }
                buf << suffix;
                
            }
            else if ( _options & USE_COLONS )
            {
                buf << prefix << d;
                if (m != 0 || s != 0)
                {
                    buf << ":" << m;
                    if (s != 0)
                    {
                        buf << ":" << s;
                    }
                }
                buf << suffix;
            }
            else
            {
                buf << prefix << d;
                if (m != 0 || s != 0)
                {
                    buf << " " << m;
                    if (s != 0)
                    {
                        buf << " " << s;
                    }
                }
                buf << suffix;
            }
        }
        break;
		case FORMAT_DEFAULT:
		default: break;
    }

    result = buf.str();
    return result;
}

#define SIGN(x) (x>=0.0?1.0:-1.0)

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
        out_value.set( SIGN(d) * (fabs(d) + m/60.0 + s/3600.0), Units::DEGREES );
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
        out_value.set( SIGN(d) * (fabs(d) + m/60.0), Units::DEGREES );
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
