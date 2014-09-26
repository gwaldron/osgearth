/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include <osgEarth/StringUtils>
#include <osgDB/FileNameUtils>
#include <cctype>

using namespace osgEarth;

StringTokenizer::StringTokenizer( const std::string& delims, const std::string& quotes ) :
_allowEmpties( true ),
_trimTokens  ( true )
{
    addDelims( delims );
    addQuotes( quotes );
}

StringTokenizer::StringTokenizer(const std::string& input, 
                                 StringVector&      output,
                                 const std::string& delims, 
                                 const std::string& quotes,
                                 bool               allowEmpties,
                                 bool               trimTokens ) :
_allowEmpties( allowEmpties ),
_trimTokens  ( trimTokens )
{
    addDelims( delims );
    addQuotes( quotes );
    tokenize( input, output );
}

void
StringTokenizer::addDelim( char delim, bool keep )
{
    _delims[delim] = keep;
}

void
StringTokenizer::addDelims( const std::string& delims, bool keep )
{
    for( unsigned i=0; i<delims.size(); ++i )
        addDelim( delims.at(i), keep );
}

void
StringTokenizer::addQuote( char quote, bool keep )
{
    _quotes[quote] = keep;
}

void
StringTokenizer::addQuotes( const std::string& quotes, bool keep )
{
    for( unsigned i=0; i<quotes.size(); ++i )
        addQuote( quotes.at(i), keep );
}

void
StringTokenizer::tokenize( const std::string& input, StringVector& output ) const
{
    output.clear();

    std::stringstream buf;
    bool quoted = false;
    char lastQuoteChar = '\0';

    for( std::string::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        char c = *i;    

        TokenMap::const_iterator q = _quotes.find( c );

        if ( quoted )
        {
            if( q != _quotes.end() && lastQuoteChar == c)
            {
                quoted = false;
                lastQuoteChar = '\0';
                if ( q->second )
                    buf << c;
            }
            else
            {
                buf << c;
            }
        }
        else
        {
            if ( q != _quotes.end() )
            {
                quoted = true;
                lastQuoteChar = c;
                if ( q->second )
                    buf << c;
            }
            else
            {
                TokenMap::const_iterator d = _delims.find( c );
                if ( d == _delims.end() )
                {
                    buf << c;
                }
                else
                {
                    std::string bufstr;
                    bufstr = buf.str();
                    std::string token = _trimTokens ? trim(bufstr) : bufstr;

                    if ( _allowEmpties || !token.empty() )
                        output.push_back( token );

                    if ( d->second == true )
                    {
                        output.push_back( std::string(1, c) );
                    }

                    buf.str("");
                }
            }
        }       
    }

    std::string bufstr;
    bufstr = buf.str();
    std::string last = _trimTokens ? trim(bufstr) : bufstr;
    if ( !last.empty() )
        output.push_back( last );
}

//--------------------------------------------------------------------------

const std::string osgEarth::EMPTY_STRING;

std::string
osgEarth::toLegalFileName( const std::string& input )
{
    //const std::string legal("ABCDEFGHIJKLMNOPQRSTUVQXYZabcdefghijklmnopqrstuvwxyz_./\\");
    static const std::string illegal("*:<>|\"\'?&");

    std::size_t pos = input.find("://");
    pos = pos == std::string::npos ? 0 : pos+3;

    std::stringstream buf;
    for( ; pos < input.size(); ++pos )
    {
        std::string::const_reference c = input.at(pos);
        if ( ::isprint(c) && !::isspace(c) && illegal.find(c) == std::string::npos )
            buf << c;
        else
            buf << "{" << std::hex << static_cast<unsigned>(c) << "}";
    }

    std::string result;
    result = buf.str();

    return result;
}

/** MurmurHash 2.0 (http://sites.google.com/site/murmurhash/) */
unsigned
osgEarth::hashString( const std::string& input )
{
    const unsigned int m = 0x5bd1e995;
    const int r = 24;
    unsigned int len = input.length();
    const char* data = input.c_str();
    unsigned int h = m ^ len; // using "m" as the seed.

    while(len >= 4)
    {
        unsigned int k = *(unsigned int *)data;
        k *= m; 
        k ^= k >> r; 
        k *= m;     		
        h *= m; 
        h ^= k;
        data += 4;
        len -= 4;
    }

    switch(len)
    {
    case 3: h ^= data[2] << 16;
    case 2: h ^= data[1] << 8;
    case 1: h ^= data[0];
        h *= m;
    };

    h ^= h >> 13;
    h *= m;
    h ^= h >> 15;

    return h;
}


/** Parses an HTML color ("#rrggbb" or "#rrggbbaa") into an OSG color. */
osg::Vec4f
osgEarth::htmlColorToVec4f( const std::string& html )
{
    std::string t = osgEarth::toLower(html);
    osg::Vec4ub c(0,0,0,255);
    if ( t.length() >= 7 ) {
        c.r() |= t[1]<='9' ? (t[1]-'0')<<4 : (10+(t[1]-'a'))<<4;
        c.r() |= t[2]<='9' ? (t[2]-'0')    : (10+(t[2]-'a'));
        c.g() |= t[3]<='9' ? (t[3]-'0')<<4 : (10+(t[3]-'a'))<<4;
        c.g() |= t[4]<='9' ? (t[4]-'0')    : (10+(t[4]-'a'));
        c.b() |= t[5]<='9' ? (t[5]-'0')<<4 : (10+(t[5]-'a'))<<4;
        c.b() |= t[6]<='9' ? (t[6]-'0')    : (10+(t[6]-'a'));
        if ( t.length() == 9 ) {
            c.a() = 0;
            c.a() |= t[7]<='9' ? (t[7]-'0')<<4 : (10+(t[7]-'a'))<<4;
            c.a() |= t[8]<='9' ? (t[8]-'0')    : (10+(t[8]-'a'));
        }
    }
    return osg::Vec4f( ((float)c.r())/255.0f, ((float)c.g())/255.0f, ((float)c.b())/255.0f, ((float)c.a())/255.0f );
}

/** Makes an HTML color ("#rrggbb" or "#rrggbbaa") from an OSG color. */
std::string
osgEarth::vec4fToHtmlColor( const osg::Vec4f& c )
{
    std::stringstream buf;
    buf << "#";
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(c.r()*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(c.g()*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(c.b()*255.0f);
    if ( c.a() < 1.0f )
        buf << std::hex << std::setw(2) << std::setfill('0') << (int)(c.a()*255.0f);
    std::string ssStr;
    ssStr = buf.str();
    return ssStr;
}

/** Parses a color string in the form "255 255 255 255" (r g b a [0..255]) into an OSG color. */
osg::Vec4ub
osgEarth::stringToColor(const std::string& str, osg::Vec4ub default_value)
{
    osg::Vec4ub color = default_value;
    std::istringstream strin(str);
    int r, g, b, a;
    if (strin >> r && strin >> g && strin >> b && strin >> a)
    {
        color.r() = (unsigned char)r;
        color.g() = (unsigned char)g;
        color.b() = (unsigned char)b;
        color.a() = (unsigned char)a;
    }
    return color;
}

/** Creates a string in the form "255 255 255 255" (r g b a [0..255]) from a color */
std::string
osgEarth::colorToString( const osg::Vec4ub& c )
{
    std::stringstream ss;
    ss << (int)c.r() << " " << (int)c.g() << " " << (int)c.b() << " " << (int)c.a();
    std::string ssStr;
    ssStr = ss.str();
    return ssStr;
}

/** Converts a string to a vec3f */
osg::Vec3f
osgEarth::stringToVec3f( const std::string& str, const osg::Vec3f& default_value )
{
    std::stringstream buf(str);
    osg::Vec3f out = default_value;
    buf >> out.x();
    if ( !buf.eof() ) {
        buf >> out.y() >> out.z();
    }
    else {
        out.y() = out.x();
        out.z() = out.x();
    }
    return out;
}

/** Converts a vec3f to a string */
std::string
osgEarth::vec3fToString( const osg::Vec3f& v )
{
    std::stringstream buf;
    buf << std::setprecision(6)
        << v.x() << " " << v.y() << " " << v.z()
        << std::endl;
    std::string result;
    result = buf.str();
    return result;
}


/** Replaces all the instances of "sub" with "other" in "s". */
std::string&
osgEarth::replaceIn( std::string& s, const std::string& sub, const std::string& other)
{
    if ( sub.empty() ) return s;
    size_t b=0;
    for( ; ; )
    {
        b = s.find( sub, b );
        if ( b == s.npos ) break;
        s.replace( b, sub.size(), other );
        b += other.size();
    }
    return s;
}

std::string&
osgEarth::ciReplaceIn( std::string& s, const std::string& pattern, const std::string& replacement )
{
    if ( pattern.empty() ) return s;
    
    std::string upperSource = s;
    std::transform( upperSource.begin(), upperSource.end(), upperSource.begin(), (int(*)(int))std::toupper );

    std::string upperPattern = pattern;
    std::transform( upperPattern.begin(), upperPattern.end(), upperPattern.begin(), (int(*)(int))std::toupper );

    for( size_t b = 0; ; )
    {
        b = upperSource.find( upperPattern, b );
        if ( b == s.npos ) break;
        s.replace( b, pattern.size(), replacement );
        upperSource.replace( b, upperPattern.size(), replacement );
        b += replacement.size();
    }

    return s;
}

/**
* Trims whitespace from the ends of a string.
* by Rodrigo C F Dias
* http://www.codeproject.com/KB/stl/stdstringtrim.aspx
*/
std::string 
osgEarth::trim( const std::string& in )
{
    std::string whitespace (" \t\f\v\n\r");
    std::string str = in;
    std::string::size_type pos = str.find_last_not_of( whitespace );
    if(pos != std::string::npos) {
        str.erase(pos + 1);
        pos = str.find_first_not_of( whitespace );
        if(pos != std::string::npos) str.erase(0, pos);
    }
    else str.erase(str.begin(), str.end());
    return str;
}


std::string
osgEarth::joinStrings( const StringVector& input, char delim )
{
    std::stringstream buf;
    for( StringVector::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        buf << *i;
        if ( (i+1) != input.end() ) buf << delim;
    }
    std::string result;
    result = buf.str();
    return result;
}

/** Returns a lower-case version of the input string. */
std::string
osgEarth::toLower( const std::string& input )
{
    std::string output = input;
    std::transform( output.begin(), output.end(), output.begin(), ::tolower );
    return output;
}

std::string
osgEarth::prettyPrintTime( double seconds )
{
    int hours = (int)floor(seconds / (3600.0) );
    seconds -= hours * 3600.0;

    int minutes = (int)floor(seconds/60.0);
    seconds -= minutes * 60.0;

    std::stringstream buf;
    buf << hours << ":" << minutes << ":" << seconds;
    return buf.str();
}

std::string
osgEarth::prettyPrintSize( double mb )
{
    std::stringstream buf;
    // Convert to terabytes
    if ( mb > 1024 * 1024 )
    {
        buf << (mb / (1024.0*1024.0)) << " TB";
    }
    else if (mb > 1024)
    {
        buf << (mb / 1024.0) << " GB";
    }
    else 
    {
        buf << mb << " MB";
    }
    return buf.str();
}


namespace
{
    template<typename charT>
    struct ci_equal {
        ci_equal( const std::locale& loc ) : _loc(loc) { }
        bool operator()(charT c1, charT c2) {
            return std::toupper(c1,_loc) == std::toupper(c2,_loc);
        }
        const std::locale& _loc;
    };
}

bool
osgEarth::ciEquals(const std::string& lhs, const std::string& rhs, const std::locale& loc )
{
    if ( lhs.length() != rhs.length() )
        return false;

    for( unsigned i=0; i<lhs.length(); ++i )
    {
        if ( std::toupper(lhs[i], loc) != std::toupper(rhs[i], loc) )
            return false;
    }

    return true;
}

bool
osgEarth::startsWith( const std::string& ref, const std::string& pattern, bool caseSensitive, const std::locale& loc )
{
    if ( pattern.length() > ref.length() )
        return false;

    if ( caseSensitive )
    {
        for( unsigned i=0; i<pattern.length(); ++i )
        {
            if ( ref[i] != pattern[i] )
                return false;
        }
    }
    else
    {
        for( unsigned i=0; i<pattern.length(); ++i )
        {
            if ( std::toupper(ref[i], loc) != std::toupper(pattern[i],loc) )
                return false;
        }
    }
    return true;
}

bool
osgEarth::endsWith( const std::string& ref, const std::string& pattern, bool caseSensitive, const std::locale& loc )
{
    if ( pattern.length() > ref.length() )
        return false;

    unsigned offset = ref.size()-pattern.length();
    if ( caseSensitive )
    {
        for( unsigned i=0; i < pattern.length(); ++i )
        {
            if ( ref[i+offset] != pattern[i] )
                return false;
        }
    }
    else
    {
        for( unsigned i=0; i < pattern.length(); ++i )
        {
            if ( std::toupper(ref[i+offset], loc) != std::toupper(pattern[i],loc) )
                return false;
        }
    }
    return true;
}
