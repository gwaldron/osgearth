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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/Notify>

using namespace osgEarth;

/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/
#include <osgEarth/Notify>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cctype>
#include <iomanip>

using namespace std;

osg::NotifySeverity osgearth_g_NotifyLevel = osg::NOTICE;

void
osgEarth::setNotifyLevel(osg::NotifySeverity severity)
{
    osgEarth::initNotifyLevel();
    osgearth_g_NotifyLevel = severity;
}

osg::NotifySeverity
osgEarth::getNotifyLevel()
{
    osgEarth::initNotifyLevel();
    return osgearth_g_NotifyLevel;
}

bool
osgEarth::initNotifyLevel()
{
    static bool s_NotifyInit = false;

    if (s_NotifyInit) return true;
    
    // g_NotifyLevel
    // =============

    osgearth_g_NotifyLevel = osg::NOTICE; // Default value

    char* OSGNOTIFYLEVEL=getenv("OSGEARTH_NOTIFY_LEVEL");
    if (!OSGNOTIFYLEVEL) OSGNOTIFYLEVEL=getenv("OSGEARTHNOTIFYLEVEL");
    if(OSGNOTIFYLEVEL)
    {

        std::string stringOSGNOTIFYLEVEL(OSGNOTIFYLEVEL);

        // Convert to upper case
        for(std::string::iterator i=stringOSGNOTIFYLEVEL.begin();
            i!=stringOSGNOTIFYLEVEL.end();
            ++i)
        {
            *i=toupper(*i);
        }

        if(stringOSGNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          osgearth_g_NotifyLevel=osg::ALWAYS;
        else if(stringOSGNOTIFYLEVEL.find("FATAL")!=std::string::npos)      osgearth_g_NotifyLevel=osg::FATAL;
        else if(stringOSGNOTIFYLEVEL.find("WARN")!=std::string::npos)       osgearth_g_NotifyLevel=osg::WARN;
        else if(stringOSGNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     osgearth_g_NotifyLevel=osg::NOTICE;
        else if(stringOSGNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) osgearth_g_NotifyLevel=osg::DEBUG_INFO;
        else if(stringOSGNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   osgearth_g_NotifyLevel=osg::DEBUG_FP;
        else if(stringOSGNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      osgearth_g_NotifyLevel=osg::DEBUG_INFO;
        else if(stringOSGNOTIFYLEVEL.find("INFO")!=std::string::npos)       osgearth_g_NotifyLevel=osg::INFO;
        else std::cout << "Warning: invalid OSG_NOTIFY_LEVEL set ("<<stringOSGNOTIFYLEVEL<<")"<<std::endl;
 
    }

    s_NotifyInit = true;

    return true;

}

bool
osgEarth::isNotifyEnabled( osg::NotifySeverity severity )
{
    return severity<=getNotifyLevel();
}

class NullStreamBuffer : public std::streambuf
{
    private:
    
        virtual streamsize xsputn (const char_type*, streamsize n)
        {
            return n;
        }
};

struct NullStream : public std::ostream
{
    NullStream() :
         std::ostream(_nsb = new NullStreamBuffer) {}
        
    virtual ~NullStream()
    {
        delete rdbuf();
        rdbuf(0);
        //delete _nsb;
    }

    NullStreamBuffer* _nsb;
};

std::ostream&
osgEarth::notify(const osg::NotifySeverity severity)
{
    // set up global notify null stream for inline notify
    static NullStream s_NotifyNulStream;

    static bool initialized = false;
    if (!initialized) 
    {
        std::cerr<<""; // dummy op to force construction of cerr, before a reference is passed back to calling code.
        std::cout<<""; // dummy op to force construction of cout, before a reference is passed back to calling code.
        initialized = osgEarth::initNotifyLevel();
    }

    if (severity<=osgearth_g_NotifyLevel)
    {
        std::ostream* out = severity <= osg::WARN ? &std::cerr : &std::cout;
        (*out) << std::setprecision(8);
        return *out;
        //if (severity<=osg::WARN) return std::cerr;
        //else return std::cout;
    }
    return s_NotifyNulStream;
}
