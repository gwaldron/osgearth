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
#include <osgEarth/Notify>

#include <osg/ApplicationUsage>
#include <osg/ref_ptr>
#include <sstream>
#include <iostream>

#include <stdlib.h>

#define OSGEARTH_INIT_SINGLETON_PROXY(ProxyName, Func) static struct ProxyName{ ProxyName() { Func; } } s_##ProxyName;

namespace osgEarth
{

class NullStreamBuffer : public std::streambuf
{
private:
    std::streamsize xsputn(const std::streambuf::char_type * /*str*/, std::streamsize n)
    {
        return n;
    }
};

struct NullStream : public std::ostream
{
public:
    NullStream():
        std::ostream(new NullStreamBuffer)
    {
        _buffer = static_cast<NullStreamBuffer *>(rdbuf());
    }

    ~NullStream()
    {
        rdbuf(0);
        delete _buffer;
    }

protected:
    NullStreamBuffer* _buffer;
};

/** Stream buffer calling notify handler when buffer is synchronized (usually on std::endl).
 * Stream stores last notification severity to pass it to handler call.
 */
struct NotifyStreamBuffer : public std::stringbuf
{
    NotifyStreamBuffer() : _severity(osg::NOTICE)
    {
        /* reduce the need to reallocate the std::ostream buffer behind osgEarth::Notify (causing multitreading issues) by pre-allocating 4095 bytes */
        str(std::string(4095, 0));
        pubseekpos(0, std::ios_base::out);
    }

    void setNotifyHandler(osg::NotifyHandler *handler) { _handler = handler; }
    osg::NotifyHandler *getNotifyHandler() const { return _handler.get(); }

    /** Sets severity for next call of notify handler */
    void setCurrentSeverity(osg::NotifySeverity severity)
    {
        if (_severity != severity)
        {
            sync();
            _severity = severity;
        }
    }

    osg::NotifySeverity getCurrentSeverity() const { return _severity; }

private:

    int sync()
    {
        sputc(0); // string termination
        if (_handler.valid())
            _handler->notify(_severity, pbase());
        pubseekpos(0, std::ios_base::out); // or str(std::string())
        return 0;
    }

    osg::ref_ptr<osg::NotifyHandler> _handler;
    osg::NotifySeverity _severity;
};

struct NotifyStream : public std::ostream
{
public:
    NotifyStream():
        std::ostream(new NotifyStreamBuffer)
    {
        _buffer = static_cast<NotifyStreamBuffer *>(rdbuf());
    }

    void setCurrentSeverity(osg::NotifySeverity severity)
    {
        _buffer->setCurrentSeverity(severity);
    }

    osg::NotifySeverity getCurrentSeverity() const
    {
        return _buffer->getCurrentSeverity();
    }

    ~NotifyStream()
    {
        rdbuf(0);
        delete _buffer;
    }

protected:
    NotifyStreamBuffer* _buffer;
};

}

using namespace osgEarth;

static osg::ApplicationUsageProxy Notify_e0(osg::ApplicationUsage::ENVIRONMENTAL_VARIABLE, "OSGEARTH_NOTIFY_LEVEL <mode>", "FATAL | WARN | NOTICE | DEBUG_INFO | DEBUG_FP | DEBUG | INFO | ALWAYS");

struct NotifySingleton
{
    NotifySingleton()
    {
        // _notifyLevel
        // =============

        _notifyLevel = osg::NOTICE; // Default value

        char* OSGNOTIFYLEVEL=getenv("OSGEARTH_NOTIFY_LEVEL");
        if (!OSGNOTIFYLEVEL) OSGNOTIFYLEVEL=getenv("OSGEARTHNOTIFYLEVEL");
        if (OSGNOTIFYLEVEL)
        {

            std::string stringOSGNOTIFYLEVEL(OSGNOTIFYLEVEL);

            // Convert to upper case
            for(std::string::iterator i=stringOSGNOTIFYLEVEL.begin();
                i!=stringOSGNOTIFYLEVEL.end();
                ++i)
            {
                *i=toupper(*i);
            }

            if(stringOSGNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          _notifyLevel=osg::ALWAYS;
            else if(stringOSGNOTIFYLEVEL.find("FATAL")!=std::string::npos)      _notifyLevel=osg::FATAL;
            else if(stringOSGNOTIFYLEVEL.find("WARN")!=std::string::npos)       _notifyLevel=osg::WARN;
            else if(stringOSGNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     _notifyLevel=osg::NOTICE;
            else if(stringOSGNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) _notifyLevel=osg::DEBUG_INFO;
            else if(stringOSGNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   _notifyLevel=osg::DEBUG_FP;
            else if(stringOSGNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      _notifyLevel=osg::DEBUG_INFO;
            else if(stringOSGNOTIFYLEVEL.find("INFO")!=std::string::npos)       _notifyLevel=osg::INFO;
            else std::cout << "Warning: invalid OSGEARTH_NOTIFY_LEVEL set ("<<stringOSGNOTIFYLEVEL<<")"<<std::endl;

        }

        // Setup standard notify handler
        NotifyStreamBuffer *buffer = dynamic_cast<NotifyStreamBuffer *>(_notifyStream.rdbuf());
        if (buffer && !buffer->getNotifyHandler())
            buffer->setNotifyHandler(new osg::StandardNotifyHandler);
    }

    osg::NotifySeverity _notifyLevel;
    NullStream     _nullStream;
    NotifyStream   _notifyStream;
};

static NotifySingleton& getNotifySingleton()
{
    static NotifySingleton s_NotifySingleton;
    return s_NotifySingleton;
}

bool osgEarth::initNotifyLevel()
{
    getNotifySingleton();
    return true;
}

// Use a proxy to force the initialization of the NotifySingleton during static initialization
OSGEARTH_INIT_SINGLETON_PROXY(NotifySingletonProxy, osgEarth::initNotifyLevel())

void osgEarth::setNotifyLevel(osg::NotifySeverity severity)
{
    getNotifySingleton()._notifyLevel = severity;
}

osg::NotifySeverity osgEarth::getNotifyLevel()
{
    return getNotifySingleton()._notifyLevel;
}

void osgEarth::setNotifyHandler(osg::NotifyHandler *handler)
{
    NotifyStreamBuffer *buffer = static_cast<NotifyStreamBuffer*>(getNotifySingleton()._notifyStream.rdbuf());
    if (buffer) buffer->setNotifyHandler(handler);
}

osg::NotifyHandler* osgEarth::getNotifyHandler()
{
    NotifyStreamBuffer *buffer = static_cast<NotifyStreamBuffer *>(getNotifySingleton()._notifyStream.rdbuf());
    return buffer ? buffer->getNotifyHandler() : 0;
}


#ifndef OSGEARTH_NOTIFY_DISABLED
bool osgEarth::isNotifyEnabled( osg::NotifySeverity severity )
{
    return severity<=getNotifySingleton()._notifyLevel;
}
#endif

std::ostream& osgEarth::notify(const osg::NotifySeverity severity)
{
    if (osgEarth::isNotifyEnabled(severity))
    {
        getNotifySingleton()._notifyStream.setCurrentSeverity(severity);
        return getNotifySingleton()._notifyStream;
    }
    return getNotifySingleton()._nullStream;
}
