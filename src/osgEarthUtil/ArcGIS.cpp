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

#include <osgEarthUtil/ArcGIS>

#include <osgEarth/IOTypes>
#include <osgEarth/URI>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::ArcGIS;
using namespace std;

Service::Service( const std::string& _name, const std::string& _type):
name( _name ),
type( _type )
{
}


/***************************************************************************************/

RESTResponse::RESTResponse()
{
}

ServiceList& 
RESTResponse::getServices()
{
    return _services;
}

const ServiceList& 
RESTResponse::getServices() const
{
    return _services;
}

FolderList&
RESTResponse::getFolders()
{
    return _folders;
}

const FolderList&
RESTResponse::getFolders() const
{
    return _folders;
}


const std::string& RESTResponse::getCurrentVersion() const
{
    return _currentVersion;
}

void RESTResponse::setCurrentVersion( const std::string& currentVersion)
{
    _currentVersion = currentVersion;
}

const std::string& RESTResponse::getServiceURL() const
{
    return _serviceURL;
}

void RESTResponse::setServiceURL( const std::string& serviceURL )
{
    _serviceURL = serviceURL;
}

bool RESTResponse::getFolder(const std::string& folder, RESTResponse& response ) const
{
    std::string folderURL = _serviceURL + "/" + folder;
    return ServiceReader::read(folderURL, 0, response );
}

/***************************************************************************************/

ServiceReader::ServiceReader()
{
}

ServiceReader::ServiceReader(const ServiceReader &rhs)
{
}

ServiceReader::~ServiceReader()
{
}

bool 
ServiceReader::read( const std::string &location, const osgDB::ReaderWriter::Options* options, RESTResponse& response )
{
    response.setServiceURL( location );
    std::string serviceLocation = location + "?f=json&pretty=true";

    ReadResult r = URI(serviceLocation).readString();
    if ( r.failed() )
    {
        OE_WARN << "Failed to read ArcGIS Services tile map file from " << serviceLocation << std::endl;
        return false;
    }

    // Read tile map into a Config:
    Config conf;
    std::stringstream buf( r.getString() );
    if (!conf.fromJSON( buf.str() ))
    {
        return false;
    }

    return read( conf, response );    
}


bool 
ServiceReader::read( const Config& conf,  RESTResponse& response )
{
    response.getServices().clear();
    response.getFolders().clear();


    if (conf.hasChild("currentVersion"))
    {
        response.setCurrentVersion( conf.value("currentVersion") );
    }

    if (conf.hasChild("services"))
    {
        ConfigSet services = conf.child("services").children();
        for (ConfigSet::iterator itr = services.begin(); itr != services.end(); ++itr)
        {
            response.getServices().push_back( Service( itr->value("name"), itr->value("type") ) );            
        }
    }

    if (conf.hasChild("folders"))
    {
        ConfigSet folders = conf.child("folders").children();
        for (ConfigSet::iterator itr = folders.begin(); itr != folders.end(); ++itr)
        {
            response.getFolders().push_back( itr->value() );            
        }
    }

    return true;
}




