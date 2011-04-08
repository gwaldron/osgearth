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

#include <osgEarthFeatures/Session>
#include <osgEarth/FileUtils>
#include <osgEarth/HTTPClient>

#define LC "[Session] "

using namespace osgEarth;
using namespace osgEarth::Features;

Session::Session( const Map* map ) :
_mapFrame( map )
{
    //nop
}

void
Session::setReferenceURI( const std::string& referenceURI )
{
    _referenceURI = referenceURI;
}

std::string
Session::resolveURI( const std::string& inputURI ) const
{
    return osgEarth::getFullPath( _referenceURI, inputURI );
}

osg::Node*
Session::getModel( const std::string& url ) const
{
    // expand the URL
    std::string absurl = resolveURI( url );

    // first, check the local repo
    {
        Threading::ScopedReadLock sharedLock( const_cast<Session*>(this)->_modelsMutex );
        ModelMap::const_iterator i = _models.find( absurl );
        if ( i != _models.end() )
            return i->second.get();
    }

    // next, try to load the model from its URL
    osg::ref_ptr<osg::Node> node;
    if ( HTTPClient::readNodeFile( absurl, node ) != HTTPClient::RESULT_OK )
    {
        OE_WARN << LC << "Failed to load model from \"" << url << "\"" << std::endl;
        return 0L;
    }

    // add it to the local model cache for next time
    {
        Session* ncthis = const_cast<Session*>(this);
        Threading::ScopedWriteLock exclusiveLock( ncthis->_modelsMutex );
        ncthis->_models[absurl] = node.get();
    }

    return node.release();
}
