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

#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <OpenThreads/Atomic>

#include <sstream>
#include <iomanip>
#include <string.h>

#include "XYZOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[XYZ driver] "
#define OE_TEST OE_DEBUG


class XYZSource : public TileSource
{
public:
    XYZSource(const TileSourceOptions& options) : 
      TileSource(options), _options(options), _rotate_iter(0u)
    {
        //nop
    }


    Status initialize(const osgDB::Options* dbOptions)
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);        

        URI xyzURI = _options.url().value();
        if ( xyzURI.empty() )
        {
            return Status::Error( "Fail: driver requires a valid \"url\" property" );
        }

        // driver requires a profile.
        if ( !getProfile() )
        {
            return Status::Error( "An explicit profile definition is required by the XYZ driver." );
        }

        _template = xyzURI.full();
        
        _rotateStart = _template.find("[");
        _rotateEnd   = _template.find("]");
        if ( _rotateStart != std::string::npos && _rotateEnd != std::string::npos && _rotateEnd-_rotateStart > 1 )
        {
            _rotateString  = _template.substr(_rotateStart, _rotateEnd-_rotateStart+1);
            _rotateChoices = _template.substr(_rotateStart+1, _rotateEnd-_rotateStart-1);
        }

        _format = _options.format().isSet() 
            ? *_options.format()
            : osgDB::getLowerCaseFileExtension( xyzURI.base() );

        return STATUS_OK;
    }


    osg::Image* createImage(const TileKey&     key,
                            ProgressCallback*  progress )
    {
        unsigned x, y;
        key.getTileXY( x, y );

        if ( _options.invertY() == true )
        {
            unsigned cols=0, rows=0;
            key.getProfile()->getNumTiles( key.getLevelOfDetail(), cols, rows );
            y = rows - y - 1;
        }

        std::string location = _template;

        // support OpenLayers template style:
        replaceIn( location, "${x}", Stringify() << x );
        replaceIn( location, "${y}", Stringify() << y );
        replaceIn( location, "${z}", Stringify() << key.getLevelOfDetail() );

        // failing that, legacy osgearth style:
        replaceIn( location, "{x}", Stringify() << x );
        replaceIn( location, "{y}", Stringify() << y );
        replaceIn( location, "{z}", Stringify() << key.getLevelOfDetail() );

        std::string cacheKey;

        if ( !_rotateChoices.empty() )
        {
            cacheKey = location;
            unsigned index = (++_rotate_iter) % _rotateChoices.size();
            replaceIn( location, _rotateString, Stringify() << _rotateChoices[index] );
        }


        URI uri( location, _options.url()->context() );
        if ( !cacheKey.empty() )
            uri.setCacheKey( cacheKey );

        OE_TEST << LC << "URI: " << uri.full() << ", key: " << uri.cacheKey() << std::endl;

        return uri.getImage( _dbOptions.get(), progress );
    }

    virtual std::string getExtension() const 
    {
        return _format;
    }

private:
    const XYZOptions       _options;
    std::string            _format;
    std::string            _template;
    std::string            _rotateChoices;
    std::string            _rotateString;
    std::string::size_type _rotateStart, _rotateEnd;
    OpenThreads::Atomic    _rotate_iter;

    osg::ref_ptr<osgDB::Options> _dbOptions;
};




class XYZTileSourceDriver : public TileSourceDriver
{
public:
    XYZTileSourceDriver()
    {
        supportsExtension( "osgearth_xyz", "XYZ Driver" );
    }

    virtual const char* className()
    {
        return "XYZ Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new XYZSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_xyz, XYZTileSourceDriver)
