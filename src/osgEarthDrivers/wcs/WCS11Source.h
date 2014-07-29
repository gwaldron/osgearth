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

#ifndef OSGEARTH_WCS_PLUGIN_WCS11SOURCE_H_
#define OSGEARTH_WCS_PLUGIN_WCS11SOURCE_H_ 1

#include <osgEarth/TileKey>
#include <osgEarth/TileSource>
#include <osgEarth/HTTPClient>
#include <osg/Image>
#include <osg/Shape>
#include <osgDB/ReaderWriter>
#include <string>
#include "WCSOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

class WCS11Source : public TileSource
{
public:
    WCS11Source( const TileSourceOptions& opt );

public: // TileSource interface

    Status initialize(const osgDB::Options* dbOptions);
    
    osg::Image* createImage( 
        const TileKey&        key,
        ProgressCallback*     progress );

    osg::HeightField* createHeightField(
        const TileKey&        key,
        ProgressCallback*     progress );
    
    std::string getExtension() const;

private:
    const WCSOptions _options;
    std::string _covFormat, _osgFormat;

    osg::ref_ptr<osgDB::Options> _dbOptions;

    HTTPRequest createRequest( const TileKey& key ) const;
};

#endif // OSGEARTH_WCS_PLUGIN_WCS11SOURCE_H_
