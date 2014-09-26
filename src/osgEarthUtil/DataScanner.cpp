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
#include <osgEarthUtil/DataScanner>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#define LC "[DataScanner] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;

namespace
{
    void traverse(const std::string&              path,
                  const std::vector<std::string>& extensions,
                  ImageLayerVector&               out_imageLayers)
    {
        if ( osgDB::fileType(path) == osgDB::DIRECTORY )
        {
            osgDB::DirectoryContents files = osgDB::getDirectoryContents(path);
            for( osgDB::DirectoryContents::const_iterator f = files.begin(); f != files.end(); ++f )
            {
                if ( f->compare(".") == 0 || f->compare("..") == 0 )
                    continue;

                std::string filepath = osgDB::concatPaths( path, *f );
                traverse( filepath, extensions, out_imageLayers );
            }
        }

        else if ( osgDB::fileType(path) == osgDB::REGULAR_FILE )
        {
            const std::string ext = osgDB::getLowerCaseFileExtension(path);

            if ( std::find(extensions.begin(), extensions.end(), ext) != extensions.end() )
            {
                GDALOptions gdal;
                gdal.url() = path;
                //gdal.interpolation() = INTERP_NEAREST;

                ImageLayerOptions options( path, gdal );
                options.cachePolicy() = CachePolicy::NO_CACHE;

                ImageLayer* layer = new ImageLayer(options);
                out_imageLayers.push_back( layer );
                OE_INFO << LC << "Found " << path << std::endl;
            }
        }
    }
}


void
DataScanner::findImageLayers(const std::string&              absRootPath,
                             const std::vector<std::string>& extensions,
                             ImageLayerVector&               out_imageLayers) const
{
    traverse( absRootPath, extensions, out_imageLayers );
}
