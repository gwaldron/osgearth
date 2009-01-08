/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/FileCache>
#include <osgEarth/md5>
#include <osgEarth/HTTPClient>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

//Static HTTPClient used by all FileCache objects
static osgEarth::HTTPClient s_httpClient;
static OpenThreads::Mutex s_httpClient_mutex;

std::string md5hash(std::string filename)
{
    MD5 context;
    unsigned int len = strlen (filename.c_str());

    context.update   ((unsigned char*)filename.c_str(), len);
    context.finalize ();

    std::stringstream out;

    out << context;
    return out.str();
}


osgEarth::FileCache::FileCache(const std::string &_cache_path)
{
    image_extension = "";
    cache_path = _cache_path;
    _offline = false;
}

osg::Image* osgEarth::FileCache::readImageFile(const std::string& filename, const osgDB::ReaderWriter::Options* options )
{
    std::string cachedFilename = getCachedImageFilename(filename);

    //osg::notify(osg::NOTICE) << "\nCached Image File for " << filename << " is " << cachedFilename << std::endl;

    osg::ref_ptr<osg::Image> image;

    if (!cachedFilename.empty())
    {
        osg::notify(osg::INFO) << "Reading " << filename << " from cache" << std::endl;
        image = osgDB::readImageFile(cachedFilename, options);
    }

    if (!image.valid())
    {
        //Fail if we are in offline mode and the filename contains a server address
        if (_offline && osgDB::containsServerAddress(filename))
        {
            osg::notify(osg::INFO) << "FileCache in offline mode, skipping read of " << filename << std::endl;
            return 0;
        }

        if (osgDB::containsServerAddress(filename) && !cachedFilename.empty())
        {
            std::string path = osgDB::getFilePath(cachedFilename);
            //If the path doesn't currently exist or we can't create the path, don't cache the file
            if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
            {
                osg::notify(osg::WARN) << "Couldn't create path " << path << std::endl;
                cachedFilename.clear();
            }

            //Try to download the file
            if (!cachedFilename.empty())
            {
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_httpClient_mutex);
                if (s_httpClient.downloadFile( filename, cachedFilename) )
                {
                    //Load the downloaded file
                    image = osgDB::readImageFile(cachedFilename, options);
                }
                else
                {
                    osg::notify(osg::INFO) << "Could not download " << filename << std::endl;
                    return 0;
                }
            }
        }

        if (!image.valid())
        {
            //Just use osgDB to read the file
            image = osgDB::readImageFile(filename, options);
        }
    }

    return image.release();
}

osg::HeightField* osgEarth::FileCache::readHeightFieldFile(const std::string& filename, const osgDB::ReaderWriter::Options* options )
{
    //TODO:  Cache heightfields
    return osgDB::readHeightFieldFile(filename, options);
}

osg::Node* osgEarth::FileCache::readNodeFile(const std::string& filename, const osgDB::ReaderWriter::Options* options )
{
    //TODO:  Cache nodes
    return osgDB::readNodeFile(filename, options);
}

std::string osgEarth::FileCache::getCachedImageFilename(const std::string &filename)
{
    std::string cachedFileName = getCachedFilename(filename);
    if (!cachedFileName.empty() && !image_extension.empty())
    {
        cachedFileName = osgDB::getNameLessExtension(cachedFileName);
        std::stringstream buf;
        buf << cachedFileName << "." << image_extension;
        cachedFileName = buf.str();
    }
    return cachedFileName;
}

std::string osgEarth::FileCache::getCachedFilename(const std::string &filename)
{
    std::string cacheFileName;
    //We are only caching files from servers
    if (osgDB::containsServerAddress(filename) && !cache_path.empty())
    {
        //MD5 hash the URL to avoid very long URLS thats you can typically get with services like WMS
        cacheFileName = cache_path + "/" + md5hash(filename) + "." + osgDB::getFileExtension(filename);
    }
    return cacheFileName;
}
