#include <osgEarth/FileCache>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>


osgEarth::FileCache::FileCache(const std::string &_cache_path)
{
    image_extension = "dds";
    cache_path = _cache_path;
    _offline = false;
}

osg::Image* osgEarth::FileCache::readImageFile(const std::string& filename, const osgDB::ReaderWriter::Options* options )
{
    std::string cachedFilename = getCachedImageFilename(filename);

    if (!cachedFilename.empty())
    {
        //If the cached file exists, return it
        if (osgDB::fileExists(cachedFilename))
        {
            osg::notify(osg::INFO) << "Reading " << filename << " from cache" << std::endl;
            return osgDB::readImageFile(cachedFilename, options);
        }
    }

    //Fail if we are in offline mode and the filename contains a server address
    if (_offline && osgDB::containsServerAddress(filename))
    {
        osg::notify(osg::INFO) << "FileCache in offline mode, skipping read of " << filename << std::endl;
        return 0;
    }

    //Load the file
    osg::Image *image = osgDB::readImageFile(filename, options);

    if (image && !cachedFilename.empty())
    {
        std::string path = osgDB::getFilePath(cachedFilename);
        //If the path doesn't currently exist or we can't create the path, don't cache the file
        if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
        {
            osg::notify(osg::WARN) << "Couldn't create path " << path << std::endl;
            cachedFilename.clear();
        }

        //Write the file to the cache
        if (!cachedFilename.empty())
        {
            osg::notify(osg::INFO) << "Writing " << filename << " to cache " << cachedFilename <<  std::endl;
            osgDB::writeImageFile(*image, cachedFilename, options);
        }
    }
    return image;
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
    if (!cachedFileName.empty())
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
    if (osgDB::containsServerAddress(filename))
    {
        if (!cache_path.empty())
        {
            cacheFileName = cache_path + "/" + 
                osgDB::getServerAddress(filename) + "/" + 
                osgDB::getServerFileName(filename);
        }
    }
    
    //Replace any ? with an _ because we can't have filenames with ?  
    std::replace(cacheFileName.begin(), cacheFileName.end(), '?', '_');


    //Remove any : that is not part of the drive specification
    for (unsigned int i = 0; i < cacheFileName.length(); ++i)
    {
        if ((cacheFileName[i] == ':') && (i != 1)) cacheFileName[i] = '_';
    }

    return cacheFileName;
}
