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

#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/Notify>
#include <list>

using namespace osgEarth;

bool osgEarth::isRelativePath(const std::string& fileName)
{
    //If it is a URL, it is not relative
    if (osgDB::containsServerAddress( fileName ) ) return false;

    std::string native = osgDB::convertFileNameToNativeStyle(fileName);
#if defined(WIN32)  && !defined(__CYGWIN__)
    //Check to see if the path begins with a drive letter like "C:\data"
    if (native.size() >= 3 && native[1] == ':' && native[2] == '\\') return false;

    //Check to see if the path is network path like "\\server1\data"
    if (native.size() >= 2 && native[0] == '\\' && native[1] == '\\') return false;

    //The path must be relative
    return true;
#else
    //Absolute paths in Unix will start with a '/'
    return !(fileName.size() >= 1 && fileName[0] == '/');
#endif
}

std::string osgEarth::getFullPath(const std::string& relativeTo, const std::string &relativePath)
{
	if (!isRelativePath(relativePath) || relativeTo.empty())
    {
        //OE_NOTICE << relativePath << " is not a relative path " << std::endl;
        return relativePath;
    }

    //Note:  Modified from VPB

    //Concatinate the paths together
    std::string filename;
    if ( !osgDB::containsServerAddress( relativeTo ) )
        filename = osgDB::concatPaths( osgDB::getFilePath( osgDB::getRealPath( relativeTo )), relativePath);
    else
        filename = osgDB::concatPaths( osgDB::getFilePath( relativeTo ), relativePath);


    std::list<std::string> directories;
    int start = 0;
    for (unsigned int i = 0; i < filename.size(); ++i)
    {
        if (filename[i] == '\\' || filename[i] == '/')
        {
            //Get the current directory
            std::string dir = filename.substr(start, i-start);

            if (dir != "..")
            {
                if (dir != ".")
                {
                  directories.push_back(dir);
                }
            }
            else if (!directories.empty())
            {
                directories.pop_back();
            }
            start = i + 1;
        }
    }

    std::string path;
    for (std::list<std::string>::iterator itr = directories.begin();
         itr != directories.end();
         ++itr)
    {
        path += *itr;
        path += "/";
    }

    path += filename.substr(start, std::string::npos);

    //OE_NOTICE << "FullPath " << path << std::endl;
    return path;
}

bool osgEarth::isZipPath(const std::string &path)
{
    return (path.find(".zip") != std::string::npos);
}