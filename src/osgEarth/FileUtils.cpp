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

#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/DateTime>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osg/Notify>

#ifdef WIN32
#  include <windows.h>
#else
#  include <stdio.h>
#  include <stdlib.h>
#endif

#include <sys/types.h>

#ifdef WIN32
#  include <sys/utime.h>
#else
#  include <utime.h>
#endif

#include <sys/stat.h>
#include <time.h>

#include <list>
#include <sstream>


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

    //If they didn't specify a relative path, just return the relativeTo
    if (relativePath.empty()) return relativeTo;


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

bool
osgEarth::isArchive(const std::string& path)
{
    osgDB::Registry::ArchiveExtensionList list = osgDB::Registry::instance()->getArchiveExtensions();
    for( osgDB::Registry::ArchiveExtensionList::const_iterator i = list.begin(); i != list.end(); ++i )
    {
        if ( osgEarth::endsWith(path, ("."+*i), false) )
            return true;
    }
    return false;
}

bool osgEarth::isZipPath(const std::string &path)
{
    return (path.find(".zip") != std::string::npos);
}

std::string osgEarth::getTempPath()
{
#if defined(WIN32)  && !defined(__CYGWIN__)
    BOOL fSuccess  = FALSE;

    TCHAR lpTempPathBuffer[MAX_PATH];    

    //  Gets the temp path env string (no guarantee it's a valid path).
    DWORD dwRetVal = ::GetTempPath(MAX_PATH,          // length of the buffer
        lpTempPathBuffer); // buffer for path     

    if (dwRetVal > MAX_PATH || (dwRetVal == 0))
    {
        OE_NOTICE << "GetTempPath failed" << std::endl;
        return ".";
    }

    return std::string(lpTempPathBuffer);
#else
    return "/tmp/";
#endif
}

std::string osgEarth::getTempName(const std::string& prefix, const std::string& suffix)
{
    //tmpname is kind of busted on Windows, it always returns a file of the form \blah which gets put in your root directory but
    //oftentimes can't get opened by some drivers b/c it doesn't have a drive letter in front of it.
    bool valid = false;
    while (!valid)
    {
        std::stringstream ss;
        ss << prefix << "~" << rand() << suffix;
        if (!osgDB::fileExists(ss.str())) return ss.str();
    }
    return "";
}


bool
osgEarth::touchFile(const std::string& path)
{
    DateTime now;
    struct ::utimbuf ut;
    ut.actime = now.asTimeStamp();
    ut.modtime = now.asTimeStamp();
    return 0 == ::utime( path.c_str(), &ut );
}


TimeStamp
osgEarth::getLastModifiedTime(const std::string& path)
{
    struct stat buf;
    if ( stat(path.c_str(), &buf) == 0 )
        return buf.st_mtime;
    else
        return 0;
}


/**************************************************/
DirectoryVisitor::DirectoryVisitor()
{
}    

void DirectoryVisitor::handleFile( const std::string& filename )
{
}

bool DirectoryVisitor::handleDir( const std::string& path )
{
	return true;
}

void DirectoryVisitor::traverse(const std::string& path )
{
	if ( osgDB::fileType(path) == osgDB::DIRECTORY )
	{            
		if (handleDir( path ))
		{
			osgDB::DirectoryContents files = osgDB::getDirectoryContents(path);
			for( osgDB::DirectoryContents::const_iterator f = files.begin(); f != files.end(); ++f )
			{
				if ( f->compare(".") == 0 || f->compare("..") == 0 )
					continue;

				std::string filepath = osgDB::concatPaths( path, *f );
				traverse( filepath );                
			}
		}
	}
	else if ( osgDB::fileType(path) == osgDB::REGULAR_FILE )
	{
		handleFile( path );            
	}
}

/**************************************************/
CollectFilesVisitor::CollectFilesVisitor()  
{
}

void CollectFilesVisitor::handleFile( const std::string& filename )
{
	filenames.push_back( filename );        
}        