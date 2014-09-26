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

#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/DateTime>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osg/Notify>
#include <stack>
#include <errno.h>

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

// currently this impl is for _all_ platforms, except as defined.
// the mac version will change soon to reflect the path scheme under osx, but
// for now, the above include is commented out, and the below code takes precedence.

#if defined(WIN32) && !defined(__CYGWIN__)
    #include <io.h>
    #define WINBASE_DECLARE_GET_MODULE_HANDLE_EX
    #include <windows.h>
    #include <winbase.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <direct.h> // for _mkdir

    #define mkdir(x,y) _mkdir((x))
    #define stat64 _stati64

    // set up for windows so acts just like unix access().
#ifndef F_OK
    #define F_OK 4
#endif

#else // unix

#if defined( __APPLE__ )
    // I'm not sure how we would handle this in raw Darwin
    // without the AvailablilityMacros.
    #include <AvailabilityMacros.h>

    //>OSG_IOS
    //IOS includes
    #include "TargetConditionals.h"

    #if (TARGET_OS_IPHONE)
        #include <Availability.h>
        // workaround a bug which appears when compiling for SDK < 4.0 and for the simulator
        #if defined(__IPHONE_4_0) && (__IPHONE_OS_VERSION_MIN_REQUIRED >= __IPHONE_4_0)
            #define stat64 stat
        #else
            #if !TARGET_IPHONE_SIMULATOR
                #define stat64 stat
            #endif
        #endif
    #endif
    //<OSG_IPHONE

    // 10.5 defines stat64 so we can't use this #define
    // By default, MAC_OS_X_VERSION_MAX_ALLOWED is set to the latest
    // system the headers know about. So I will use this as the control
    // variable. (MIN_ALLOWED is set low by default so it is
    // unhelpful in this case.)
    // Unfortunately, we can't use the label MAC_OS_X_VERSION_10_4
    // for older OS's like Jaguar, Panther since they are not defined,
    // so I am going to hardcode the number.
    #if (MAC_OS_X_VERSION_MAX_ALLOWED <= 1040)
        #define stat64 stat
    #endif
#elif defined(__CYGWIN__) || defined(__FreeBSD__) || (defined(__hpux) && !defined(_LARGEFILE64_SOURCE))
    #define stat64 stat
#endif

    #include <stdlib.h>
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
#endif

    // set up _S_ISDIR()
#if !defined(S_ISDIR)
#  if defined( _S_IFDIR) && !defined( __S_IFDIR)
#    define __S_IFDIR _S_IFDIR
#  endif
#  define S_ISDIR(mode)    (mode&__S_IFDIR)
#endif


#define LC "[FileUtils] "


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

bool
osgEarth::isPathToArchivedFile(const std::string& path)
{
    osgDB::Registry::ArchiveExtensionList list = osgDB::Registry::instance()->getArchiveExtensions();
    for( osgDB::Registry::ArchiveExtensionList::const_iterator i = list.begin(); i != list.end(); ++i )
    {
        if (path.find("."+*i+"/")  != std::string::npos ||
            path.find("."+*i+"\\") != std::string::npos)
        {
            return true;
        }
    }
    return false;
}

bool osgEarth::isZipPath(const std::string &path)
{
    OE_WARN << LC << "FileUtils::isZipPath is deprecated; use isPathToArchivedFile instead" << std::endl;
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

bool osgEarth::makeDirectory( const std::string &path )
{    
    if (path.empty())
    {
        OSG_DEBUG << "osgDB::makeDirectory(): cannot create an empty directory" << std::endl;
        return false;
    }

    struct stat64 stbuf;
#ifdef OSG_USE_UTF8_FILENAME
    if( _wstat64( OSGDB_STRING_TO_FILENAME(path).c_str(), &stbuf ) == 0 )
#else
    if( stat64( path.c_str(), &stbuf ) == 0 )
#endif
    {
        if( S_ISDIR(stbuf.st_mode))
            return true;
        else
        {
            OSG_DEBUG << "osgDB::makeDirectory(): "  <<
                    path << " already exists and is not a directory!" << std::endl;
            return false;
        }
    }

    std::string dir = path;
    std::stack<std::string> paths;
    while( true )
    {
        if( dir.empty() )
            break;

#ifdef OSG_USE_UTF8_FILENAME
        if( _wstat64( OSGDB_STRING_TO_FILENAME(dir).c_str(), &stbuf ) < 0 )
#else
        if( stat64( dir.c_str(), &stbuf ) < 0 )
#endif
        {
            switch( errno )
            {
                case ENOENT:
                case ENOTDIR:                    
                    paths.push( dir );
                    break;

                default:
                    OSG_DEBUG << "osgDB::makeDirectory(): "  << strerror(errno) << std::endl;
                    return false;
            }
        }
        dir = osgDB::getFilePath(std::string(dir));
    }

    while( !paths.empty() )
    {
        std::string dir = paths.top();

        #if defined(WIN32)
            //catch drive name
            if (dir.size() == 2 && dir.c_str()[1] == ':') {
                paths.pop();
                continue;
            }
        #endif

#ifdef OSG_USE_UTF8_FILENAME
        if ( _wmkdir(OSGDB_STRING_TO_FILENAME(dir).c_str())< 0 )
#else
        if( mkdir( dir.c_str(), 0755 )< 0 )
#endif
        {            
            if (osgDB::fileExists(dir))
            {
                OE_DEBUG << "Attempt to create directory that already exists " << dir << std::endl;
            }
            else            
            {
                OSG_DEBUG << "osgDB::makeDirectory(): "  << strerror(errno) << std::endl;
                return false;
            }
        }
        paths.pop();
    }
    return true;
}

bool osgEarth::makeDirectoryForFile( const std::string &path )
{
    return osgEarth::makeDirectory( osgDB::getFilePath( path ));
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

