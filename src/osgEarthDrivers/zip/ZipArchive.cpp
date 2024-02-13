/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
*
* This library is open source and may be redistributed and/or modified under
* the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
* (at your option) any later version.  The full license is in LICENSE file
* included with this distribution, and on the openscenegraph.org website.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* OpenSceneGraph Public License for more details.
*/

/**
* This is a modified version of the ZipArchive class in the OpenSceneGraph repository that uses libzip
* instead of unzip.cpp to support zip files that are greater than 2GB in size.
*/

#include "ZipArchive.h"

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgEarth/Threading>

#include <sys/types.h>
#include <sys/stat.h>

#include <sstream>
#include <cstdio>

ZipArchive::ZipArchive()  :
_zipLoaded( false )
{
}

ZipArchive::~ZipArchive()
{
    close();
}

/** close the archive (on all threads) */
void ZipArchive::close()
{
    if ( _zipLoaded )
    {
        std::lock_guard<std::mutex> lock(_zipMutex);
        if ( _zipLoaded )
        {
            // close the file (on one thread since it's a shared file)
            const PerThreadData& data = getDataNoLock();

            zip_close(data._zipHandle);
            // clear out the file handles
            _perThreadData.clear();

            // clear out the index.
            _zipIndex.clear();

            _zipLoaded = false;
        }
    }
}

/** return true if file exists in archive.*/
bool ZipArchive::fileExists(const std::string& filename) const
{
    zip_uint64_t idx;
    return GetZipIndex(filename, idx);
}

/** Get the file name which represents the master file recorded in the Archive.*/
std::string ZipArchive::getMasterFileName() const
{
    return std::string();
}

std::string ZipArchive::getArchiveFileName() const
{
    std::string result;
    if( _zipLoaded )
    {
        result = _filename;
    }
    return result;
}

/** Get the full list of file names available in the archive.*/
bool ZipArchive::getFileNames(osgDB::Archive::FileNameList& fileNameList) const
{
    if(_zipLoaded)
    {
        ZipEntryMap::const_iterator iter = _zipIndex.begin();

        for(;iter != _zipIndex.end(); ++iter)
        {
            fileNameList.push_back((*iter).first);
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool ZipArchive::open(const std::string& file, ArchiveStatus /*status*/, const osgDB::ReaderWriter::Options* options)
{
    if ( !_zipLoaded )
    {
        // exclusive lock when we open for the first time:
        std::lock_guard<std::mutex> lock(_zipMutex);

        if ( !_zipLoaded ) // double-check avoids race condition
        {
            std::string ext = osgDB::getLowerCaseFileExtension(file);
            if (!acceptsExtension(ext)) return false;

            // save the filename + password so other threads can open the file
            _filename = osgDB::findDataFile( file, options );
            if (_filename.empty()) return false;

            _password = ReadPassword(options);

            // open the zip file in this thread:
            const PerThreadData& data = getDataNoLock();

            // establish a shared (read-only) index:
            if ( data._zipHandle != NULL )
            {
                IndexZipFiles( data._zipHandle );
                _zipLoaded = true;
            }
        }
    }

    return _zipLoaded;
}

osgDB::ReaderWriter::ReadResult ZipArchive::readObject(const std::string& file, const osgDB::ReaderWriter::Options* options) const
{
    osgDB::ReaderWriter::ReadResult rresult = osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::string ext = osgDB::getLowerCaseFileExtension(file);
    if (!_zipLoaded || !acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::stringstream buffer;

    osgDB::ReaderWriter* rw = ReadFromZipIndex(file, options, buffer);
    if (rw != NULL)
    {
        // Setup appropriate options
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = options ?
            static_cast<osgDB::ReaderWriter::Options*>(options->clone(osg::CopyOp::SHALLOW_COPY)) :
            new osgDB::ReaderWriter::Options;

        local_opt->setPluginStringData("STREAM_FILENAME", osgDB::getSimpleFileName(file));

        osgDB::ReaderWriter::ReadResult readResult = rw->readObject(buffer, local_opt.get());
        if (readResult.success())
        {
            return readResult;
        }
    }
    return rresult;
}

osgDB::ReaderWriter::ReadResult ZipArchive::readImage(const std::string& file,const osgDB::ReaderWriter::Options* options) const
{
    osgDB::ReaderWriter::ReadResult rresult = osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::string ext = osgDB::getLowerCaseFileExtension(file);
    if (!_zipLoaded || !acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::stringstream buffer;
    osgDB::ReaderWriter* rw = ReadFromZipIndex(file, options, buffer);
    if (rw != NULL)
    {
        // Setup appropriate options
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = options ?
            static_cast<osgDB::ReaderWriter::Options*>(options->clone(osg::CopyOp::SHALLOW_COPY)) :
            new osgDB::ReaderWriter::Options;

        local_opt->setPluginStringData("STREAM_FILENAME", osgDB::getSimpleFileName(file));

        osgDB::ReaderWriter::ReadResult readResult = rw->readImage(buffer, local_opt.get());
        if (readResult.success())
        {
            return readResult;
        }
    }
   return rresult;
}

osgDB::ReaderWriter::ReadResult ZipArchive::readHeightField(const std::string& file,const osgDB::ReaderWriter::Options* options) const
{
    osgDB::ReaderWriter::ReadResult rresult = osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::string ext = osgDB::getLowerCaseFileExtension(file);
    if (!_zipLoaded || !acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::stringstream buffer;

    osgDB::ReaderWriter* rw = ReadFromZipIndex(file, options, buffer);
    if (rw != NULL)
    {
        // Setup appropriate options
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = options ?
            static_cast<osgDB::ReaderWriter::Options*>(options->clone(osg::CopyOp::SHALLOW_COPY)) :
            new osgDB::ReaderWriter::Options;

        local_opt->setPluginStringData("STREAM_FILENAME", osgDB::getSimpleFileName(file));

        osgDB::ReaderWriter::ReadResult readResult = rw->readHeightField(buffer, local_opt.get());
        if (readResult.success())
        {
            return readResult;
        }
    }

    return rresult;
}

osgDB::ReaderWriter::ReadResult ZipArchive::readNode(const std::string& file,const osgDB::ReaderWriter::Options* options) const
{
    osgDB::ReaderWriter::ReadResult rresult = osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::string ext = osgDB::getLowerCaseFileExtension(file);
    if (!_zipLoaded || !acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::stringstream buffer;

    osgDB::ReaderWriter* rw = ReadFromZipIndex(file, options, buffer);
    if (rw != NULL)
    {
        // Setup appropriate options
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = options ?
            static_cast<osgDB::ReaderWriter::Options*>(options->clone(osg::CopyOp::SHALLOW_COPY)) :
            new osgDB::ReaderWriter::Options;

        local_opt->setPluginStringData("STREAM_FILENAME", osgDB::getSimpleFileName(file));

        osgDB::ReaderWriter::ReadResult readResult = rw->readNode(buffer, local_opt.get());
        if (readResult.success())
        {
            return readResult;
        }
    }

    return rresult;
}

osgDB::ReaderWriter::ReadResult ZipArchive::readScript(const std::string& file,const osgDB::ReaderWriter::Options* options) const
{
    osgDB::ReaderWriter::ReadResult rresult = osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::string ext = osgDB::getLowerCaseFileExtension(file);
    if (!_zipLoaded || !acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::stringstream buffer;

    osgDB::ReaderWriter* rw = ReadFromZipIndex(file, options, buffer);
    if (rw != NULL)
    {
        // Setup appropriate options
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = options ?
            static_cast<osgDB::ReaderWriter::Options*>(options->clone(osg::CopyOp::SHALLOW_COPY)) :
            new osgDB::ReaderWriter::Options;

        local_opt->setPluginStringData("STREAM_FILENAME", osgDB::getSimpleFileName(file));

        osgDB::ReaderWriter::ReadResult readResult = rw->readScript(buffer, local_opt.get());
        if (readResult.success())
        {
            return readResult;
        }
    }

    return rresult;
}

osgDB::ReaderWriter::ReadResult ZipArchive::readShader(const std::string& file,const osgDB::ReaderWriter::Options* options) const
{
    osgDB::ReaderWriter::ReadResult rresult = osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::string ext = osgDB::getLowerCaseFileExtension(file);
    if (!_zipLoaded || !acceptsExtension(ext)) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;

    std::stringstream buffer;

    osgDB::ReaderWriter* rw = ReadFromZipIndex(file, options, buffer);
    if (rw != NULL)
    {
        // Setup appropriate options
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = options ?
            static_cast<osgDB::ReaderWriter::Options*>(options->clone(osg::CopyOp::SHALLOW_COPY)) :
            new osgDB::ReaderWriter::Options;

        local_opt->setPluginStringData("STREAM_FILENAME", osgDB::getSimpleFileName(file));

        osgDB::ReaderWriter::ReadResult readResult = rw->readShader(buffer, local_opt.get());
        if (readResult.success())
        {
            return readResult;
        }
    }

    return rresult;
}

osgDB::ReaderWriter::WriteResult ZipArchive::writeObject(const osg::Object& /*obj*/,const std::string& /*fileName*/,const osgDB::ReaderWriter::Options*) const
{
    return osgDB::ReaderWriter::WriteResult(osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED);
}

osgDB::ReaderWriter::WriteResult ZipArchive::writeScript(const osg::Script& /*obj*/,const std::string& /*fileName*/,const osgDB::ReaderWriter::Options*) const
{
    return osgDB::ReaderWriter::WriteResult(osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED);
}

osgDB::ReaderWriter::WriteResult ZipArchive::writeImage(const osg::Image& /*image*/,const std::string& /*fileName*/,const osgDB::ReaderWriter::Options*) const
{
    return osgDB::ReaderWriter::WriteResult(osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED);
}

osgDB::ReaderWriter::WriteResult ZipArchive::writeHeightField(const osg::HeightField& /*heightField*/,const std::string& /*fileName*/,const osgDB::ReaderWriter::Options*) const
{
    return osgDB::ReaderWriter::WriteResult(osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED);
}

osgDB::ReaderWriter::WriteResult ZipArchive::writeNode(const osg::Node& /*node*/,const std::string& /*fileName*/,const osgDB::ReaderWriter::Options*) const
{
    return osgDB::ReaderWriter::WriteResult(osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED);
}

osgDB::ReaderWriter::WriteResult ZipArchive::writeShader(const osg::Shader& /*shader*/,const std::string& /*fileName*/,const osgDB::ReaderWriter::Options*) const
{
    return osgDB::ReaderWriter::WriteResult(osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED);
}

osgDB::ReaderWriter* ZipArchive::ReadFromZipIndex(const std::string& filename, const osgDB::ReaderWriter::Options* options, std::stringstream& streamIn) const
{
    zip_uint64_t idx;
    if (GetZipIndex(filename, idx))
    {
        // fetch the handle for the current thread:
        const PerThreadData& data = getData();
        if (data._zipHandle != NULL)
        {
            zip_file_t* zf;
            if ((zf = zip_fopen_index(data._zipHandle, idx, 0)) != NULL)
            {
                char buf[8192];
                zip_int64_t n;
                while ((n = zip_fread(zf, buf, sizeof(buf))) > 0) {
                    streamIn.write(buf, (size_t)n);
                }
                zip_fclose(zf);

                std::string file_ext = osgDB::getFileExtension(filename);
                osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(file_ext);
                if (rw != NULL)
                {
                    return rw;
                }

            }
        }
    }

    return NULL;
}


void CleanupFileString(std::string& strFileOrDir)
{
    if (strFileOrDir.empty())
    {
        return;
    }

    // convert all separators to unix-style for conformity
    for (unsigned int i = 0; i < strFileOrDir.length(); ++i)
    {
        if (strFileOrDir[i] == '\\')
        {
            strFileOrDir[i] = '/';
        }
    }

    // get rid of trailing separators
    if (strFileOrDir[strFileOrDir.length()-1] == '/')
    {
        strFileOrDir = strFileOrDir.substr(0, strFileOrDir.length()-1);
    }
}

void ZipArchive::IndexZipFiles(zip_t* zip)
{
    if (zip != NULL && !_zipLoaded)
    {
        zip_uint64_t  count = zip_get_num_entries(zip, 0);
        for (zip_uint64_t i = 0; i < count; i++)
        {
            std::string name(zip_get_name(zip, i, 0));
            CleanupFileString(name);
            if (!name.empty())
            {
                _zipIndex.insert(ZipEntryMapping(name, i));
            }
        }
    }
}

bool ZipArchive::GetZipIndex(const std::string& filename, zip_uint64_t& idx) const
{
    const ZipEntryMap::iterator iter = const_cast<ZipEntryMap*>(&_zipIndex)->find(filename);
    if (iter != _zipIndex.end())
    {
        idx = (*iter).second;
        return true;
    }
    return false;
}

osgDB::FileType ZipArchive::getFileType(const std::string& filename) const
{
    zip_uint64_t idx;
    if (GetZipIndex(filename, idx))
    {
        return osgDB::REGULAR_FILE;
    }

    return osgDB::FILE_NOT_FOUND;
}

osgDB::DirectoryContents ZipArchive::getDirectoryContents(const std::string& dirName) const
{
    osgDB::DirectoryContents dirContents;

    ZipEntryMap::const_iterator iter = _zipIndex.begin();
    ZipEntryMap::const_iterator iterEnd = _zipIndex.end();

    for(; iter != iterEnd; ++iter)
    {
        std::string searchPath = dirName;
        CleanupFileString(searchPath);

        if(iter->first.size() > searchPath.size())
        {
            size_t endSubElement = iter->first.find(searchPath);

            //we match the whole string in the beginning of the path
            if(endSubElement == 0)
            {
                std::string remainingFile = iter->first.substr(searchPath.size() + 1, std::string::npos);
                size_t endFileToken = remainingFile.find_first_of('/');

                if(endFileToken == std::string::npos)
                {
                    dirContents.push_back(remainingFile);
                }
            }
        }
    }

    return dirContents;
}

std::string ZipArchive::ReadPassword(const osgDB::ReaderWriter::Options* options) const
{
    //try pulling it off the options first
    std::string password = "";
    if(options != NULL)
    {
        const osgDB::AuthenticationMap* credentials = options->getAuthenticationMap();
        if(credentials != NULL)
        {
            const osgDB::AuthenticationDetails* details = credentials->getAuthenticationDetails("ZipPlugin");
            if(details != NULL)
            {
                password = details->password;
            }
        }
    }

    //if no password, try the registry
    if(password.empty())
    {
        osgDB::Registry* reg = osgDB::Registry::instance();
        if(reg != NULL)
        {
            const osgDB::AuthenticationMap* credentials = reg->getAuthenticationMap();
            if(credentials != NULL)
            {
                const osgDB::AuthenticationDetails* details = credentials->getAuthenticationDetails("ZipPlugin");
                if(details != NULL)
                {
                    password = details->password;
                }
            }
        }
    }

    return password;
}

const ZipArchive::PerThreadData&
ZipArchive::getData() const
{
    std::lock_guard<std::mutex> lock(_zipMutex);
    return getDataNoLock();
}


const ZipArchive::PerThreadData&
ZipArchive::getDataNoLock() const
{
    // get/create data for the currently running thread:
    auto current = std::this_thread::get_id();

    PerThreadDataMap::const_iterator i = _perThreadData.find( current );

    if ( i == _perThreadData.end() || i->second._zipHandle == NULL )
    {
        // cache pattern: cast to const for caching purposes
        ZipArchive* ncThis = const_cast<ZipArchive*>(this);

        // data does not already exist, so open the ZIP with a handle exclusively for this thread:
        PerThreadData& data = ncThis->_perThreadData[current];
        if ( !_filename.empty() )
        {
            int errorCode;
            data._zipHandle = zip_open(_filename.c_str(), ZIP_RDONLY, &errorCode);
            if (!data._zipHandle)
            {
                zip_error_t error;
                zip_error_init_with_code(&error, errorCode);
                OSG_WARN << "Failed to open zip " << _filename << ": " << zip_error_strerror(&error) << std::endl;
                zip_error_fini(&error);
            }
        }
        else
        {
            data._zipHandle = NULL;
        }
        return data;
    }
    else
    {
        return i->second;
    }
}
