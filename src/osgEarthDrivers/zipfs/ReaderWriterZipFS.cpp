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

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ReentrantMutex>

#include <sstream>
#include <string.h>

#include "zip.h"

using namespace osg;
using namespace osgDB;

static OpenThreads::ReentrantMutex s_mutex;

/**
* The ZipFS plugin allows you to treat zip files almost like a virtual file system.
* You can read and write objects from zips using paths like c:/data/models.zip/cow.osg where cow.osg is a file within the models.zip file.
*/
class ReaderWriterZipFS : public osgDB::ReaderWriter
{
public:

    enum ObjectType
    {
        OBJECT,
        IMAGE,
        HEIGHTFIELD,
        NODE
    };

    ReaderWriterZipFS()
    {
        supportsExtension( "zipfs", "Zip virtual file system" );
    }

    virtual const char* className()
    {
        return "ZIP virtual file system";
    }

    virtual ReadResult readNode(const std::string& file_name, const Options* options) const
    {
        return readFile(NODE, file_name, options);
    }

    virtual ReadResult readImage(const std::string& file_name, const Options* options) const
    {
        return readFile(IMAGE, file_name, options);
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        return readFile(OBJECT, file_name, options);
    }

    virtual ReadResult readHeightField(const std::string& file_name, const Options* options) const
    {
        return readFile(HEIGHTFIELD, file_name, options);
    }

    virtual WriteResult writeObject(const osg::Object& obj, const std::string& fileName, const osgDB::ReaderWriter::Options* options) const
    {
        return writeFile(OBJECT, &obj, fileName, options);
    }

    virtual WriteResult writeImage(const osg::Image& image, const std::string& fileName, const osgDB::ReaderWriter::Options* options) const
    {
        return writeFile(IMAGE, &image, fileName, options);
    }

    virtual WriteResult writeHeightField(const osg::HeightField& hf, const std::string& fileName, const osgDB::ReaderWriter::Options* options) const
    {
        return writeFile(HEIGHTFIELD, &hf, fileName, options);
    }

    virtual WriteResult writeNode(const osg::Node& node, const std::string& fileName, const osgDB::ReaderWriter::Options* options) const
    {
        return writeFile(NODE, &node, fileName,options);
    }

    WriteResult writeFile(ObjectType objectType, const osg::Object* object, osgDB::ReaderWriter* rw, std::ostream& fout, const osgDB::ReaderWriter::Options* options) const
    {
        switch (objectType)
        {
        case(OBJECT): return rw->writeObject(*object, fout, options);
        case(IMAGE): return rw->writeImage(*(dynamic_cast<const osg::Image*>(object)), fout, options);
        case(HEIGHTFIELD): return rw->writeHeightField(*(dynamic_cast<const osg::HeightField*>(object)), fout, options);
        case(NODE): return rw->writeNode(*(dynamic_cast<const osg::Node*>(object)), fout,options);
        default: break;
        }
        return WriteResult::FILE_NOT_HANDLED;
    }


    ReadResult readFile(ObjectType objectType, osgDB::ReaderWriter* rw, std::istream& fin, const osgDB::ReaderWriter::Options* options) const
    {
        switch (objectType)
        {
          case(OBJECT): return rw->readObject(fin, options);
          case(IMAGE): return rw->readImage(fin, options);
          case(HEIGHTFIELD): return rw->readHeightField(fin, options);
          case(NODE): return rw->readNode(fin, options);
          default: break;
        }
        return ReadResult::FILE_NOT_HANDLED;
    }

    ReadResult readFile(ObjectType objectType, const std::string &fullFileName, const osgDB::ReaderWriter::Options* options) const
    {
        OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(s_mutex);

        //This plugin allows you to treat zip files almost like virtual directories.  So, the pathname to the file you want in the zip should
        //be of the format c:\data\myzip.zip\images\foo.png

        std::string::size_type len = fullFileName.find(".zip");
        if (len == std::string::npos)
        {
            osg::notify(osg::INFO) << "ReaderWriterZipFS: Path does not contain zip file" << std::endl;
            return ReadResult::FILE_NOT_HANDLED;
        }

        std::string zipFile = fullFileName.substr(0, len + 4);
        zipFile = osgDB::findDataFile(zipFile);
        zipFile = osgDB::convertFileNameToNativeStyle( zipFile );

        //Return if the file doesn't exist
        if (!osgDB::fileExists( zipFile )) return ReadResult::FILE_NOT_FOUND;

        osg::notify(osg::INFO) << "ReaderWriterZipFS::readFile  ZipFile path is " << zipFile << std::endl;

        std::string zipEntry = fullFileName.substr(len+4);


        //Strip the leading slash from the zip entry
        if ((zipEntry.length() > 0) && 
            ((zipEntry[0] == '/') || (zipEntry[0] == '\\')))
        {
            zipEntry = zipEntry.substr(1);
        }


        //Lipzip returns filenames with '/' rather than '\\', even on Windows.  So, convert the zip entry to Unix style
        zipEntry = osgDB::convertFileNameToUnixStyle(zipEntry);
        osg::notify(osg::INFO) << "Zip Entry " << zipEntry << std::endl;

        //See if we can get a ReaderWriter for the zip entry before we even try to unzip the file
         ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(osgDB::getFileExtension(zipEntry));
         if (!rw)
         {
             osg::notify(osg::NOTICE) << "Could not find ReaderWriter for " << zipEntry << std::endl;
             return ReadResult::FILE_NOT_HANDLED;
         }


        int err;

        //Open the zip file
        struct zip* pZip = zip_open(zipFile.c_str(), ZIP_CHECKCONS, &err);
        if (pZip)
        {
            //List the files
            /*int numFiles = zip_get_num_files(pZip);
            osg::notify(osg::NOTICE) << zipFile << " has " << numFiles << " files " << std::endl;
            for (int i = 0; i < numFiles; ++i)
            {
            osg::notify(osg::NOTICE) << i << ": " << zip_get_name(pZip, i, 0) << std::endl;
            }*/

            //Find the index within the zip file for the given zip entry
            int zipIndex = zip_name_locate(pZip, zipEntry.c_str(), 0);

            osg::notify(osg::INFO) << "ReaderWriterZipFS: ZipFile index " << zipIndex << std::endl;
            if (zipIndex < 0)
            {
                osg::notify(osg::INFO) << "Could not find zip entry " << zipEntry << " in " << zipFile << std::endl;
                //Make sure to close the zipfile
                zip_close(pZip);
                return ReadResult::FILE_NOT_FOUND;  
            }
            
            //Open the entry for reading
            zip_file* pZipFile = zip_fopen_index(pZip, zipIndex, 0);

            if (pZipFile) 
            {
                //Read the data from the entry into a std::string
                int dataSize = 0;
                std::string data;
                do{
                    char buffer[1024];
                    dataSize = zip_fread(pZipFile, buffer, 1024);
                    if (dataSize > 0)
                    {
                       data.append((char*)buffer, dataSize);
                    }
                }while (dataSize > 0);

                //Close the zip entry and the actual zip file itself
                zip_fclose(pZipFile);
                zip_close(pZip);

                std::stringstream strstream(data);
                return readFile(objectType, rw, strstream, options);
            }
        }
        else
        {
            osg::notify(osg::NOTICE) << "ReaderWriterZipFS::readFile couldn't open zip " << zipFile << " full filename " << fullFileName << std::endl;
        }
        return ReadResult::FILE_NOT_HANDLED;
    }

    WriteResult writeFile(ObjectType objectType, const osg::Object* object, const std::string& fullFileName, const osgDB::ReaderWriter::Options* options) const
    {       
        OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(s_mutex);

        std::string::size_type len = fullFileName.find(".zip");
        if (len == std::string::npos)
        {
            osg::notify(osg::INFO) << "ReaderWriterZipFS: Path does not contain zip file" << std::endl;
            return WriteResult::FILE_NOT_HANDLED;
        }

        //It is possible that the zip file doesn't currently exist, so we just use getRealPath instead of findDataFile as in the readFile method
        std::string zipFile = osgDB::getRealPath(fullFileName.substr(0, len + 4));

        std::string path = osgDB::getFilePath(zipFile);
        //If the path doesn't currently exist, create it
        if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
        {
            osg::notify(osg::WARN) << "Couldn't create path " << path << std::endl;
        }

        osg::notify(osg::INFO) << "ReaderWriterZipFS::writeFile ZipFile path is " << zipFile << std::endl;

        std::string zipEntry = fullFileName.substr(len+4);


        //Strip the leading slash from the zip entry
        if ((zipEntry.length() > 0) && 
            ((zipEntry[0] == '/') || (zipEntry[0] == '\\')))
        {
            zipEntry = zipEntry.substr(1);
        }


        //Lipzip returns filenames with '/' rather than '\\', even on Windows.  So, convert the zip entry to Unix style
        zipEntry = osgDB::convertFileNameToUnixStyle(zipEntry);
        osg::notify(osg::INFO) << "Zip Entry " << zipEntry << std::endl;

        //See if we can get a ReaderWriter for the zip entry before we even try to unzip the file
         ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(osgDB::getFileExtension(zipEntry));
         if (!rw)
         {
             osg::notify(osg::INFO) << "Could not find ReaderWriter for " << zipEntry << std::endl;
             return WriteResult::FILE_NOT_HANDLED;
         }

        int err;

        //Open the zip file
        struct zip* pZip = zip_open(zipFile.c_str(), ZIP_CREATE|ZIP_CHECKCONS, &err);
        if (pZip)
        {
            //Write the data to the stream
            std::ostringstream strstream;
            writeFile(objectType, object, rw, strstream, options);

            char *data = new char[strstream.str().length()];
            memcpy(data, strstream.str().c_str(), strstream.str().size());

            WriteResult wr;
            struct zip_source *zs = zip_source_buffer(pZip, data, strstream.str().length(), 0);
            if (zs)
            {
                if (zip_add(pZip, zipEntry.c_str(), zs) != -1) 
                {
                    wr = WriteResult::FILE_SAVED;
                }
                else
                {
                  osg::notify(osg::NOTICE) << "Couldn't add zip source " << std::endl;
                  wr = WriteResult::ERROR_IN_WRITING_FILE;
                }
            }
            else
            {
                osg::notify(osg::NOTICE) << "Couldn't create zip source " << std::endl;
                wr = WriteResult::ERROR_IN_WRITING_FILE;
            }
            zip_close(pZip);
            delete[] data;
            return wr;
        }
        else
        {
            osg::notify(osg::NOTICE) << "ReaderWriterZipFS::writeFile couldn't open zip " << zipFile << " full filename " << fullFileName << std::endl;
        }
        return WriteResult::FILE_NOT_HANDLED;      
    }
};

REGISTER_OSGPLUGIN(zipfs, ReaderWriterZipFS)

