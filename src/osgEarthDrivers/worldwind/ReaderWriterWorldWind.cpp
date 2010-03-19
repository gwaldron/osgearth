/* -*-c++-*- */
/* osgEarth worldwind plugin - for the osgEarth toolkit
* based on the yahoo plugin provided as part of the osgearth distribution
* Produced by Matt Franklin
* Contact: MattFranklin1 at gmail.com
*
* Please note that the use of this plugin requires the user to be accept
* the license agreements provided by NASA
*
* This plugin is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
*/

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/Archive>

#include "zip.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

#ifdef remove
#undef remove
#endif

using namespace osgEarth;

class WorldWindSource : public TileSource
{
    int _maxLOD;
    std::string _worldwind_cache;
public:
    WorldWindSource( const PluginOptions* options ) : TileSource( options )
    {
        const Config& conf = options->config();
        _maxLOD = options->config().value( "max_lod", 11 );

        _worldwind_cache = conf.value<std::string>( "worldwind_cache", "" );

    }

public:
    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {

        setProfile( Profile::create(
            "epsg:4326",
            -180.0, -90.0, 180.0, 90.0,
            18, 9 ) );
    }

    osg::HeightField* createHeightFieldFromBil(char* buf,int buflength)
    {

        osg::HeightField* hf = new osg::HeightField;
        //osg::notify(osg::NOTICE) << "Read heightfield image" << std::endl;
        hf->allocate(150, 150);

        for( unsigned int row=0; row < 150; row++ )
        {
            for( unsigned int col=0; col < 150; col++ )
            {
                short* ptr = (short*)buf;
                short val = ptr[col + ((150-row -1)*150)];

                hf->setHeight( col, row, (float)val * 0.3048 );
            }
        }
        return hf;
    }

public:
    osg::Image* createImage( const TileKey* key ,
        ProgressCallback* progress)
    {
        //NOP
        return NULL;
    }


    osg::HeightField* createHeightField( const TileKey* key,
        ProgressCallback* progress)
    {
        if ( _maxLOD <= key->getLevelOfDetail()) return NULL;

        osg::HeightField *hf = NULL;
        std::string cachefilepath = _worldwind_cache + "/" + createCachePath(key);
        std::string cachefilename = createCacheName(key) + ".bil";
        std::string fullcachefilename = cachefilepath + "/" + cachefilename;
        OE_DEBUG << "Cached name " << fullcachefilename << std::endl;
        if (osgDB::fileExists(fullcachefilename))
        {
            // read file
            std::ifstream fin;
            fin.open(fullcachefilename.c_str(), std::ios::in | std::ios::binary);
            if (!fin)
            {
                OE_NOTICE << "Coud not open cache " << fullcachefilename << std::endl;
                return NULL;
            }
            int fullsize = 150*150*2;
            char *buf = new char[fullsize];
            OE_DEBUG << "Loading from cache " << fullcachefilename << std::endl;
            if ( !fin.read(buf, fullsize))
            {
                OE_NOTICE << "Coud not read from cache " << fullcachefilename << std::endl;
                delete[] buf;
                fin.close();
                return NULL;
            }
            hf = createHeightFieldFromBil((char*)buf,fullsize);
            delete[] buf;
            fin.close();
        } else {
            // download file
            HTTPResponse out_response;
            std::string URI = createURI(key);

            OE_DEBUG << "Requesting " << URI << std::endl;
            out_response = HTTPClient::get( URI, getOptions(), progress );

            if ( !out_response.isOK() )
            {
                OE_NOTICE << "No Response received for " << URI << std::endl;
                return NULL;
            }

            // store downloaded part as a zip file
            // Useful to store a local copy as the same file is requested many times
            unsigned int part_num = out_response.getNumParts() > 1? 1 : 0;
            std::string zipfilename;
            out_response.getPartHeader(part_num, zipfilename);
            std::istream& input_stream = out_response.getPartStream( part_num );

            if ( !osgDB::fileExists(cachefilepath) )
            {
                osgDB::makeDirectory(cachefilepath);
            }
            std::ofstream fout;
            std::string tempname = fullcachefilename + ".zip";
            fout.open(tempname.c_str(), std::ios::out | std::ios::binary);

            if ( !fout )
            {
                OE_NOTICE << "Could not write zip file" << std::endl;
                return NULL;
            }
            input_stream.seekg (0, std::ios::end);
            int length = input_stream.tellg();
            input_stream.seekg (0, std::ios::beg);

            char *buffer = new char[length];
            input_stream.read(buffer, length);
            fout.write(buffer, length);
            delete[] buffer;
            fout.close();


            //Unzip the file
            int err;

            //Open the zip file
            struct zip* pZip = zip_open(tempname.c_str(), ZIP_CHECKCONS, &err);
            if (pZip)
            {
                //List the files
                int numFiles = zip_get_num_files(pZip);
                //OE_DEBUG <<  tempname << " has " << numFiles << " files " << std::endl;
                /*for (int i = 0; i < numFiles; ++i)
                {
                    OE_NOTICE << i << ": " << zip_get_name(pZip, i, 0) << std::endl;
                }*/

                //Find the index within the zip file for the given zip entry
                int zipIndex = 0;

                //Open the first file for reading
                zip_file* pZipFile = zip_fopen_index(pZip, 0, 0);

                if (pZipFile) 
                {
                    //Read the data from the entry into a std::string
                    int dataSize = 0;
                    std::string data;
                    do{
                        char* buf = new char[1024];
                        dataSize = zip_fread(pZipFile, buf, 1024);
                        if (dataSize == 0)
                        {
                            delete [](buf);
                            buf = NULL;
                        }
                        if (buf)
                        {
                            data.append((char*)buf, dataSize);
                        }
                    }while (dataSize > 0);

                    //Close the zip entry and the actual zip file itself
                    zip_fclose(pZipFile);
                    zip_close(pZip);

                    //Write the BIL file to the cache
                    fout.open(fullcachefilename.c_str(), std::ios::out | std::ios::binary);
                    if ( !fout )
                    {
                        std::cout << "Cannot write bil file"<< std::endl;
                        return NULL;
                    }

                    fout.write((char*)data.c_str(), data.size());
                    fout.close(); 

                    hf = createHeightFieldFromBil((char*)data.c_str(),data.size());                    
                    // delete zip file as it has now been processed
                    remove(tempname.c_str());
                }               
            }
        }
        return hf;
    }

    std::string createCachePath( const TileKey* key ) const
    {
        unsigned int x, y;
        key->getTileXY(x, y);

        unsigned int lod = key->getLevelOfDetail();

        std::stringstream buf;
        buf << "" << lod
            << "/" << std::setw(4) << std::setfill('0') << x;
        std::string bufStr;
        bufStr = buf.str();
        return bufStr;
    }

    std::string createCacheName( const TileKey* key ) const
    {
        unsigned int x, y;
        key->getTileXY(x, y);

        unsigned int lod = key->getLevelOfDetail();

        // flip the y based on level
        int flippedy = ((9 * powf((int)2,(int)lod)) - 1) - y;
        //printf("Key %i, %i, %i\n", lod,x,flippedy);

        std::stringstream buf;
        buf << "" << std::setw(4) << std::setfill('0') << x
            << "_" << std::setw(4) << std::setfill('0') << flippedy;
        std::string bufStr;
        bufStr = buf.str();
        return bufStr;
    }

    std::string createURI( const TileKey* key ) const
    {
        unsigned int x, y;
        key->getTileXY(x, y);

        unsigned int lod = key->getLevelOfDetail();

        // flip the y based on level
        int flippedy = ((9 * powf((int)2,(int)lod)) - 1) - y;

        std::stringstream buf;
        buf << "http://worldwind25.arc.nasa.gov/wwelevation/wwelevation.aspx?T=srtm30pluszip"
            << "&L=" << lod
            << "&X=" << x
            << "&Y=" << flippedy;
        std::string bufStr;
        bufStr = buf.str();
        return bufStr;
    }

    virtual int getPixelsPerTile() const
    {
        return 150;
    }

    virtual std::string getExtension()  const
    {
        return "bil";
    }

private:
};


class ReaderWriterWorldWind : public osgDB::ReaderWriter
{
public:
    ReaderWriterWorldWind() {}

    virtual const char* className()
    {
        return "WorldWind Reader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "osgearth_WorldWind" );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
    {
        std::string ext = osgDB::getFileExtension( file_name );
        if ( !acceptsExtension( ext ) )
        {
            return ReadResult::FILE_NOT_HANDLED;
        }

        return new WorldWindSource( static_cast<const PluginOptions*>(opt) );
    }
};

REGISTER_OSGPLUGIN(osgearth_WorldWind, ReaderWriterWorldWind) 