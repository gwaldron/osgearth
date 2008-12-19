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

#include <osgEarth/MapConfig>
#include <osgEarth/Mercator>
#include <osgEarth/FileCache>
#include <osgEarth/ImageToHeightFieldConverter>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <ctype.h>

#include <sstream>

using namespace osgEarth;

#define PROPERTY_URL        "url"
#define PROPERTY_DATASET    "dataset"
#define PROPERTY_MAP_CONFIG "map_config"



class MSVESource : public TileSource
{
public:
    MSVESource( const osgDB::ReaderWriter::Options* _options ) :
      options( _options ),
      map_config(0)
    {
        //Set the profile to global mercator
        _profile = TileGridProfile(TileGridProfile::GLOBAL_MERCATOR);

        if ( options.valid() )
        {
            if ( options->getPluginData( PROPERTY_URL ) )
                url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_DATASET ) )
                dataset = std::string( (const char*)options->getPluginData( PROPERTY_DATASET ) );

            if ( options->getPluginData( PROPERTY_MAP_CONFIG ) )
                map_config = (const MapConfig*)options->getPluginData(PROPERTY_MAP_CONFIG);
        }

        // validate dataset
        if ( dataset == "hybrid" ) dataset = "h";
        else if ( dataset == "roads" ) dataset = "r";
        else if ( dataset == "aerial" || dataset == "satellite" ) dataset = "a";
        else if ( dataset == "terrain") dataset = "t";
        if ( dataset.empty() ) dataset = "h"; // default to the "hybrid" dataset (imagery+labels)

        // validate/default URL
        if ( url.empty() )
            url = "http://h0.ortho.tiles.virtualearth.net/tiles";
    }

    osg::Image* createImage( const TileKey* key )
    {
        //If we are given a PlateCarreTileKey, use the MercatorTileConverter to create the image
        if ( dynamic_cast<const PlateCarreTileKey*>( key ) )
        {
            MercatorTileConverter converter( this );
            return converter.createImage( static_cast<const PlateCarreTileKey*>( key ) );
        }


        // a=aerial(jpg), r=map(png), h=hybrid(jpg), t=elev(wmphoto?)

        std::stringstream buf;

        std::string myurl = url;

        //Round Robin the server number [0,3]
        if (osgDB::containsServerAddress(url))
        {
          //Assume URL is in the form http://[type][server].ortho.tiles.virtualearth.net/tiles/[type][location].[format]?g=45
          char server = key->str().length() > 0? key->str()[key->str().length()-1] : '0';
          if (url.size() >=9 && isdigit(myurl[8]))
          {
              myurl[8] = server;
          }
        }
        


        buf << myurl << "/" << dataset << key->str();
        //Only add the ?g=1 if we are connecting to a server address
        if (osgDB::containsServerAddress(url)) buf << "?g=1&";
        buf << "." << getFormat();

        osg::notify(osg::INFO) << "Loading MSVE tile " << key->str() << " from " << buf.str() << std::endl;

        //osg::Image* image = osgDB::readImageFile( buf.str() );

        osg::notify(osg::INFO) 
            << "[osgEarth] MSVE: option string = "
            << (options.valid()? options->getOptionString() : "<empty>")
            << std::endl;

        std::string cache_path = map_config ? map_config->getFullCachePath() : std::string("");
        bool offline = map_config ? map_config->getOfflineHint() : false;
        osgEarth::FileCache fc(cache_path);
        fc.setOffline(offline);
        osg::ref_ptr<osg::Image> image = fc.readImageFile( buf.str(), options.get() );
        /*if (!image)
        {
            //HACK:  The "no image" image that is returned from MSVE will have a JPG extension but it is really a
            //       PNG.  Try loading it as a PNG instead
            osg::notify(osg::INFO) << "Trying to read as a PNG " << std::endl;
            std::stringstream os;
            os << osgDB::getNameLessExtension(buf.str()) << ".png";

            image = osgDB::readImageFile(os.str(), options.get());

            if (image.valid())
            {
                osg::notify(osg::INFO) << "Read as a PNG successfully" << std::endl;
                osgDB::writeImageFile(*image, "bad_image.png");
            }
        }*/

        if (isBadImage(image.get())) return NULL;
        return image.release();
    }

    std::string getFormat() const
    {
        if (dataset == "h" || dataset == "a")
        {
            return "jpg";
        }
        else if (dataset == "t")
        {
            return "tif";
        }
        else
        {
            return "png";
        }
    }    

    osg::HeightField* createHeightField( const TileKey* key )
    {
        osg::Image* image = createImage(key);
        if (!image)
        {
            osg::notify(osg::WARN) << "Failed to read heightfield from " << key->str() << std::endl;
        }
        return (ImageToHeightFieldConverter::convert(image));  
    }

    /*
     * Determines if the given image is the "bad" image that MSVE uses to denote no imagery in an area
     */
    static bool isBadImage(osg::Image* image)
    {
        if (image)
        {
            static bool tried_to_load_bad_image = false;
            if (!tried_to_load_bad_image)
            {
                if (!bad_image.valid())
                {
                    bad_image = osgDB::readImageFile("msve_bad_image.png");
                    tried_to_load_bad_image = true;
                    if (!bad_image.valid()) osg::notify(osg::WARN) << "Could not load msve_bad_image.png" << std::endl;
                }
            }

            if (bad_image.valid())
            {
                if ((image->s() == bad_image->s()) && (image->t() == bad_image->t()))
                {
                    for (int c = 0; c < image->s(); ++c)
                    {
                        for (int r = 0; r < image->t(); ++r)
                        {
                            if (*image->data(c, r) != *bad_image->data(c, r))
                            {
                                return false;
                            }
                        }
                    }
                    osg::notify(osg::NOTICE) << "Detected bad image" << std::endl;
                    return true;
                }
            }
        }
        return false;
    }        

private:
    osg::ref_ptr<const osgDB::ReaderWriter::Options> options;
    std::string url;
    std::string dataset;

    const MapConfig* map_config;

    static osg::ref_ptr<osg::Image> bad_image;
};

osg::ref_ptr<osg::Image> MSVESource::bad_image;


class ReaderWriterMSVE : public osgDB::ReaderWriter
{
    public:
        ReaderWriterMSVE() {}

        virtual const char* className()
        {
            return "MSVE Imagery ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "msve" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
            return new MSVESource(options);
        }
};

REGISTER_OSGPLUGIN(msve, ReaderWriterMSVE)
