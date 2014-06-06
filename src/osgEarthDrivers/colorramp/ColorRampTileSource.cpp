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
#include "ColorRampOptions"

#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarthSymbology/Color>
#include <osgDB/FileNameUtils>
#include <osg/TransferFunction>

#include <cstring>

#define LC "[ColorRamp Driver] "

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;


class ColorRampTileSource : public TileSource
{
public:
    ColorRampTileSource( const TileSourceOptions& options ) :
      TileSource( options ),            
      _options( options )
    {
        //nop
    }

    Status initialize( const osgDB::Options* dbOptions )
    {
        osg::ref_ptr<osgDB::Options> localOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);        

        if (!_options.elevationLayer().isSet())
        {
            return Status::Error("Please specify a heightfield layer for the color ramp");
        }
        
        _layer = new ElevationLayer(*_options.elevationLayer() );
        if (!_layer.valid())
        {
            return Status::Error("Failed to initialize the Please specify a heightfield layer for the color ramp");
        }

        setProfile(_layer->getProfile());

        initTransferFunction();
               

        return STATUS_OK;
    }

    double fToC(float f)
    {
        return (f - 32.0) * (5.0/9.0);
    }

    void initTransferFunction()
    {
        _transferFunction = new osg::TransferFunction1D();
        _transferFunction->setColor(fToC(-20), Color("#A99ADB"));
        _transferFunction->setColor(fToC(-10), Color("#3654B6"));
        _transferFunction->setColor(fToC(0), Color("#3275C6"));
        _transferFunction->setColor(fToC(10), Color("#4FA9C3"));
        _transferFunction->setColor(fToC(20), Color("#5240A4"));
        _transferFunction->setColor(fToC(30), Color("#264F2F"));
        _transferFunction->setColor(fToC(40), Color("#197625"));
        _transferFunction->setColor(fToC(50), Color("#26C233"));
        _transferFunction->setColor(fToC(60), Color("#BAD111"));
        _transferFunction->setColor(fToC(70), Color("#F7A003"));
        _transferFunction->setColor(fToC(80), Color("#CD4B00"));
        _transferFunction->setColor(fToC(90), Color("#AF211F"));
        _transferFunction->setColor(fToC(100), Color("#870C0E"));        
    }
      

    osg::Image*
    createImage( const TileKey& key, ProgressCallback* progress )
    {
        // Use the underlying ElevationLayer to create a heightfield and then color it.
        GeoHeightField geoHF = _layer->createHeightField(key, progress);
        if (geoHF.valid())
        {
            osg::HeightField* hf = geoHF.getHeightField(); 
            osg::Image* image = new osg::Image();
            image->allocateImage(hf->getNumColumns(),hf->getNumRows(),1, GL_RGBA, GL_UNSIGNED_BYTE);
            memset(image->data(), 0, image->getImageSizeInBytes());
            ImageUtils::PixelWriter writer(image);
            for (unsigned int c = 0; c < hf->getNumColumns(); c++)
            {
                for (unsigned int r = 0; r < hf->getNumRows(); r++)
                {
                    float v = hf->getHeight(c, r );
                    if (v != NO_DATA_VALUE)
                    {                        
                        osg::Vec4 color = _transferFunction->getColor(v);
                        writer(color, c, r);
                    }                    
                }
            } 
            return image;

        }
        return NULL;
    }


private:
    const ColorRampOptions _options;
    osg::ref_ptr< ElevationLayer > _layer;
    osg::ref_ptr< osg::TransferFunction1D> _transferFunction;
};


class ColorRampTileSourceFactory : public TileSourceDriver
{
public:
    ColorRampTileSourceFactory()
    {
        supportsExtension( "osgearth_colorramp", "Color ramp driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "ColorRamp Image Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new ColorRampTileSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_colorramp, ColorRampTileSourceFactory)

