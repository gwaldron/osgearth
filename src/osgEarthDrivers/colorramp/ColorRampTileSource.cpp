/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgDB/FileUtils>
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
            return Status::Error(Status::ConfigurationError, "Please specify a heightfield layer for the color ramp");
        }
        
        _layer = new ElevationLayer(*_options.elevationLayer() );
        _layer->open();

        setProfile(_layer->getProfile());

        initTransferFunction();
               

        return STATUS_OK;
    }

    void initTransferFunction()
    {     
        _transferFunction = loadCLRFile(_options.ramp()->full());
        if (!_transferFunction.valid())
        {
            OE_WARN << LC << "Failed to load transfer function from " << _options.ramp()->full() << std::endl;
            
            // Just create a default ramp
            _transferFunction = new osg::TransferFunction1D();
            _transferFunction->setColor(0, osg::Vec4(1,0,0,1));
            _transferFunction->setColor(100, osg::Vec4(0,1,0,1));
        }
    }  

    osg::TransferFunction1D* loadCLRFile(const std::string& filename)
    {
        if (osgDB::fileExists(filename))
        {
            osg::TransferFunction1D* transfer = new osg::TransferFunction1D();

            std::ifstream in(filename.c_str());
            float value;
            unsigned int r, g, b, a;
            while(in >> value >> r >> g >> b >> a)
            {                
                transfer->setColor(value, osg::Vec4((float)r/255.0, (float)g/255.0, (float)b/255.0, (float)a/255.0));
            }
            return transfer;
        }
        return NULL;        
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

    virtual const char* className() const
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

