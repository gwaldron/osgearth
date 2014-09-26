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
#include <osgEarth/CompositeTileSource>
#include <osgEarth/ImageUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgEarth/HeightFieldUtils>
#include <osgDB/FileNameUtils>

#define LC "[CompositeTileSource] "

using namespace osgEarth;

//------------------------------------------------------------------------

CompositeTileSourceOptions::CompositeTileSourceOptions( const TileSourceOptions& options ) :
TileSourceOptions( options )
{
    setDriver( "composite" );
    fromConfig( _conf );
}

void
CompositeTileSourceOptions::add( const ImageLayerOptions& options )
{
    Component c;
    c._imageLayerOptions = options;
    _components.push_back( c );
}

void
CompositeTileSourceOptions::add( const ElevationLayerOptions& options )
{
    Component c;
    c._elevationLayerOptions = options;
    _components.push_back( c );
}

Config 
CompositeTileSourceOptions::getConfig() const
{    
    Config conf = TileSourceOptions::newConfig();

    for( ComponentVector::const_iterator i = _components.begin(); i != _components.end(); ++i )
    {
        if ( i->_imageLayerOptions.isSet() )
            conf.add( "image", i->_imageLayerOptions->getConfig() );
    }

    return conf;
}

void 
CompositeTileSourceOptions::mergeConfig( const Config& conf )
{
    TileSourceOptions::mergeConfig( conf );
    fromConfig( conf );
}

void 
CompositeTileSourceOptions::fromConfig( const Config& conf )
{    
    const ConfigSet& images = conf.children("image");
    for( ConfigSet::const_iterator i = images.begin(); i != images.end(); ++i )
    {
        add( ImageLayerOptions( *i ) );
    }

    const ConfigSet& elevations = conf.children("elevation");
    for( ConfigSet::const_iterator i = elevations.begin(); i != elevations.end(); ++i )
    {
        add( ElevationLayerOptions( *i ) );
    }

    const ConfigSet& heightfields = conf.children("heightfield");
    for( ConfigSet::const_iterator i = heightfields.begin(); i != heightfields.end(); ++i )
    {
        add( ElevationLayerOptions( *i ) );
    }

    if (conf.children("model").size() > 0 || conf.children("overlay").size() > 0 )
    {
        OE_WARN << LC << "Illegal - composite driver only supports image and elevation layers" << std::endl;
    }
}

//------------------------------------------------------------------------

namespace
{
    struct ImageInfo
    {
        ImageInfo()
        {
            image = 0;
            opacity = 1;
            dataInExtents = false;
        }

        ImageInfo(osg::Image* image, float opacity, bool dataInExtents)
        {
            this->image = image;
            this->opacity = opacity;
            this->dataInExtents = dataInExtents;
        }

        bool dataInExtents;
        float opacity;
        osg::ref_ptr< osg::Image> image;
    };

    // some helper types.    
    typedef std::vector<ImageInfo> ImageMixVector;   
}

//-----------------------------------------------------------------------

CompositeTileSource::CompositeTileSource( const TileSourceOptions& options ) :
TileSource  ( options ),
_options    ( options ),
_initialized( false ),
_dynamic    ( false )
{
    //nop
}

osg::Image*
CompositeTileSource::createImage(const TileKey&    key,
                                 ProgressCallback* progress )
{    
    ImageMixVector images;
    images.reserve(_imageLayers.size());

    // Try to get an image from each of the layers for the given key.
    for (ImageLayerVector::const_iterator itr = _imageLayers.begin(); itr != _imageLayers.end(); ++itr)
    {
        ImageLayer* layer = itr->get();
        ImageInfo imageInfo;
        imageInfo.dataInExtents = layer->getTileSource()->hasDataInExtent( key.getExtent() );
        imageInfo.opacity = layer->getOpacity();

        if (imageInfo.dataInExtents)
        {
            GeoImage image = layer->createImage(key, progress);
            if (image.valid())
            {
                imageInfo.image = image.getImage();
            }
        }

        images.push_back(imageInfo);
    }

    // Determine the output texture size to use based on the image that were creatd.
    unsigned numValidImages = 0;
    osg::Vec2s textureSize;
    for (unsigned int i = 0; i < images.size(); i++)
    {
        ImageInfo& info = images[i];
        if (info.image.valid())
        {
            if (numValidImages == 0)
            {
                textureSize.set( info.image->s(), info.image->t());
            }
            numValidImages++;        
        }
    } 

    // Create fallback images if we have some valid data but not for all the layers
    if (numValidImages > 0 && numValidImages < images.size())
    {
        for (unsigned int i = 0; i < images.size(); i++)
        {
            ImageInfo& info = images[i];
            ImageLayer* layer = _imageLayers[i].get();
            if (!info.image.valid() && info.dataInExtents)
            {                      
                TileKey parentKey = key.createParentKey();

                GeoImage image;
                while (!image.valid() && parentKey.valid())
                {
                    image = layer->createImage(parentKey, progress);
                    if (image.valid())
                    {
                        break;
                    }
                    parentKey = parentKey.createParentKey();
                }

                if (image.valid())
                {                                        
                    // TODO:  Bilinear options?
                    GeoImage cropped = image.crop( key.getExtent(), true, textureSize.x(), textureSize.y(), true);
                    info.image = cropped.getImage();
                }                    
            }
        }
    }

    // Now finally create the output image.
    //Recompute the number of valid images
    numValidImages = 0;
    for (unsigned int i = 0; i < images.size(); i++)
    {
        ImageInfo& info = images[i];
        if (info.image.valid()) numValidImages++;        
    }    

    if ( progress && progress->isCanceled() )
    {
        return 0L;
    }
    else if ( numValidImages == 0 )
    {
        return 0L;
    }
    else if ( numValidImages == 1 )
    {
        //We only have one valid image, so just return it and don't bother with compositing
        for (unsigned int i = 0; i < images.size(); i++)
        {
            ImageInfo& info = images[i];
            if (info.image.valid())
                return info.image.release();
        }
        return 0L;
    }
    else
    {
        osg::Image* result = 0;
        for (unsigned int i = 0; i < images.size(); i++)
        {
            ImageInfo& imageInfo = images[i];
            if (!result)
            {
                if (imageInfo.image.valid())
                {
                    result = new osg::Image( *imageInfo.image.get());
                }
            }
            else
            {
                if (imageInfo.image.valid())
                {
                    ImageUtils::mix( result, imageInfo.image, imageInfo.opacity );
                }
            }            
        }        
        return result;
    }



}

osg::HeightField* CompositeTileSource::createHeightField(
            const TileKey&        key,
            ProgressCallback*     progress )
{    
    unsigned int size = *getOptions().tileSize();    
    bool hae = false;
    osg::ref_ptr< osg::HeightField > heightField = new osg::HeightField();
    heightField->allocate(size, size);

    // Initialize the heightfield to nodata
    for (unsigned int i = 0; i < heightField->getFloatArray()->size(); i++)
    {
        heightField->getFloatArray()->at( i ) = NO_DATA_VALUE;
    }  

    // Populate the heightfield and return it if it's valid
    if (_elevationLayers.populateHeightField(heightField.get(), key, 0, INTERP_AVERAGE, progress))
    {                
        return heightField.release();
    }
    else
    {        
        return NULL;
    }
}

bool
CompositeTileSource::add( ImageLayer* layer )
{
    if ( _initialized )
    {
        OE_WARN << LC << "Illegal: cannot modify TileSource after initialization" << std::endl;
        return false;
    }

    if ( !layer )
    {
        OE_WARN << LC << "Illegal: tried to add a NULL layer" << std::endl;
        return false;
    }

    _imageLayers.push_back( layer );
    CompositeTileSourceOptions::Component comp;
    comp._layer = layer;
    comp._imageLayerOptions = layer->getImageLayerOptions();
    _options._components.push_back( comp );    

    return true;
}

bool
CompositeTileSource::add( ElevationLayer* layer )
{
    if ( _initialized )
    {
        OE_WARN << LC << "Illegal: cannot modify TileSource after initialization" << std::endl;
        return false;
    }

    if ( !layer )
    {
        OE_WARN << LC << "Illegal: tried to add a NULL layer" << std::endl;
        return false;
    }

    _elevationLayers.push_back( layer );
    CompositeTileSourceOptions::Component comp;
    comp._layer = layer;
    comp._elevationLayerOptions = layer->getElevationLayerOptions();
    _options._components.push_back( comp );    

    return true;
}


TileSource::Status
CompositeTileSource::initialize(const osgDB::Options* dbOptions)
{
    _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

    osg::ref_ptr<const Profile> profile = getProfile();    

    for(CompositeTileSourceOptions::ComponentVector::iterator i = _options._components.begin();
        i != _options._components.end(); )
    {        
        if ( i->_imageLayerOptions.isSet() )
        {
            osg::ref_ptr< ImageLayer > layer = new ImageLayer(*i->_imageLayerOptions);
            if (!layer->getTileSource())
            {
                OE_WARN << LC << "Could not find a TileSource for driver [" << i->_imageLayerOptions->driver()->getDriver() << "]" << std::endl;
            }
            else
            {
                i->_layer = layer;
                _imageLayers.push_back( layer );
            }            
        }
        else if (i->_elevationLayerOptions.isSet())
        {
            osg::ref_ptr< ElevationLayer > layer = new ElevationLayer(*i->_elevationLayerOptions);            
            if (!layer->getTileSource())
            {
                   OE_WARN << LC << "Could not find a TileSource for driver [" << i->_elevationLayerOptions->driver()->getDriver() << "]" << std::endl;
            }
            else
            {
                i->_layer = layer;
                _elevationLayers.push_back( layer.get() );                
            }
        }

        if ( !i->_layer.valid() )
        {
            OE_WARN << LC << "A component has no valid TerrainLayer ... removing." << std::endl;
            i = _options._components.erase( i );
        }
        else
        {            
            TileSource* source = i->_layer->getTileSource();

            // If no profile is specified assume they want to use the profile of the first layer in the list.
            if (!profile.valid())
            {
                profile = source->getProfile();
            }

            _dynamic = _dynamic || source->isDynamic();
            
            // gather extents                        
            const DataExtentList& extents = source->getDataExtents();            
            for( DataExtentList::const_iterator j = extents.begin(); j != extents.end(); ++j )
            {                
                // Convert the data extent to the profile that is actually used by this TileSource
                DataExtent dataExtent = *j;                
                GeoExtent ext = dataExtent.transform(profile->getSRS());
                unsigned int minLevel = 0;
                unsigned int maxLevel = profile->getEquivalentLOD( source->getProfile(), *dataExtent.maxLevel() );                                        
                dataExtent = DataExtent(ext, minLevel, maxLevel);                                
                getDataExtents().push_back( dataExtent );
            }          
        }

        ++i;
    }

    // set the new profile that was derived from the components
    setProfile( profile.get() );

    _initialized = true;
    return STATUS_OK;
}

//------------------------------------------------------------------------

namespace
{
    struct CompositeTileSourceDriver : public TileSourceDriver
    {
        CompositeTileSourceDriver()
        {
            supportsExtension( "osgearth_composite", "Composite tile source driver" );
        }

        virtual const char* className()
        {
            return "CompositeTileSourceDriver";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new CompositeTileSource( getTileSourceOptions(options) );
        }
    };
}
REGISTER_OSGPLUGIN(osgearth_composite, CompositeTileSourceDriver)
