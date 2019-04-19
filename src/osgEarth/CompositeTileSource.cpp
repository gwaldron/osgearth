/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/Progress>

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
    Config conf = TileSourceOptions::getConfig();

    for( ComponentVector::const_iterator i = _components.begin(); i != _components.end(); ++i )
    {
        if ( i->_imageLayerOptions.isSet() )
            conf.add( "image", i->_imageLayerOptions->getConfig() );
        else if ( i->_elevationLayerOptions.isSet() )
            conf.add( "elevation", i->_elevationLayerOptions->getConfig() );
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
    const ConfigSet& images = conf.hasChild("images") ? conf.child("images").children() : conf.children("image");
    for( ConfigSet::const_iterator i = images.begin(); i != images.end(); ++i )
    {
        add( ImageLayerOptions( *i ) );
    }

    const ConfigSet& elevations = conf.hasChild("elevations") ? conf.child("elevations").children() : conf.children("elevation");
    for( ConfigSet::const_iterator i = elevations.begin(); i != elevations.end(); ++i )
    {
        add( ElevationLayerOptions( *i ) );
    }
    
    const ConfigSet& heightfields = conf.hasChild("heightfields") ? conf.child("heightfields").children() : conf.children("heightfield");
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
            mayHaveDataForKey = false;
        }

        ImageInfo(osg::Image* image, float opacity, bool mayHaveDataForKey)
        {
            this->image = image;
            this->opacity = opacity;
            this->mayHaveDataForKey = mayHaveDataForKey;
        }

        bool mayHaveDataForKey;
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
        imageInfo.mayHaveDataForKey = layer->mayHaveData(key);
        imageInfo.opacity = layer->getOpacity();

        if (imageInfo.mayHaveDataForKey)
        {
            GeoImage image = layer->createImage(key, progress);
            if (image.valid())
            {
                imageInfo.image = image.getImage();
            }

            // If the progress got cancelled (due to any reason, including network error)
            // then return NULL to prevent this tile from being built and cached with
            // incomplete or partial data.
            if (progress && progress->isCanceled())
            {
                OE_DEBUG << LC << " createImage was cancelled or needs retry for " << key.str() << std::endl;
                return 0L;
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

            // If we didn't get any data for the tilekey, but the extents do overlap,
            // we will try to fall back on lower LODs and get data there instead:
            if (!info.image.valid() && layer->getDataExtentsUnion().intersects(key.getExtent()))
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

                    // If the progress got cancelled or it needs a retry then return NULL to prevent this tile from being built and cached with incomplete or partial data.
                    if (progress && progress->isCanceled())
                    {
                        OE_DEBUG << LC << " createImage was cancelled or needs retry for " << key.str() << std::endl;
                        return 0L;
                    }

                    parentKey = parentKey.createParentKey();
                }

                if (image.valid())
                {                                        
                    // TODO:  Bilinear options?
                    bool bilinear = layer->isCoverage() ? false : true;
                    GeoImage cropped = image.crop( key.getExtent(), true, textureSize.x(), textureSize.y(), bilinear);
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
                    ImageUtils::mix( result, imageInfo.image.get(), imageInfo.opacity );
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
    unsigned size = getPixelsPerTile(); //int size = *getOptions().tileSize();    
    osg::ref_ptr< osg::HeightField > heightField = new osg::HeightField();
    heightField->allocate(size, size);

    // Initialize the heightfield to nodata
    for (unsigned int i = 0; i < heightField->getFloatArray()->size(); i++)
    {
        heightField->getFloatArray()->at( i ) = NO_DATA_VALUE;
    }  

    // Populate the heightfield and return it if it's valid
    if (_elevationLayers.populateHeightFieldAndNormalMap(heightField.get(), 0L, key, 0, INTERP_BILINEAR, progress))
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
    comp._imageLayerOptions = layer->options();
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
    comp._elevationLayerOptions = layer->options();
    _options._components.push_back( comp );    

    return true;
}


Status
CompositeTileSource::initialize(const osgDB::Options* dbOptions)
{
    _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

    osg::ref_ptr<const Profile> profile = getProfile();

    bool dataExtentsValid = true;

    for(CompositeTileSourceOptions::ComponentVector::iterator i = _options._components.begin();
        i != _options._components.end(); )
    {        
        if ( i->_imageLayerOptions.isSet() && !i->_layer.valid() )
        {
            // Disable the l2 cache for composite layers so that we don't get run out of memory on very large datasets.
            i->_imageLayerOptions->driver()->L2CacheSize() = 0;

            osg::ref_ptr< ImageLayer > layer = new ImageLayer(*i->_imageLayerOptions);
            layer->setReadOptions(_dbOptions.get());
            Status status = layer->open();
            if (status.isOK())
            {
                i->_layer = layer.get();
                _imageLayers.push_back( layer.get() );
                OE_INFO << LC << "Added image layer " << layer->getName() << " (" << i->_imageLayerOptions->driver()->getDriver() << ")\n";
            }
            else
            {
                setStatus(Status(Status::ResourceUnavailable, Stringify()
                    << "Could not open sublayer (" << layer->getName() << ") ... " << status.message()));
                return getStatus();
            }            
        }
        else if (i->_elevationLayerOptions.isSet() && !i->_layer.valid())
        {
            // Disable the l2 cache for composite layers so that we don't get run out of memory on very large datasets.
            i->_elevationLayerOptions->driver()->L2CacheSize() = 0;

            osg::ref_ptr< ElevationLayer > layer = new ElevationLayer(*i->_elevationLayerOptions);   
            layer->setReadOptions(_dbOptions.get());
            Status status = layer->open();
            if (status.isOK())
            {
                i->_layer = layer;
                _elevationLayers.push_back( layer.get() );   
                OE_INFO << LC << "Added elevation layer " << layer->getName() << " (" << i->_elevationLayerOptions->driver()->getDriver() << ")\n";             
            }
            else
            {
                setStatus(Status(Status::ResourceUnavailable, Stringify()
                    << "Could not open sublayer (" << layer->getName() << ") ... " << status.message()));
                return getStatus();
            }
        }

        if ( !i->_layer.valid() )
        {
            OE_DEBUG << LC << "A component has no valid TerrainLayer ... removing." << std::endl;            
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

            // If even one of the layers' data extents is unknown, the entire composite
            // must have unknown data extents:
            if (extents.empty())
            {
                dataExtentsValid = false;
                getDataExtents().clear();
            }

            if (dataExtentsValid)
            {
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
    }

    // If there is no profile set by the user or by a component, fall back
    // on a default profile. This will allow the Layer to continue to operate
    // off the cache even if all components fail to initialize for some reason.
    if (profile.valid() == false)
    {
        profile = Profile::create("global-geodetic");
    }

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

        const char* className() const //override
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
