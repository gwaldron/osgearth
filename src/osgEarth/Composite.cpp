/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/Composite>
#include <osgEarth/Progress>

using namespace osgEarth;

#undef  LC
#define LC "[CompositeImageLayer] "

//........................................................................

Config
CompositeImageLayerOptions::getConfig() const
{
    Config conf = ImageLayerOptions::getConfig();
    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for( Layers::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        {
            layersConf.add(i->getConfig());
        }
        conf.set(layersConf);
    }
    return conf;
}

void
CompositeImageLayerOptions::fromConfig(const Config& conf)
{
    const ConfigSet& layers = conf.child("layers").children();
    for( ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i )
    {
        _layers.push_back(ConfigOptions(*i));
    }
}

//........................................................................

namespace osgEarth { namespace Composite
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
} }

REGISTER_OSGEARTH_LAYER(compositeimage, CompositeImageLayer);

void
CompositeImageLayer::addLayer(ImageLayer* layer)
{
    if (_open)
    {
        OE_WARN << LC << "Illegal call to addLayer when layer is already open" << std::endl;
    }
    else if (layer)
    {
        _layers.push_back(layer);
    }
}

void
CompositeImageLayer::init()
{
    ImageLayer::init();
    _open = false;
    setTileSourceExpected(false);
}

const Status&
CompositeImageLayer::open()
{
    _open = true;

    osg::ref_ptr<const Profile> profile;

    bool dataExtentsValid = true;

    // You may not call addLayers() and also put layers in the options.
    if (_layers.empty() == false && options().layers().empty() == false)
    {
        return setStatus(Status(Status::ConfigurationError, 
            "Illegal to add layers both by options and by API"));
    }

    // If the user didn't call addLayer(), try to read them from the options.
    if (_layers.empty())
    {
        for(CompositeImageLayerOptions::Layers::const_iterator i = options().layers().begin();
            i != options().layers().end();
            ++i)
        {
            osg::ref_ptr<Layer> newLayer = Layer::create(*i);
            ImageLayer* layer = dynamic_cast<ImageLayer*>(newLayer.get());
            if (layer)
            {
                // Add to the list
                _layers.push_back(layer);
            }
            else
            {
                OE_WARN << LC << "This composite can only contains ImageLayers; discarding layer " << layer->getName() << std::endl;
            }
        }
    }

    // otherwise, we need to add these layers to the options in case
    // someone calls getConfig.
    else
    {
        for(ImageLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            ImageLayer* layer = i->get();
            options().layers().push_back(layer->getConfig());
        }
    }

    for(ImageLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        ImageLayer* layer = i->get();

        // TODO: disable the L2 cache...used to be in the tile source
        // TODO: replacement for "dynamic" tile sources

        layer->setReadOptions(getReadOptions());

        Status status = layer->open();

        if (status.isError())
        {
            return setStatus(status);
        }

        OE_INFO << LC << "...opened " << layer->getName() << " OK" << std::endl;

        // If no profile is specified assume they want to use the profile of the first layer in the list.
        if (!profile.valid())
        {
            profile = layer->getProfile();
            if (!profile.valid())
            {
                return setStatus(Status(Status::ResourceUnavailable, 
                    Stringify()<<"Cannot establish profile for layer " << layer->getName()));
            }
        }

        // gather extents                        
        const DataExtentList& extents = layer->getDataExtents();  

        // If even one of the layers' data extents is unknown, the entire composite
        // must have unknown data extents:
        if (extents.empty())
        {
            dataExtentsValid = false;
            dataExtents().clear();
        }

        if (dataExtentsValid)
        {
            for( DataExtentList::const_iterator j = extents.begin(); j != extents.end(); ++j )
            {                
                // Convert the data extent to the profile that is actually used by this TileSource
                DataExtent dataExtent = *j;                
                GeoExtent ext = dataExtent.transform(profile->getSRS());
                unsigned int minLevel = 0;
                unsigned int maxLevel = profile->getEquivalentLOD(layer->getProfile(), *dataExtent.maxLevel() );                                        
                dataExtent = DataExtent(ext, minLevel, maxLevel);                                
                dataExtents().push_back( dataExtent );
            }
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

    return ImageLayer::open();
}

GeoImage
CompositeImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    Composite::ImageMixVector images;
    images.reserve(_layers.size());

    // Try to get an image from each of the layers for the given key.
    for (ImageLayerVector::const_iterator itr = _layers.begin(); itr != _layers.end(); ++itr)
    {
        ImageLayer* layer = itr->get();
        Composite::ImageInfo imageInfo;
        imageInfo.dataInExtents = layer->mayHaveData(key);
        imageInfo.opacity = layer->getOpacity();

        if (imageInfo.dataInExtents)
        {
            GeoImage image = layer->createImage(key, progress);
            if (image.valid())
            {
                imageInfo.image = image.getImage();
            }

            // If the progress got cancelled or it needs a retry then return NULL to prevent this tile from being built and cached with incomplete or partial data.
            if (progress && progress->isCanceled())
            {
                OE_DEBUG << LC << " createImage was cancelled or needs retry for " << key.str() << std::endl;
                return GeoImage::INVALID;
            }
        }

        images.push_back(imageInfo);
    }

    // Determine the output texture size to use based on the image that were creatd.
    unsigned numValidImages = 0;
    osg::Vec2s textureSize;
    for (unsigned int i = 0; i < images.size(); i++)
    {
        Composite::ImageInfo& info = images[i];
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
            Composite::ImageInfo& info = images[i];
            ImageLayer* layer = _layers[i].get();
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

                    // If the progress got cancelled or it needs a retry then return NULL to prevent this tile from being built and cached with incomplete or partial data.
                    if (progress && progress->isCanceled())
                    {
                        OE_DEBUG << LC << " createImage was cancelled or needs retry for " << key.str() << std::endl;
                        return GeoImage::INVALID;
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
        Composite::ImageInfo& info = images[i];
        if (info.image.valid())
            numValidImages++;        
    }    

    if ( progress && progress->isCanceled() )
    {
        return GeoImage::INVALID;
    }

    else if ( numValidImages == 0 )
    {
        return GeoImage::INVALID;
    }

    else if ( numValidImages == 1 )
    {
        //We only have one valid image, so just return it and don't bother with compositing
        for (unsigned int i = 0; i < images.size(); i++)
        {
            Composite::ImageInfo& info = images[i];
            if (info.image.valid())
            {
                return GeoImage(info.image.release(), key.getExtent());
            }
        }
        return GeoImage::INVALID;
    }

    else
    {
        osg::Image* result = 0;
        for (unsigned int i = 0; i < images.size(); i++)
        {
            Composite::ImageInfo& imageInfo = images[i];
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
        return GeoImage(result, key.getExtent());
    }
}

//........................................................................

Config
CompositeElevationLayerOptions::getConfig() const
{
    Config conf = ElevationLayerOptions::getConfig();
    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for( Layers::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        {
            layersConf.add(i->getConfig());
        }
        conf.set(layersConf);
    }
    return conf;
}

void
CompositeElevationLayerOptions::fromConfig(const Config& conf)
{
    const ConfigSet& layers = conf.child("layers").children();
    for( ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i )
    {
        _layers.push_back(ConfigOptions(*i));
    }
}



namespace osgEarth { namespace Composite
{
    struct HeightFieldInfo
    {
        HeightFieldInfo()
        {
            dataInExtents = false;
        }

        HeightFieldInfo(osg::HeightField* hf, bool dataInExtents)
        {
            this->hf = hf;
            this->dataInExtents = dataInExtents;
        }

        bool dataInExtents;
        osg::ref_ptr< osg::HeightField> hf;
    };

    // some helper types.    
    typedef std::vector<HeightFieldInfo> HeightFieldMixVector;   
} }

REGISTER_OSGEARTH_LAYER(compositeelevation, CompositeElevationLayer);

void
CompositeElevationLayer::addLayer(ElevationLayer* layer)
{
    if (_open)
    {
        OE_WARN << LC << "Illegal call to addLayer when layer is already open" << std::endl;
    }
    else if (layer)
    {
        _layers.push_back(layer);
    }
}

void
CompositeElevationLayer::init()
{
    ElevationLayer::init();
    _open = false;
    setTileSourceExpected(false);
}

const Status&
CompositeElevationLayer::open()
{
    _open = true;

    osg::ref_ptr<const Profile> profile;

    bool dataExtentsValid = true;

    // You may not call addLayers() and also put layers in the options.
    if (_layers.empty() == false && options().layers().empty() == false)
    {
        return setStatus(Status(Status::ConfigurationError, 
            "Illegal to add layers both by options and by API"));
    }

    // If the user didn't call addLayer(), try to read them from the options.
    if (_layers.empty())
    {
        for(CompositeElevationLayerOptions::Layers::const_iterator i = options().layers().begin();
            i != options().layers().end();
            ++i)
        {
            osg::ref_ptr<Layer> newLayer = Layer::create(*i);
            ElevationLayer* layer = dynamic_cast<ElevationLayer*>(newLayer.get());
            if (layer)
            {
                // Add to the list
                _layers.push_back(layer);
            }
            else
            {
                OE_WARN << LC << "This composite can only contains ElevationLayers; discarding layer " << layer->getName() << std::endl;
            }
        }
    }

    // otherwise, we need to add these layers to the options in case
    // someone calls getConfig.
    else
    {
        for(ElevationLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            ElevationLayer* layer = i->get();
            options().layers().push_back(layer->getConfig());
        }
    }

    for(ElevationLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        ElevationLayer* layer = i->get();

        // TODO: disable the L2 cache...used to be in the tile source
        // TODO: replacement for "dynamic" tile sources

        layer->setReadOptions(getReadOptions());

        Status status = layer->open();

        if (status.isError())
        {
            return setStatus(status);
        }

        OE_INFO << LC << "...opened " << layer->getName() << " OK" << std::endl;

        // If no profile is specified assume they want to use the profile of the first layer in the list.
        if (!profile.valid())
        {
            profile = layer->getProfile();
            if (!profile.valid())
            {
                return setStatus(Status(Status::ResourceUnavailable, 
                    Stringify()<<"Cannot establish profile for layer " << layer->getName()));
            }
        }

        // gather extents                        
        const DataExtentList& extents = layer->getDataExtents();  

        // If even one of the layers' data extents is unknown, the entire composite
        // must have unknown data extents:
        if (extents.empty())
        {
            dataExtentsValid = false;
            dataExtents().clear();
        }

        if (dataExtentsValid)
        {
            for( DataExtentList::const_iterator j = extents.begin(); j != extents.end(); ++j )
            {                
                // Convert the data extent to the profile that is actually used by this TileSource
                DataExtent dataExtent = *j;                
                GeoExtent ext = dataExtent.transform(profile->getSRS());
                unsigned int minLevel = 0;
                unsigned int maxLevel = profile->getEquivalentLOD(layer->getProfile(), *dataExtent.maxLevel() );                                        
                dataExtent = DataExtent(ext, minLevel, maxLevel);                                
                dataExtents().push_back( dataExtent );
            }
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

    return ElevationLayer::open();
}

GeoHeightField
CompositeElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    unsigned size = getTileSize();
    osg::ref_ptr< osg::HeightField > heightField = new osg::HeightField();
    heightField->allocate(size, size);

    // Initialize the heightfield to nodata
    heightField->getFloatArray()->assign(size*size, NO_DATA_VALUE);

    // Populate the heightfield and return it if it's valid
    if (_layers.populateHeightFieldAndNormalMap(heightField.get(), 0L, key, 0, INTERP_BILINEAR, progress))
    {                
        return GeoHeightField(heightField.release(), key.getExtent());
    }
    else
    {        
        return GeoHeightField::INVALID;
    }
}
