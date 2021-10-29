/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
CompositeImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for( std::vector<ConfigOptions>::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        {
            layersConf.add(i->getConfig());
        }
        conf.set(layersConf);
    }
    conf.set("composite_function", "blend", function(), FUNCTION_BLEND);
    conf.set("composite_function", "less", function(), FUNCTION_LESS);
    conf.set("composite_function", "greater", function(), FUNCTION_GREATER);
    conf.set("composite_function", "more", function(), FUNCTION_GREATER);
    return conf;
}

void
CompositeImageLayer::Options::fromConfig(const Config& conf)
{
    function().setDefault(FUNCTION_BLEND);

    const ConfigSet& layers = conf.child("layers").children();
    for( ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i )
    {
        _layers.push_back(ConfigOptions(*i));
    }
    conf.get("composite_function", "blend", function(), FUNCTION_BLEND);
    conf.get("composite_function", "less", function(), FUNCTION_LESS);
    conf.get("composite_function", "greater", function(), FUNCTION_GREATER);
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
        }

        TileKey bestAvailableKey;
        //bool mayHaveData;
        float opacity;
        osg::ref_ptr<const osg::Image> image;
    };

    // some helper types.    
    typedef std::vector<ImageInfo> ImageMixVector;   
} }

REGISTER_OSGEARTH_LAYER(compositeimage, CompositeImageLayer);

void
CompositeImageLayer::addLayer(ImageLayer* layer)
{
    if (isOpen())
    {
        OE_WARN << LC << "Illegal call to addLayer when layer is already open" << std::endl;
    }
    else if (layer)
    {
        _layers.push_back(layer);
    }
}

const ImageLayerVector&
CompositeImageLayer::getLayers() const
{
    return _layers;
}

void
CompositeImageLayer::init()
{
    ImageLayer::init();
}

osg::Node*
CompositeImageLayer::getNode() const
{
    return _layerNodes.get();
}

void
CompositeImageLayer::addedToMap(const Map* map)
{
    for(ImageLayerVector::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        i->get()->addedToMap(map);
    }
}

void
CompositeImageLayer::removedFromMap(const Map* map)
{
    for(ImageLayerVector::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        i->get()->removedFromMap(map);
    }
}

Status
CompositeImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // If we're in cache-only mode, do not attempt to open the component layers!
    if (getCacheSettings()->cachePolicy()->isCacheOnly())
        return Status::NoError;

    osg::ref_ptr<const Profile> profile = getProfile();

    bool dataExtentsValid = true;

    // If the user didn't call addLayer(), try to read them from the options.
    if (_layers.empty())
    {
        for(std::vector<ConfigOptions>::const_iterator i = options().layers().begin();
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
                OE_WARN << LC << "This composite can only contains ImageLayers; discarding a layer" << std::endl;
            }
        }
    }

    else
    {
        // the user added layers through the API, so store each layer's options
        // for serialization
        for(ImageLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            ImageLayer* layer = i->get();
            options().layers().push_back(layer->getConfig());
        }
    }

    for(ImageLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        ImageLayer* layer = i->get();

        layer->setReadOptions(getReadOptions());

        Status status = layer->open();

        if (status.isOK())
        {
            OE_DEBUG << LC << "...opened " << layer->getName() << " OK" << std::endl;

            // If no profile is specified assume they want to use the profile of the first layer in the list
            if (!profile.valid())
            {
                profile = layer->getProfile();
                if (!profile.valid())
                {
                    return Status(
                        Status::ResourceUnavailable,
                        Stringify() << "Cannot establish profile for layer " << layer->getName());
                }
                else if (profile->getSRS()->getVerticalDatum() != NULL)
                {
                    // Remove the VDATUM! Otherwise we will process it twice,
                    // once in the component layer and again in the composite.
                    ProfileOptions po = profile->toProfileOptions();
                    po.vsrsString().unset();
                    profile = Profile::create(po);
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

            // If the sublayer has a Node, add it to the group.
            if (layer->getNode())
            {
                if (!_layerNodes.valid())
                    _layerNodes = new osg::Group();

                _layerNodes->addChild(layer->getNode());
            }
        }

        else
        {
            OE_WARN << LC << "...failed to open " << layer->getName() << ": " << status.message() << std::endl;
            if (getCacheSettings()->isCacheEnabled())
            {
                OE_WARN << LC << "...cache writes will be DISABLED for this layer" << std::endl;
                getCacheSettings()->integrateCachePolicy(CachePolicy(CachePolicy::USAGE_READ_ONLY));
            }
        }
    }

    if (!getProfile())
    {
        // If there is no profile set by the user or by a component, fall back
        // on a default profile. This will allow the Layer to continue to operate
        // off the cache even if all components fail to initialize for some reason.
        if (profile.valid() == false)
        {
            if (getCacheSettings()->isCacheEnabled())
            {
                profile = Profile::create(Profile::GLOBAL_GEODETIC);
            }
            else
            {
                return Status(Status::ResourceUnavailable, "Unable to open any component layers");
            }
        }

        setProfile(profile.get());
    }

    return Status::NoError;
}

Status
CompositeImageLayer::closeImplementation()
{
    for(ImageLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        ImageLayer* layer = i->get();
        layer->close();
    }

    if (_layerNodes.valid())
    {
        _layerNodes->removeChildren(0, _layerNodes->getNumChildren());
    }

    dataExtents().clear();
    return Status::OK();
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
        imageInfo.opacity = layer->getOpacity();
        imageInfo.bestAvailableKey = layer->getBestAvailableTileKey(key);
        
        // if there is possibly actual data for this key...
        if (imageInfo.bestAvailableKey == key)
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

    // Determine the output texture size to use based on the image that were created.
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
            if (info.image.valid() == false && info.bestAvailableKey.valid())
            {
                TileKey currentKey = info.bestAvailableKey; //key.createParentKey();

                GeoImage image;
                while (!image.valid() && currentKey.valid())
                {
                    image = layer->createImage(currentKey, progress);
                    if (image.valid())
                    {
                        break;
                    }

                    // If the progress got cancelled or it needs a retry then return INVALID
                    // to prevent this tile from being built and cached with incomplete or partial data.
                    if (progress && progress->isCanceled())
                    {
                        OE_DEBUG << LC << " createImage was cancelled or needs retry for " << key.str() << std::endl;
                        return GeoImage::INVALID;
                    }

                    currentKey = currentKey.createParentKey();
                }

                if (image.valid())
                {
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
                    if (options().function() == options().FUNCTION_BLEND)
                    {
                        ImageUtils::mix(result, imageInfo.image.get(), imageInfo.opacity);
                    }
                    else
                    {
                        ImageUtils::PixelReader readOne(result);
                        ImageUtils::PixelReader readTwo(imageInfo.image.get());
                        ImageUtils::PixelWriter writeOne(result);
                        osg::Vec4 pixelOne, pixelTwo;

                        std::function<bool(float, float)> compare;
                        if (options().function() == options().FUNCTION_LESS)
                            compare = [](float a, float b) { return a < b; };
                        else // FUNCTION_MORE
                            compare = [](float a, float b) { return a > b; };
                            
                        ImageUtils::ImageIterator iter(readOne);
                        iter.forEachPixel([&]()
                            {
                                readOne(pixelOne, iter.s(), iter.t());
                                readTwo(pixelTwo, iter.s(), iter.t());
                                if (compare(pixelTwo.r(), pixelOne.r()))
                                    writeOne(pixelTwo, iter.s(), iter.t());
                            });
                    }
                }
            }            
        }        
        return GeoImage(result, key.getExtent());
    }
}

//........................................................................

Config
CompositeElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for( std::vector<ConfigOptions>::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        {
            layersConf.add(i->getConfig());
        }
        conf.set(layersConf);
    }
    return conf;
}

void
CompositeElevationLayer::Options::fromConfig(const Config& conf)
{
    const ConfigSet& layers = conf.child("layers").children();
    for( ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i )
    {
        _layers.push_back(ConfigOptions(*i));
    }
}


REGISTER_OSGEARTH_LAYER(compositeelevation, CompositeElevationLayer);

void
CompositeElevationLayer::addLayer(ElevationLayer* layer)
{
    if (isOpen())
    {
        OE_WARN << LC << "Illegal call to addLayer when layer is already open" << std::endl;
    }
    else if (layer)
    {
        _layers.push_back(layer);
    }
}

const ElevationLayerVector&
CompositeElevationLayer::getLayers() const
{
    return _layers;
}

void
CompositeElevationLayer::init()
{
    ElevationLayer::init();
}

osg::Node*
CompositeElevationLayer::getNode() const
{
    return _layerNodes.get();
}

void
CompositeElevationLayer::addedToMap(const Map* map)
{
    for(ElevationLayerVector::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        i->get()->addedToMap(map);
    }
}

void
CompositeElevationLayer::removedFromMap(const Map* map)
{
    for(ElevationLayerVector::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        i->get()->removedFromMap(map);
    }
}

Status
CompositeElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;    
    
    // If we're in cache-only mode, do not attempt to open the component layers!
    if (getCacheSettings() && getCacheSettings()->cachePolicy()->isCacheOnly())
        return Status::NoError;

    osg::ref_ptr<const Profile> profile;

    bool dataExtentsValid = true;

    // If the user didn't call addLayer(), try to read them from the options.
    if (_layers.empty())
    {
        for(std::vector<ConfigOptions>::const_iterator i = options().layers().begin();
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
                OE_WARN << LC << "This composite can only contains ElevationLayers; discarding a layer" << std::endl;
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

        layer->setReadOptions(getReadOptions());

        Status status = layer->open();

        if (status.isOK())
        {
            OE_DEBUG << LC << "...opened " << layer->getName() << " OK" << std::endl;

            // If no profile is specified assume they want to use the profile of the first layer in the list.
            if (!profile.valid())
            {
                profile = layer->getProfile();
                if (!profile.valid())
                {
                    return Status(Status::ResourceUnavailable, 
                        Stringify()<<"Cannot establish profile for layer " << layer->getName());
                }
                else if (profile->getSRS()->getVerticalDatum() != NULL)
                {
                    // Remove the VDATUM! Otherwise we will process it twice,
                    // once in the component layer and again in the composite.
                    ProfileOptions po = profile->toProfileOptions();
                    po.vsrsString().unset();
                    profile = Profile::create(po);
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

            // If the sublayer has a Node, add it to the group.
            if (layer->getNode())
            {
                if (!_layerNodes.valid())
                    _layerNodes = new osg::Group();

                _layerNodes->addChild(layer->getNode());
            }
        }

        else
        {
            OE_WARN << LC << "...failed to open " << layer->getName() << ": " << status.message() << std::endl;
            if (getCacheSettings()->isCacheEnabled())
            {
                OE_WARN << LC << "...cache writes will be DISABLED for this layer" << std::endl;
                getCacheSettings()->integrateCachePolicy(CachePolicy(CachePolicy::USAGE_READ_ONLY));
            }
        }
    }

    // If there is no profile set by the user or by a component, fall back
    // on a default profile. This will allow the Layer to continue to operate
    // off the cache even if all components fail to initialize for some reason.
    if (profile.valid() == false)
    {
        if (getCacheSettings()->isCacheEnabled())
        {
            profile = Profile::create(Profile::GLOBAL_GEODETIC);
        }
        else
        {
            return Status(Status::ResourceUnavailable, "Unable to open any component layers");
        }
    }

    setProfile( profile.get() );

    return Status::NoError;
}

Status
CompositeElevationLayer::closeImplementation()
{
    for(ElevationLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        ElevationLayer* layer = i->get();
        layer->close();
    }

    if (_layerNodes.valid())
    {
        _layerNodes->removeChildren(0, _layerNodes->getNumChildren());
    }

    dataExtents().clear();
    return Status::OK();
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
    if (_layers.populateHeightField(heightField.get(), NULL, key, 0, INTERP_BILINEAR, progress))
    {                
        return GeoHeightField(heightField.release(), key.getExtent());
    }
    else
    {        
        return GeoHeightField::INVALID;
    }
}


#undef  LC
#define LC "[CompositeLandCoverLayer] "

//........................................................................

Config
CompositeLandCoverLayer::Options::getConfig() const
{
    Config conf = LandCoverLayer::Options::getConfig();
    if (_layers.empty() == false)
    {
        Config layersConf("layers");
        for( std::vector<ConfigOptions>::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        {
            layersConf.add(i->getConfig());
        }
        conf.set(layersConf);
    }
    return conf;
}

void
CompositeLandCoverLayer::Options::fromConfig(const Config& conf)
{
    const ConfigSet& layers = conf.child("layers").children();
    for( ConfigSet::const_iterator i = layers.begin(); i != layers.end(); ++i )
    {
        _layers.push_back(ConfigOptions(*i));
    }
}

//........................................................................

REGISTER_OSGEARTH_LAYER(compositelandcover, CompositeLandCoverLayer);

void
CompositeLandCoverLayer::addLayer(LandCoverLayer* layer)
{
    if (isOpen())
    {
        OE_WARN << LC << "Illegal call to addLayer when layer is already open" << std::endl;
    }
    else if (layer)
    {
        _layers.push_back(layer);
    }
}

const LandCoverLayerVector&
CompositeLandCoverLayer::getLayers() const
{
    return _layers;
}

void
CompositeLandCoverLayer::init()
{
    LandCoverLayer::init();
}

osg::Node*
CompositeLandCoverLayer::getNode() const
{
    return _layerNodes.get();
}

void
CompositeLandCoverLayer::addedToMap(const Map* map)
{
    for(LandCoverLayerVector::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        i->get()->addedToMap(map);
    }
}

void
CompositeLandCoverLayer::removedFromMap(const Map* map)
{
    for(LandCoverLayerVector::iterator i = _layers.begin();
        i != _layers.end();
        ++i)
    {
        i->get()->removedFromMap(map);
    }
}

Status
CompositeLandCoverLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation(); // skipping LandCoverLayer::open...
    if (parent.isError())
        return parent;

    // If we're in cache-only mode, do not attempt to open the component layers!
    if (getCacheSettings()->cachePolicy()->isCacheOnly())
        return Status::NoError;

    osg::ref_ptr<const Profile> profile;

    bool dataExtentsValid = true;

    // You may not call addLayers() and also put layers in the options.
    if (_layers.empty() == false && options().layers().empty() == false)
    {
        return Status(Status::ConfigurationError, 
            "Illegal to add layers both by options and by API");
    }

    // If the user didn't call addLayer(), try to read them from the options.
    if (_layers.empty())
    {
        for(std::vector<ConfigOptions>::const_iterator i = options().layers().begin();
            i != options().layers().end();
            ++i)
        {
            osg::ref_ptr<Layer> newLayer = Layer::create(*i);
            LandCoverLayer* layer = dynamic_cast<LandCoverLayer*>(newLayer.get());
            if (layer)
            {
                // Add to the list
                _layers.push_back(layer);
            }
            else
            {
                OE_WARN << LC << "This composite can only contains LandCoverLayers; discarding a layer" << std::endl;
            }
        }
    }

    else
    {
        // the user added layers through the API, so store each layer's options
        // for serialization
        for(LandCoverLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            LandCoverLayer* layer = i->get();
            options().layers().push_back(layer->getConfig());
        }
    }

    for(LandCoverLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        LandCoverLayer* layer = i->get();

        layer->setReadOptions(getReadOptions());

        Status status = layer->open();

        if (status.isOK())
        {
            OE_DEBUG << LC << "...opened " << layer->getName() << " OK" << std::endl;

            // If no profile is specified assume they want to use the profile of the first layer in the list.
            if (!profile.valid())
            {
                profile = layer->getProfile();
                if (!profile.valid())
                {
                    return Status(
                        Status::ResourceUnavailable, 
                        Stringify()<<"Cannot establish profile for layer " << layer->getName());
                }
                else if (profile->getSRS()->getVerticalDatum() != NULL)
                {
                    // Remove the VDATUM! Otherwise we will process it twice,
                    // once in the component layer and again in the composite.
                    ProfileOptions po = profile->toProfileOptions();
                    po.vsrsString().unset();
                    profile = Profile::create(po);
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

            // If the sublayer has a Node, add it to the group.
            if (layer->getNode())
            {
                if (!_layerNodes.valid())
                    _layerNodes = new osg::Group();

                _layerNodes->addChild(layer->getNode());
            }
        }

        else
        {
            OE_WARN << LC << "...failed to open " << layer->getName() << ": " << status.message() << std::endl;
            if (getCacheSettings()->isCacheEnabled())
            {
                OE_WARN << LC << "...cache writes will be DISABLED for this layer" << std::endl;
                getCacheSettings()->integrateCachePolicy(CachePolicy(CachePolicy::USAGE_READ_ONLY));
            }
        }
    }

    // If there is no profile set by the user or by a component, fall back
    // on a default profile. This will allow the Layer to continue to operate
    // off the cache even if all components fail to initialize for some reason.
    if (profile.valid() == false)
    {
        if (getCacheSettings()->isCacheEnabled())
        {
            profile = Profile::create(Profile::GLOBAL_GEODETIC);
        }
        else
        {
            return Status(Status::ResourceUnavailable, "Unable to open any component layers");
        }
    }

    setProfile( profile.get() );

    return Status::NoError;
}

Status
CompositeLandCoverLayer::closeImplementation()
{
    for(LandCoverLayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
    {
        LandCoverLayer* layer = i->get();
        layer->close();
    }

    if (_layerNodes.valid())
    {
        _layerNodes->removeChildren(0, _layerNodes->getNumChildren());
    }

    dataExtents().clear();
    return Status::OK();
}

GeoImage
CompositeLandCoverLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    unsigned size = getTileSize();

    osg::ref_ptr<osg::Image> image;

    // Populate the heightfield and return it if it's valid
    if (_layers.populateLandCoverImage(image, key, progress))
    {                
        return GeoImage(image.get(), key.getExtent());
    }
    else
    {        
        return GeoImage::INVALID;
    }
}
