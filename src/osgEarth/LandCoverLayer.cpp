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
#include <osgEarth/LandCoverLayer>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/SimplexNoise>
#include <osgEarth/Progress>
#include <osgEarth/Random>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[LandCoverLayer] "

REGISTER_OSGEARTH_LAYER(landcover, LandCoverLayer);

//........................................................................

#undef  LC
#define LC "[LandCoverLayerOptions] "

void
LandCoverLayer::Options::fromConfig(const Config& conf)
{
    source().get(conf, "source");

    ConfigSet mappingsConf = conf.child("land_cover_mappings").children("mapping");
    for (ConfigSet::const_iterator i = mappingsConf.begin(); i != mappingsConf.end(); ++i)
    {
        osg::ref_ptr<LandCoverValueMapping> mapping = new LandCoverValueMapping(*i);
        mappings().push_back(mapping.get());
    }
}

Config
LandCoverLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();

    source().set(conf, "source");

    if (conf.hasChild("land_cover_mappings") == false)
    {   
        Config mappingConf("land_cover_mappings");
        conf.add(mappingConf);
        for(LandCoverValueMappingVector::const_iterator i = _mappings.begin();
            i != _mappings.end();
            ++i)
        {
            LandCoverValueMapping* mapping = i->get();
            if (mapping)
                mappingConf.add(mapping->getConfig());
        }
    }
    return conf;
}

//...........................................................................

#undef  LC
#define LC "[LandCoverLayer] "

void
LandCoverLayer::init()
{
    options().coverage() = true;

    ImageLayer::init();

    setRenderType(RENDERTYPE_NONE);

    layerHints().L2CacheSize() = 64;

    _waterCode = -1;
    _beachCode = -1;
}

Config
LandCoverLayer::getConfig() const
{
    Config c = ImageLayer::getConfig();
    return c;
}

void
LandCoverLayer::setSource(ImageLayer* value)
{
    options().source().setLayer(value);
}

ImageLayer*
LandCoverLayer::getSource() const
{
    return options().source().getLayer();
}

LandCoverValueMappingVector&
LandCoverLayer::getLandCoverValueMappings()
{
    return options().mappings();
}

const LandCoverValueMappingVector&
LandCoverLayer::getLandCoverValueMappings() const
{
    return options().mappings();
}

void
LandCoverLayer::map(int value, const std::string& classname)
{
    options().mappings().push_back(new LandCoverValueMapping(value, classname));
}

Status
LandCoverLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    const Profile* profile = getProfile();
    if (!profile)
    {
        profile = Profile::create(Profile::GLOBAL_GEODETIC);
        setProfile(profile);
    }

    // We never want to cache data from a coverage, because the "parent" layer
    // will be caching the entire result of a multi-coverage composite.
    options().sourceEmbeddedOptions()->cachePolicy() = CachePolicy::NO_CACHE;

    // Try to open it.
    Status cs = options().source().open(getReadOptions());
    if (cs.isError())
        return cs;

    // Pull this layer's extents from the coverage layer.
    ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(options().source().getLayer());
    if (!imageLayer)
        return Status(Status::ResourceUnavailable, "Cannot access source image layer");

    // Normally we would collect and store the layer's DataExtents here.
    // Since this is possibly a composited layer with warping, we just
    // let it default so we can oversample the data with warping.
    // TODO: review this statement
    //dataExtents() = getSource()->getDataExtents();

    // TODO: review this since we are setting a cache on this layer itself
    // via the layerHints()
    
    // GW: do we really need this? Probably not
    getSource()->setUpL2Cache(9u);

    // Force the image source into coverage mode.
    getSource()->setCoverage(true);

    return Status::NoError;
}

void
LandCoverLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    // Find a land cover dictionary if there is one.
    // There had better be one, or we are not going to get very far!
    // Note. If the land cover dictionary isn't already in the Map...this will fail! (TODO)
    // Consider a LayerReference. (TODO)
    _lcDictionary = map->getLayer<LandCoverDictionary>();

    if (_lcDictionary.valid())
    {
        if (getSource())
        {
            getSource()->addedToMap(map);
            buildCodeMap(_codemap);
        }

#if 0
        const LandCoverClass* water = _lcDictionary->getClassByName("water");
        if (water) _waterCode = water->getValue();
        const LandCoverClass* beach = _lcDictionary->getClassByName("desert");
        if (beach) _beachCode = beach->getValue();
#endif
    }
    else
    {
        OE_WARN << LC << "Did not find a LandCoverDictionary in the Map!" << std::endl;
    }
}

void
LandCoverLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);
    options().source().removedFromMap(map);
}

bool
LandCoverLayer::readMetaImage(MetaImage& metaImage, const TileKey& key, int s, int t, osg::Vec4& output, ProgressCallback* progress) const
{
    int tilesize = (int)getTileSize();

    int dx = s<0 ? -1 : s>tilesize-1 ? +1 : 0;
    int dy = t<0 ? -1 : t>tilesize-1 ? +1 : 0;

    TileKey actualKey = (dx==0 && dy==0)? key : key.createNeighborKey(dx, -dy);

    if (actualKey.valid())
    {
        MetaImageComponent& comp = metaImage[dx+1][dy+1];
        if (!comp.failed && !comp.image.valid())
        {
            // Always use the immediate parent for fractal refinement.
            TileKey parentKey = actualKey.createParentKey();

            GeoImage tile = const_cast<LandCoverLayer*>(this)->createImage(parentKey, progress);
            if (tile.valid())
            {
                comp.image = tile.getImage();
                actualKey.getExtent().createScaleBias(parentKey.getExtent(), comp.scaleBias);
                comp.pixel.setImage(comp.image.get());
            }
            else
            {
                comp.failed = true;
            }
        }

        if (comp.image.valid())
        {
            s = s<0? tilesize+s : s>tilesize-1 ? s-tilesize : s;
            t = t<0? tilesize+t : t>tilesize-1 ? t-tilesize : t;
            s = (int)((double)s*comp.scaleBias(0,0)) + (int)(comp.scaleBias(3,0)*(double)tilesize);
            t = (int)((double)t*comp.scaleBias(1,1)) + (int)(comp.scaleBias(3,1)*(double)tilesize);

            comp.pixel(output, s, t);

            return true;
        }
    }

    output.set(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE);
    return false;
}

GeoImage
LandCoverLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoImage::INVALID;

    if (key.getLOD() > getMaxDataLevel())
        return GeoImage::INVALID;

    ImageLayer* imageLayer = getSource();

    if (key.getLOD() > imageLayer->getMaxDataLevel())
        return GeoImage::INVALID;

    TileKey parentKey = key.createParentKey();

    TileKey bestKey = imageLayer->getBestAvailableTileKey(key);
    if (bestKey.valid() == false)
        return GeoImage::INVALID;

    if (bestKey == key || !parentKey.valid())
    {
        GeoImage img = imageLayer->createImage(key, progress);
        if (!img.valid())
            return img;

        osg::ref_ptr<osg::Image> output = LandCover::createImage(getTileSize());

        ImageUtils::PixelReader read(img.getImage());
        ImageUtils::PixelWriter write(output.get());

        osg::Vec4 pixel;
        bool wrotePixel;
        unsigned pixelsWritten = 0u;

        // Transcode the layer-specific codes into the dictionary codes:
        for (int t = 0; t < output->t(); ++t)
        {
            for (int s = 0; s < output->s(); ++s)
            {
                read(pixel, s, t);

                wrotePixel = false;

                if (pixel.r() != NO_DATA_VALUE)
                {
                    if (pixel.r() < 1.0f)
                    {
                        // normalized code; convert to unnormalized.
                        // e.g., data coming from a server might be encoded this way
                        int code = (int)(pixel.r()*255.0f);
                        if (code < _codemap.size())
                        {
                            int value = _codemap[code];
                            if (value >= 0)
                            {
                                pixel.r() = (float)value;
                                write(pixel, s, t);
                                wrotePixel = true;
                                pixelsWritten++;
                            }
                        }
                    }
                    else
                    {
                        // unnormalized
                        int code = (int)pixel.r();
                        if (code < _codemap.size() && _codemap[code] >= 0)
                        {
                            pixel.r() = (float)_codemap[code];
                            write(pixel, s, t);
                            wrotePixel = true;
                            pixelsWritten++;
                        }
                    }
                }

                if (!wrotePixel)
                {
                    pixel.r() = NO_DATA_VALUE;
                    write(pixel, s, t);
                }
            }
        }

        if (pixelsWritten > 0)
            return GeoImage(output.get(), key.getExtent());
        else
            return GeoImage::INVALID;
    }

    // No data, but want more levels? Fractal refinement starts here.
    if (options().maxDataLevel().isSet() && getMaxDataLevel() > key.getLOD())
    {
        return createFractalEnhancedImage(key, progress);
    }
    else
    {
        return GeoImage::INVALID;
    }
}


GeoImage
LandCoverLayer::createFractalEnhancedImage(const TileKey& key, ProgressCallback* progress) const
{
    MetaImage metaImage;

    // Allocate the working image, which includes a border for 
    // holding values from adjacent tiles.
    osg::ref_ptr<osg::Image> workspace = LandCover::createImage(getTileSize() + 3);

    ImageUtils::PixelWriter writeToWorkspace(workspace.get());
    ImageUtils::PixelReader readFromWorkspace(workspace.get());

    // Allocate the output image:
    osg::ref_ptr<osg::Image> output = LandCover::createImage(getTileSize());

    Random prng(key.getTileX()*key.getTileY()*key.getLOD());

    // working variables
    osg::Vec4 pixel;
    osg::Vec4 p0, p1, p2, p3;
    unsigned r;
    int s, t;

    // temporarily hard-coded sand and water values for beach generation
    bool generateBeach = _beachCode>=0 && _waterCode>=0;
    const float S=_beachCode;
    const osg::Vec4 sand(S,S,S,1);
    const float W=_waterCode;
    const osg::Vec4 water(W,W,W,1);
    float k0,k1,k2,k3;
    unsigned beachLOD = 14; //13;

    // First pass: loop over the grid and populate even pixels with
    // values from the ancestors.
    for (t = 0; t < workspace->t(); t += 2)
    {
        for (s = 0; s < workspace->s(); s += 2)
        {
            readMetaImage(metaImage, key, s-2, t-2, pixel, progress);
            writeToWorkspace(pixel, s, t);

            if (progress && progress->isCanceled())
                return GeoImage::INVALID;
        }

        if (progress && progress->isCanceled())
            return GeoImage::INVALID;
    }

    // Second pass: diamond
    for (t = 1; t < workspace->t()-1; t+=2) //++t)
    {
        for (s = 1; s < workspace->s()-1; s+=2) //++s)
        {
            //if ((s & 1) == 1 && (t & 1) == 1)
            {
                r = prng.next(4u);

                // Diamond: pick one of the four diagonals to copy into the
                // center pixel, attempting to preserve curves. When there is
                // no clear choice, go random.
                readFromWorkspace(p0, s - 1, t - 1); k0=p0.r();
                readFromWorkspace(p1, s + 1, t - 1); k1=p1.r();
                readFromWorkspace(p2, s + 1, t + 1); k2=p2.r();
                readFromWorkspace(p3, s - 1, t + 1); k3=p3.r();   

                if (generateBeach && key.getLOD()==beachLOD)
                {
                    // all the same, copy
                    if (k0==k1 && k1==k2 && k2==k3) pixel = p0;

                    // if water is across from non-water and non-sand, make it sand.
                    else if (k0==W && k2!=W && k2!=S) pixel=sand;
                    else if (k1==W && k3!=W && k3!=S) pixel=sand;
                    else if (k2==W && k0!=W && k0!=S) pixel=sand;
                    else if (k3==W && k1!=W && k1!=S) pixel=sand;

                    // three the same
                    else if (k0==k1 && k1==k2 && k2 != k3) pixel = p0;
                    else if (k1==k2 && k2==k3 && k3 != k0) pixel = p1;
                    else if (k2==k3 && k3==k0 && k0 != k1) pixel = p2;
                    else if (k3==k0 && k0==k1 && k1 != k2) pixel = p3;

                    // continuations - don't break up a run
                    else if (k0==k2 && k0!=k1 && k0!=k3) pixel=p0;
                    else if (k1==k3 && k1!=k2 && k1!=k0) pixel=p1;
                    
                    // all else, rando.
                    else pixel = (r==0)? p0 : (r==1)? p1 : (r==2)? p2 : p3;
                }
                else
                {
                    // three the same
                    if (k0==k1 && k1==k2 && k2 != k3) pixel = p0;
                    else if (k1==k2 && k2==k3 && k3 != k0) pixel = p1;
                    else if (k2==k3 && k3==k0 && k0 != k1) pixel = p2;
                    else if (k3==k0 && k0==k1 && k1 != k2) pixel = p3;

                    // continuations
                    else if (k0==k2 && k0!=k1 && k0!=k3) pixel=p0;
                    else if (k1==k3 && k1!=k2 && k1!=k0) pixel=p1;

                    // all else, rando.
                    else pixel = (r==0)? p0 : (r==1)? p1 : (r==2)? p2 : p3;
                }
                writeToWorkspace(pixel, s, t);
            }
        }
    }

    // Third pass: square
    for (t = 2; t < workspace->t()-1; ++t)
    {
        for (s = 2; s < workspace->s()-1; ++s)
        {
            if (((s & 1) == 1 && (t & 1) == 0) || ((s & 1) == 0 && (t & 1) == 1))
            {
                r = prng.next(4u);

                // Square: pick one of the four adjacents to copy into the
                // center pixel, attempting to preserve curves. When there is
                // no clear choice, go random.
                readFromWorkspace(p0, s - 1, t); k0=p0.r();
                readFromWorkspace(p1, s, t - 1); k1=p1.r();
                readFromWorkspace(p2, s + 1, t); k2=p2.r();
                readFromWorkspace(p3, s, t + 1); k3=p3.r();

                if (generateBeach && key.getLOD()==beachLOD)
                {
                    // all the same, copy
                    if (k0==k1 && k1==k2 && k2==k3) pixel = p0;

                    // if water is across from non-water and non-sand, make it sand.
                    else if (k0==W && k2!=W && k2!=S) pixel=sand;
                    else if (k1==W && k3!=W && k3!=S) pixel=sand;
                    else if (k2==W && k0!=W && k0!=S) pixel=sand;
                    else if (k3==W && k1!=W && k1!=S) pixel=sand;

                    // three the same
                    else if (k0==k1 && k1==k2 && k2 != k3) pixel = p0;
                    else if (k1==k2 && k2==k3 && k3 != k0) pixel = p1;
                    else if (k2==k3 && k3==k0 && k0 != k1) pixel = p2;
                    else if (k3==k0 && k0==k1 && k1 != k2) pixel = p3;

                    // continuations
                    else if (k0==k2 && k0!=k1 && k0!=k3) pixel=p0;
                    else if (k1==k3 && k1!=k2 && k1!=k0) pixel=p1;

                    // all else, rando.
                    else pixel = (r==0)? p0 : (r==1)? p1 : (r==2)? p2 : p3;
                }
                else
                {
                    // three the same
                    if (k0==k1 && k1==k2 && k2 != k3) pixel = p0;
                    else if (k1==k2 && k2==k3 && k3 != k0) pixel = p1;
                    else if (k2==k3 && k3==k0 && k0 != k1) pixel = p2;
                    else if (k3==k0 && k0==k1 && k1 != k2) pixel = p3;

                    // continuations
                    else if (k0==k2 && k0!=k1 && k0!=k3) pixel=p0;
                    else if (k1==k3 && k1!=k2 && k1!=k0) pixel=p1;

                    // all else, rando.
                    else pixel = (r==0)? p0 : (r==1)? p1 : (r==2)? p2 : p3;
                }
                writeToWorkspace(pixel, s, t);
            }
        }
    }

    ImageUtils::PixelWriter writeToOutput(output.get());

    for (t = 0; t < output->t(); ++t)
    {
        for (s = 0; s < output->s(); ++s)
        {
            readFromWorkspace(pixel, s + 2, t + 2);
            writeToOutput(pixel, s, t);
        }
    }

    if (progress && progress->isCanceled())
    {
        return GeoImage::INVALID;
    }

    return GeoImage(output.get(), key.getExtent());
}

// Constructs a code map (int to int) for a coverage layer. We will use this
// code map to map coverage layer codes to dictionary codes.
void
LandCoverLayer::buildCodeMap(CodeMap& codemap)
{
    if (options().mappings().empty()) {
        OE_WARN << LC << "ILLEGAL: no coverage mappings\n";
        return;
    }
    if (!_lcDictionary.valid()) {
        OE_WARN << LC << "ILLEGAL: coverage dictionary not set in buildCodeMap\n";
        return;
    }

    //OE_INFO << LC << "Building code map for " << coverage->getName() << "..." << std::endl;

    int highestValue = 0;

    for (LandCoverValueMappingVector::const_iterator k = options().mappings().begin();
        k != options().mappings().end();
        ++k)
    {
        const LandCoverValueMapping* mapping = k->get();
        int value = mapping->getValue();
        if (value > highestValue)
            highestValue = value;
    }

    codemap.assign(highestValue + 1, -1);

    for (LandCoverValueMappingVector::const_iterator k = options().mappings().begin();
        k != options().mappings().end();
        ++k)
    {
        const LandCoverValueMapping* mapping = k->get();
        int value = mapping->getValue();
        const LandCoverClass* lcClass = _lcDictionary->getClassByName(mapping->getLandCoverClassName());
        if (lcClass)
        {
            codemap[value] = lcClass->getValue();
        }
    }
}

//........................................................................

#undef  LC
#define LC "[LandCoverLayerVector] "

LandCoverLayerVector::LandCoverLayerVector()
{
    //nop
}


LandCoverLayerVector::LandCoverLayerVector(const LandCoverLayerVector& rhs) :
    osg::MixinVector< osg::ref_ptr<LandCoverLayer> >( rhs )
{
    //nop
}

// Composites a vector of land cover images into a single image.
bool
LandCoverLayerVector::populateLandCoverImage(
    osg::ref_ptr<osg::Image>& output,
    const TileKey&            key,
    ProgressCallback*         progress ) const
{
    if (empty())
        return false;

    // Special case of one image - no compositing necessary.
    if (size() == 1)
    {
        if (begin()->get()->isOpen())
        {
            GeoImage r = begin()->get()->createImage(key, progress);
            output = const_cast<osg::Image*>(r.getImage());
        }
        return output.valid();
    }

    bool fallback = false;          // whether to fall back on parent tiles for a component
    bool needsClone = false;        // whether to clone the output image

    osg::Vec4 value;
    unsigned numValues = 0u;
    unsigned numNoDataValues = 1u;

    // Iterate backwards since the last image has the highest priority.
    // If we get an image with all valid values (no NO_DATA), we are finished
    for(const_reverse_iterator i = rbegin(); i != rend() && numNoDataValues > 0u; ++i)
    {
        LandCoverLayer* layer = i->get();

        if (!layer->isOpen())
            continue;

        GeoImage comp;

        osg::Matrixd compScaleBias;
        
        // Fetch the image for the current component. If necessary, fall back on
        // ancestor tilekeys until we get a result. This is necessary if we 
        // already have some data but there are NO DATA values that need filling.
        if (fallback)
        {
            TileKey compKey = key;
            while(comp.valid() == false && compKey.valid())
            {
                comp = layer->createImage(compKey, progress);
                if (!comp.valid())
                    compKey = compKey.createParentKey();
                else
                    key.getExtent().createScaleBias(compKey.getExtent(), compScaleBias);
            }
        }
        else
        {
            comp = layer->createImage(key, progress);
        }

        if (!comp.valid())
            continue;  
        
        ImageUtils::PixelReader readInput(comp.getImage());

        // scale and bias to read an ancestor (fallback) tile if necessary.
        double 
            scale = compScaleBias(0,0), 
            sbias = compScaleBias(3,0)*readInput.s(),
            tbias = compScaleBias(3,1)*readInput.t();

        // If this is the first image, scan the image for NO_DATA values.
        if (!output.valid())
        {
            numNoDataValues = 0u;

            for(int t=0; t<readInput.t() && numNoDataValues == 0u; ++t)
            {
                for(int s=0; s<readInput.s() && numNoDataValues == 0u; ++s)
                {
                    readInput(value, (int)(s*scale+sbias), (int)(t*scale+tbias));
                    if (value.r() == NO_DATA_VALUE)
                    {
                        numNoDataValues++;
                    }
                }
            }

            output = const_cast<osg::Image*>(comp.getImage());
            numValues = output->s() * output->t();
            needsClone = true;
            fallback = true;
            continue;
        }

        // The second image to arrive requires that we clone the data
        // since we are going to modify it.
        if (needsClone)
        {
            output = osg::clone(output.get(), osg::CopyOp::DEEP_COPY_ALL);
            needsClone = false;
        }

        // now composite this image under the previous one, 
        // accumulating a count of NO_DATA values along the way.
        ImageUtils::PixelReader readOutput(output.get());
        ImageUtils::PixelWriter writeOutput(output.get());

        numNoDataValues = 0u;

        for(int t=0; t<readOutput.t(); ++t)
        {
            for(int s=0; s<readOutput.s(); ++s)
            {
                readOutput(value, s, t);

                if (value.r() == NO_DATA_VALUE)
                {
                    readInput(value, (int)(s*scale+sbias), (int)(t*scale+tbias));

                    if (value.r() == NO_DATA_VALUE)
                        numNoDataValues++;
                    else
                        writeOutput(value, s, t);
                }
            }
        }
    }

    // If the image is ALL nodata ... return NULL.
    if (numNoDataValues == numValues)
    {
        output = NULL;
        return false;
    }
    else
    {
        return output.valid();
    }
}
