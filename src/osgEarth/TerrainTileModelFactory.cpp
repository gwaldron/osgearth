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
#include <osgEarth/TerrainTileModelFactory>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/ImageToHeightFieldConverter>

#include <osg/Texture2D>

#define LC "[TerrainTileModelFactory] "

using namespace osgEarth;

//.........................................................................

TerrainTileModelFactory::TerrainTileModelFactory(const TerrainOptions& options) :
_options         ( options ),
_heightFieldCache( true, 128 )
{
    _heightFieldCacheEnabled = (::getenv("OSGEARTH_MEMORY_PROFILE") == 0L);
}

TerrainTileModel*
TerrainTileModelFactory::createTileModel(const MapFrame&                  frame,
                                         const TileKey&                   key,
                                         const TerrainEngineRequirements* requirements,
                                         ProgressCallback*                progress)
{
    // Make a new model:
    osg::ref_ptr<TerrainTileModel> model = new TerrainTileModel(
        key,
        frame.getRevision() );

    // assemble all the components:
    addImageLayers( model.get(), frame, key, progress );

    if ( requirements == 0L || requirements->elevationTexturesRequired() )
    {
        addElevation( model.get(), frame, key, progress );
    }

    if ( requirements == 0L || requirements->normalTexturesRequired() )
    {
        addNormalMap( model.get(), frame, key, progress );
    }

    // done.
    return model.release();
}

void
TerrainTileModelFactory::addImageLayers(TerrainTileModel*            model,
                                        const MapFrame&              frame,
                                        const TileKey&               key,
                                        ProgressCallback*            progress)
{
    OE_START_TIMER(fetch_image_layers);

    int order = 0;

    for(ImageLayerVector::const_iterator i = frame.imageLayers().begin();
        i != frame.imageLayers().end();
        ++i, ++order )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() && layer->isKeyInRange(key) )
        {
            // This will only go true if we are requesting a ROOT TILE but we have to
            // fall back on lower resolution data to create it.
            bool isFallback = false;

            GeoImage geoImage;

            const Profile* layerProfile = layer->getProfile();
            
            // If this is a ROOT tile, we will try to fall back on lower-resolution
            // data if we can't find something at the optimal LOD.
            bool isRootKey =
                (key.getLOD() == 0) || // should never be
                (key.getLOD()-1 == _options.firstLOD().get() );

            TileSource* tileSource = layer->getTileSource();

            // Only try to get data from the source if it actually intersects the key extent
            bool hasDataInExtent = true;
            if ( tileSource && layerProfile )
            {
                GeoExtent ext = key.getExtent();
                if (!layerProfile->getSRS()->isEquivalentTo( ext.getSRS() ))
                {
                    ext = layerProfile->clampAndTransformExtent( ext );
                }
                hasDataInExtent = tileSource->hasDataInExtent( ext );
            }
            
            // fetch the image from the layer if it's available:
            if ( hasDataInExtent && layer->isKeyInRange(key) )
            {
                geoImage = layer->createImage( key, progress );
            }
            
            if ( geoImage.valid() )
            {
                TerrainTileImageLayerModel* layerModel = new TerrainTileImageLayerModel();
                layerModel->setImageLayer( layer );

                // preserve layer ordering. Without this, layer draw order can get out of whack
                // if you have a layer that doesn't appear in the model until a higher LOD. Instead
                // of just getting appended to the draw set, the Order will make sure it gets 
                // inserted in the correct position according to the map model.
                layerModel->setOrder( order );

                // made an image. Store as a texture with an identity matrix.
                osg::Texture* texture;
                if ( layer->isCoverage() )
                    texture = createCoverageTexture(geoImage.getImage(), layer);
                else
                    texture = createImageTexture(geoImage.getImage(), layer);

                layerModel->setTexture( texture );


                if ( layer->isShared() )
                    model->sharedLayers().push_back( layerModel );

                if ( layer->getVisible() )
                    model->colorLayers().push_back( layerModel );

                if ( layer->isDynamic() )
                    model->setRequiresUpdateTraverse( true );
            }
        }
    }

    if (progress)
        progress->stats()["fetch_imagery_time"] += OE_STOP_TIMER(fetch_image_layers);
}


void
TerrainTileModelFactory::addElevation(TerrainTileModel*            model,
                                      const MapFrame&              frame,
                                      const TileKey&               key,
                                      ProgressCallback*            progress)
{    
    // make an elevation layer.
    OE_START_TIMER(fetch_elevation);

    const MapInfo& mapInfo = frame.getMapInfo();

    const osgEarth::ElevationInterpolation& interp =
        frame.getMapOptions().elevationInterpolation().get();

    // Request a heightfield from the map.
    osg::ref_ptr<osg::HeightField> mainHF;

    if (getOrCreateHeightField(frame, key, SAMPLE_FIRST_VALID, interp, mainHF, progress) && mainHF.valid())
    {
        osg::ref_ptr<TerrainTileElevationModel> layerModel = new TerrainTileElevationModel();
        layerModel->setHeightField( mainHF.get() );

        // pre-calculate the min/max heights:
        for( unsigned col = 0; col < mainHF->getNumColumns(); ++col )
        {
            for( unsigned row = 0; row < mainHF->getNumRows(); ++row )
            {
                float h = mainHF->getHeight(col, row);
                if ( h > layerModel->getMaxHeight() )
                    layerModel->setMaxHeight( h );
                if ( h < layerModel->getMinHeight() )
                    layerModel->setMinHeight( h );
            }
        }        

        // needed for normal map generation
        model->heightFields().setNeighbor(0, 0, mainHF.get());

        // convert the heightfield to a 1-channel 32-bit fp image:
        ImageToHeightFieldConverter conv;
        osg::Image* image = conv.convert( mainHF.get(), 32 ); // 32 = GL_FLOAT

        if ( image )
        {
            // Made an image, so store this as a texture with no matrix.
            osg::Texture* texture = createElevationTexture( image );
            layerModel->setTexture( texture );
            model->elevationModel() = layerModel.get();
        }
    }

    if (progress)
        progress->stats()["fetch_elevation_time"] += OE_STOP_TIMER(fetch_elevation);
}

void
TerrainTileModelFactory::addNormalMap(TerrainTileModel*            model,
                                      const MapFrame&              frame,
                                      const TileKey&               key,
                                      ProgressCallback*            progress)
{
    OE_START_TIMER(fetch_normalmap);

    const osgEarth::ElevationInterpolation& interp =
        frame.getMapOptions().elevationInterpolation().get();

    // Can only generate the normal map if the center heightfield was built:
    osg::Image* image = HeightFieldUtils::convertToNormalMap(
        model->heightFields(),
        key.getProfile()->getSRS() );

    if ( image )
    {
        TerrainTileImageLayerModel* layerModel = new TerrainTileImageLayerModel();
        layerModel->setName( "oe_normal_map" );

        // Made an image, so store this as a texture with no matrix.
        osg::Texture* texture = createNormalTexture( image );
        layerModel->setTexture( texture );
        model->normalModel() = layerModel;
    }

    if (progress)
        progress->stats()["fetch_normalmap_time"] += OE_STOP_TIMER(fetch_normalmap);
}

bool
TerrainTileModelFactory::getOrCreateHeightField(const MapFrame&                 frame,
                                                const TileKey&                  key,
                                                ElevationSamplePolicy           samplePolicy,
                                                ElevationInterpolation          interpolation,
                                                osg::ref_ptr<osg::HeightField>& out_hf,
                                                ProgressCallback*               progress)
{
    // check the quick cache.
    HFCacheKey cachekey;
    cachekey._key          = key;
    cachekey._revision     = frame.getRevision();
    cachekey._samplePolicy = samplePolicy;

    if (progress)
        progress->stats()["hfcache_try_count"] += 1;

    bool hit = false;
    HFCache::Record rec;
    if ( _heightFieldCacheEnabled && _heightFieldCache.get(cachekey, rec) )
    {
        out_hf = rec.value().get();

        if (progress)
        {
            progress->stats()["hfcache_hit_count"] += 1;
            progress->stats()["hfcache_hit_rate"] = progress->stats()["hfcache_hit_count"]/progress->stats()["hfcache_try_count"];
        }

        return true;
    }

    if ( !out_hf.valid() )
    {
        // This sets the elevation tile size; query size for all tiles.
        out_hf = HeightFieldUtils::createReferenceHeightField(
            key.getExtent(), 257, 257, true );
    }

    bool populated = frame.populateHeightField(
        out_hf,
        key,
        true, // convertToHAE
        progress );

#ifdef TREAT_ALL_ZEROS_AS_MISSING_TILE
    // check for a real tile with all zeros and treat it the same as non-existant data.
    if ( populated )
    {
        bool isEmpty = true;
        for(osg::FloatArray::const_iterator f = out_hf->getFloatArray()->begin(); f != out_hf->getFloatArray()->end(); ++f)
        {
            if ( (*f) != 0.0f )
            {
                isEmpty = false;
                break;
            }
        }
        if ( isEmpty )
        {
            populated = false;
        }
    }
#endif

    if ( populated )
    {
        // Treat Plate Carre specially by scaling the height values. (There is no need
        // to do this with an empty heightfield)
        const MapInfo& mapInfo = frame.getMapInfo();
        if ( mapInfo.isPlateCarre() )
        {
            HeightFieldUtils::scaleHeightFieldToDegrees( out_hf.get() );
        }

        // cache it.
        if (_heightFieldCacheEnabled )
            _heightFieldCache.insert( cachekey, out_hf.get() );
    }

    return populated;
}

osg::Texture*
TerrainTileModelFactory::createImageTexture(osg::Image*       image,
                                            const ImageLayer* layer) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );

    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint(false);

    osg::Texture::FilterMode magFilter = 
        layer ? layer->getImageLayerOptions().magFilter().get() : osg::Texture::LINEAR;
    osg::Texture::FilterMode minFilter =
        layer ? layer->getImageLayerOptions().minFilter().get() : osg::Texture::LINEAR;

    tex->setFilter( osg::Texture::MAG_FILTER, magFilter );
    tex->setFilter( osg::Texture::MIN_FILTER, minFilter );
    tex->setMaxAnisotropy( 4.0f );

    // Disable mip mapping for npot tiles
    if (!ImageUtils::isPowerOfTwo( image ) || (!image->isMipmap() && ImageUtils::isCompressed(image)))
    {
        tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    }

    return tex;
}

osg::Texture*
TerrainTileModelFactory::createCoverageTexture(osg::Image*       image,
                                               const ImageLayer* layer) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );

    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint(false);

    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );

    tex->setMaxAnisotropy( 1.0f );

    return tex;
}

osg::Texture*
TerrainTileModelFactory::createElevationTexture(osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setInternalFormat(GL_LUMINANCE32F_ARB);
    tex->setSourceFormat(GL_LUMINANCE);
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint( false );
    tex->setMaxAnisotropy( 1.0f );
    return tex;
}

osg::Texture*
TerrainTileModelFactory::createNormalTexture(osg::Image* image) const
{
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    tex->setWrap  ( osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap  ( osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
    tex->setResizeNonPowerOfTwoHint( false );
    tex->setMaxAnisotropy( 1.0f );
    return tex;
}
