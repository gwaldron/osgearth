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
#include "LandUseTileSource"
#include <osgEarth/ImageLayer>
#include <osgEarth/MapFrame>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarthUtil/SimplexNoise>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[LandUseTileSource] "

namespace
{
    osg::Vec2 getSplatCoords(const TileKey& key, float baseLOD, const osg::Vec2& covUV)
    {
        osg::Vec2 out;

        float dL = (float)key.getLOD() - baseLOD;
        float factor = pow(2.0f, dL);
        float invFactor = 1.0/factor;
        out.set( covUV.x()*invFactor, covUV.y()*invFactor ); 

        // For upsampling we need to calculate an offset as well
        if ( factor >= 1.0 )
        {
            unsigned wide, high;
            key.getProfile()->getNumTiles(key.getLOD(), wide, high);

            float tileX = (float)key.getTileX();
            float tileY = (float)(wide-1-key.getTileY()); // swap Y. (not done in the shader version.)

            osg::Vec2 a( floor(tileX*invFactor), floor(tileY*invFactor) );
            osg::Vec2 b( a.x()*factor, a.y()*factor );
            osg::Vec2 c( (a.x()+1.0f)*factor, (a.y()+1.0f)*factor );
            osg::Vec2 offset( (tileX-b.x())/(c.x()-b.x()), (tileY-b.y())/(c.y()-b.y()) );

            out += offset;
        }

        return out;
    }

    osg::Vec2 warpCoverageCoords(const osg::Vec2& covIn, float noise, float warp)
    {
        float n1 = 2.0 * noise - 1.0;
        return osg::Vec2(
            osg::clampBetween( covIn.x() + n1*warp, 0.0f, 1.0f ),
            osg::clampBetween( covIn.y() + n1*warp, 0.0f, 1.0f ) );
    }

    float getNoise(osgEarth::Util::SimplexNoise& noiseGen, const osg::Vec2& uv)
    {
        // TODO: check that u and v are 0..s and not 0..s-1
        double n = noiseGen.getTiledValue(uv.x(), uv.y());
        n = osg::clampBetween(n, 0.0, 1.0);
        //out = n;
        return n;
    }
}


LandUseTileSource::LandUseTileSource(const LandUseOptions& options) :
TileSource( options ),
_options  ( options )
{
    //nop
}

Status
LandUseTileSource::initialize(const osgDB::Options* dbOptions)
{
    _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

    const Profile* profile = getProfile();
    if ( !profile )
    {
        profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        setProfile( profile );
    }

    // load all the image layers:
    _imageLayers.assign( _options.imageLayerOptionsVector().size(), 0L );
    _warps.assign( _options.imageLayerOptionsVector().size(), 0.0f );

    for(unsigned i=0; i<_options.imageLayerOptionsVector().size(); ++i)
    {
        ImageLayerOptions ilo = _options.imageLayerOptionsVector()[i];
        ilo.cachePolicy() = CachePolicy::NO_CACHE;
        ImageLayer* layer = new ImageLayer( ilo );
        layer->setTargetProfileHint( profile );
        layer->setReadOptions(_dbOptions.get());
        layer->open();
        _imageLayers[i] = layer;

        Config conf = ilo.getConfig();
        _warps[i] = conf.value("warp", _options.warpFactor().get());
    }

    // set up the noise generator.
    const float F[4] = { 4.0f, 16.0f, 4.0f, 8.0f };
    const float P[4] = { 0.8f,  0.6f, 0.8f, 0.9f };
    const float L[4] = { 2.2f,  1.7f, 3.0f, 4.0f };
    
    // Configure the noise function:
    _noiseGen.setNormalize  ( true );
    _noiseGen.setRange      ( 0.0, 1.0 );
    _noiseGen.setFrequency  ( F[0] );
    _noiseGen.setPersistence( P[0] );
    _noiseGen.setLacunarity ( L[0] );
    _noiseGen.setOctaves    ( 8 );

    return STATUS_OK;
}

CachePolicy
LandUseTileSource::getCachePolicyHint() const
{
    return CachePolicy::NO_CACHE;
}

namespace
{
    struct ILayer 
    {
        GeoImage  image;
        float     scale;
        osg::Vec2 bias;
        bool      valid;
        float     warp;
        ImageUtils::PixelReader* read;

        ILayer() : valid(true), read(0L), scale(1.0f), warp(0.0f) { }

        ~ILayer() { if (read) delete read; }

        void load(const TileKey& key, ImageLayer* sourceLayer, float sourceWarp, ProgressCallback* progress)
        {
            if ( sourceLayer->getEnabled() && sourceLayer->getVisible() && sourceLayer->isKeyInRange(key) )
            {
                for(TileKey k = key; k.valid() && !image.valid(); k = k.createParentKey())
                {
                    image = sourceLayer->createImage(k, progress);
                } 
            }

            valid = image.valid();

            if ( valid )
            {
                scale = key.getExtent().width() / image.getExtent().width();
                bias.x() = (key.getExtent().xMin() - image.getExtent().xMin()) / image.getExtent().width();
                bias.y() = (key.getExtent().yMin() - image.getExtent().yMin()) / image.getExtent().height();

                read = new ImageUtils::PixelReader(image.getImage());

                // cannot interpolate coverage data:
                read->setBilinear( false );

                warp = sourceWarp;
            }
        }
    };
}

osg::Image*
LandUseTileSource::createImage(const TileKey&    key,
                               ProgressCallback* progress)
{
    if ( _imageLayers.empty() )
        return 0L;

    std::vector<ILayer> layers(_imageLayers.size());

    // Allocate the new coverage image; it will contain unnormalized values.
    osg::Image* out = new osg::Image();
    ImageUtils::markAsUnNormalized(out, true);

    // Allocate a suitable format:
    GLenum dataType;
    GLint  internalFormat;
    
    if ( _options.bits().isSetTo(16u) )
    {
        // 16-bit float:
        dataType       = GL_FLOAT;
        internalFormat = GL_LUMINANCE16F_ARB;
    }
    else //if ( _options.bits().isSetTo(32u) )
    {
        // 32-bit float:
        dataType       = GL_FLOAT;
        internalFormat = GL_LUMINANCE32F_ARB;
    }
    
    int tilesize = getPixelsPerTile();

    out->allocateImage(tilesize, tilesize, 1, GL_LUMINANCE, dataType);
    out->setInternalTextureFormat(internalFormat);

    float noiseLOD = _options.baseLOD().get();
    float warp     = _options.warpFactor().get();

    osg::Vec2 cov;    // coverage coordinates
    float     noise;  // noise value
    osg::Vec2 noiseCoords;

    ImageUtils::PixelWriter write( out );

    float du = 1.0f / (float)(out->s()-1);
    float dv = 1.0f / (float)(out->t()-1);

    osg::Vec4 nodata;
    if (internalFormat == GL_LUMINANCE16F_ARB)
        nodata.set(-32768, -32768, -32768, -32768);
    else
        nodata.set(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE);

    for(float u=0.0f; u<=1.0f; u+=du)
    {
        for(float v=0.0f; v<=1.0f; v+=dv)
        {
            bool wrotePixel = false;
            for(int L = layers.size()-1; L >= 0 && !wrotePixel; --L)
            {
                ILayer& layer = layers[L];
                if ( !layer.valid )
                    continue;

                if ( !layer.image.valid() )
                    layer.load(key, _imageLayers[L], _warps[L], progress);

                if ( !layer.valid )
                    continue;

                osg::Vec2 cov(layer.scale*u + layer.bias.x(), layer.scale*v + layer.bias.y());

                if ( cov.x() >= 0.0f && cov.x() <= 1.0f && cov.y() >= 0.0f && cov.y() <= 1.0f )
                {
                    // Noise is like a repeating overlay at the noiseLOD. So sample it using
                    // straight U/V tile coordinates.
                    noiseCoords = getSplatCoords( key, noiseLOD, osg::Vec2(u,v) );
                    noise = getNoise( _noiseGen, noiseCoords );

                    cov = warpCoverageCoords(cov, noise, layer.warp);

                    osg::Vec4 texel = (*layer.read)(cov.x(), cov.y());
                    if ( texel.r() != NO_DATA_VALUE )
                    {
                        write.f(texel, u, v);
                        wrotePixel = true;
                    }
                }
            }

            if ( !wrotePixel )
            {
                write.f(nodata, u, v);
            }
        }
    }

    return out;
}
