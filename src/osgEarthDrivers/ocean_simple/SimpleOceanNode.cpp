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
#include "SimpleOceanNode"
#include "ElevationProxyImageLayer"
#include "SimpleOceanShaders"
#include <osgEarth/Map>
#include <osgEarth/ShaderFactory>
#include <osgEarth/TextureCompositor>
#include <osgEarth/ImageUtils>
#include <osgEarth/CullingUtils>
#include <osgEarthUtil/SimplexNoise>
#include <osgEarthDrivers/engine_mp/MPTerrainEngineOptions>

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Texture2D>

#define LC "[SimpleOceanNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::SimpleOcean;
using namespace osgEarth::Drivers::MPTerrainEngine;

namespace
{
#define SIR 512

    osg::Image* createSurfaceImage()
    {
        static const double twoPI = 2.0*osg::PI;

        osg::Image* image = new osg::Image();
        image->allocateImage(SIR, SIR, 1, GL_LUMINANCE, GL_UNSIGNED_BYTE);

        SimplexNoise noise;
        noise.setFrequency(SIR*512.0);
        noise.setOctaves(16);
        noise.setRange(0.0, 1.0);

        double n0=DBL_MAX, n1=-DBL_MAX;

        ImageUtils::PixelWriter write(image);

        for(int s=0; s<image->s(); ++s)
        {
            for(int t=0; t<image->t(); ++t)
            {
                double a = (double)s / (double)image->s();
                double b = (double)t / (double)image->t();

                double n = noise.getTiledValue(a, b);

                //write( osg::Vec4(0.25*n, 0.3*n, 0.35*n, 0.5), s, t );

                write( osg::Vec4(n,n,n,n), s, t );

                if (n<n0) n0=n;
                if (n>n1) n1=n;
            }
        }

        OE_DEBUG << "Min = " << n0 << ", Max = " << n1 << "\n";

        return image;
    }
}




SimpleOceanNode::SimpleOceanNode(const SimpleOceanOptions& options,
                                 MapNode*                  mapNode) :
OceanNode(options),
SimpleOceanOptions(options),
_parentMapNode(mapNode)
{
    // set the node mask so that our custom EarthManipulator will NOT find this node.
    setNodeMask( 0xFFFFFFFE );
    setSRS( mapNode? mapNode->getMapSRS() : 0L );
    rebuild();
}


void
SimpleOceanNode::rebuild()
{
    this->removeChildren( 0, this->getNumChildren() );

    osg::ref_ptr<MapNode> mapNode;
    if (_parentMapNode.lock(mapNode))
    {
        const MapOptions&     parentMapOptions     = mapNode->getMap()->getMapOptions();
        const MapNodeOptions& parentMapNodeOptions = mapNode->getMapNodeOptions();

        // set up the map to "match" the parent map:
        MapOptions mo;
        mo.coordSysType() = parentMapOptions.coordSysType();
        mo.profile()      = mapNode->getMap()->getProfile()->toProfileOptions();

        // new data model for the ocean:
        Map* oceanMap = new Map( mo );

        // ditto with the map node options:
        MapNodeOptions mno;
        if ( mno.enableLighting().isSet() )
            mno.enableLighting() = *mno.enableLighting();

        MPTerrainEngineOptions mpoptions;
        mpoptions.heightFieldSkirtRatio() = 0.0;      // don't want to see skirts
        mpoptions.minLOD() = maxLOD().get(); // weird, I know

        // so we can the surface from underwater:
        mpoptions.clusterCulling() = false;       // want to see underwater

        mpoptions.enableBlending() = true;        // gotsta blend with the main node

        mpoptions.color() = baseColor().get();

        mno.setTerrainOptions( mpoptions );

        // make the ocean's map node:
        MapNode* oceanMapNode = new MapNode( oceanMap, mno );

        // if the caller requested a mask layer, install that now.
        if ( maskLayer().isSet() )
        {
            if ( !maskLayer()->maxLevel().isSet() )
            {
                // set the max subdivision level if it's not already specified in the 
                // mask layer options:
                maskLayer()->maxLevel() = maxLOD().get();
            }

            // make sure the mask is shared (so we can access it from our shader)
            // and invisible (so we can't see it)
            maskLayer()->shared() = true;
            maskLayer()->visible() = false;

            ImageLayer* layer = new ImageLayer("ocean-mask", maskLayer().get());
            oceanMap->addImageLayer( layer );
        }

        // otherwise, install a "proxy layer" that will use the elevation data in the map
        // to determine where the ocean is. This approach is limited in that it cannot
        // detect the difference between ocean and inland areas that are below sea level.
        else
        {
            // install an "elevation proxy" layer that reads elevation tiles from the
            // parent map and turns them into encoded images for our shader to use.
            ImageLayerOptions epo( "ocean-proxy" );
            epo.cachePolicy() = CachePolicy::NO_CACHE;
            oceanMap->addImageLayer( new ElevationProxyImageLayer(mapNode->getMap(), epo) );
        }

        this->addChild( oceanMapNode );

        // set up the shaders.
        osg::StateSet* ss = this->getOrCreateStateSet();

        // install the shaders on the ocean map node.
        VirtualProgram* vp = VirtualProgram::getOrCreate( ss );
        vp->setName( "osgEarth SimpleOcean" );
        
        Shaders shaders;
        shaders.define("USE_OCEAN_MASK", maskLayer().isSet());
        shaders.loadAll(vp, 0L);

        //// use the appropriate shader for the active technique:
        //std::string vertSource = maskLayer().isSet() ? source_vertMask : source_vertProxy;
        //std::string fragSource = maskLayer().isSet() ? source_fragMask : source_fragProxy;

        //vp->setFunction( "oe_ocean_vertex",   vertSource, ShaderComp::LOCATION_VERTEX_VIEW );
        //vp->setFunction( "oe_ocean_fragment", fragSource, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.6f );

        // install the slot attribute(s)
        ss->getOrCreateUniform( "ocean_data", osg::Uniform::SAMPLER_2D )->set( 0 );

        // set up the options uniforms.

        _seaLevel = new osg::Uniform(osg::Uniform::FLOAT, "ocean_seaLevel");
        ss->addUniform( _seaLevel.get() );

        _lowFeather = new osg::Uniform(osg::Uniform::FLOAT, "ocean_lowFeather");
        ss->addUniform( _lowFeather.get() );

        _highFeather = new osg::Uniform(osg::Uniform::FLOAT, "ocean_highFeather");
        ss->addUniform( _highFeather.get() );

        _baseColor = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "ocean_baseColor");
        ss->addUniform( _baseColor.get() );

        _maxRange = new osg::Uniform(osg::Uniform::FLOAT, "ocean_max_range");
        ss->addUniform( _maxRange.get() );

        _fadeRange = new osg::Uniform(osg::Uniform::FLOAT, "ocean_fade_range");
        ss->addUniform( _fadeRange.get() );

        _alphaUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_ocean_alpha");
        ss->addUniform( _alphaUniform.get() );

        // disable depth writes.
        ss->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false) );

        // load up a surface texture
        osg::ref_ptr<osg::Image> surfaceImage;
        ss->getOrCreateUniform( "ocean_has_surface_tex", osg::Uniform::BOOL )->set( false );
        if ( textureURI().isSet() )
        {
            //TODO: enable cache support here?
            surfaceImage = textureURI()->getImage();
        }

        if ( !surfaceImage.valid() )
        {
            surfaceImage = createSurfaceImage();
        }

        if ( surfaceImage.valid() )
        {
            osg::Texture2D* tex = new osg::Texture2D( surfaceImage.get() );
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            tex->setWrap  ( osg::Texture::WRAP_S, osg::Texture::REPEAT );
            tex->setWrap  ( osg::Texture::WRAP_T, osg::Texture::REPEAT );

            ss->setTextureAttributeAndModes( 2, tex, 1 );
            ss->getOrCreateUniform( "ocean_surface_tex", osg::Uniform::SAMPLER_2D )->set( 2 );
            ss->getOrCreateUniform( "ocean_has_surface_tex", osg::Uniform::BOOL )->set( true );
        }

        // remove backface culling so we can see underwater
        // (use OVERRIDE since the terrain engine sets back face culling.)
        ss->setAttributeAndModes( 
            new osg::CullFace(), 
            osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

        // Material.
        osg::Material* m = new osg::Material();
        m->setAmbient(m->FRONT_AND_BACK, osg::Vec4(.5,.5,.5,1));
        m->setDiffuse(m->FRONT_AND_BACK, osg::Vec4(1,1,1,1));
        m->setSpecular(m->FRONT_AND_BACK, osg::Vec4(0.2,0.2,0.2,1));
        m->setEmission(m->FRONT_AND_BACK, osg::Vec4(0,0,0,1));
        m->setShininess(m->FRONT_AND_BACK, 40.0);
        ss->setAttributeAndModes(m, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

        // force apply options:
        applyOptions();
    }
}

void
SimpleOceanNode::applyOptions()
{
    if (_seaLevel.valid())
    {
        setSeaLevel(seaLevel().get());
        _lowFeather->set(lowFeatherOffset().get());
        _highFeather->set(highFeatherOffset().get());
        _baseColor->set(baseColor().get());
        _maxRange->set(maxRange().get());
        _fadeRange->set(fadeRange().get());
        _alphaUniform->set(getAlpha());
        this->getOrCreateStateSet()->setRenderBinDetails(renderBinNumber().get(), "DepthSortedBin");
    }
}

void
SimpleOceanNode::onSetSeaLevel()
{
    _seaLevel->set( getSeaLevel() );
}

void
SimpleOceanNode::onSetAlpha()
{
    _alphaUniform->set(getAlpha());
}
