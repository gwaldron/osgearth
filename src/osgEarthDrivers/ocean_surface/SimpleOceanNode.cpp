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
#include "SimpleOceanNode"
#include "ElevationProxyImageLayer"
#include "SimpleOceanShaders"
#include <osgEarth/Map>
#include <osgEarth/ShaderFactory>
#include <osgEarth/TextureCompositor>
#include <osgEarthDrivers/engine_mp/MPTerrainEngineOptions>

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Texture2D>

#define LC "[SimpleOceanNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::SimpleOcean;


SimpleOceanNode::SimpleOceanNode(const SimpleOceanOptions& options,
                                 MapNode*                  mapNode) :
_parentMapNode( mapNode ),
_options      ( options )
{
    // set the node mask so that our custom EarthManipulator will NOT find this node.
    setNodeMask( 0xFFFFFFFE );
    rebuild();
}


void
SimpleOceanNode::rebuild()
{
    this->removeChildren( 0, this->getNumChildren() );

    if ( _parentMapNode.valid() )
    {
        const MapOptions&     parentMapOptions     = _parentMapNode->getMap()->getMapOptions();
        const MapNodeOptions& parentMapNodeOptions = _parentMapNode->getMapNodeOptions();

        // set up the map to "match" the parent map:
        MapOptions mo;
        mo.coordSysType() = parentMapOptions.coordSysType();
        mo.profile()      = _parentMapNode->getMap()->getProfile()->toProfileOptions();

        // new data model for the ocean:
        Map* oceanMap = new Map( mo );

        // ditto with the map node options:
        MapNodeOptions mno;
        if ( mno.enableLighting().isSet() )
            mno.enableLighting() = *mno.enableLighting();

        MPTerrainEngineOptions mpoptions;
        mpoptions.heightFieldSkirtRatio() = 0.0;  // don't want to see skirts
        mpoptions.clusterCulling() = false;       // want to see underwater
        mpoptions.enableBlending() = true;        // gotsta blend with the main node
        mno.setTerrainOptions( mpoptions );

        // make the ocean's map node:
        MapNode* oceanMapNode = new MapNode( oceanMap, mno );
        
        // install a custom compositor. Must do this before adding any image layers.
        //oceanMapNode->setCompositorTechnique( new OceanCompositor(_options) );

        // if the caller requested a mask layer, install that now.
        if ( _options.maskLayer().isSet() )
        {
            if ( !_options.maskLayer()->maxLevel().isSet() )
            {
                // set the max subdivision level if it's not already specified in the 
                // mask layer options:
                _options.maskLayer()->maxLevel() = *_options.maxLOD();
            }

            // make sure the mask is shared (so we can access it from our shader)
            // and invisible (so we can't see it)
            _options.maskLayer()->shared() = true;
            _options.maskLayer()->visible() = false;

            ImageLayer* maskLayer = new ImageLayer( "ocean-mask", *_options.maskLayer() );
            oceanMap->addImageLayer( maskLayer );
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
            epo.maxLevel() = *_options.maxLOD();
            oceanMap->addImageLayer( new ElevationProxyImageLayer(_parentMapNode->getMap(), epo) );
        }

        this->addChild( oceanMapNode );

        // set up the shaders.
        osg::StateSet* ss = this->getOrCreateStateSet();

        // install the shaders on the ocean map node.
        VirtualProgram* vp = VirtualProgram::getOrCreate( ss );
        vp->setName( "osgEarth SimpleOcean" );

        // install a default lighting shader
        // TODO: eventually deprecate this.
        Registry::shaderFactory()->installLightingShaders( vp );

        // use the appropriate shader for the active technique:
        std::string vertSource = _options.maskLayer().isSet() ? source_vertMask : source_vertProxy;
        std::string fragSource = _options.maskLayer().isSet() ? source_fragMask : source_fragProxy;

        vp->setFunction( "oe_ocean_vertex",   vertSource, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "oe_ocean_fragment", fragSource, ShaderComp::LOCATION_FRAGMENT_COLORING );

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

        // trick to mitigate z-fighting..
        ss->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false) );
        ss->setRenderBinDetails( 15, "RenderBin" );

        // load up a surface texture
        ss->getOrCreateUniform( "ocean_has_surface_tex", osg::Uniform::BOOL )->set( false );
        if ( _options.textureURI().isSet() )
        {
            //TODO: enable cache support here:
            osg::Image* image = _options.textureURI()->getImage();
            if ( image )
            {
                osg::Texture2D* tex = new osg::Texture2D( image );
                tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
                tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
                tex->setWrap  ( osg::Texture::WRAP_S, osg::Texture::REPEAT );
                tex->setWrap  ( osg::Texture::WRAP_T, osg::Texture::REPEAT );

                ss->setTextureAttributeAndModes( 2, tex, 1 );
                ss->getOrCreateUniform( "ocean_surface_tex", osg::Uniform::SAMPLER_2D )->set( 2 );
                ss->getOrCreateUniform( "ocean_has_surface_tex", osg::Uniform::BOOL )->set( true );
            }
        }

        // remove backface culling so we can see underwater
        // (use OVERRIDE since the terrain engine sets back face culling.)
        ss->setAttributeAndModes( 
            new osg::CullFace(), 
            osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

        // force apply options:
        applyOptions();
    }
}

void
SimpleOceanNode::applyOptions()
{
    setSeaLevel( *_options.seaLevel() );

    _lowFeather->set( *_options.lowFeatherOffset() );
    _highFeather->set( *_options.highFeatherOffset() );
    _baseColor->set( *_options.baseColor() );
    _maxRange->set( *_options.maxRange() );
    _fadeRange->set( *_options.fadeRange() );

}

void
SimpleOceanNode::onSetSeaLevel()
{
    _seaLevel->set( getSeaLevel() );
}
