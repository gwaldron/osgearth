/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include "OceanSurfaceContainer"
#include "OceanCompositor"
#include "ElevationProxyImageLayer"
#include <osgEarth/Map>
#include <osgEarth/ShaderComposition>
#include <osgEarth/TextureCompositor>
#include <osgEarthDrivers/osg/OSGOptions>
#include <osgEarthDrivers/engine_osgterrain/OSGTerrainOptions>

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Texture2D>

#define LC "[OceanSurface] "

OceanSurfaceContainer::OceanSurfaceContainer( MapNode* mapNode, const OceanSurfaceOptions& options ) :
_parentMapNode( mapNode )
{
    // set the node mask so that our custom EarthManipulator will NOT find this node.
    setNodeMask( 0xFFFFFFFE );

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

        OSGTerrainOptions to;
        to.heightFieldSkirtRatio() = 0.0;  // don't want to see skirts
        to.clusterCulling() = false;       // want to see underwater
        to.enableBlending() = true;        // gotsta blend with the main node
        mno.setTerrainOptions( to );

        // make the ocean's map node:
        MapNode* oceanMapNode = new MapNode( oceanMap, mno );
        
        // install a custom compositor. Must do this before adding any image layers.
        oceanMapNode->setCompositorTechnique( new OceanCompositor() );

        // install an "elevation proxy" layer that reads elevation tiles from the
        // parent map and turns them into encoded images for our shader to use.
        ImageLayerOptions epo( "ocean-proxy" );
        epo.maxLevel() = *options.maxLOD();
        oceanMap->addImageLayer( new ElevationProxyImageLayer(_parentMapNode->getMap(), epo) );

        this->addChild( oceanMapNode );

        // set up the options uniforms.
        osg::StateSet* ss = this->getOrCreateStateSet();

        _seaLevel = new osg::Uniform(osg::Uniform::FLOAT, "seaLevel");
        ss->addUniform( _seaLevel.get() );

        _lowFeather = new osg::Uniform(osg::Uniform::FLOAT, "lowFeather");
        ss->addUniform( _lowFeather.get() );

        _highFeather = new osg::Uniform(osg::Uniform::FLOAT, "highFeather");
        ss->addUniform( _highFeather.get() );

        _baseColor = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "baseColor");
        ss->addUniform( _baseColor.get() );

        // trick to prevent z-fighting..
        ss->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false) );
        ss->setRenderBinDetails( 15, "RenderBin" );

        // load up a surface texture
        ss->getOrCreateUniform( "has_tex1", osg::Uniform::BOOL )->set( false );
        if ( options.textureURI().isSet() )
        {
            //TODO: enable cache support here:
            osg::Image* image = options.textureURI()->readImage().releaseImage();
            if ( image )
            {
                osg::Texture2D* tex = new osg::Texture2D( image );
                tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
                tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
                tex->setWrap  ( osg::Texture::WRAP_S, osg::Texture::REPEAT );
                tex->setWrap  ( osg::Texture::WRAP_T, osg::Texture::REPEAT );

                ss->setTextureAttributeAndModes( 1, tex, 1 );
                ss->getOrCreateUniform( "tex1", osg::Uniform::SAMPLER_2D )->set( 1 );
                ss->getOrCreateUniform( "has_tex1", osg::Uniform::BOOL )->set( true );
            }
        }

        // remove backface culling so we can see underwater
        // (use OVERRIDE since the terrain engine sets back face culling.)
        ss->setAttributeAndModes( new osg::CullFace(), osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

        apply( options );
    }
}

void
OceanSurfaceContainer::apply( const OceanSurfaceOptions& options )
{
    OE_DEBUG << LC << "Ocean Options = " << options.getConfig().toJSON(true) << std::endl;

    _seaLevel->set( *options.seaLevel() );
    _lowFeather->set( *options.lowFeatherOffset() );
    _highFeather->set( *options.highFeatherOffset() );
    _baseColor->set( *options.baseColor() );
}


// removed, because it was causing conflicts with the ACPH and with entity data
#if 0
void
OceanSurfaceContainer::traverse( osg::NodeVisitor& nv )
{
    osgUtil::CullVisitor* cv = 0L;
    osg::CullSettings::ComputeNearFarMode mode;

    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        // temporarily disable near/far computation so that the ocean surface doesn't
        // play into the clip plane math
        cv = static_cast<osgUtil::CullVisitor*>(&nv);
        mode = cv->getComputeNearFarMode();

        cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    }

    osg::Group::traverse( nv );

    if ( cv )
    {
        cv->setComputeNearFarMode( mode );
    }
}
#endif
