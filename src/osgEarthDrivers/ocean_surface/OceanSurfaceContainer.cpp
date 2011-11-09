/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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

#include <osg/Depth>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/IntersectVisitor>


OceanSurfaceContainer::OceanSurfaceContainer( MapNode* mapNode, const OceanSurfaceOptions& options ) :
_parentMapNode( mapNode )
{
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
        mno.enableLighting() = false;
        //if ( mno.enableLighting().isSet() )
        //    mno.enableLighting() = *mno.enableLighting();

        OSGTerrainOptions to;
        to.heightFieldSkirtRatio() = 0.0;
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

#if 0
        // now add a layer for the surface imager.
        OSGOptions tso;
        tso.url() = "../data/watersurface1.png"; //.int"; //*options.alphaImageURI();
        tso.maxDataLevel() = 16;

        ImageLayerOptions ilo( "ocean-tex", tso );
        ilo.cachePolicy() = CachePolicy::NO_CACHE;
        ilo.profile() = *mo.profile();

        ImageLayer* seaLayer = new ImageLayer( ilo );
        oceanMap->addImageLayer( seaLayer );
#endif

        this->addChild( oceanMapNode );

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

        apply( options );
    }
}

void
OceanSurfaceContainer::apply( const OceanSurfaceOptions& options )
{
    _seaLevel->set( *options.seaLevel() );
    _lowFeather->set( *options.lowFeatherOffset() );
    _highFeather->set( *options.highFeatherOffset() );
    _baseColor->set( *options.baseColor() );
}

void
OceanSurfaceContainer::traverse( osg::NodeVisitor& nv )
{
    if ( dynamic_cast<osgUtil::IntersectionVisitor*>(&nv) )
        return;

    if ( dynamic_cast<osgUtil::IntersectVisitor*>(&nv) )
        return;

    osg::Group::traverse( nv );
}
