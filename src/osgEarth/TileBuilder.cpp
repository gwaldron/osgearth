/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/TileBuilder>
#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/GeographicTileBuilder>
#include <osgEarth/PlateCarre>
#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/TexEnvCombine>
#include <osgFX/MultiTextureControl>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;


TileBuilder*
TileBuilder::create( MapConfig* map, const std::string& url_template, const osgDB::ReaderWriter::Options* options )
{
    TileBuilder* result = NULL;
    if ( map )
    {
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = options ? 
            new osgDB::ReaderWriter::Options( *local_options ) :
            NULL;

        // transcribe proxy settings:
        if ( !map->getProxyHost().empty() )
        {
            if ( !local_options.valid() )
                local_options = new osgDB::ReaderWriter::Options();

            std::stringstream buf;
            buf << local_options->getOptionString()
                << "OSG_CURL_PROXY=" << map->getProxyHost() << " "
                << "OSG_CURL_PROXYPORT=" << map->getProxyPort();
            local_options->setOptionString( buf.str() );
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] TileBuilder: options string = " 
            << (local_options.valid()? local_options->getOptionString() : "<empty>")
            << std::endl;

        if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
        {
            result = new GeocentricTileBuilder( map, url_template, local_options.get() );
        }
        else
        {
            result = new GeographicTileBuilder( map, url_template, local_options.get() );
        }
    }
    return result;
}

static void
addSources(const MapConfig* mapConfig, const SourceConfigList& from, 
           std::vector< osg::ref_ptr<TileSource> >& to,
           const std::string& url_template,
           const osgDB::ReaderWriter::Options* global_options)
{        
    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        SourceConfig* source = i->get();

        osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ?
            new osgDB::ReaderWriter::Options( *global_options ) : 
            new osgDB::ReaderWriter::Options();

        //Setup the plugin options for the source
        for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
        {
            local_options->setPluginData( p->first, (void*)p->second.c_str() );
        }

        local_options->setPluginData("map_config", (void*)mapConfig); 

        //Add the source to the list
        TileSource* tile_source = new ReaderWriterTileSource( source->getDriver(), local_options.get() );
        to.push_back( tile_source );
    }
}

TileBuilder::TileBuilder(MapConfig* _map, 
                         const std::string& _url_template,
                         const osgDB::ReaderWriter::Options* options ) :
map( _map ),
url_template( _url_template )
{
    if ( map.valid() )
    {
        addSources( map.get(), map->getImageSources(), image_sources, url_template, options );
        addSources( map.get(), map->getHeightFieldSources(), heightfield_sources, url_template, options );
    }
}

std::string
TileBuilder::createURI( const TileKey* key )
{
    return key->getName() + "." + url_template;
    //std::stringstream buf;
    //buf << key->getTypeCode() << key->str() << "." << url_template;
    //return buf.str();
}

MapConfig*
TileBuilder::getMapConfig() const
{
    return map.get();
}



osg::Node*
TileBuilder::createNode( const TileKey* key )
{
    osg::ref_ptr<osg::Group> top;
    osg::Group* parent = NULL;

    //osg::notify(osg::NOTICE) << "[osgEarth] TileBuilder::createNode( " << key->str() << ")" << std::endl;

    if ( key->getLevelOfDetail() == 0 )
    {
        // Note: CSN must always be at the top
        osg::CoordinateSystemNode* csn = createCoordinateSystemNode();
        parent = csn;
        top = csn;

        //If there is more than one image source, use TexEnvCombine to blend them together
        if ( map->getImageSources().size() > 1 )
        {
#if 1
            osg::StateSet* stateset = parent->getOrCreateStateSet();
            for (unsigned int i = 0; i < map->getImageSources().size(); ++i)
            {    
                //Blend the textures together from the bottom up
                stateset->setTextureMode(i, GL_TEXTURE_2D, osg::StateAttribute::ON);
                
                //Interpolate the current texture with the previous combiner result using the textures SRC_ALPHA
                osg::TexEnvCombine * tec = new osg::TexEnvCombine;
                tec->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                
                tec->setSource0_RGB(osg::TexEnvCombine::TEXTURE);
                tec->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);

                tec->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
                tec->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                
                tec->setSource2_RGB(osg::TexEnvCombine::TEXTURE);
                tec->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttribute(i, tec, osg::StateAttribute::ON);
            }

            //Modulate the result with the primary color to get proper lighting
            osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
            texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
            texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
            texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
            texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
            texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
            stateset->setTextureAttribute(map->getImageSources().size(), texenv, osg::StateAttribute::ON);
            stateset->setTextureMode(map->getImageSources().size(), GL_TEXTURE_2D, osg::StateAttribute::ON);
#else
            //Decorate the scene with a multi-texture control to control blending between textures
            osgFX::MultiTextureControl *mt = new osgFX::MultiTextureControl;
            parent->addChild( mt );

            float r = 1.0f/ map->getImageSources().size();
            for (unsigned int i = 0; i < map->getImageSources().size(); ++i)
            {
                mt->setTextureWeight(i, r);
            }
            parent = mt;
#endif
        }

        osgTerrain::Terrain* terrain = new osgEarth::EarthTerrain;//new osgTerrain::Terrain();
        terrain->setVerticalScale( map->getVerticalScale() );
        parent->addChild( terrain );
        parent = terrain;
    }
    else
    {
        top = new osg::Group();
        top->setName( key->str() );
        parent = top.get();
    }

    if (!addChildren( parent, key ))
    {
        top = 0;
    }

    return top.release();
}

osg::HeightField*
TileBuilder::createValidHeightField(osgEarth::TileSource* tileSource, const osgEarth::TileKey *key)
{
    //Try to create the heightfield with the given key
    osg::ref_ptr<osg::HeightField> hf;
    osg::ref_ptr<const TileKey> hf_key = key;
    hf = tileSource->createHeightField( key );        

    if (!hf.valid())
    {
        //We could not load the heightfield from the given key, so try to load from parent tiles
        hf_key = key->getParentKey();

        while (hf_key.valid())
        {
            hf = tileSource->createHeightField(hf_key.get());
            if (hf.valid()) break;
            hf_key = hf_key->getParentKey();
        }

        //Use a HeightFieldExtractor to sample the parent tile
        if (hf.valid())
        {
            osg::ref_ptr<HeightFieldExtractor> hfe = new HeightFieldExtractor(hf_key.get(), hf.get());
            hf = hfe->extractChild(key, hf->getNumColumns(), hf->getNumRows());
        }
    }

    return hf.release();
}

bool
TileBuilder::createValidImage(osgEarth::TileSource* tileSource, const osgEarth::TileKey *key, osgEarth::TileBuilder::ImageTileKeyPair &imageTile)
{
    //Try to create the image with the given key
    osg::ref_ptr<osg::Image> image = tileSource->createImage(key);
    osg::ref_ptr<const TileKey> image_key = key;
    
    if (!image.valid())
    {
        //Could not directly create the image from the given TileKey, so try to load from parent tiles
        image_key = key->getParentKey();

        while (image_key.valid())
        {
            image = tileSource->createImage(image_key.get());
            if (image.valid()) break;
            image_key = image_key->getParentKey();
        }
    }

    if (image.valid())
    {
        imageTile.first = image.get();
        imageTile.second = image_key.get();
        return true;
    }
    return false;
}

