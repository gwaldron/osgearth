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

#include <osgEarth/Map>
#include <osgEarth/GeocentricMap>
#include <osgEarth/ProjectedMap>
#include <osgEarth/FindNode>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>

using namespace osgEarth;

Map::Map(const MapConfig& mapConfig)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( MapEngine::s_mapEngineCacheMutex );

    const osgDB::ReaderWriter::Options* global_options = mapConfig.getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    // transcribe proxy settings:
    if ( !mapConfig.getProxyHost().empty() )
    {
        if ( !local_options.valid() )
            local_options = new osgDB::ReaderWriter::Options();

        std::stringstream buf;
        buf << local_options->getOptionString() << " "
            << "OSG_CURL_PROXY=" << mapConfig.getProxyHost() << " "
            << "OSG_CURL_PROXYPORT=" << mapConfig.getProxyPort();
        local_options->setOptionString( buf.str() );
    }

    osg::notify(osg::INFO) 
        << "[osgEarth] Map: options string = " 
        << (local_options.valid()? local_options->getOptionString() : "<empty>")
        << std::endl;

    
    MapConfig newMapConfig( mapConfig );
    newMapConfig.setGlobalOptions( local_options.get() );  

    if (mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC || 
        mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE )
    {     
        _engine = new GeocentricMap( newMapConfig );
    }
    else // if (mapConfig->getCoordinateSystemType() == MapConfig::CSTYPE_PROJECTED)
    {
        _engine = new ProjectedMap( newMapConfig );
    }

    updateStateSet();

    addChild( _engine->initialize() );
}

Map::~Map()
{
    //NOP
}


MapEngine*
Map::getEngine() const
{
    return _engine.get();
}

const Profile*
Map::getProfile() const 
{
    return _engine->getProfile();
}

bool
Map::isOK() const
{
    return _engine->isOK();
}

Map*
Map::findMapNode( osg::Node* graph )
{
    return findTopMostNodeOfType<Map>( graph );
}

void
Map::addLayer( Layer* layer )
{
    _engine->addLayer( layer );
    if (_layerCallback.valid()) _layerCallback->layerAdded(layer);
    updateStateSet();
}

void
Map::removeLayer( Layer* layer )
{
    //Take a reference to the layer before removing it so we can use it in the callback
    osg::ref_ptr<Layer> layerRef = layer;
    _engine->removeLayer( layer );
    if (_layerCallback.valid()) _layerCallback->layerRemoved(layerRef.get());
    updateStateSet();
}

void
Map::moveLayer( Layer* layer, int position )
{
    int prevIndex = getLayerIndex( layer );
    _engine->moveLayer( layer, position );
    if (_layerCallback.valid()) _layerCallback->layerMoved( layer, prevIndex, position );
    updateStateSet();
}

unsigned int
Map::getNumLayers() const
{
    return _engine->getNumLayers();
}

int
Map::getLayerIndex( Layer* layer ) const
{
    return _engine->getLayerIndex( layer );
}

Layer*
Map::getLayer( unsigned int i ) const
{
    return _engine->getLayer( i );
}

void 
Map::getImageLayers( ImageLayerList& layers ) const
{
    _engine->getImageLayers( layers );
}

void 
Map::getElevationLayers( ElevationLayerList& layers ) const
{
    _engine->getElevationLayers( layers );
}

TileSource* 
Map::createTileSource( const SourceConfig& sourceConfig )
{
    return _engine->createTileSource( sourceConfig );
}

void Map::updateStateSet()
{
    if (_engine->getMapConfig().getCombineLayers())
    {
        osg::notify(osg::NOTICE) << "Applying texenvcombine" << std::endl;
        ImageLayerList imageLayers;
        getImageLayers(imageLayers);

        int numLayers = imageLayers.size();

        osg::StateSet* stateset = getOrCreateStateSet();

        if (numLayers == 1)
        {
            osg::TexEnv* texenv = new osg::TexEnv(osg::TexEnv::MODULATE);
            stateset->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
        }
        else if (numLayers >= 2)
        {
            //Blend together textures 0 and 1 on unit 0
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);

                texenv->setSource1_RGB(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
            }


            //For textures 2 and beyond, blend them together with the previous
            for (int unit = 1; unit < numLayers-1; ++unit)
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);

                texenv->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttributeAndModes(unit, texenv, osg::StateAttribute::ON);
            }

            //Modulate the colors to get proper lighting on the last unit
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
                texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                stateset->setTextureAttributeAndModes(numLayers-1, texenv, osg::StateAttribute::ON);
            }
        }
    }
    else
    {
        osg::notify(osg::NOTICE) << "NOt applying texenvcombine" << std::endl;
    }
}
