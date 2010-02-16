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
#include <osgEarth/EarthFile>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

EarthFile::EarthFile()
{
    //nop
}

EarthFile::EarthFile( Map* map, const MapEngineProperties& props ) :
_map( map ),
_engineProps( props )
{
    //nop
}

void
EarthFile::setMap( Map* map ) {
    _map = map;
}

void
EarthFile::setMapEngineProperties( const MapEngineProperties& props ) {
    _engineProps = props;
}

Map*
EarthFile::getMap() {
    return _map.get();
}

MapEngineProperties& 
EarthFile::getMapEngineProperties() {
    return _engineProps;
}

#define ELEM_MAP                      "map"
#define ATTR_NAME                     "name"
#define ATTR_CSTYPE                   "type"
#define ELEM_IMAGE                    "image"
#define ELEM_HEIGHTFIELD              "heightfield"
#define ELEM_VERTICAL_SCALE           "vertical_scale"
#define ELEM_MIN_TILE_RANGE           "min_tile_range_factor"
#define ELEM_USE_MERCATOR_LOCATOR     "use_mercator_locator"
#define ATTR_DRIVER                   "driver"
#define ELEM_SKIRT_RATIO              "skirt_ratio"
#define ELEM_SAMPLE_RATIO             "sample_ratio"
#define ELEM_PROXY_HOST               "proxy_host"
#define ELEM_PROXY_PORT               "proxy_port"
#define ATTR_CACHE_ONLY               "cache_only"
#define ELEM_NORMALIZE_EDGES          "normalize_edges"
#define ELEM_COMBINE_LAYERS           "combine_layers"
#define ELEM_PREEMPTIVE_LOD           "preemptive_lod"
#define ATTR_MIN_LEVEL                "min_level"
#define ATTR_MAX_LEVEL                "max_level"
#define ELEM_CACHE                    "cache"
#define ATTR_TYPE                     "type"
#define ELEM_LAYERING_TECHNIQUE       "layering_technique"
#define VALUE_MULTIPASS               "multipass"
#define VALUE_MULTITEXTURE            "multitexture"
#define ELEM_NODATA_IMAGE             "nodata_image"
#define ELEM_TRANSPARENT_COLOR        "transparent_color"
#define ELEM_CACHE_FORMAT             "cache_format"
#define ELEM_CACHE_ENABLED            "cache_enabled"
#define ELEM_MODEL                    "model"
#define ELEM_MAX_LOD                  "max_lod"
#define ELEM_LIGHTING                 "lighting"
#define ELEM_MASK_MODEL               "mask_model"
#define ELEM_OPACITY                  "opacity"
#define ELEM_ENABLED                  "enabled"

#define VALUE_TRUE                    "true"
#define VALUE_FALSE                   "false"

#define ELEM_PROFILE                  "profile"
#define ATTR_MINX                     "xmin"
#define ATTR_MINY                     "ymin"
#define ATTR_MAXX                     "xmax"
#define ATTR_MAXY                     "ymax"
#define ATTR_SRS                      "srs"

#define ATTR_LOADING_WEIGHT           "loading_weight"
#define ELEM_LOADING_POLICY           "loading_policy"
#define ATTR_MODE                     "mode"
#define ATTR_LOADING_THREADS_PER_LOGICAL_PROCESSOR "loading_threads_per_logical_processor"
#define ATTR_LOADING_THREADS_PER_CORE "loading_threads_per_core"
#define ATTR_LOADING_THREADS          "loading_threads"
#define ATTR_TILE_GEN_THREADS         "tile_generation_threads"


static osg::Vec4ub
getColor(const std::string& str, osg::Vec4ub default_value)
{
    osg::Vec4ub color = default_value;
    std::istringstream strin(str);
    int r, g, b, a;
    if (strin >> r && strin >> g && strin >> b && strin >> a)
    {
        color.r() = (unsigned char)r;
        color.g() = (unsigned char)g;
        color.b() = (unsigned char)b;
        color.a() = (unsigned char)a;
    }
    return color;
}

static std::string
toColor( const osg::Vec4ub& c )
{
    std::stringstream ss;
    ss << c.r() << " " << c.g() << " " << c.b() << " " << c.a();
    std::string ssStr;
	ssStr = ss.str();
	return ssStr;
}

static ModelLayer*
readModelLayer( const Config& conf )
{
    ModelLayer* layer = new ModelLayer(
        conf.attr( "name" ),
        conf.attr( "driver" ),
        conf );

    return layer;
}

static MapLayer*
readMapLayer( const Config& conf, const Config& additional )
{
    // divine layer type
    MapLayer::Type layerType =
        conf.name() == ELEM_HEIGHTFIELD ? MapLayer::TYPE_HEIGHTFIELD :
        MapLayer::TYPE_IMAGE;

    // combine the layer conf children with the additional config to create the
    // driver-specific configuration.
    Config driverConf;
    driverConf.add( conf.children() );
    driverConf.add( additional.children() );

    MapLayer* layer = new MapLayer(
        conf.attr( "name" ),
        layerType,
        conf.attr( "driver" ),
        driverConf );

    int minLevel = conf.value<int>( ATTR_MIN_LEVEL, -1 );
    if ( minLevel >= 0 )
        layer->minLevel() = minLevel;

    int maxLevel = conf.value<int>( ATTR_MAX_LEVEL, -1 );
    if ( maxLevel >= 0 )
        layer->maxLevel() = maxLevel;

    std::string noDataImage = conf.value( ELEM_NODATA_IMAGE );
	if ( !noDataImage.empty() )
		layer->noDataImageFilename() = noDataImage;

    layer->setCacheFormat( conf.value( ELEM_CACHE_FORMAT ) );

    std::string cacheEnabled = conf.value( ELEM_CACHE_ENABLED );
    if (cacheEnabled == VALUE_TRUE)
    {
        layer->setCacheEnabled( true );
    }
    else if ( cacheEnabled == VALUE_FALSE)
    {
        layer->setCacheEnabled( false );
    }

    if ( !conf.value( ELEM_TRANSPARENT_COLOR ).empty() )
		layer->transparentColor() = getColor( conf.value( ELEM_TRANSPARENT_COLOR ), osg::Vec4ub() );

	// Check for an explicit profile override:
    if ( conf.hasChild( ELEM_PROFILE ) )
        layer->profileConfig() = ProfileConfig( conf.child( ELEM_PROFILE ) );
        
    if ( conf.hasValue( ATTR_LOADING_WEIGHT ) )
        layer->setLoadWeight( conf.value<float>( ATTR_LOADING_WEIGHT, layer->getLoadWeight() ) );

    if ( conf.hasValue( ELEM_OPACITY ) )
        layer->setOpacity( conf.value<float>( ELEM_OPACITY, layer->getOpacity() ) );

    std::string enabledString = conf.value( ELEM_ENABLED );
    if (enabledString == VALUE_TRUE)
    {
        layer->setEnabled( true );
    }
    else if (enabledString == VALUE_FALSE)
    {
        layer->setEnabled( false);
    }


    return layer;
}

static Config
writeLayer( MapLayer* layer, const std::string& typeName ="" )
{
    Config conf;

    conf.name() =
        !typeName.empty() ? typeName :
        layer->getType() == MapLayer::TYPE_HEIGHTFIELD ? ELEM_HEIGHTFIELD :
        ELEM_IMAGE;

    conf.attr( ATTR_NAME ) = layer->getName();
    conf.attr( ATTR_DRIVER ) = layer->getDriver();

    //Add all the properties
    conf.add( layer->getDriverConfig().children() );

	if ( layer->profileConfig().isSet() )
        conf.addChild( layer->profileConfig()->toConfig() );

	if ( layer->noDataImageFilename().isSet() )
        conf.addChild( ELEM_NODATA_IMAGE, layer->noDataImageFilename().get() );

	if ( layer->transparentColor().isSet() )
        conf.addChild( ELEM_TRANSPARENT_COLOR, toColor( layer->transparentColor().get() ) );

    if ( !layer->getCacheFormat().empty() )
        conf.addChild( ELEM_CACHE_FORMAT, layer->getCacheFormat() );

    return conf;
}

static Config
writeLayer( ModelLayer* layer, const std::string& typeName ="" )
{
    Config conf;

    conf.name() = !typeName.empty() ? typeName : ELEM_MODEL;

    conf.attr( ATTR_NAME ) = layer->getName();
    conf.attr( ATTR_DRIVER ) = layer->getDriver();

    //Add all the properties
    conf.add( layer->getDriverConfig().children() );

    return conf;
}


static bool
readMap( const Config& conf, const std::string& referenceURI, EarthFile* earth )
{
//    osg::notify(osg::NOTICE) << conf.toString() << std::endl;

    bool success = true;

    Map::CoordinateSystemType cstype = Map::CSTYPE_GEOCENTRIC;

    std::string a_cstype = conf.value( ATTR_CSTYPE );
    if ( a_cstype == "geocentric" || a_cstype == "round" || a_cstype == "globe" || a_cstype == "earth" )
        cstype = Map::CSTYPE_GEOCENTRIC;
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" || a_cstype == "projected")
        cstype = Map::CSTYPE_PROJECTED;
    else if ( a_cstype == "cube" )
        cstype = Map::CSTYPE_GEOCENTRIC_CUBE;

    osg::ref_ptr<Map> map = new Map( cstype );
    map->setReferenceURI( referenceURI );

    map->setName( conf.value( ATTR_NAME ) );

    std::string use_merc_locator = conf.value( ELEM_USE_MERCATOR_LOCATOR );
    if (use_merc_locator == VALUE_TRUE )
        map->setUseMercatorLocator( true );
    else if ( use_merc_locator == VALUE_FALSE )
        map->setUseMercatorLocator( false );

    
    MapEngineProperties ep;

    std::string combine_layers = conf.value( ELEM_COMBINE_LAYERS );
    if (combine_layers == VALUE_TRUE)
        ep.combineLayers() = true;
    else if (combine_layers == VALUE_FALSE)
        ep.combineLayers() = false;

    std::string normalizeEdges = conf.value( ELEM_NORMALIZE_EDGES );
    if (normalizeEdges == VALUE_TRUE)
        ep.normalizeEdges() = true;
    else if (normalizeEdges == VALUE_FALSE)
        ep.normalizeEdges() = false;
        
    // Read the loading policy
    if ( conf.hasChild( ELEM_LOADING_POLICY ) )
    {
        LoadingPolicy lp;
        Config lpConf = conf.child( ELEM_LOADING_POLICY );

        if ( lpConf.attr( ATTR_MODE ) == "standard" )
        {
            lp.mode() = LoadingPolicy::MODE_STANDARD;
        }
        else if ( lpConf.attr( ATTR_MODE ) == "sequential" ) 
        {            
            lp.mode() = LoadingPolicy::MODE_SEQUENTIAL;
        }
        else if ( lpConf.attr( ATTR_MODE ) == "preemptive" )
        {
            lp.mode() = LoadingPolicy::MODE_PREEMPTIVE;
        }

        if ( lpConf.hasValue( ATTR_LOADING_THREADS_PER_LOGICAL_PROCESSOR ) )
        {
            lp.numThreadsPerCore() = lpConf.value<int>( 
                ATTR_LOADING_THREADS_PER_LOGICAL_PROCESSOR, lp.numThreadsPerCore().defaultValue() );
        }
        if ( lpConf.hasValue( ATTR_LOADING_THREADS_PER_CORE ) )
        {
            lp.numThreadsPerCore() = lpConf.value<int>(
                ATTR_LOADING_THREADS_PER_CORE, lp.numThreadsPerCore().defaultValue() );
        }
        if ( lpConf.hasValue( ATTR_LOADING_THREADS ) )
        {
            lp.numThreads() = lpConf.value<int>(
                ATTR_LOADING_THREADS, lp.numThreads().defaultValue() );
        }
        if ( lpConf.hasValue( ATTR_TILE_GEN_THREADS ) )
        {
            lp.numTileGeneratorThreads() = lpConf.value<int>(
                ATTR_TILE_GEN_THREADS, lp.numTileGeneratorThreads().defaultValue() );
        }

        ep.loadingPolicy() = lp;
    }

	std::string technique = conf.value( ELEM_LAYERING_TECHNIQUE );
	if (technique == VALUE_MULTIPASS )
        ep.layeringTechnique() = MapEngineProperties::LAYERING_MULTIPASS;
    else if ( technique == VALUE_MULTITEXTURE )
        ep.layeringTechnique() = MapEngineProperties::LAYERING_MULTITEXTURE;

    if ( conf.hasValue( ELEM_VERTICAL_SCALE ) )
        ep.verticalScale() = conf.value<float>(
            ELEM_VERTICAL_SCALE, ep.verticalScale().defaultValue() );

    if ( conf.hasValue( ELEM_MIN_TILE_RANGE ) )
        ep.minTileRangeFactor() = conf.value<float>(
            ELEM_MIN_TILE_RANGE, ep.minTileRangeFactor().defaultValue() );

    if ( conf.hasValue( ELEM_SKIRT_RATIO ) )
        ep.heightFieldSkirtRatio() = conf.value<float>(
            ELEM_SKIRT_RATIO, ep.heightFieldSkirtRatio().defaultValue() );

    if ( conf.hasValue( ELEM_SAMPLE_RATIO ) )
        ep.heightFieldSampleRatio() = conf.value<float>(
            ELEM_SAMPLE_RATIO, ep.heightFieldSampleRatio().defaultValue() );

    if ( conf.hasValue( ELEM_PROXY_HOST ) )
        ep.proxySettings()->hostName() = conf.value( ELEM_PROXY_HOST );

    if ( conf.hasValue( ELEM_PROXY_PORT ) )
        ep.proxySettings()->port() = conf.value<int>(
            ELEM_PROXY_PORT, ep.proxySettings()->port() );
          
    if ( conf.hasValue( ELEM_MAX_LOD ) )
        ep.maxLOD() = conf.value<int>(
            ELEM_MAX_LOD, ep.maxLOD().defaultValue() );

    if ( conf.value( ELEM_LIGHTING ) == VALUE_TRUE )
        ep.enableLighting() = true;
    else if ( conf.value( ELEM_LIGHTING ) == VALUE_FALSE )
        ep.enableLighting() = false;

    //Read the profile definition
    if ( conf.hasChild( ELEM_PROFILE ) )
        map->profileConfig() = ProfileConfig( conf.child( ELEM_PROFILE ) );

    //Try to read the global map cache if one is specifiec
    if ( conf.hasChild( ELEM_CACHE ) )
    {
        map->cacheConfig() = CacheConfig( conf.child( ELEM_CACHE ) );

		//Create and set the Cache for the Map
		CacheFactory factory;
		map->setCache( factory.create( map->cacheConfig().get()) );
    }

	if ( osgEarth::Registry::instance()->getCacheOverride() )
	{
		osg::notify(osg::NOTICE) << "Overriding map cache with global cache override" << std::endl;
		map->setCache( osgEarth::Registry::instance()->getCacheOverride() );
	}

    // Read the layers in LAST (otherwise they will not benefit from the cache/profile configuration)
    ConfigSet images = conf.children( ELEM_IMAGE );
    for( ConfigSet::const_iterator i = images.begin(); i != images.end(); i++ )
    {
        Config additional;
        additional.add( "default_tile_size", "256" );

        MapLayer* layer = readMapLayer( *i, additional );
        if ( layer )
            map->addMapLayer( layer );
    }

    ConfigSet heightfields = conf.children( ELEM_HEIGHTFIELD );
    for( ConfigSet::const_iterator i = heightfields.begin(); i != heightfields.end(); i++ )
    {
        Config additional;
        additional.add( "default_tile_size", "16" );

        MapLayer* layer = readMapLayer( *i, additional );
        if ( layer )
            map->addMapLayer( layer );
    }

    ConfigSet models = conf.children( ELEM_MODEL );
    for( ConfigSet::const_iterator i = models.begin(); i != models.end(); i++ )
    {
        ModelLayer* layer = readModelLayer( *i );
        if ( layer )
            map->addModelLayer( layer );
    }

    Config maskModel = conf.child( ELEM_MASK_MODEL );
    if ( !maskModel.empty() )
    {
        ModelLayer* layer = readModelLayer( maskModel );
        if ( layer )
            map->setTerrainMaskLayer( layer );
    }

    earth->setMap( map.get() );
    earth->setMapEngineProperties( ep );

    return success;
}


static Config
mapToConfig( Map* map, const MapEngineProperties& ep )
{
    Config conf( ELEM_MAP );

    conf.attr( ATTR_NAME ) = map->getName();

    //Write the coordinate system
    std::string cs;
    if (map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC) cs = "geocentric";
    else if (map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED) cs = "projected";
    else if ( map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE) cs = "cube";
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth::EarthFile] Unhandled CoordinateSystemType " << std::endl;
        return Config();
    }
    conf.attr( ATTR_CSTYPE ) = cs;

    conf.add( ELEM_USE_MERCATOR_LOCATOR, toString<bool>(map->getUseMercatorLocator()) );

    if ( ep.normalizeEdges().isSet() )
        conf.add( ELEM_NORMALIZE_EDGES, toString<bool>(ep.normalizeEdges().value()) );

    if ( ep.combineLayers().isSet() )
        conf.add( ELEM_COMBINE_LAYERS, toString<bool>(ep.combineLayers().value()) );

    if ( ep.verticalScale().isSet() )
        conf.add( ELEM_VERTICAL_SCALE, toString<float>( ep.verticalScale().value() ) );

    if ( ep.minTileRangeFactor().isSet() )
        conf.add( ELEM_MIN_TILE_RANGE, toString<float>( ep.minTileRangeFactor().value()) );

    if ( ep.heightFieldSkirtRatio().isSet() )
        conf.add( ELEM_SKIRT_RATIO, toString<float>( ep.heightFieldSkirtRatio().value() ) );

    if ( ep.heightFieldSampleRatio().isSet() )
        conf.add( ELEM_SAMPLE_RATIO, toString<float>( ep.heightFieldSampleRatio().value() ) );

    if ( ep.maxLOD().isSet() )
        conf.add( ELEM_MAX_LOD, toString<unsigned int>( ep.maxLOD().value()) );

    if ( ep.proxySettings().isSet() )
    {
        conf.add( ELEM_PROXY_HOST, ep.proxySettings()->hostName() );
        conf.add( ELEM_PROXY_PORT, toString<int>( ep.proxySettings()->port() ) );
    }

    // LOADING POLICY
    if ( ep.loadingPolicy().isSet() )
    {
        const LoadingPolicy& lp = ep.loadingPolicy().value();

        Config lpConf( ELEM_LOADING_POLICY );
        if ( lp.mode() == LoadingPolicy::MODE_SEQUENTIAL )
            lpConf.attr(ATTR_MODE) = "sequential";
        else if ( lp.mode() == LoadingPolicy::MODE_PREEMPTIVE )
            lpConf.attr(ATTR_MODE) = "preemptive";
        else if ( lp.mode() == LoadingPolicy::MODE_STANDARD )
            lpConf.attr(ATTR_MODE) = "standard";

        if ( lp.numThreads().isSet() )
            lpConf.add( ATTR_LOADING_THREADS, toString<int>( lp.numThreads().value() ) );
        if ( lp.numThreadsPerCore().isSet() )
            lpConf.add( ATTR_LOADING_THREADS_PER_CORE, toString<int>( lp.numThreadsPerCore().value() ) );
        if ( lp.numTileGeneratorThreads().isSet() )
            lpConf.add( ATTR_TILE_GEN_THREADS, toString<int>( lp.numTileGeneratorThreads().value() ) );

        conf.addChild( lpConf );
    }

    // LAYERING TECHNIQUE
	if ( ep.layeringTechnique().isSet() )
	{
		if (ep.layeringTechnique() == MapEngineProperties::LAYERING_MULTIPASS)
		{
            conf.add( ELEM_LAYERING_TECHNIQUE, VALUE_MULTIPASS );
		}
		else if (ep.layeringTechnique() == MapEngineProperties::LAYERING_MULTITEXTURE)
		{
            conf.add( ELEM_LAYERING_TECHNIQUE, VALUE_MULTITEXTURE );
		}
	}

    //Write all the image sources
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
        conf.add( writeLayer( i->get() ) );
    }

    //Write all the heightfield sources
    for (MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        conf.add( writeLayer( i->get() ) );
    }

    //Write all the model layers
    for(ModelLayerList::const_iterator i = map->getModelLayers().begin(); i != map->getModelLayers().end(); i++ )
    {
        conf.add( writeLayer( i->get() ) );
    }

    //Terrain mask layer, if necc.
    if ( map->getTerrainMaskLayer() )
    {
        conf.add( writeLayer( map->getTerrainMaskLayer(), ELEM_MASK_MODEL ) );
    }

	//TODO:  Get this from the getCache call itself, not a CacheConfig.
    if ( map->cacheConfig().isSet() )
    {
        conf.add( map->cacheConfig()->toConfig( ELEM_CACHE ) );
    }

    if ( map->profileConfig().isSet() )
    {
        conf.add( map->profileConfig()->toConfig( ELEM_PROFILE ) );
    }

    return conf;
}

bool
EarthFile::readXML( std::istream& input, const std::string& location )
{
    bool success = false;
    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( input );
    if ( doc.valid() )
    {
        Config conf = doc->toConfig().child( ELEM_MAP );
        success = readMap( conf, location, this );
    }
    return success;
}

bool
EarthFile::readXML( const std::string& location )
{
    bool success = false;

    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPResponse response = HTTPClient::get( location );
        if ( response.isOK() && response.getNumParts() > 0 )
        {
            success = readXML( response.getPartStream( 0 ), location );
        }
    }
    else
    {
        if (osgDB::fileExists(location) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            std::ifstream in( location.c_str() );
            success = readXML( in, location );
        }
    }

    if ( success )
    {
        std::string filename = location;
        if (!osgDB::containsServerAddress(filename))
        {
            filename = osgDB::getRealPath( location );
        }
        _map->setReferenceURI( filename );
    }

    return success;
}

bool
EarthFile::writeXML( const std::string& location )
{
    if ( !_map.valid() )
        return false;

    std::ofstream out( location.c_str() );
    return writeXML( out );
}

bool
EarthFile::writeXML( std::ostream& output )
{
    if ( !_map.valid() )
        return false;

    ScopedReadLock lock( _map->getMapDataMutex() );

    Config conf = mapToConfig( _map.get(), _engineProps );
    osg::ref_ptr<XmlDocument> doc = new XmlDocument( conf );
    doc->store( output );

    return true;
}
