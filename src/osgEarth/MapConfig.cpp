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

#include <osgEarth/MapConfig>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgEarth/TileSourceFactory>

#include <osg/Notify>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <fstream>

using namespace osgEarth;

MapConfig::MapConfig()
{
    _id = 0;
    _model_cstype = MapConfig::CSTYPE_GEOCENTRIC;
    _vertical_scale = 1.0f;
    _skirt_ratio = 0.02f;
    _sample_ratio = 1.0f;
    _proxy_port = 8080;
    _min_tile_range_factor = 5;
    _cache_only = false;
    _normalize_edges = false;
    _combine_layers = true;
    _filename = "";
}

MapConfig::MapConfig( const MapConfig& rhs ) :
_id( rhs._id ),
_model_cstype( rhs._model_cstype ),
_vertical_scale( rhs._vertical_scale ),
_skirt_ratio( rhs._skirt_ratio ),
_sample_ratio( rhs._sample_ratio ),
_proxy_host( rhs._proxy_host ),
_proxy_port( rhs._proxy_port ),
_min_tile_range_factor( rhs._min_tile_range_factor ),
_cache_only( rhs._cache_only ),
_combine_layers( rhs._combine_layers),
_normalize_edges( rhs._normalize_edges ),
_filename( rhs._filename ),
_imageSourceConfigs( rhs._imageSourceConfigs),
_heightFieldSourceConfigs( rhs._heightFieldSourceConfigs ),
_imageSources( rhs._imageSources ),
_heightFieldSources( rhs._heightFieldSources),
_cache_config( rhs._cache_config ),
_profile_config( rhs._profile_config ),
_profile( rhs._profile ),
_global_options( rhs._global_options.get() )
{
    //NOP
}

unsigned int
MapConfig::getId() const
{
    return _id;
}

void
MapConfig::setId( unsigned int id )
{
    _id = id;
}

void
MapConfig::setName( const std::string& name )
{
    _name = name;
}

const std::string&
MapConfig::getName() const
{
    return _name;
}

void
MapConfig::setFilename(const std::string& filename)
{
    _filename = filename;
}

const std::string&
MapConfig::getFilename() const
{
    return _filename;
}

const MapConfig::CoordinateSystemType&
MapConfig::getCoordinateSystemType() const
{
    return _model_cstype;
}

void
MapConfig::setCoordinateSystemType( const CoordinateSystemType& type )
{
    _model_cstype = type;
}

void
MapConfig::setVerticalScale( float value )
{
    _vertical_scale = value;
}

float
MapConfig::getVerticalScale() const
{
    return _vertical_scale;
}

void
MapConfig::setSampleRatio(float sample_ratio)
{
    _sample_ratio = sample_ratio;
}

float
MapConfig::getSampleRatio() const
{
    return _sample_ratio;
}

void
MapConfig::setSkirtRatio( float value )
{
    _skirt_ratio = value;
}

float
MapConfig::getSkirtRatio() const
{
    return _skirt_ratio;
}

SourceConfigList&
MapConfig::getImageSourceConfigs()
{
    return _imageSourceConfigs;
}

const SourceConfigList&
MapConfig::getImageSourceConfigs() const 
{
    return _imageSourceConfigs;
}

SourceConfigList&
MapConfig::getHeightFieldSourceConfigs()
{
    return _heightFieldSourceConfigs;
}

const SourceConfigList&
MapConfig::getHeightFieldSourceConfigs() const
{
    return _heightFieldSourceConfigs;
}

OpenThreads::ReadWriteMutex&
MapConfig::getSourceMutex() const {
    return const_cast<MapConfig*>(this)->_sourceMutex;
}


void
MapConfig::setProxyHost( const std::string& value )
{
    _proxy_host = value;
}

const std::string&
MapConfig::getProxyHost() const 
{
    return _proxy_host;
}

void
MapConfig::setProxyPort( unsigned short value )
{
    _proxy_port = value;
}

unsigned short
MapConfig::getProxyPort() const
{
    return _proxy_port;
}

void
MapConfig::setMinTileRangeFactor( float value )
{
    _min_tile_range_factor = value;
}

float
MapConfig::getMinTileRangeFactor() const
{
    return _min_tile_range_factor;
}

void
MapConfig::setCacheOnly(bool cacheOnly)
{
    _cache_only = cacheOnly;
}

bool
MapConfig::getCacheOnly() const

{
    return _cache_only;
}

void
MapConfig::setCombineLayers(bool combineLayers)
{
    _combine_layers = combineLayers;
}

bool
MapConfig::getCombineLayers() const
{
    return _combine_layers;
}

const CacheConfig&
MapConfig::getCacheConfig() const
{
    return _cache_config;
}

void
MapConfig::setCacheConfig( const CacheConfig& cache_config )
{
    _cache_config = cache_config;
}


bool
MapConfig::getNormalizeEdges() const
{
    return _normalize_edges;
}

void
MapConfig::setNormalizeEdges(bool normalize_edges)
{
    _normalize_edges = normalize_edges;
}

const ProfileConfig&
MapConfig::getProfileConfig() const
{
    return _profile_config;
}

void
MapConfig::setProfileConfig(const ProfileConfig& profile_config)
{
    _profile_config = profile_config;
}

void
MapConfig::setGlobalOptions( const osgDB::ReaderWriter::Options* options )
{
    _global_options = options;
}

const osgDB::ReaderWriter::Options*
MapConfig::getGlobalOptions() const
{
    return _global_options.get();
}

const Profile*
MapConfig::getProfile() const
{
    return _profile.get();
}

bool
MapConfig::isOK() const
{
    if ( !_profile.valid() )
    {
        osg::notify(osg::NOTICE) << "Error: Unable to determine a map profile." << std::endl;
        return false;
    }

    //Check to see if we are trying to do a Geocentric database with a Projected profile.
    if ( _profile->getProfileType() == Profile::TYPE_LOCAL && 
        getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC)
    {
        osg::notify(osg::NOTICE) << "[osgEarth::MapConfig] Error: Cannot create a geocentric scene using projected datasources.  Please specify type=\"flat\" on the map element in the .earth file." << std::endl;
        return false;
    }

    //TODO: Other cases?
    return true;
}


static const Profile*
getSuitableMapProfileFor( const Profile* candidate )
{
    if ( candidate->getProfileType() == Profile::TYPE_GEODETIC )
        return osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    else if ( candidate->getProfileType() == Profile::TYPE_MERCATOR )
        return osgEarth::Registry::instance()->getGlobalMercatorProfile();
    else
        return candidate;
}


// figures out what the Map profile should be. there are multiple ways of setting it.
// In order of priority:
//
//   1. Use an explicit "named" profile (e.g., "global-geodetic")
//   2. Use the profile of one of the TileSources
//   3. Use an explicitly defined profile
//   4. Scan the TileSources and use the first profile found
//
// Once we locate the profile to use, set the MAP profile accordingly. If the map profile
// is not LOCAL/PROJECTED, it must be one of the NAMED profiles (global-geodetic/mercator).
// This is done so that caches are stored consistently.
//
void
MapConfig::initialize()
{
    //All the SourceConfig's have been loaded, so initialize the TileSource's
    TileSourceFactory factory;

    //Initialize the image sources
    for (SourceConfigList::const_iterator i = _imageSourceConfigs.begin(); i != _imageSourceConfigs.end(); ++i)
    {
        TileSource* tileSource = factory.createMapTileSource( *i, *this);
        if (tileSource)
        {
            _imageSources.push_back(tileSource);
        }
    }

    //Initialize the heightfield sources
    for (SourceConfigList::const_iterator i = _heightFieldSourceConfigs.begin(); i != _heightFieldSourceConfigs.end(); ++i)
    {
        TileSource* tileSource = factory.createMapTileSource( *i, *this);
        if (tileSource)
        {
            _heightFieldSources.push_back(tileSource);
        }
    }

    TileSource* ref_source = NULL;

    if (getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
    {
        //If the map type if Geocentric, set the profile to global-geodetic
        _profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        osg::notify(osg::INFO) << "[osgEarth::MapConfig] Setting Profile to global-geodetic for geocentric scene" << std::endl;
    }
    else if ( getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE )
    {
        //If the map type is a Geocentric Cube, set the profile to the cube profile.
        _profile = osgEarth::Registry::instance()->getCubeProfile();
        osg::notify(osg::INFO) << "[osgEarth::MapConfig] Using cube profile for geocentric scene" << std::endl;
    }

    // First check for an explicit profile declaration:
    if ( !_profile.valid() && getProfileConfig().defined() )
    {
        // Check for a "well known named" profile:
        std::string namedProfile = getProfileConfig().getNamedProfile();
        if ( !namedProfile.empty() )
        {
            _profile = osgEarth::Registry::instance()->getNamedProfile( namedProfile );
            if ( _profile.valid() )
            {
                osg::notify(osg::INFO) << "[osgEarth::MapConfig] Set map profile to " << namedProfile << std::endl;
            }
            else
            {
                osg::notify(osg::WARN) << "[osgEarth::MapConfig] " << namedProfile << " is not a known profile name" << std::endl;
                //TODO: continue on? or fail here?
            }
        }

        // Check for a TileSource reference (i.e. get the map profile from a particular TileSource)
        if ( !_profile.valid() )
        {
            std::string refLayer = getProfileConfig().getRefLayer();
            if ( !refLayer.empty() )
            {
                //Search through the image sources to find the reference TileSource
                for (TileSourceList::iterator itr = _imageSources.begin(); itr != _imageSources.end(); ++itr)
                {
                    if (itr->get()->getName() == refLayer)
                    {
                        ref_source = itr->get();
                        break; 
                    }
                }

                if (ref_source == NULL)
                {
                    //Search through the heightfield sources to find the reference TileSource
                    for (TileSourceList::iterator itr = _heightFieldSources.begin(); itr != _heightFieldSources.end(); ++itr)
                    {
                        if (itr->get()->getName() == refLayer)
                        {
                            ref_source = itr->get();
                            break; 
                        }
                    }
                }

                if ( ref_source )
                {
                    const Profile* ref_profile = ref_source->initProfile( NULL, _filename );
                    if ( ref_profile )
                    {
                        _profile = getSuitableMapProfileFor( ref_profile );
                        osg::notify(osg::INFO) << "[osgEarth::MapConfig] Setting profile from \"" << refLayer << "\"" << std::endl;
                    }
                }
                else
                {
                    osg::notify(osg::WARN) << "[osgEarth::MapConfig] Source \"" << refLayer << "\" does not have a valid profile" << std::endl;
                }
            }
        }

        // Try to create a profile from an explicit definition (the SRS and extents)
        if ( !_profile.valid() )
        {
            if ( getProfileConfig().areExtentsValid() )
            {
                double minx, miny, maxx, maxy;
                getProfileConfig().getExtents( minx, miny, maxx, maxy );

                // TODO: should we restrict this? This is fine for LOCAL/PROJECTED, but since we are not
                // constraining non-local map profiles to the "well known" types, should we let the user
                // override that? probably...
                _profile = Profile::create( getProfileConfig().getSRS(), minx, miny, maxx, maxy );

                if ( _profile.valid() )
                {
                    osg::notify( osg::INFO ) << "[[osgEarth::MapEngine] Set map profile from SRS: " 
                        << _profile->getSRS()->getName() << std::endl;
                }
            }
        }
    }

    // At this point we MIGHT have a profile.

    // Finally, try scanning the loaded sources and taking the first one we get. At the
    // same time, remove any incompatible sources.

    for( TileSourceList::iterator i = _imageSources.begin(); i != _imageSources.end(); )
    {
        // skip the reference source since we already initialized it
        if ( i->get() != ref_source )
        {
            osg::ref_ptr<const Profile> sourceProfile = (*i)->initProfile( _profile.get(), _filename );

            if ( !_profile.valid() && sourceProfile.valid() )
            {
                _profile = getSuitableMapProfileFor( sourceProfile.get() );
            }
            else if ( !sourceProfile.valid() )
            {
                osg::notify(osg::WARN) << "[osgEarth::MapEngine] Removing invalid TileSource " << i->get()->getName() << std::endl;
                i = _imageSources.erase(i);
                continue;
            }
        }

        if ( osg::getNotifyLevel() >= osg::INFO )
        {
            std::string prof_str = i->get()->getProfile()? i->get()->getProfile()->toString() : "none";
            osg::notify(osg::INFO) 
                << "[osgEarth::MapEngine] Tile source \"" 
                << i->get()->getName() << "\" : profile = " << prof_str << std::endl;
        }

        i++;
    }

    for (TileSourceList::iterator i = _heightFieldSources.begin(); i != _heightFieldSources.end(); )
    {        
        if ( i->get() != ref_source )
        {
            osg::ref_ptr<const Profile> sourceProfile = (*i)->initProfile( _profile.get(), _filename );

            if ( !_profile.valid() && sourceProfile.valid() )
            {
                _profile = getSuitableMapProfileFor( sourceProfile.get() );
            }
            else if ( !sourceProfile.valid() )
            {
                osg::notify(osg::WARN) << "[osgEarth::MapEngine] Removing invalid TileSource " << i->get()->getName() << std::endl;
                i = _heightFieldSources.erase(i);
                continue;
            }
        }

        if ( osg::getNotifyLevel() >= osg::INFO )
        {
            std::string prof_str = i->get()->getProfile()? i->get()->getProfile()->toString() : "none";
            osg::notify(osg::INFO)
                << "[osgEarth::MapEngine] Tile source \""
                << i->get()->getName() << "\" : profile = " << prof_str << std::endl;
        }
        i++;
    }
}




/************************************************************************/


SourceConfig::SourceConfig():
_reprojectBeforeCaching(false)
{
    //NOP
}

SourceConfig::SourceConfig( const SourceConfig& rhs ) :
_name( rhs._name ),
_driver( rhs._driver ),
_reprojectBeforeCaching( rhs._reprojectBeforeCaching ),
_properties( rhs._properties ),
_profile_config( rhs._profile_config ),
_cache_config( rhs._cache_config )
{
    //NOP
}

SourceConfig::SourceConfig(const std::string& name,
                           const std::string& driver ) :
_name( name ),
_driver( driver ),
_reprojectBeforeCaching(false)
{
    //NOP
}

SourceConfig::SourceConfig(const std::string& name,
                           const std::string& driver,
                           const SourceProperties& props ) :
_name( name ),
_driver( driver ),
_properties( props ),
_reprojectBeforeCaching(false)
{
    //NOP
}

bool
SourceConfig::isValid() const
{
    return !_name.empty() && !_driver.empty();
}

void
SourceConfig::setName( const std::string& name )
{
    _name = name;
}

const std::string&
SourceConfig::getName() const
{
    return _name;
}

void
SourceConfig::setDriver( const std::string& driver )
{

    _driver = driver;
}

const std::string&
SourceConfig::getDriver() const
{
    return _driver;
}

bool
SourceConfig::getReprojectBeforeCaching() const
{
	return _reprojectBeforeCaching;
}

void
SourceConfig::setReprojectBeforeCaching( bool reprojectBeforeCaching )
{
	_reprojectBeforeCaching = reprojectBeforeCaching;
}

SourceProperties&
SourceConfig::getProperties()
{
    return _properties;
}

const SourceProperties&
SourceConfig::getProperties() const
{
    return _properties;
}

void
SourceConfig::setProperty( const std::string& name, const std::string& value )
{
    _properties[name] = value;
}

static std::string EMPTY_STRING;

const std::string&
SourceConfig::operator [] ( const std::string& name ) const
{
    SourceProperties::const_iterator i = _properties.find( name );
    return i != _properties.end()? i->second : EMPTY_STRING;
}
        
std::string&
SourceConfig::operator [] ( const std::string& name )
{
    return _properties[name];
}

const CacheConfig&
SourceConfig::getCacheConfig() const
{
    return _cache_config;
}

void
SourceConfig::setCacheConfig(const CacheConfig& cache_config)
{
    _cache_config = cache_config;
}

const ProfileConfig&
SourceConfig::getProfileConfig() const
{
    return _profile_config;
}

void
SourceConfig::setProfileConfig( const ProfileConfig& profile_config )
{
    _profile_config = profile_config;
}
    

/************************************************************************/


CacheConfig::CacheConfig() :
_type( CacheConfig::TYPE_UNDEFINED )
{
    //NOP
}

CacheConfig::CacheConfig( const CacheConfig& rhs ) :
_type( rhs._type ),
_properties( rhs._properties )
{
    //NOP
}

bool
CacheConfig::defined() const
{
    return _type != CacheConfig::TYPE_UNDEFINED; //_type.empty() && _properties.size() == 0;
}

const CacheConfig::CacheType&
CacheConfig::getType() const
{
    return _type;
}

void
CacheConfig::setType(const CacheConfig::CacheType& type)
{
    _type = type;
}

/**
* Gets the collection of name/value pairs for the cache.
*/
CacheProperties&
CacheConfig::getProperties()
{
    return _properties;
}

const CacheProperties& CacheConfig::getProperties() const
{
    return _properties;
}

void CacheConfig::inheritFrom(const CacheConfig& rhs)
{
    if ( _type != TYPE_NONE )
    {
        //Inherit the type if applicable:
        if ( rhs.getType() != TYPE_UNDEFINED )
        {
            setType( rhs.getType() );
        }

        //Inherit the properites
        for (CacheProperties::const_iterator itr = rhs.getProperties().begin(); itr != rhs.getProperties().end(); ++itr)
        {
            getProperties()[itr->first] = itr->second;
        }
    }
}

/***********************************************************************/

ProfileConfig::ProfileConfig():
_minX(DBL_MAX),
_minY(DBL_MAX),
_maxX(-DBL_MAX),
_maxY(-DBL_MAX),
_empty(true)
{
    //NOP
}

ProfileConfig::ProfileConfig( const ProfileConfig& rhs ) :
_minX( rhs._minX ),
_minY( rhs._minY ),
_maxX( rhs._maxX ),
_maxY( rhs._maxY ),
_empty( rhs._empty ),
_srs( rhs._srs ),
_namedProfile( rhs._namedProfile ),
_refLayer( rhs._refLayer )
{
    //NOP
}

ProfileConfig::ProfileConfig( const std::string& namedProfile )
{
    _namedProfile = namedProfile;
    _empty = false;
}

bool
ProfileConfig::defined() const
{
    return !_empty;
}

const std::string&
ProfileConfig::getNamedProfile() const
{
    return _namedProfile;
}

void
ProfileConfig::setNamedProfile( const std::string& namedProfile)
{
    _namedProfile = namedProfile;
    _empty = false;
}

const std::string&
ProfileConfig::getRefLayer() const
{
    return _refLayer;
}

void
ProfileConfig::setRefLayer(const std::string& refLayer)
{
    _refLayer = refLayer;
    _empty = false;
}

const std::string&
ProfileConfig::getSRS() const
{
    return _srs;
}

void
ProfileConfig::setSRS(const std::string& srs)
{
    _srs = srs;
    _empty = false;
}

bool ProfileConfig::areExtentsValid() const
{
    return _maxX >= _minX && _maxY >= _minY;
}

void
ProfileConfig::getExtents(double &minX, double &minY, double &maxX, double &maxY) const
{
    minX = _minX;
    minY = _minY;
    maxX = _maxX;
    maxY = _maxY;
}

void ProfileConfig::setExtents(double minX, double minY, double maxX, double maxY)
{
    _minX = minX;
    _minY = minY;
    _maxX = maxX;
    _maxY = maxY;
    _empty = false;
}

/***********************************************************************/

#define ELEM_MAP                      "map"
#define ATTR_NAME                     "name"
#define ATTR_CSTYPE                   "type"
#define ELEM_IMAGE                    "image"
#define ELEM_HEIGHTFIELD              "heightfield"
#define ELEM_VERTICAL_SCALE           "vertical_scale"
#define ELEM_MIN_TILE_RANGE           "min_tile_range_factor"
#define ATTR_DRIVER                   "driver"
#define ATTR_REPROJECT_BEFORE_CACHING "reproject_before_caching"
#define ELEM_SKIRT_RATIO              "skirt_ratio"
#define ELEM_SAMPLE_RATIO             "sample_ratio"
#define ELEM_PROXY_HOST               "proxy_host"
#define ELEM_PROXY_PORT               "proxy_port"
#define ELEM_CACHE_ONLY               "cache_only"
#define ELEM_NORMALIZE_EDGES          "normalize_edges"
#define ELEM_COMBINE_LAYERS           "combine_layers"

#define ELEM_CACHE                    "cache"
#define ATTR_TYPE                     "type"

#define VALUE_TRUE                    "true"
#define VALUE_FALSE                   "false"

#define ELEM_PROFILE                  "profile"
#define ATTR_MINX                     "xmin"
#define ATTR_MINY                     "ymin"
#define ATTR_MAXX                     "xmax"
#define ATTR_MAXY                     "ymax"
#define ATTR_SRS                      "srs"
#define ATTR_USELAYER                 "use"

static CacheConfig
readCache( XmlElement* e_cache )
{
    CacheConfig cache;
    std::string type_token = e_cache->getAttr( ATTR_TYPE );
    if ( type_token == "tms" || type_token.empty() ) cache.setType( CacheConfig::TYPE_TMS );
    else if ( type_token == "tilecache" ) cache.setType( CacheConfig::TYPE_TILECACHE );
    else if ( type_token == "none" ) cache.setType( CacheConfig::TYPE_NONE );

    //cache.setType( e_cache->getAttr( ATTR_TYPE ) );

    const XmlNodeList& e_props = e_cache->getChildren();
    for( XmlNodeList::const_iterator i = e_props.begin(); i != e_props.end(); i++ )
    {
        XmlElement* e_prop = dynamic_cast<XmlElement*>( i->get() );
        if ( e_prop )
        {
            std::string name = e_prop->getName();
            std::string value = e_prop->getText();
            if ( !name.empty() && !value.empty() )
            {
                cache.getProperties()[name] = value;
            }
        }
    }
    return cache;
}

static void
writeCache( const CacheConfig& cache, XmlElement* e_cache )
{
    e_cache->getAttrs()[ATTR_TYPE] = 
        cache.getType() == CacheConfig::TYPE_NONE? "none" :
        cache.getType() == CacheConfig::TYPE_TILECACHE? "tilecache" :
        cache.getType() == CacheConfig::TYPE_TMS? "tms" :
        "tms";

    //Add all the properties
    for (CacheProperties::const_iterator i = cache.getProperties().begin(); i != cache.getProperties().end(); i++ )
    {
        e_cache->addSubElement(i->first, i->second);
    }
}

static ProfileConfig
readProfileConfig( XmlElement* e_profile )
{
    ProfileConfig profile;

    profile.setNamedProfile( e_profile->getText() );
    profile.setRefLayer( e_profile->getAttr( ATTR_USELAYER ) );

    std::string srs_text = e_profile->getSubElementText( ATTR_SRS );
    std::string srs_attr = e_profile->getAttr( ATTR_SRS );
    profile.setSRS( !srs_attr.empty()? srs_attr : srs_text );

    double minx, miny, maxx, maxy;
    profile.getExtents(minx, miny, maxx, maxy);

    //Get the bounding box (sub element or attr is OK)
    minx = as<double>(e_profile->getSubElementText( ATTR_MINX ), as<double>(e_profile->getAttr( ATTR_MINX ), minx) );
    miny = as<double>(e_profile->getSubElementText( ATTR_MINY ), as<double>(e_profile->getAttr( ATTR_MINY ), miny) );
    maxx = as<double>(e_profile->getSubElementText( ATTR_MAXX ), as<double>(e_profile->getAttr( ATTR_MAXX ), maxx) );
    maxy = as<double>(e_profile->getSubElementText( ATTR_MAXY ), as<double>(e_profile->getAttr( ATTR_MAXY ), maxy) );

    //miny = as<double>(e_profile->getAttr( ATTR_MINY ), miny);
    //maxx = as<double>(e_profile->getAttr( ATTR_MAXX ), maxx);
    //maxy = as<double>(e_profile->getAttr( ATTR_MAXY ), maxy);

    profile.setExtents(minx, miny, maxx, maxy);

    return profile;
}

static void
writeProfileConfig(const ProfileConfig& profile, XmlElement* e_profile )
{
    e_profile->getChildren().push_back(new XmlText(profile.getNamedProfile()));
    e_profile->getAttrs()[ATTR_USELAYER] = profile.getRefLayer();
    e_profile->getAttrs()[ATTR_SRS] = profile.getSRS();

    if (profile.areExtentsValid())
    {
        double minx, miny, maxx, maxy;
        profile.getExtents(minx, miny, maxx, maxy);
        e_profile->getAttrs()[ATTR_MINX] = toString(minx);
        e_profile->getAttrs()[ATTR_MINY] = toString(miny);
        e_profile->getAttrs()[ATTR_MAXX] = toString(maxx);
        e_profile->getAttrs()[ATTR_MAXX] = toString(maxy);
    }
}


static SourceConfig
readSource( XmlElement* e_source )
{
    SourceConfig source;

    source.setName( e_source->getAttr( ATTR_NAME ) );
    source.setDriver( e_source->getAttr( ATTR_DRIVER ) );
	std::string reprojectBeforeCaching = e_source->getAttr( ATTR_REPROJECT_BEFORE_CACHING );

	if (reprojectBeforeCaching == VALUE_TRUE )
        source.setReprojectBeforeCaching( true );
    else if (reprojectBeforeCaching == VALUE_FALSE)
        source.setReprojectBeforeCaching( false );


    //Try to read the cache for the source if one exists
    XmlElement* e_cache = static_cast<XmlElement*>(e_source->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        source.setCacheConfig( readCache( e_cache) );
    }

    // Check for an explicit profile override:
    XmlElement* e_profile = static_cast<XmlElement*>( e_source->getSubElement( ELEM_PROFILE ) );
    if ( e_profile )
    {
        source.setProfileConfig( readProfileConfig( e_profile ) );
    }

    const XmlNodeList& e_props = e_source->getChildren();
    for( XmlNodeList::const_iterator i = e_props.begin(); i != e_props.end(); i++ )
    {
        XmlElement* e_prop = dynamic_cast<XmlElement*>( i->get() );
        if ( e_prop )
        {
            std::string name = e_prop->getName();
            std::string value = e_prop->getText();
            if ( !name.empty() && !value.empty() )
            {
                source.setProperty( name, value );
            }
        }
    }

    return source;
}

static void
writeSource( const SourceConfig& source, XmlElement* e_source )
{
    e_source->getAttrs()[ATTR_NAME] = source.getName();
    e_source->getAttrs()[ATTR_DRIVER] = source.getDriver();
	e_source->getAttrs()[ATTR_REPROJECT_BEFORE_CACHING] = toString<bool>(source.getReprojectBeforeCaching());

    //Add all the properties
    for (SourceProperties::const_iterator i = source.getProperties().begin(); i != source.getProperties().end(); i++ )
    {
        e_source->addSubElement(i->first, i->second);
    }

    if ( source.getCacheConfig().defined() )
    {
       XmlElement* e_cache = new XmlElement(ELEM_CACHE);
       writeCache(source.getCacheConfig(), e_cache);
       e_source->getChildren().push_back(e_cache);
    }
}


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



static bool
readMap( XmlElement* e_map, MapConfig& out_map )
{
    bool success = true;

    out_map.setName( e_map->getAttr( ATTR_NAME ) );
    
    std::string a_cstype = e_map->getAttr( ATTR_CSTYPE );
    if ( a_cstype == "geocentric" || a_cstype == "round" || a_cstype == "globe" || a_cstype == "earth" )
        out_map.setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC );
    else if ( a_cstype == "geographic" || a_cstype == "flat" || a_cstype == "plate carre" || a_cstype == "projected")
        out_map.setCoordinateSystemType( MapConfig::CSTYPE_PROJECTED );
    else if ( a_cstype == "cube" )
        out_map.setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC_CUBE );

    std::string cache_only = e_map->getSubElementText(ELEM_CACHE_ONLY);
    if (cache_only == VALUE_TRUE )
        out_map.setCacheOnly(true);
    else if (cache_only == VALUE_FALSE)
        out_map.setCacheOnly(false);

    std::string combine_layers = e_map->getSubElementText(ELEM_COMBINE_LAYERS);
    if (combine_layers == VALUE_TRUE)
        out_map.setCombineLayers(true);
    else if (combine_layers == VALUE_FALSE)
        out_map.setCombineLayers(false);

    std::string normalizeEdges = e_map->getSubElementText(ELEM_NORMALIZE_EDGES);
    if (normalizeEdges == VALUE_TRUE)
        out_map.setNormalizeEdges(true);
    else if (normalizeEdges == VALUE_FALSE)
        out_map.setNormalizeEdges(false);

    out_map.setVerticalScale( as<float>( e_map->getSubElementText( ELEM_VERTICAL_SCALE ), out_map.getVerticalScale() ) );
    out_map.setMinTileRangeFactor( as<float>( e_map->getSubElementText( ELEM_MIN_TILE_RANGE ), out_map.getMinTileRangeFactor() ) );
    out_map.setSkirtRatio(as<float>(e_map->getSubElementText( ELEM_SKIRT_RATIO ), out_map.getSkirtRatio()));
    out_map.setSampleRatio(as<float>(e_map->getSubElementText( ELEM_SAMPLE_RATIO ), out_map.getSampleRatio()));


    out_map.setProxyHost( as<std::string>( e_map->getSubElementText( ELEM_PROXY_HOST ), out_map.getProxyHost() ) );
    out_map.setProxyPort( as<unsigned short>( e_map->getSubElementText( ELEM_PROXY_PORT ), out_map.getProxyPort() ) );

    //Read the profile definition
    XmlElement* e_profile = static_cast<XmlElement*>(e_map->getSubElement( ELEM_PROFILE ));
    if (e_profile)
    {
        out_map.setProfileConfig( readProfileConfig( e_profile ) );
    }



    XmlNodeList e_images = e_map->getSubElements( ELEM_IMAGE );
    for( XmlNodeList::const_iterator i = e_images.begin(); i != e_images.end(); i++ )
    {
        SourceConfig image_source = readSource( static_cast<XmlElement*>( i->get() ) );
        if ( image_source.isValid() )
        {
            image_source.setProperty( "default_tile_size", "256" ); //->getProperties()["default_tile_size"] = "256";
            out_map.getImageSourceConfigs().push_back( image_source );
        }
    }

    XmlNodeList e_heightfields = e_map->getSubElements( ELEM_HEIGHTFIELD );
    for( XmlNodeList::const_iterator i = e_heightfields.begin(); i != e_heightfields.end(); i++ )
    {
        SourceConfig heightfield_source = readSource( static_cast<XmlElement*>( i->get() ) );
        if ( heightfield_source.isValid() )
        {
            heightfield_source.setProperty( "default_tile_size", "32" );
            out_map.getHeightFieldSourceConfigs().push_back( heightfield_source );
        }
    }

    //Try to read the global map cache if one is specifiec
    XmlElement* e_cache = static_cast<XmlElement*>(e_map->getSubElement( ELEM_CACHE ));
    if (e_cache)
    {
        out_map.setCacheConfig( readCache( e_cache) );
    }

    return success;
}

XmlDocument*
mapToXmlDocument( const MapConfig& map )
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    
    //Create the root "map" node
    osg::ref_ptr<XmlElement> e_map = new XmlElement( "map" );
    doc->getChildren().push_back( e_map.get() );

    //Write the map's name
    e_map->getAttrs()[ATTR_NAME] = map.getName();

    //Write the coordinate system
    std::string cs;
    if (map.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC) cs = "geocentric";
    else if (map.getCoordinateSystemType() == MapConfig::CSTYPE_PROJECTED) cs = "projected";
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth::MapConfig] Unhandled CoordinateSystemType " << std::endl;
        return NULL;
    }
    e_map->getAttrs()[ATTR_CSTYPE] = cs;

    e_map->addSubElement( ELEM_CACHE_ONLY, toString<bool>(map.getCacheOnly()));
    e_map->addSubElement( ELEM_NORMALIZE_EDGES, toString<bool>(map.getNormalizeEdges()));
    e_map->addSubElement( ELEM_COMBINE_LAYERS, toString<bool>(map.getCombineLayers()));

    e_map->addSubElement( ELEM_VERTICAL_SCALE, toString<float>( map.getVerticalScale() ) );
    e_map->addSubElement( ELEM_MIN_TILE_RANGE, toString<float>( map.getMinTileRangeFactor() ) );
    e_map->addSubElement( ELEM_SKIRT_RATIO, toString<float>( map.getSkirtRatio() ) );
    e_map->addSubElement( ELEM_SAMPLE_RATIO, toString<float>( map.getSampleRatio() ) );

    e_map->addSubElement( ELEM_PROXY_HOST, map.getProxyHost() );
    e_map->addSubElement( ELEM_PROXY_PORT, toString<unsigned short>( map.getProxyPort() ) );

    //Write all the image sources
    for (SourceConfigList::const_iterator i = map.getImageSourceConfigs().begin(); i != map.getImageSourceConfigs().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_IMAGE );
        writeSource( *i, e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    //Write all the heightfield sources
    for (SourceConfigList::const_iterator i = map.getHeightFieldSourceConfigs().begin(); i != map.getHeightFieldSourceConfigs().end(); i++)
    {
        osg::ref_ptr<XmlElement> e_source = new XmlElement( ELEM_HEIGHTFIELD );
        writeSource( *i, e_source.get());
        e_map->getChildren().push_back( e_source.get() );
    }

    if ( map.getCacheConfig().defined() )
    {
        XmlElement* e_cache = new XmlElement(ELEM_CACHE);
        writeCache(map.getCacheConfig(), e_cache);
        e_map->getChildren().push_back(e_cache);
    }

    if ( map.getProfileConfig().defined() )
    {
        XmlElement* e_profile = new XmlElement(ELEM_PROFILE);
        writeProfileConfig(map.getProfileConfig(), e_profile);
        e_map->getChildren().push_back(e_profile);
    }

    return doc.release();
}

bool
MapConfigReaderWriter::readXml( std::istream& input, MapConfig& out_map )
{
    bool success = false;
    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( input );
    if ( doc.valid() )
    {
        success = readMap( doc->getSubElement( ELEM_MAP ), out_map );
    }
    return success;
}

bool
MapConfigReaderWriter::readXml( const std::string& location, MapConfig& out_map )
{
    bool success = false;

    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPClient client;
        osg::ref_ptr<HTTPResponse> response = client.get( location );
        if ( response->isOK() && response->getNumParts() > 0 )
        {
            success = readXml( response->getPartStream( 0 ), out_map );
        }
    }
    else
    {
        if (osgDB::fileExists(location) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            std::ifstream in( location.c_str() );
            success = readXml( in, out_map );
        }
    }

    if ( success )
    {
        std::string filename = location;
        if (!osgDB::containsServerAddress(filename))
        {
            filename = osgDB::getRealPath( location );
        }
        out_map.setFilename( filename );
    }

    return success;
}

bool
MapConfigReaderWriter::writeXml( const std::string& location, const MapConfig& map )
{
    std::ofstream out(location.c_str());
    return writeXml( out, map );
}

bool
MapConfigReaderWriter::writeXml( std::ostream& output, const MapConfig& map )
{
    osg::ref_ptr<XmlDocument> doc = mapToXmlDocument( map );    
    doc->store( output );
    return true;
}