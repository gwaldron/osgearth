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

#include <osgEarth/MapEngine>
#include <osgEarth/DirectReadTileSource>
#include <osgEarth/Caching>
#include <osgEarth/Mercator>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Compositing>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/TileSourceFactory>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/ElevationManager>

#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/TexEnvCombine>
#include <osgFX/MultiTextureControl>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <OpenThreads/ReentrantMutex>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

//static
OpenThreads::ReentrantMutex MapEngine::s_mapEngineCacheMutex;
static unsigned int s_mapEngineID = 0;
//Caches the maps that have been created
typedef std::map<unsigned int, osg::observer_ptr<MapEngine> > MapEngineCache;


char vert_source[] ="varying vec2 texCoord0;\n"
                    "varying vec2 texCoord1;\n"
                    "varying vec2 texCoord2;\n"
                    "varying vec2 texCoord3;\n"
                    "uniform bool lightingEnabled; \n"
                    "uniform bool light0Enabled; \n"
                    "uniform bool light1Enabled; \n"
                    "uniform bool light2Enabled; \n"
                    "uniform bool light3Enabled; \n"
                    "uniform bool light4Enabled; \n"
                    "uniform bool light5Enabled; \n"
                    "uniform bool light6Enabled; \n"
                    "uniform bool light7Enabled; \n"
                    "\n"
                    "\n"
                    "void directionalLight(in int i, \n"
                    "                      in vec3 normal, \n"
                    "                      inout vec4 ambient, \n"
                    "                      inout vec4 diffuse, \n"
                    "                      inout vec4 specular) \n"
                    "{ \n"
                    "   float nDotVP;         // normal . light direction \n"
                    "   float nDotHV;         // normal . light half vector \n"
                    "   float pf;             // power factor \n"
                    " \n"
                    "   nDotVP = max(0.0, dot(normal, normalize(vec3 (gl_LightSource[i].position)))); \n"
                    "   nDotHV = max(0.0, dot(normal, vec3 (gl_LightSource[i].halfVector))); \n"
                    " \n"
                    "   if (nDotVP == 0.0) \n"
                    "   { \n"
                    "       pf = 0.0; \n"
                    "   } \n"
                    "   else \n"
                    "   { \n"
                    "       pf = pow(nDotHV, gl_FrontMaterial.shininess); \n"
                    " \n"
                    "   } \n"
                    "   ambient  += gl_LightSource[i].ambient; \n"
                    "   diffuse  += gl_LightSource[i].diffuse * nDotVP; \n"
                    "   specular += gl_LightSource[i].specular * pf; \n"
                    "} \n"
                    "\n"
                    "vec3 fnormal(void)\n"
                    "{\n"
                    "    //Compute the normal \n"
                    "    vec3 normal = gl_NormalMatrix * gl_Normal;\n"
                    "    normal = normalize(normal);\n"
                    "    return normal;\n"
                    "}\n"
                    "\n"
                    "void main (void)\n"
                    "{\n"
                    "    if (lightingEnabled)\n"
                    "    {\n"
                    "    vec4 ambient = vec4(0.0); \n"
                    "    vec4 diffuse = vec4(0.0); \n"
                    "    vec4 specular = vec4(0.0); \n"
                    " \n"
                    "    vec3 normal = fnormal(); \n"
                    " \n"
                    "    if (light0Enabled) directionalLight(0, normal, ambient, diffuse, specular); \n"
                    "    if (light1Enabled) directionalLight(1, normal, ambient, diffuse, specular); \n"
                    "    if (light2Enabled) directionalLight(2, normal, ambient, diffuse, specular); \n"
                    "    if (light3Enabled) directionalLight(3, normal, ambient, diffuse, specular); \n"
                    "    if (light4Enabled) directionalLight(4, normal, ambient, diffuse, specular); \n"
                    "    if (light5Enabled) directionalLight(5, normal, ambient, diffuse, specular); \n"
                    "    if (light6Enabled) directionalLight(6, normal, ambient, diffuse, specular); \n"
                    "    if (light7Enabled) directionalLight(7, normal, ambient, diffuse, specular); \n"

                    "    vec4 color = gl_FrontLightModelProduct.sceneColor + \n"
                    "                 ambient  * gl_FrontMaterial.ambient + \n"
                    "                 diffuse  * gl_FrontMaterial.diffuse + \n"
                    "                 specular * gl_FrontMaterial.specular; \n"
                    " \n"
                    "    gl_FrontColor = color; \n"
                    "   }\n"
                    " else\n"
                    " {\n"
                    " gl_FrontColor = gl_Color;\n"
                    " }\n"
                    "    gl_Position = ftransform();\n"
                    "\n"
                    "	 texCoord0 = gl_MultiTexCoord0.st;\n"
                    "    texCoord1 = gl_MultiTexCoord1.st;\n"
                    "    texCoord2 = gl_MultiTexCoord2.st;\n"
                    "    texCoord3 = gl_MultiTexCoord3.st;\n"
                    "}\n";


char frag_source[] = "uniform sampler2D osgEarth_Layer0_unit;\n"
                    "uniform float osgEarth_Layer0_opacity;\n"
                    "varying vec2 texCoord0;\n"
                    "uniform bool osgEarth_Layer0_enabled;\n"
                    "\n"
                    "uniform sampler2D osgEarth_Layer1_unit;\n"
                    "uniform float osgEarth_Layer1_opacity;\n"
                    "varying vec2 texCoord1;\n"
                    "uniform bool osgEarth_Layer1_enabled;\n"
                    "\n"
                    "uniform sampler2D osgEarth_Layer2_unit;\n"
                    "uniform float osgEarth_Layer2_opacity;\n"
                    "varying vec2 texCoord2;\n"
                    "uniform bool osgEarth_Layer2_enabled;\n"
                    "\n"
                    "uniform sampler2D osgEarth_Layer3_unit;\n"
                    "uniform float osgEarth_Layer3_opacity;\n"
                    "varying vec2 texCoord3;\n"
                    "uniform bool osgEarth_Layer3_enabled;\n"
                    "\n"
                    "void main (void )\n"
                    "{\n"
                    "  //Get the fragment for texture 0\n"
                    "  int numLayersOn = 0;\n"
                    "  vec4 tex0 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer0_enabled)\n"
                    "  {\n"
                    "    tex0 = texture2D(osgEarth_Layer0_unit, texCoord0);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex0.a *= osgEarth_Layer0_opacity;\n"
                    "\n"
                    "  //Get the fragment for texture 1\n"
                    "  vec4 tex1 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer1_enabled)\n"
                    "  {\n"
                    "    tex1 = texture2D(osgEarth_Layer1_unit, texCoord1);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex1.a *= osgEarth_Layer1_opacity;\n"
                    "\n"
                    "  //Get the fragment for texture 2\n"
                    "  vec4 tex2 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer2_enabled)\n"
                    "  {\n"
                    "    tex2 = texture2D(osgEarth_Layer2_unit, texCoord2);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex2.a *= osgEarth_Layer2_opacity;\n"
                    "\n"
                    "  //Get the fragment for texture 2\n"
                    "  vec4 tex3 = vec4(0.0,0.0,0.0,0.0);\n"
                    "  if (osgEarth_Layer3_enabled)\n"
                    "  {\n"
                    "    tex3 = texture2D(osgEarth_Layer3_unit, texCoord3);\n"
                    "    numLayersOn++;\n"
                    "  }\n"
                    "  tex3.a *= osgEarth_Layer3_opacity;\n"
                    "\n"
                    "  if (numLayersOn > 0)\n"
                    "  {\n"
                    "    //Interpolate the color between the first layer and second\n"
                    "    vec3 c = mix(tex0.rgb, tex1.rgb, tex1.a);\n"
                    "\n"
                    "    c = mix(c, tex2.rgb, tex2.a);\n"
                    " \n"
                    "    c = mix(c, tex3.rgb, tex3.a);\n"
                    " \n"
                    "    //Take the maximum alpha for the final destination alpha\n"
                    "    float a = tex0.a;\n"
                    "    if (a < tex1.a) a = tex1.a;\n"
                    "    if (a < tex2.a) a = tex2.a;\n"
                    "    if (a < tex3.a) a = tex3.a;\n"
                    "\n"
                    "    gl_FragColor = gl_Color * vec4(c, a);\n"
                    "  }\n"
                    "  else\n"
                    "  {\n"
                    "    //No layers are on, so just use the underlying color\n"
                    "    gl_FragColor = gl_Color;\n"
                    "  }\n"
                    "}\n"
                    "\n";

static
MapEngineCache& getMapEngineCache()
{
    static MapEngineCache s_cache;
    return s_cache;
}


void
MapEngine::registerMapEngine(MapEngine* map)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapEngineCacheMutex);
    getMapEngineCache()[map->id] = map;
    osg::notify(osg::INFO) << "Registered " << map->id << std::endl;
}

void
MapEngine::unregisterMapEngine(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapEngineCacheMutex);
    MapEngineCache::iterator k = getMapEngineCache().find( id);
    if (k != getMapEngineCache().end())
    {
        getMapEngineCache().erase(k);
        osg::notify(osg::INFO) << "Unregistered " << id << std::endl;
    }
}

MapEngine*
MapEngine::getMapEngineById(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapEngineCacheMutex);
    MapEngineCache::const_iterator k = getMapEngineCache().find( id);
    if (k != getMapEngineCache().end()) return k->second.get();
    return 0;
}

unsigned int
MapEngine::getId() const
{
    return id;
}

const Profile*
MapEngine::getProfile() const
{
    return _profile.get();
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

void
MapEngine::addSources(const MapConfig& map_conf,
                      const SourceConfigList& from, 
                      std::vector< osg::ref_ptr<TileSource> >& to )
{       
    TileSourceFactory factory;

    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        const SourceConfig& source_conf = *i;

        osg::ref_ptr<TileSource> tileSource = factory.createMapTileSource( source_conf, map_conf );

        if ( tileSource.valid() )
        {
            to.push_back( tileSource.get() );
        }
    }
}

MapEngine::MapEngine( const MapConfig& mapConfig ):
_mapConfig( mapConfig )
{
    //if ( !mapConfig )
    //    return;  

    // the state set that will hold the layer effect uniforms:
    uniformStateSet = new osg::StateSet();

    // load all the startup layers.
    initializeLayers();

    // Set the MapConfig's Profile to the computed profile so that the TileSource's can query it when they are loaded
    //_mapConfig.setProfile( _profile.get() );

    id = s_mapEngineID++;

    //Register the map
    registerMapEngine( this );
}

MapEngine::~MapEngine()
{
    //osg::notify(osg::NOTICE) << "Deleting Map " << getId() << std::endl;
    unregisterMapEngine( getId() );
}

std::string
MapEngine::createURI( const TileKey* key )
{
    std::stringstream ss;
    ss << key->str() << "." <<id<<".earth_tile";
    return ss.str();
}

const MapConfig&
MapEngine::getMapConfig() const
{
    return _mapConfig; //.get();
}

osg::CoordinateSystemNode* MapEngine::createCoordinateSystemNode() const
{
    return new osg::CoordinateSystemNode();
}


// Make a MatrixTransform suitable for use with a Locator object based on the given extents.
// Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
// _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
osg::Matrixd
MapEngine::getTransformFromExtents(double minX, double minY, double maxX, double maxY) const
{
    osg::Matrixd transform;
    transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0); 
    return transform;
}


bool
MapEngine::isOK() const
{
    if ( !_profile.valid() )
    {
        osg::notify(osg::NOTICE) << "Error: Unable to determine a map profile." << std::endl;
        return false;
    }

    //if (getImageSources().size() == 0 && getHeightFieldSources().size() == 0)
    //{
    //    osg::notify(osg::NOTICE) << "Error: Map does not contain any image or heightfield sources." << std::endl;
    //    return false;
    //}

    //Check to see if we are trying to do a Geocentric database with a Projected profile.
    if ( _profile->getProfileType() == Profile::TYPE_LOCAL && 
        _mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC)
    {
        osg::notify(osg::NOTICE) << "Error: Cannot create a geocentric scene using projected datasources.  Please specify type=\"flat\" on the map element in the .earth file." << std::endl;
        return false;
    }

    //TODO: Other cases?
    return true;
}


osg::Node*
MapEngine::createNode( const TileKey* key )
{
    osg::ref_ptr<osg::Group> parent = new osg::Group;
    if (!addChildren( parent.get(), key ))
    {
        parent = 0;
    }
    return parent.release();
}

GeoImage*
MapEngine::createValidGeoImage(TileSource* tileSource, const TileKey* key)
{
    //Try to create the image with the given key
    osg::ref_ptr<const TileKey> image_key = key;

    osg::ref_ptr<GeoImage> geo_image;

    while (image_key.valid())
    {
        if ( tileSource->isKeyValid(image_key.get()) )
        {
            geo_image = createGeoImage( image_key.get(), tileSource );
            if (geo_image.valid()) return geo_image.release();
        }
        image_key = image_key->createParentKey();
    }
    return 0;
}

bool
MapEngine::hasMoreLevels( const TileKey* key )
{
    ImageLayerList imageLayers;
    getImageLayers( imageLayers );

    ElevationLayerList elevationLayers;
    getElevationLayers( elevationLayers );

    bool more_levels = false;
    int max_level = 0;

    for (ImageLayerList::const_iterator i = imageLayers.begin(); i != imageLayers.end(); i++)
    {
        if ( key->getLevelOfDetail() < i->get()->getTileSource()->getMaxLevel() )
        {
            more_levels = true;
            break;
        }
    }
    if ( !more_levels )
    {
        for( ElevationLayerList::const_iterator j = elevationLayers.begin(); j != elevationLayers.end(); j++)
        {
            if ( key->getLevelOfDetail() < j->get()->getTileSource()->getMaxLevel() )
            {
                more_levels = true;
                break;
            }
        }
    }

    return more_levels;
}

bool
MapEngine::addChildren( osg::Group* tile_parent, const TileKey* key )
{
    bool all_quadrants_created = false;

    osg::ref_ptr<osg::Node> q0, q1, q2, q3;

    q0 = createQuadrant( key->getSubkey(0) );
    q1 = createQuadrant( key->getSubkey(1) );
    q2 = createQuadrant( key->getSubkey(2) );
    q3 = createQuadrant( key->getSubkey(3) );

    all_quadrants_created = (q0.valid() && q1.valid() && q2.valid() && q3.valid());

    if (all_quadrants_created)
    {
        if (q0.valid()) tile_parent->addChild(q0.get());
        if (q1.valid()) tile_parent->addChild(q1.get());
        if (q2.valid()) tile_parent->addChild(q2.get());
        if (q3.valid()) tile_parent->addChild(q3.get());
    }
    else
    {
        osg::notify(osg::INFO) << "Couldn't create all quadrants for " << key->str() << " time to stop subdividing!" << std::endl;
    }
    return all_quadrants_created;
}


GeoImage*
MapEngine::createGeoImage(const TileKey* mapKey, TileSource* source)
{
    GeoImage* result = NULL;
    const Profile* mapProfile = mapKey->getProfile();

    //If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( source->getProfile() ) )
    {
        osg::Image* image = source->createImage( mapKey );
        if ( image )
        {
            result = new GeoImage( image, mapKey->getGeoExtent() );
        }
    }

    // Otherwise, we need to process the tiles.
    else
    {
        Compositor comp;
        osg::ref_ptr<GeoImage> mosaic = comp.mosaicImages( mapKey, source );

        if ( mosaic.valid() )
        {
            if ( ! (mosaic->getSRS()->isEquivalentTo( mapKey->getProfile()->getSRS()) ) &&
                !(mosaic->getSRS()->isMercator() && mapKey->getProfile()->getSRS()->isGeographic() ) )
            {
                //We actually need to reproject the image.  Note:  The GeoImage::reprojection function will automatically
                //crop the image to the correct extents, so there is no need to crop after reprojection.
                //osgDB::writeImageFile(*mosaic->getImage(), "c:/temp/mosaic_" + mapKey->str() + ".png");
                result = mosaic->reproject( mapKey->getProfile()->getSRS(), &mapKey->getGeoExtent() );
                //osgDB::writeImageFile(*result->getImage(), "c:/temp/reprojected_" + mapKey->str() + ".png");
                //osg::notify(osg::NOTICE) << "Reprojected mosaic" << std::endl;
            }
            else
            {
                // crop to fit the map key extents
                GeoExtent clampedMapExt = source->getProfile()->clampAndTransformExtent( mapKey->getGeoExtent() );
                result = mosaic->crop(clampedMapExt);
            }
        }
    }

    return result;
}

bool
MapEngine::isCached(const osgEarth::TileKey *key)
{
    const Profile* mapProfile = key->getProfile();

    //Check the imagery layers
    ImageLayerList imageLayers;
    getImageLayers(imageLayers);

    for (unsigned int i = 0; i < imageLayers.size(); ++i)
    {
        std::vector< osg::ref_ptr< const TileKey > > keys;

        if ( _profile->isEquivalentTo( imageLayers[i]->getTileSource()->getProfile() ) )
        {
            keys.push_back(key);
        }
        else
        {
            imageLayers[i]->getTileSource()->getProfile()->getIntersectingTiles(key, keys);
        }

        for (unsigned int j = 0; j < keys.size(); ++j)
        {
            if (imageLayers[i]->getTileSource()->isKeyValid( keys[j].get() ) )
            {
                if (!imageLayers[i]->getTileSource()->isCached(keys[j].get()))
                {
                    return false;
                }
            }
        }
    }

    //Check the elevation layers
    ElevationLayerList elevationLayers;
    getElevationLayers(elevationLayers);

    for (unsigned int i = 0; i < elevationLayers.size(); ++i)
    {
        std::vector< osg::ref_ptr< const TileKey > > keys;

        if ( _profile->isEquivalentTo( elevationLayers[i]->getTileSource()->getProfile() ) )
        {
            keys.push_back(key);
        }
        else
        {
            elevationLayers[i]->getTileSource()->getProfile()->getIntersectingTiles(key, keys);
        }


        for (unsigned int j = 0; j < keys.size(); ++j)
        {
            if (elevationLayers[i]->getTileSource()->isKeyValid( keys[j].get() ) )
            {
                if (!elevationLayers[i]->getTileSource()->isCached(keys[j].get()))
                {
                    return false;
                }
            }
        }
    }

    return true;
}



// figures out what the map profile should be. there are multiple ways of setting it.
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
MapEngine::initializeLayers()
{
    //Collect the image and heightfield sources.
    TileSourceList image_sources;
    TileSourceList heightfield_sources;

    addSources( _mapConfig, _mapConfig.getImageSources(), image_sources );
    addSources( _mapConfig, _mapConfig.getHeightFieldSources(), heightfield_sources );

    TileSource* ref_source = NULL;

    if (_mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
    {
        //If the map type if Geocentric, set the profile to global-geodetic
        _profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        osg::notify(osg::INFO) << "[osgEarth] Setting Profile to global-geodetic for geocentric scene" << std::endl;
    }
    else if ( _mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE )
    {
        //If the map type is a Geocentric Cube, set the profile to the cube profile.
        _profile = osgEarth::Registry::instance()->getCubeProfile();
        osg::notify(osg::INFO) << "[osgEarth] Using cube profile for geocentric scene" << std::endl;
    }

    // First check for an explicit profile declaration:
    if ( !_profile.valid() && _mapConfig.getProfileConfig().defined() )
    {
        // Check for a "well known named" profile:
        std::string namedProfile = _mapConfig.getProfileConfig().getNamedProfile();
        if ( !namedProfile.empty() )
        {
            _profile = osgEarth::Registry::instance()->getNamedProfile( namedProfile );
            if ( _profile.valid() )
            {
                osg::notify(osg::INFO) << "[osgEarth] Set map profile to " << namedProfile << std::endl;
            }
            else
            {
                osg::notify(osg::WARN) << "[osgEarth] " << namedProfile << " is not a known profile name" << std::endl;
                //TODO: continue on? or fail here?
            }
        }

        // Check for a TileSource reference (i.e. get the map profile from a particular TileSource)
        if ( !_profile.valid() )
        {
            std::string refLayer = _mapConfig.getProfileConfig().getRefLayer();
            if ( !refLayer.empty() )
            {
                //Search through the image sources to find the reference TileSource
                for (TileSourceList::iterator itr = image_sources.begin(); itr != image_sources.end(); ++itr)
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
                    for (TileSourceList::iterator itr = heightfield_sources.begin(); itr != heightfield_sources.end(); ++itr)
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
                    const Profile* ref_profile = ref_source->initProfile( NULL, _mapConfig.getFilename() );
                    if ( ref_profile )
                    {
                        _profile = getSuitableMapProfileFor( ref_profile );
                        osg::notify(osg::INFO) << "[osgEarth] Setting profile from \"" << refLayer << "\"" << std::endl;
                    }
                }
                else
                {
                    osg::notify(osg::WARN) << "[osgEarth] Source \"" << refLayer << "\" does not have a valid profile" << std::endl;
                }
            }
        }

        // Try to create a profile from an explicit definition (the SRS and extents)
        if ( !_profile.valid() )
        {
            if ( _mapConfig.getProfileConfig().areExtentsValid() )
            {
                double minx, miny, maxx, maxy;
                _mapConfig.getProfileConfig().getExtents( minx, miny, maxx, maxy );

                // TODO: should we restrict this? This is fine for LOCAL/PROJECTED, but since we are not
                // constraining non-local map profiles to the "well known" types, should we let the user
                // override that? probably...
                _profile = Profile::create( _mapConfig.getProfileConfig().getSRS(), minx, miny, maxx, maxy );

                if ( _profile.valid() )
                {
                    osg::notify( osg::INFO ) << "[osgEarth::Map] Set map profile from SRS: " 
                        << _profile->getSRS()->getName() << std::endl;
                }
            }
        }
    }

    // At this point we MIGHT have a profile.

    // Finally, try scanning the loaded sources and taking the first one we get. At the
    // same time, remove any incompatible sources.

    for( TileSourceList::iterator i = image_sources.begin(); i != image_sources.end(); )
    {
        // skip the reference source since we already initialized it
        if ( i->get() != ref_source )
        {
            osg::ref_ptr<const Profile> sourceProfile = (*i)->initProfile( _profile.get(), _mapConfig.getFilename() );

            if ( !_profile.valid() && sourceProfile.valid() )
            {
                _profile = getSuitableMapProfileFor( sourceProfile.get() );
            }
            else if ( !sourceProfile.valid() )
            {
                osg::notify(osg::WARN) << "[osgEarth] Removing invalid TileSource " << i->get()->getName() << std::endl;
                i =image_sources.erase(i);
                continue;
            }
        }

        if ( osg::getNotifyLevel() >= osg::INFO )
        {
            std::string prof_str = i->get()->getProfile()? i->get()->getProfile()->toString() : "none";
            osg::notify(osg::INFO)
                << "Tile source \"" << i->get()->getName() << "\" : profile = " << prof_str << std::endl;
        }

        i++;
    }

    for (TileSourceList::iterator i = heightfield_sources.begin(); i != heightfield_sources.end(); )
    {        
        if ( i->get() != ref_source )
        {
            osg::ref_ptr<const Profile> sourceProfile = (*i)->initProfile( _profile.get(), _mapConfig.getFilename() );

            if ( !_profile.valid() && sourceProfile.valid() )
            {
                _profile = getSuitableMapProfileFor( sourceProfile.get() );
            }
            else if ( !sourceProfile.valid() )
            {
                osg::notify(osg::WARN) << "[osgEarth] Removing invalid TileSource " << i->get()->getName() << std::endl;
                i = heightfield_sources.erase(i);
                continue;
            }
        }

        if ( osg::getNotifyLevel() >= osg::INFO )
        {
            std::string prof_str = i->get()->getProfile()? i->get()->getProfile()->toString() : "none";
            osg::notify(osg::INFO)
                << "Tile source \"" << i->get()->getName() << "\" : profile = " << prof_str << std::endl;
        }

        i++;
    }

    //Add all the elevation layers
    for (TileSourceList::iterator itr = heightfield_sources.begin(); itr != heightfield_sources.end(); ++itr)
    {
        ElevationLayer *elevationLayer = new ElevationLayer( itr->get() );
        elevationLayer->setName( itr->get()->getName() );
        addLayer( elevationLayer );
    }

    //Add all the imagery layers
    for (TileSourceList::iterator itr = image_sources.begin(); itr != image_sources.end(); ++itr)
    {
        ImageLayer *imageLayer = new ImageLayer( itr->get() );
        imageLayer->setName( itr->get()->getName() );
        addLayer( imageLayer );
    }
}



osg::HeightField*
MapEngine::createHeightField( const TileKey* key, bool fallback )
{   
    osg::ref_ptr< ElevationManager > em = new ElevationManager;
    ElevationLayerList elevationLayers;
    getElevationLayers( elevationLayers );

    for (ElevationLayerList::iterator itr = elevationLayers.begin(); itr != elevationLayers.end(); ++itr)
    {
        em->getElevationSources().push_back( itr->get()->getTileSource() );
    }
    return em->createHeightField( key, 0, 0, fallback );
}

osg::HeightField*
MapEngine::createEmptyHeightField( const TileKey* key )
{
    //Get the bounds of the key
    double minx, miny, maxx, maxy;
    key->getGeoExtent().getBounds(minx, miny, maxx, maxy);

    osg::HeightField *hf = new osg::HeightField();
    hf->allocate( 16, 16 );
    for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
        hf->getHeightList()[i] = 0.0;

    hf->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
    hf->setXInterval( (maxx - minx)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (maxy - miny)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );
    return hf;
}

TileSource*
MapEngine::createTileSource(const osgEarth::SourceConfig& sourceConfig)
{
    //Create the TileSource
    TileSourceFactory factory;
    osg::ref_ptr<TileSource> tileSource = factory.createMapTileSource(sourceConfig, _mapConfig );

    if (tileSource.valid())
    {
        const Profile* profile = tileSource->initProfile( getProfile(), getMapConfig().getFilename() );
        if (!profile)
        {
            osg::notify(osg::NOTICE) << "Could not initialize profile " << std::endl;
            tileSource = NULL;
        }
    }
    return tileSource.release();
}

OpenThreads::ReadWriteMutex&
MapEngine::getLayersMutex() {
    return _layersMutex;
}

void
MapEngine::dirtyLayers()
{
    updateUniforms();
}


void
MapEngine::updateUniforms()
{
    ImageLayerList imageLayers;
    getImageLayers( imageLayers );

    unsigned int maxLayers = 4;
    for (unsigned int i = 0; i < maxLayers; ++i)
    {        
        ImageLayer* layer = i < imageLayers.size() ? imageLayers[i].get() : 0;

        std::stringstream ss;
        ss << "osgEarth_Layer" << i << "_unit";
        osg::Uniform* unitUniform = uniformStateSet->getOrCreateUniform(ss.str(), osg::Uniform::INT);
        unitUniform->set( (int)i );
        
        ss.str("");
        ss << "osgEarth_Layer" << i << "_enabled";
        osg::Uniform* enabledUniform = uniformStateSet->getOrCreateUniform(ss.str(), osg::Uniform::BOOL);
        enabledUniform->set(layer ? layer->getEnabled() : false);

        ss.str("");
        ss << "osgEarth_Layer" << i << "_opacity";
        osg::Uniform* opacityUniform = uniformStateSet->getOrCreateUniform(ss.str(), osg::Uniform::FLOAT);
        opacityUniform->set(layer ? layer->getOpacity() : 0.0f);
    }
}



osg::Node*
MapEngine::initialize()
{    
    // Note: CSN must always be at the top
    osg::CoordinateSystemNode* csn = createCoordinateSystemNode();

    // apply the uniform state set to this node
    csn->setStateSet( uniformStateSet.get() );

    // go through and build the root nodesets.
    int faces_ok = 0;
    for( int face = 0; face < getProfile()->getNumFaces(); face++ )
    {
        EarthTerrain* terrain = new EarthTerrain;
        terrain->setVerticalScale( _mapConfig.getVerticalScale() );
        terrain->setSampleRatio( _mapConfig.getSampleRatio() );
        csn->addChild( terrain );
        _terrains.push_back( terrain );

        std::vector< osg::ref_ptr<TileKey> > keys;
        getProfile()->getFaceProfile( face )->getRootKeys( keys, face );

        int numAdded = 0;
        for (unsigned int i = 0; i < keys.size(); ++i)
        {
            osg::Node* node = createNode( keys[i].get() );
            if (node)
            {
                terrain->addChild(node);
                numAdded++;
            }
            else
            {
                osg::notify(osg::NOTICE) << "Couldn't get tile for " << keys[i]->str() << std::endl;
            }
        }
        if ( numAdded == keys.size() )
            faces_ok++;
    }

    //if ( faces_ok == getProfile()->getNumFaces() )
        //    if (numAdded == keys.size())
    //{
        //csn->addChild( csn.release() );
    //}


    // set up the layer effect uniforms.

    osg::Program* program = new osg::Program;
    program->addShader(new osg::Shader( osg::Shader::VERTEX, vert_source ) );
    program->addShader(new osg::Shader( osg::Shader::FRAGMENT, frag_source ) );

    uniformStateSet->setAttributeAndModes(program, osg::StateAttribute::ON);

    //Enable blending
    uniformStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);

    updateUniforms();

    return csn;
}


void
MapEngine::addLayer( Layer* layer )
{
    osg::notify(osg::INFO) << "[osgEarth::MapEngine::addLayer] Begin " << std::endl;
    if (layer)
    {        
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::addLayer] Waiting for lock..." << std::endl;
        OpenThreads::ScopedWriteLock lock(_layersMutex);
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::addLayer] Obtained lock " << std::endl;

        //Add the layer to the list
        _layers.push_back( layer );
        //layer->setMap( this );

        bool addedImage = (dynamic_cast<ImageLayer*>( layer ) != NULL);
        bool addedElevation = (dynamic_cast<ElevationLayer*>( layer ) != NULL);

        //Add the color layer to the end of the list
        for (unsigned int i = 0; i < _terrains.size(); ++i)
        {            
            EarthTerrain* terrain = static_cast<EarthTerrain*>(_terrains[i].get());
            EarthTerrain::TerrainTileList tiles;
            terrain->getTerrainTiles( tiles );
            osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

            for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
            {
                OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());

                //Create a TileKey from the TileID
                osgTerrain::TileID tileId = itr->get()->getTileID();
                osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );

                if ( addedImage )
                {
                    osg::ref_ptr< GeoImage > geoImage = createValidGeoImage( ((ImageLayer*)layer)->getTileSource(), key.get() );

                    if (geoImage.valid())
                    {
                        double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

                        //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
                        osg::ref_ptr<osgTerrain::Locator> img_locator; // = key->getProfile()->getSRS()->createLocator();

                        // Use a special locator for mercator images (instead of reprojecting)
                        if ( geoImage->getSRS()->isMercator() )
                        {
                            GeoExtent geog_ext = geoImage->getExtent().transform(geoImage->getExtent().getSRS()->getGeographicSRS());
                            geog_ext.getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                            img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                            img_locator = new MercatorLocator( *img_locator.get(), geoImage->getExtent() );
                            //Transform the mercator extents to geographic
                        }
                        else
                        {
                            geoImage->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                            img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                        }

                        //Set the CS to geocentric is we are dealing with a geocentric map
                        if (_mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC || _mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE)
                        {
                            img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
                        }

                        osgTerrain::ImageLayer* img_layer = new osgEarthImageLayer( layer->getId(), geoImage->getImage() );
                        img_layer->setLocator( img_locator.get());

                        unsigned int newLayer = _layers.size()-1;
                        osg::notify(osg::INFO) << "Inserting layer at position " << newLayer << std::endl;
                        itr->get()->setColorLayer(newLayer, img_layer );
                    }
                }
                else
                {
                    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
                    if (heightFieldLayer)
                    {
                        osg::HeightField* hf = createHeightField( key, true );
                        if (!hf) hf = createEmptyHeightField( key.get() );
                        heightFieldLayer->setHeightField( hf );
                        hf->setSkirtHeight( itr->get()->getBound().radius() * _mapConfig.getSkirtRatio() );
                    }
                }
                itr->get()->setDirty(true);
            }
        }

        updateUniforms();
    }
}

void
MapEngine::removeLayer( Layer* layer )
{
    if (layer)
    {
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::removeLayer] Begin " << std::endl;
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::removeLayer] Waiting for lock" << std::endl;
        OpenThreads::ScopedWriteLock lock(_layersMutex);
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::removeLayer] Obtained for lock" << std::endl;
        
        //Store the layer ID
        int layerId = layer->getId();

        //Determine what kind of layer was removed
        bool imageLayerRemoved = (dynamic_cast<osgEarth::ImageLayer*>(layer) != NULL);
        bool elevationLayerRemoved = (dynamic_cast<osgEarth::ElevationLayer*>(layer) != NULL);

        //Erase the layer from the list       
        LayerList::iterator itr = std::find(_layers.begin(), _layers.end(), layer);
        if (itr != _layers.end())
        {
            _layers.erase(itr);
        }
        else
        {
            osg::notify(osg::NOTICE) << "[osgEarth::MapEngine::removeLayer] Could not find image layer with ID " << layerId << std::endl;
        }

        for (unsigned int i = 0; i < _terrains.size(); ++i)
        {            
            EarthTerrain* terrain = static_cast<EarthTerrain*>(_terrains[i].get());
            EarthTerrain::TerrainTileList tiles;
            terrain->getTerrainTiles( tiles );
            //osg::notify(osg::NOTICE) << "Found " << tiles.size() << std::endl;

            for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
            {
                OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());
                if (imageLayerRemoved)
                {
                    //An image layer was removed, so reorganize the color layers in the tiles to account for it's removal
                    std::vector< osg::ref_ptr< osgEarth::osgEarthImageLayer > > layers;
                    for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
                    {              
                        osgEarthImageLayer* imageLayer = dynamic_cast<osgEarthImageLayer*>(itr->get()->getColorLayer( i ));
                        if (imageLayer && imageLayer->getLayerId() != layerId)
                        {
                            layers.push_back(imageLayer);
                        }
                        //Set the current value to NULL
                        itr->get()->setColorLayer( i, NULL);
                    }

                    //Reset the color layers to the correct order
                    for (unsigned int i = 0; i < layers.size(); ++i)
                    {
                        itr->get()->setColorLayer( i, layers[i].get() );
                    }
                }
                else if (elevationLayerRemoved)
                {
                    osgTerrain::TileID tileId = itr->get()->getTileID();
                    osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );
                    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
                    if (heightFieldLayer)
                    {
                        osg::HeightField* hf = createHeightField( key, true );
                        if (!hf) hf = createEmptyHeightField( key.get() );
                        heightFieldLayer->setHeightField( hf );
                        hf->setSkirtHeight( itr->get()->getBound().radius() * _mapConfig.getSkirtRatio() );
                    }
                }
                itr->get()->setDirty(true);
            }
        }
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::removeLayer] end " << std::endl;
        updateUniforms();
    }
}

void
MapEngine::moveLayer( Layer* layer, int position )
{
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveLayer] Begin" << std::endl;
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveLayer] Waiting for lock..." << std::endl;
    OpenThreads::ScopedWriteLock lock(_layersMutex);
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveLayer] Obtained lock" << std::endl;
    if (layer)
    {

        //Determine what kind of layer was removed
        bool movedImage = (dynamic_cast<osgEarth::ImageLayer*>(layer) != NULL);
        bool movedElevation = (dynamic_cast<osgEarth::ElevationLayer*>(layer) != NULL);

        int relativeIndexOrig= -1;
        int relativeIndexNew = -1;

        if (movedImage)
        {
            ImageLayerList imageLayers;
            getImageLayers( imageLayers );
            for (unsigned int i = 0; i < imageLayers.size(); ++i) if (imageLayers[i].get() == layer) { relativeIndexOrig = i; break;}
            osg::notify(osg::INFO) << "Image layer relative position=" << relativeIndexOrig << std::endl;
        }
        else if (movedElevation)
        {
            ElevationLayerList elevationLayers;
            getElevationLayers( elevationLayers );
            for (unsigned int i = 0; i < elevationLayers.size(); ++i) if (elevationLayers[i].get() == layer) { relativeIndexOrig = i; break;}
            osg::notify(osg::INFO) << "Elevation layer relative position=" << relativeIndexOrig << std::endl;
        }

        //Find the original position of the Layer
        unsigned int index = -1;
        for (unsigned int i = 0; i < _layers.size(); ++i)
        {
            if (_layers[i].get() == layer)
            {
                index = i;
                break;
            }
        }
        osg::notify(osg::INFO) << "Found layer at " << index << std::endl;

        osg::ref_ptr<Layer> l = layer;
        _layers.erase(_layers.begin() + index);
        _layers.insert(_layers.begin() + position, l.get());

        if (movedImage)
        {
            ImageLayerList imageLayers;
            getImageLayers( imageLayers );
            for (unsigned int i = 0; i < imageLayers.size(); ++i) if (imageLayers[i].get() == layer) { relativeIndexNew = i; break;}
            osg::notify(osg::INFO) << "New Image layer relative position=" << relativeIndexNew << std::endl;
        }
        else if (movedElevation)
        {
            ElevationLayerList elevationLayers;
            getElevationLayers( elevationLayers );
            for (unsigned int i = 0; i < elevationLayers.size(); ++i) if (elevationLayers[i].get() == layer) { relativeIndexNew = i; break;}
            osg::notify(osg::INFO) << "New Elevation layer relative position=" << relativeIndexNew << std::endl;
        }

        for (unsigned int i = 0; i < _terrains.size(); ++i)
        {            
            EarthTerrain* terrain = static_cast<EarthTerrain*>(_terrains[i].get());
            EarthTerrain::TerrainTileList tiles;
            terrain->getTerrainTiles( tiles );
            osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

            for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
            {
                OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());
                if (movedImage)
                {
                    std::vector< osg::ref_ptr< osgEarth::osgEarthImageLayer > > layers;
                    for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
                    {              
                        layers.push_back((osgEarthImageLayer*)itr->get()->getColorLayer(i));
                    }

                    //Swap the original position
                    osg::ref_ptr< osgEarth::osgEarthImageLayer > layer = layers[relativeIndexOrig];
                    layers.erase(layers.begin() + relativeIndexOrig);
                    layers.insert(layers.begin() + relativeIndexNew, layer.get());

                    for (unsigned int i = 0; i < layers.size(); ++i)
                    {
                        itr->get()->setColorLayer( i, layers[i].get() );
                    }
                }
                else if (movedElevation)
                {
                    osgTerrain::TileID tileId = itr->get()->getTileID();
                    osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );
                    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
                    if (heightFieldLayer)
                    {
                        osg::HeightField* hf = createHeightField( key, true );
                        if (!hf) hf = createEmptyHeightField( key.get() );
                        heightFieldLayer->setHeightField( hf );
                        hf->setSkirtHeight( itr->get()->getBound().radius() * _mapConfig.getSkirtRatio() );
                    }
                }
                itr->get()->setDirty(true);
            }
        }
        
        updateUniforms();
    }
    osg::notify(osg::INFO) << "[osgEarth::MapEngine::moveLayer] end " << std::endl;
}

Layer*
MapEngine::getLayer( unsigned int i ) const
{
    return i < _layers.size() ? _layers[i].get() : 0;
}

unsigned int
MapEngine::getNumLayers() const
{
    return _layers.size();
}

void
MapEngine::getElevationLayers(osgEarth::ElevationLayerList& layers) const
{
    layers.clear();
    for (LayerList::const_iterator itr = _layers.begin(); itr != _layers.end(); ++itr)
    {
        ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(itr->get());
        if (elevationLayer)
        {
            layers.push_back(elevationLayer);
        }
    }
}

void
MapEngine::getImageLayers(osgEarth::ImageLayerList& layers) const
{
    layers.clear();
    for (LayerList::const_iterator itr = _layers.begin(); itr != _layers.end(); ++itr)
    {
        ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(itr->get());
        if (imageLayer)
        {
            layers.push_back(imageLayer);
        }
    }
}
