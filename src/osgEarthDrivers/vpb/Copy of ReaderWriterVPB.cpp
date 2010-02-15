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

#include <osgEarth/Registry>
#include <osgEarth/TileSource>
#include <osgEarth/HTTPClient>

#include <osg/Notify>
#include <osg/io_utils>
#include <osg/Version>
#include <osgTerrain/Terrain>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include <sstream>

using namespace osgEarth;
using namespace OpenThreads;

#define PROPERTY_URL                    "url"
#define PROPERTY_PRIMARY_SPLIT_LEVEL    "primary_split_level"
#define PROPERTY_SECONDARY_SPLIT_LEVEL  "secondary_split_level"
#define PROPERTY_DIRECTORY_STRUCTURE    "directory_structure"
#define PROPERTY_LAYER_NUM              "layer"
#define PROPERTY_NUM_TILES_WIDE_AT_LOD0 "num_tiles_wide_at_lod0"
#define PROPERTY_NUM_TILES_HIGH_AT_LOD0 "num_tiles_high_at_lod0"
#define PROPERTY_BASE_NAME              "base_name"


class CollectTiles : public osg::NodeVisitor
{
public:

    CollectTiles(): 
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }
    
    void reset()
    {
        _terrainTiles.clear();
    }
    
    void apply(osg::Group& group)
    {
        osgTerrain::TerrainTile* terrainTile = dynamic_cast<osgTerrain::TerrainTile*>(&group);
        if (terrainTile)
        {
            osg::notify(osg::INFO)<<"Found terrain tile TileID("<<
				TileKey::getLOD(terrainTile->getTileID())<<", "<<
                terrainTile->getTileID().x<<", "<<
                terrainTile->getTileID().y<<")"<<std::endl;
            
            _terrainTiles.push_back(terrainTile);
        }
        else
        {
            traverse(group);
        }
    }
    
    osgTerrain::Locator* getLocator()
    {
        for(unsigned int i=0; i<_terrainTiles.size(); ++i)
        {
            osgTerrain::TerrainTile* tile = _terrainTiles[i].get();
            osgTerrain::Locator* locator = tile->getLocator();
            if (locator) return locator;
        }
        return 0;
    }

    bool getRange(double& min_x, double& min_y, double& max_x, double& max_y) const
    {
        min_x = DBL_MAX;
        max_x = -DBL_MAX;
        min_y = DBL_MAX;
        max_y = -DBL_MAX;
        
        typedef std::vector<osg::Vec3d> Corners;
        Corners corners;
        corners.push_back(osg::Vec3d(0.0f,0.0f,0.0f));
        corners.push_back(osg::Vec3d(1.0f,0.0f,0.0f));
        corners.push_back(osg::Vec3d(1.0f,1.0f,0.0f));
        corners.push_back(osg::Vec3d(1.0f,1.0f,0.0f));
        
        for(unsigned int i=0; i<_terrainTiles.size(); ++i)
        {
            osgTerrain::TerrainTile* tile = _terrainTiles[i].get();
            osgTerrain::Locator* locator = tile->getLocator();
            if (locator)
            {
                for(Corners::iterator itr = corners.begin();
                    itr != corners.end();
                    ++itr)
                {
                    osg::Vec3d& local = *itr;
                    osg::Vec3d projected = local * locator->getTransform();

                    if (projected.x()<min_x) min_x = projected.x();
                    if (projected.x()>max_x) max_x = projected.x();

                    if (projected.y()<min_y) min_y = projected.y();
                    if (projected.y()>max_y) max_y = projected.y();
                }
            }
        }
        
        return min_x <= max_x;
    }
    
    typedef std::vector< osg::ref_ptr<osgTerrain::TerrainTile> > TerrainTiles;
    TerrainTiles _terrainTiles;
};


class VPBDatabase : public osg::Referenced
{
public:

    enum DirectoryStructure
    {
        FLAT,
        FLAT_TASK_DIRECTORIES,
        NESTED_TASK_DIRECTORIES
    };

    VPBDatabase(const PluginOptions* in_options ) :
        options( in_options),
        primary_split_level(-1),
        secondary_split_level(-1),
        directory_structure(FLAT_TASK_DIRECTORIES),
        maxNumTilesInCache(128),
        profile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() ),
        initialized( false )
    {
        //NOP
    }

    void init()
    {
        if ( initialized ) return;
        initialized = true;

        unsigned int numTilesWideAtLod0, numTilesHighAtLod0;
        profile->getNumTiles(0, numTilesWideAtLod0, numTilesHighAtLod0);

        if ( options.valid() )
        {
            const Config& conf = options->config();

            url = conf.value( PROPERTY_URL );
            primary_split_level = conf.value<int>( PROPERTY_PRIMARY_SPLIT_LEVEL, -1 );
            secondary_split_level = conf.value<int>( PROPERTY_SECONDARY_SPLIT_LEVEL, -1 );

            std::string dir_string = conf.value( PROPERTY_DIRECTORY_STRUCTURE );
            if (dir_string=="nested" || dir_string=="nested_task_directories" ) directory_structure = NESTED_TASK_DIRECTORIES;
            if (dir_string=="task" || dir_string=="flat_task_directories") directory_structure = FLAT_TASK_DIRECTORIES;
            if (dir_string=="flat") directory_structure = FLAT;
            
            base_name = conf.value( PROPERTY_BASE_NAME );
        }

        // validate dataset
        if (!url.empty())
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> localOptions = new osgDB::ReaderWriter::Options;
            localOptions->setPluginData("osgearth_vpb Plugin",(void*)(1));
            root_node = osgDB::readNodeFile(url, localOptions.get());


            if (root_node.valid())
            {
                path = osgDB::getFilePath(url);
                if ( base_name.empty() )
                    base_name = osgDB::getStrippedName(url);
                extension = osgDB::getFileExtension(url);
                
                osg::notify(osg::INFO)<<"VPBasebase constructor: Loaded root "<<url<<", path="<<path<<" base_name="<<base_name<<" extension="<<extension<<std::endl;
                
                std::string srs = profile->getSRS()->getInitString(); //.srs();
                
                osg::CoordinateSystemNode* csn = dynamic_cast<osg::CoordinateSystemNode*>(root_node.get());
                if (csn)
                {
                    osg::notify(osg::INFO)<<"CordinateSystemNode found, coordinate system is : "<<csn->getCoordinateSystem()<<std::endl;
                    
                    srs = csn->getCoordinateSystem();
                }

                CollectTiles ct;
                root_node->accept(ct);

                    
                osgTerrain::Locator* locator = ct.getLocator();
                if (locator)
                {
                    double min_x, max_x, min_y, max_y;
                    ct.getRange(min_x, min_y, max_x, max_y);

                    osg::notify(osg::INFO)<<"range("<<min_x<<", "<<min_y<<", "<<max_x<<", "<<max_y<<std::endl;

                    srs = locator->getCoordinateSystem();
                
                    //osgEarth::Profile::ProfileType ptype = osgEarth::Profile::TYPE_UNKNOWN;

                    //switch(locator->getCoordinateSystemType())
                    //{
                    //    case(osgTerrain::Locator::GEOCENTRIC):
                    //        ptype = Profile::TYPE_GEODETIC; //profile.setProfileType(osgEarth::Profile::GLOBAL_GEODETIC);
                    //        break;
                    //    case(osgTerrain::Locator::GEOGRAPHIC):
                    //        ptype = Profile::TYPE_LOCAL; //profile.setProfileType(osgEarth::Profile::PROJECTED);
                    //        break;
                    //    case(osgTerrain::Locator::PROJECTED):
                    //        ptype = Profile::TYPE_LOCAL; //profile.setProfileType(osgEarth::Profile::PROJECTED);
                    //        break;
                    //}

                    double aspectRatio = (max_x-min_x)/(max_y-min_y);
                    
                    osg::notify(osg::INFO)<<"aspectRatio = "<<aspectRatio<<std::endl;

                    if (aspectRatio>1.0)
                    {
                        numTilesWideAtLod0 = static_cast<unsigned int>(floor(aspectRatio+0.499999));
                        numTilesHighAtLod0 = 1;
                    }
                    else
                    {
                        numTilesWideAtLod0 = 1;
                        numTilesHighAtLod0 = static_cast<unsigned int>(floor(1.0/aspectRatio+0.499999));
                    }
                    
                    osg::notify(osg::INFO)<<"computed numTilesWideAtLod0 = "<<numTilesWideAtLod0<<std::endl;
                    osg::notify(osg::INFO)<<"computed numTilesHightAtLod0 = "<<numTilesHighAtLod0<<std::endl;
                    
                    if ( options.valid() )
                    {
                        if ( options->getPluginData( PROPERTY_NUM_TILES_WIDE_AT_LOD0 ) )
                            numTilesWideAtLod0 = atoi( (const char*)options->getPluginData( PROPERTY_NUM_TILES_WIDE_AT_LOD0 ) );

                        if ( options->getPluginData( PROPERTY_NUM_TILES_HIGH_AT_LOD0 ) )
                            numTilesHighAtLod0 = atoi( (const char*)options->getPluginData( PROPERTY_NUM_TILES_HIGH_AT_LOD0 ) );
                    }

                    osg::notify(osg::INFO)<<"final numTilesWideAtLod0 = "<<numTilesWideAtLod0<<std::endl;
                    osg::notify(osg::INFO)<<"final numTilesHightAtLod0 = "<<numTilesHighAtLod0<<std::endl;
                   
                    profile = osgEarth::Profile::create( 
                        srs,
                        osg::RadiansToDegrees(min_x), 
                        osg::RadiansToDegrees(min_y), 
                        osg::RadiansToDegrees(max_x), 
                        osg::RadiansToDegrees(max_y),
                        numTilesWideAtLod0,
                        numTilesHighAtLod0 );
                }
                
            }
            else
            {
                osg::notify(osg::NOTICE)<<"Unable to read file "<<url<<std::endl;
                url = "";
            }
        }
        else 
        {
            osg::notify(osg::NOTICE)<<"No data referenced "<<std::endl;
        }

    }
    
    std::string createTileName( int level, int tile_x, int tile_y )
    {
        init();
        std::stringstream buf;
        if (directory_structure==FLAT)
        {
             buf<<path<<"/"<<base_name<<"_L"<<level<<"_X"<<tile_x/2<<"_Y"<<tile_y/2<<"_subtile."<<extension;
        }
        else
        {
            if (level<primary_split_level)
            {
                buf<<path<<"/"<<base_name<<"_root_L0_X0_Y0/"<<
                     base_name<<"_L"<<level<<"_X"<<tile_x/2<<"_Y"<<tile_y/2<<"_subtile."<<extension;

            }
            else if (level<secondary_split_level)
            {
                tile_x /= 2;
                tile_y /= 2;

                int split_x = tile_x >> (level - primary_split_level);
                int split_y = tile_y >> (level - primary_split_level);

                buf<<path<<"/"<<base_name<<"_subtile_L"<<primary_split_level<<"_X"<<split_x<<"_Y"<<split_y<<"/"<<
                     base_name<<"_L"<<level<<"_X"<<tile_x<<"_Y"<<tile_y<<"_subtile."<<extension;
            }
            else if (directory_structure==NESTED_TASK_DIRECTORIES)
            {
                tile_x /= 2;
                tile_y /= 2;

                int split_x = tile_x >> (level - primary_split_level);
                int split_y = tile_y >> (level - primary_split_level);

                int secondary_split_x = tile_x >> (level - secondary_split_level);
                int secondary_split_y = tile_y >> (level - secondary_split_level);

                buf<<path<<"/"<<base_name<<"_subtile_L"<<primary_split_level<<"_X"<<split_x<<"_Y"<<split_y<<"/"<<
                     base_name<<"_subtile_L"<<secondary_split_level<<"_X"<<secondary_split_x<<"_Y"<<secondary_split_y<<"/"<< 
                     base_name<<"_L"<<level<<"_X"<<tile_x<<"_Y"<<tile_y<<"_subtile."<<extension;
            }
            else
            {
                tile_x /= 2;
                tile_y /= 2;

                int split_x = tile_x >> (level - secondary_split_level);
                int split_y = tile_y >> (level - secondary_split_level);

                buf<<path<<"/"<<base_name<<"_subtile_L"<<secondary_split_level<<"_X"<<split_x<<"_Y"<<split_y<<"/"<<
                     base_name<<"_L"<<level<<"_X"<<tile_x<<"_Y"<<tile_y<<"_subtile."<<extension;
            }
        }
        
        osg::notify(osg::INFO)<<"VPBDatabase::createTileName(), buf.str()=="<<buf.str()<<std::endl;
        
        return buf.str();
    }
    
    osgTerrain::TerrainTile* getTerrainTile( const TileKey* key, ProgressCallback* progress )
    {
        init();
        int level = key->getLevelOfDetail();
        unsigned int tile_x, tile_y;
        key->getTileXY( tile_x, tile_y );
        
        int max_x = (2 << level) - 1;
        int max_y = (1 << level) - 1;
        
        tile_y = max_y - tile_y;

        osgTerrain::TileID tileID(level, tile_x, tile_y);

        osg::ref_ptr<osgTerrain::TerrainTile> tile = findTile(tileID, false);
        if (tile.valid()) return tile.get();
        
        osg::notify(osg::INFO)<<"Max_x = "<<max_x<<std::endl;
        osg::notify(osg::INFO)<<"Max_y = "<<max_y<<std::endl;

        osg::notify(osg::INFO)<<"base_name = "<<base_name<<" psl="<<primary_split_level<<" ssl="<<secondary_split_level<<std::endl;
        osg::notify(osg::INFO)<<"level = "<<level<<", x = "<<tile_x<<", tile_y = "<<tile_y<<std::endl;
        osg::notify(osg::INFO)<<"tile_name "<<createTileName(level, tile_x, tile_y)<<std::endl;
        osg::notify(osg::INFO)<<"thread "<<OpenThreads::Thread::CurrentThread()<<std::endl;

        std::string filename = createTileName(level, tile_x, tile_y);
        
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(blacklistMutex);
            if (blacklistedFilenames.count(filename)==1)
            {
                osg::notify(osg::INFO)<<"file has been found in black list : "<<filename<<std::endl;

                insertTile(tileID, 0);
                return 0;
            }
        }
        

        osg::ref_ptr<osgDB::ReaderWriter::Options> localOptions = new osgDB::ReaderWriter::Options;
        localOptions->setPluginData("osgearth_vpb Plugin",(void*)(1));

        //osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename, localOptions.get());
        osg::ref_ptr<osg::Node> node;
        if (osgDB::containsServerAddress( filename ) )
        {
            node = HTTPClient::readNodeFile( filename, localOptions.get(), progress );
        }
        else
        {
            node = osgDB::readNodeFile( filename, localOptions.get() );
        }

        if (node.valid())
        {
            osg::notify(osg::INFO)<<"Loaded model "<<filename<<std::endl;
            CollectTiles ct;
            node->accept(ct);

            int base_x = (tile_x / 2) * 2;
            int base_y = (tile_y / 2) * 2;
            
            double min_x, max_x, min_y, max_y;
            ct.getRange(min_x, min_y, max_x, max_y);

            double center_x = (min_x + max_x)*0.5;
            double center_y = (min_y + max_y)*0.5;

            osg::Vec3d local(0.5,0.5,0.0);
            for(unsigned int i=0; i<ct._terrainTiles.size(); ++i)
            {
                osgTerrain::TerrainTile* tile = ct._terrainTiles[i].get();
                osgTerrain::Locator* locator = tile->getLocator();
                if (locator)
                {
                    osg::Vec3d projected = local * locator->getTransform();
                    
                    int local_x = base_x + ((projected.x() > center_x) ? 1 : 0);
                    int local_y = base_y + ((projected.y() > center_y) ? 1 : 0);
                    osgTerrain::TileID local_tileID(level, local_x, local_y);
                    
                    tile->setTileID(local_tileID);
                    insertTile(local_tileID, tile);
                }

            }

        }
        else
        {
            //Only blacklist if the request wasn't cancelled by the callback.
            if (!progress || (!progress->isCanceled()))
            {
                osg::notify(osg::INFO)<<"Black listing : "<<filename<<std::endl;
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(blacklistMutex);
                blacklistedFilenames.insert(filename);
            }
        }
        
        return findTile(tileID, true);
    }
    
    void insertTile(const osgTerrain::TileID& tileID, osgTerrain::TerrainTile* tile)
    {
        init();
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(tileMapMutex);

        if ( tileMap.find(tileID) == tileMap.end() )
        {
            tileMap[tileID] = tile;

            tileFIFO.push_back(tileID);

            if (tileFIFO.size()>maxNumTilesInCache)
            {
                osgTerrain::TileID tileToRemove = tileFIFO.front();
                tileFIFO.pop_front();
                tileMap.erase(tileToRemove);

			    osg::notify(osg::INFO)<<"Pruned tileID ("<<TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<")"<<std::endl;
            }

            osg::notify(osg::INFO)<<"insertTile ("
                << TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<") " 
                << " tileFIFO.size()=="<<tileFIFO.size()<<std::endl;
        }
        else
        {
            osg::notify(osg::INFO)<<"insertTile ("
                << TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<") " 
                << " ...already in cache!"<<std::endl;
        }
    }

    osgTerrain::TerrainTile* findTile(const osgTerrain::TileID& tileID, bool insertBlankTileIfNotFound = false)
    {
        init();
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(tileMapMutex);
            TileMap::iterator itr = tileMap.find(tileID);
            if (itr!=tileMap.end()) return itr->second.get();
        }

        if (insertBlankTileIfNotFound) insertTile(tileID, 0);

        return 0;
    }

    const Profile* getProfile() 
    {
        init();
        return profile.get();
    }

    const std::string& getExtension()
    {
        init();
        return extension;
    }

    bool initialized;

    osg::ref_ptr<const PluginOptions> options;
    std::string url;
    std::string path;
    std::string base_name;
    std::string extension;
    int primary_split_level;
    int secondary_split_level;
    DirectoryStructure directory_structure;

    osg::ref_ptr<const Profile> profile;
    osg::ref_ptr<osg::Node> root_node;
    
    unsigned int maxNumTilesInCache;
    
    typedef std::map<osgTerrain::TileID, osg::ref_ptr<osgTerrain::TerrainTile> > TileMap;
    TileMap tileMap;
    OpenThreads::Mutex tileMapMutex;
    
    typedef std::list<osgTerrain::TileID> TileIDList;
    TileIDList tileFIFO;
    
    typedef std::set<std::string> StringSet;
    StringSet blacklistedFilenames;
    OpenThreads::Mutex blacklistMutex;
    
};

class VPBSource : public TileSource
{
public:
    VPBSource( VPBDatabase* vpbDatabase, const PluginOptions* in_options) :  
      TileSource(in_options),
      _vpbDatabase(vpbDatabase),
      _layerNum(0)
    {
        if ( in_options )
        {
            _layerNum = in_options->config().value<int>( PROPERTY_LAYER_NUM, _layerNum );
        }
    }

    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
		if ( overrideProfile)
		{
			setProfile( overrideProfile );
		}
		else
		{
			setProfile( _vpbDatabase->getProfile() ); //profile.get() );
		}
    }
    
    osg::Image* createImage( const TileKey* key,
                             ProgressCallback* progress)
    {
        //TODO:  Make VPB driver use progress callback
        osg::ref_ptr<osgTerrain::TerrainTile> tile = _vpbDatabase->getTerrainTile(key, progress);                
        if (tile.valid())
        {        
            if (_layerNum < tile->getNumColorLayers())
            {
                osgTerrain::Layer* layer = tile->getColorLayer(_layerNum);
                osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>(layer);
                if (imageLayer)
                {
                    return imageLayer->getImage();
                }
            }
        }
        
        return 0;
    }

    osg::HeightField* createHeightField( const TileKey* key,
                                         ProgressCallback* progress
                                         )
    {
        osg::ref_ptr<osgTerrain::TerrainTile> tile = _vpbDatabase->getTerrainTile(key, progress);                
        if (tile.valid())
        {        
            osgTerrain::Layer* elevationLayer = tile->getElevationLayer();
            osgTerrain::HeightFieldLayer* hfLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(elevationLayer);
            if (hfLayer) 
            {
                return hfLayer->getHeightField();
            }
        }

        return 0;
    }

    virtual std::string getExtension()  const 
    {
        //All VPB tiles are in IVE format
        return _vpbDatabase->getExtension();
    }

private:
    osg::ref_ptr<VPBDatabase>   _vpbDatabase;
    unsigned int                _layerNum;
    osg::ref_ptr<PluginOptions> _options;
};


class ReaderWriterVPB : public osgDB::ReaderWriter
{
    public:
        ReaderWriterVPB()
        {
            supportsExtension( "osgearth_vpb", "VirtualPlanetBuilder data" );
        }

        virtual const char* className()
        {
            return "VirtualPlanetBuilder ReaderWriter";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            const PluginOptions* pluginOpts = static_cast<const PluginOptions*>( options );

            //osg::notify(osg::NOTICE) << pluginOpts->config().toString() << std::endl;

            std::string url = pluginOpts->config().value( PROPERTY_URL );
            if ( !url.empty() )
            {                
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(vpbDatabaseMapMutex);
                osg::observer_ptr<VPBDatabase>& db_ptr = vpbDatabaseMap[url];
                
                if (!db_ptr) db_ptr = new VPBDatabase( pluginOpts );
                
                if (db_ptr.valid()) return new VPBSource(db_ptr.get(), pluginOpts );
                else return ReadResult::FILE_NOT_FOUND;               
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        }
        
        typedef std::map<std::string, osg::observer_ptr<VPBDatabase> > VPBDatabaseMap;
        mutable OpenThreads::Mutex vpbDatabaseMapMutex;
        mutable VPBDatabaseMap vpbDatabaseMap;
};

REGISTER_OSGPLUGIN(osgearth_vpb, ReaderWriterVPB)

