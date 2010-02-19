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

#include <sstream>

#include "VPBOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

//#define PROPERTY_URL                    "url"
//#define PROPERTY_PRIMARY_SPLIT_LEVEL    "primary_split_level"
//#define PROPERTY_SECONDARY_SPLIT_LEVEL  "secondary_split_level"
//#define PROPERTY_DIRECTORY_STRUCTURE    "directory_structure"
//#define PROPERTY_LAYER_NUM              "layer"
//#define PROPERTY_NUM_TILES_WIDE_AT_LOD0 "num_tiles_wide_at_lod0"
//#define PROPERTY_NUM_TILES_HIGH_AT_LOD0 "num_tiles_high_at_lod0"
//#define PROPERTY_BASE_NAME              "base_name"


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

    //enum DirectoryStructure
    //{
    //    FLAT,
    //    FLAT_TASK_DIRECTORIES,
    //    NESTED_TASK_DIRECTORIES
    //};

    VPBDatabase( const VPBOptions* in_options ) :
        _options( in_options ),
        //_directory_structure( FLAT_TASK_DIRECTORIES ),
        _maxNumTilesInCache( 128 ),
        _profile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() )
    {
        unsigned int numTilesWideAtLod0, numTilesHighAtLod0;
        _profile->getNumTiles(0, numTilesWideAtLod0, numTilesHighAtLod0);
        
        //if ( _options.valid() )
        //{
        //    const Config& conf = options->config();

        //    url = conf.value( PROPERTY_URL );
        //    primary_split_level = conf.value<int>( PROPERTY_PRIMARY_SPLIT_LEVEL, -1 );
        //    secondary_split_level = conf.value<int>( PROPERTY_SECONDARY_SPLIT_LEVEL, -1 );

        //    std::string dir_string = conf.value( PROPERTY_DIRECTORY_STRUCTURE );
        //    if (dir_string=="nested" || dir_string=="nested_task_directories" ) directory_structure = NESTED_TASK_DIRECTORIES;
        //    if (dir_string=="task" || dir_string=="flat_task_directories") directory_structure = FLAT_TASK_DIRECTORIES;
        //    if (dir_string=="flat") directory_structure = FLAT;
        //    
        //    base_name = conf.value( PROPERTY_BASE_NAME );
        //}

        // validate dataset
        _url = _options->url().value();

        if ( !_url.empty() )
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> localOptions = new osgDB::ReaderWriter::Options;
            localOptions->setPluginData("osgearth_vpb Plugin",(void*)(1));
            _rootNode = osgDB::readNodeFile( _url, localOptions.get() );


            if ( _rootNode.valid() )
            {
                _baseNameToUse = _options->baseName().value();

                _path = osgDB::getFilePath(_url);
                if ( _baseNameToUse.empty() )
                    _baseNameToUse = osgDB::getStrippedName(_url);
                _extension = osgDB::getFileExtension(_url);
                
                osg::notify(osg::INFO)<<"VPBasebase constructor: Loaded root "<<_url<<", path="<<_path<<" base_name="<<_baseNameToUse<<" extension="<<_extension<<std::endl;
                
                std::string srs = _profile->getSRS()->getInitString(); //.srs();
                
                osg::CoordinateSystemNode* csn = dynamic_cast<osg::CoordinateSystemNode*>(_rootNode.get());
                if (csn)
                {
                    osg::notify(osg::INFO)<<"CordinateSystemNode found, coordinate system is : "<<csn->getCoordinateSystem()<<std::endl;
                    
                    srs = csn->getCoordinateSystem();
                }

                CollectTiles ct;
                _rootNode->accept(ct);

                    
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
                    
                    if ( _options.valid() )
                    {
                        if ( _options->numTilesWideAtLod0().isSet() )
                            numTilesWideAtLod0 = _options->numTilesWideAtLod0().value();

                        if ( _options->numTilesHighAtLod0().isSet() )
                            numTilesHighAtLod0 = _options->numTilesHighAtLod0().value();

                        //if ( options->getPluginData( PROPERTY_NUM_TILES_WIDE_AT_LOD0 ) )
                        //    numTilesWideAtLod0 = atoi( (const char*)options->getPluginData( PROPERTY_NUM_TILES_WIDE_AT_LOD0 ) );

                        //if ( options->getPluginData( PROPERTY_NUM_TILES_HIGH_AT_LOD0 ) )
                        //    numTilesHighAtLod0 = atoi( (const char*)options->getPluginData( PROPERTY_NUM_TILES_HIGH_AT_LOD0 ) );
                    }

                    osg::notify(osg::INFO)<<"final numTilesWideAtLod0 = "<<numTilesWideAtLod0<<std::endl;
                    osg::notify(osg::INFO)<<"final numTilesHightAtLod0 = "<<numTilesHighAtLod0<<std::endl;
                   
                    _profile = osgEarth::Profile::create( 
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
                osg::notify(osg::NOTICE)<<"Unable to read file "<<_url<<std::endl;
                _url = "";
            }
        }
        else 
        {
            osg::notify(osg::NOTICE)<<"No data referenced "<<std::endl;
        }

    }
    
    std::string createTileName( int level, int tile_x, int tile_y )
    {
        std::stringstream buf;
        if ( _options->directoryStructure() == VPBOptions::DS_FLAT )
        {
             buf<<_path<<"/"<<_baseNameToUse<<"_L"<<level<<"_X"<<tile_x/2<<"_Y"<<tile_y/2<<"_subtile."<<_extension;
        }
        else
        {
            int psl = _options->primarySplitLevel().value();
            int ssl = _options->secondarySplitLevel().value();

            if (level<psl)
            {
                buf<<_path<<"/"<<_baseNameToUse<<"_root_L0_X0_Y0/"<<
                     _baseNameToUse<<"_L"<<level<<"_X"<<tile_x/2<<"_Y"<<tile_y/2<<"_subtile."<<_extension;

            }
            else if (level<ssl)
            {
                tile_x /= 2;
                tile_y /= 2;

                int split_x = tile_x >> (level - psl);
                int split_y = tile_y >> (level - psl);

                buf<<_path<<"/"<<_baseNameToUse<<"_subtile_L"<<psl<<"_X"<<split_x<<"_Y"<<split_y<<"/"<<
                     _baseNameToUse<<"_L"<<level<<"_X"<<tile_x<<"_Y"<<tile_y<<"_subtile."<<_extension;
            }
            else if ( _options->directoryStructure() == VPBOptions::DS_TASK )
            {
                tile_x /= 2;
                tile_y /= 2;

                int split_x = tile_x >> (level - psl);
                int split_y = tile_y >> (level - psl);

                int secondary_split_x = tile_x >> (level - ssl);
                int secondary_split_y = tile_y >> (level - ssl);

                buf<<_path<<"/"<<_baseNameToUse<<"_subtile_L"<<psl<<"_X"<<split_x<<"_Y"<<split_y<<"/"<<
                     _baseNameToUse<<"_subtile_L"<<ssl<<"_X"<<secondary_split_x<<"_Y"<<secondary_split_y<<"/"<< 
                     _baseNameToUse<<"_L"<<level<<"_X"<<tile_x<<"_Y"<<tile_y<<"_subtile."<<_extension;
            }
            else
            {
                tile_x /= 2;
                tile_y /= 2;

                int split_x = tile_x >> (level - ssl);
                int split_y = tile_y >> (level - ssl);

                buf<<_path<<"/"<<_baseNameToUse<<"_subtile_L"<<ssl<<"_X"<<split_x<<"_Y"<<split_y<<"/"<<
                     _baseNameToUse<<"_L"<<level<<"_X"<<tile_x<<"_Y"<<tile_y<<"_subtile."<<_extension;
            }
        }
        
		std::string bufStr;
		bufStr = buf.str();
        osg::notify(osg::INFO)<<"VPBDatabase::createTileName(), buf.str()=="<< bufStr <<std::endl;
        
		return bufStr;
    }
    
    osgTerrain::TerrainTile* getTerrainTile( const TileKey* key, ProgressCallback* progress )
    {
        int level = key->getLevelOfDetail();
        unsigned int tile_x, tile_y;
        key->getTileXY( tile_x, tile_y );
        
        int max_x = (2 << level) - 1;
        int max_y = (1 << level) - 1;
        
        tile_y = max_y - tile_y;

        osgTerrain::TileID tileID(level, tile_x, tile_y);

        osg::ref_ptr<osgTerrain::TerrainTile> tile = findTile(tileID, false);
        if (tile.valid()) return tile.get();
        
        //osg::notify(osg::INFO)<<"Max_x = "<<max_x<<std::endl;
        //osg::notify(osg::INFO)<<"Max_y = "<<max_y<<std::endl;

        //osg::notify(osg::INFO)<<"base_name = "<<base_name<<" psl="<<primary_split_level<<" ssl="<<secondary_split_level<<std::endl;
        //osg::notify(osg::INFO)<<"level = "<<level<<", x = "<<tile_x<<", tile_y = "<<tile_y<<std::endl;
        //osg::notify(osg::INFO)<<"tile_name "<<createTileName(level, tile_x, tile_y)<<std::endl;
        //osg::notify(osg::INFO)<<"thread "<<OpenThreads::Thread::CurrentThread()<<std::endl;

        std::string filename = createTileName(level, tile_x, tile_y);
        
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_blacklistMutex);
            if (_blacklistedFilenames.count(filename)==1)
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
        HTTPClient::ResultCode result = HTTPClient::readNodeFile( filename, node, localOptions.get(), progress );
        if ( result == HTTPClient::RESULT_OK && node.valid() )
        {
            //osg::notify(osg::INFO)<<"Loaded model "<<filename<<std::endl;
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
            // in the case of an "unrecoverable" error, black-list the URL for this tile.
            if ( ! HTTPClient::isRecoverable( result ) )
            {
                //osg::notify(osg::NOTICE)<<"Black listing : "<< filename<< " (" << result << ")" << std::endl;
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_blacklistMutex);
                _blacklistedFilenames.insert(filename);
            }

            //if ( progress && !progress->isCanceled() && !progress->message().empty() )
            //{
            //    osg::notify(osg::NOTICE) << "[osgEarth] [vpb] error '" << progress->message() << "' on " << filename << 
            //        std::endl;
            //}
        }
        
        return findTile(tileID, false);
    }
    
    void insertTile(const osgTerrain::TileID& tileID, osgTerrain::TerrainTile* tile)
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_tileMapMutex);

        if ( _tileMap.find(tileID) == _tileMap.end() )
        {
            _tileMap[tileID] = tile;

            _tileFIFO.push_back(tileID);

            if (_tileFIFO.size() > _maxNumTilesInCache)
            {
                osgTerrain::TileID tileToRemove = _tileFIFO.front();
                _tileFIFO.pop_front();
                _tileMap.erase(tileToRemove);

			    osg::notify(osg::INFO)<<"Pruned tileID ("<<TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<")"<<std::endl;
            }

            osg::notify(osg::INFO)<<"insertTile ("
                << TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<") " 
                << " tileFIFO.size()=="<<_tileFIFO.size()<<std::endl;
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
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_tileMapMutex);
            TileMap::iterator itr = _tileMap.find(tileID);
            if (itr != _tileMap.end()) return itr->second.get();
        }

        if (insertBlankTileIfNotFound) insertTile(tileID, 0);

        return 0;
    }

    osg::ref_ptr<const VPBOptions> _options;
    std::string _url;
    std::string _path;
    //std::string base_name;
    std::string _extension;
    //int primary_split_level;
    //int secondary_split_level;
    //DirectoryStructure directory_structure;

    std::string _baseNameToUse;

    osg::ref_ptr<const Profile> _profile;
    osg::ref_ptr<osg::Node> _rootNode;
    
    unsigned int _maxNumTilesInCache;
    
    typedef std::map<osgTerrain::TileID, osg::ref_ptr<osgTerrain::TerrainTile> > TileMap;
    TileMap _tileMap;
    OpenThreads::Mutex _tileMapMutex;
    
    typedef std::list<osgTerrain::TileID> TileIDList;
    TileIDList _tileFIFO;
    
    typedef std::set<std::string> StringSet;
    StringSet _blacklistedFilenames;
    OpenThreads::Mutex _blacklistMutex;
    
};

class VPBSource : public TileSource
{
public:
    VPBSource( VPBDatabase* vpbDatabase, const VPBOptions* in_options) :  
        TileSource(in_options),
        _vpbDatabase(vpbDatabase)
    {
        _options = in_options;
        //if ( in_options )
        //{
        //    layerNum = in_options->config().value<int>( PROPERTY_LAYER_NUM, layerNum );
        //}
    }

    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
		if ( overrideProfile)
		{
			setProfile( overrideProfile );
		}
		else
		{
			setProfile(_vpbDatabase->_profile.get());
		}
    }
    
    osg::Image* createImage( const TileKey* key,
                             ProgressCallback* progress)
    {
        //TODO:  Make VPB driver use progress callback
        osg::ref_ptr<osgTerrain::TerrainTile> tile = _vpbDatabase->getTerrainTile(key, progress);                
        if (tile.valid())
        {        
            int layerNum = _options->layer().value();

            if (layerNum < tile->getNumColorLayers())
            {
                osgTerrain::Layer* layer = tile->getColorLayer(layerNum);
                osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>(layer);
                if (imageLayer)
                {
                    //return imageLayer->getImage();
                    return new osg::Image( *imageLayer->getImage() );
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
                //return hfLayer->getHeightField();
                return new osg::HeightField(*hfLayer->getHeightField());
            }
        }

        return 0;
    }

    virtual std::string getExtension()  const 
    {
        //All VPB tiles are in IVE format
        return _vpbDatabase->_extension;
    }

private:
    osg::ref_ptr<VPBDatabase> _vpbDatabase;
    osg::ref_ptr<const VPBOptions>  _options;
    //unsigned int                                        layerNum;
};


class VPBSourceFactory : public osgDB::ReaderWriter
{
    public:
        VPBSourceFactory()
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

            osg::ref_ptr<const VPBOptions> settings = dynamic_cast<const VPBOptions*>( pluginOpts );
            if ( !settings.valid() )
            {
                settings = new VPBOptions( pluginOpts );
            }

            //osg::notify(osg::NOTICE) << pluginOpts->config().toString() << std::endl;

            std::string url = settings->url().value();
            if ( !url.empty() )
            {                
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(vpbDatabaseMapMutex);
                osg::observer_ptr<VPBDatabase>& db_ptr = vpbDatabaseMap[url]; //get or create
                
                if (!db_ptr) db_ptr = new VPBDatabase( settings.get() );
                
                if (db_ptr.valid())
                    return new VPBSource( db_ptr.get(), settings.get() );
                else
                    return ReadResult::FILE_NOT_FOUND;               
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

REGISTER_OSGPLUGIN(osgearth_vpb, VPBSourceFactory)

