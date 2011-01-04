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

#include <osgEarth/Registry>
#include <osgEarth/TileSource>
#include <osgEarth/HTTPClient>
#include <osgEarth/FileUtils>

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
            OE_DEBUG<<"VPB: Found terrain tile TileID("<<
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
    VPBDatabase( const VPBOptions& in_options ) :
        _options( in_options ),
        //_directory_structure( FLAT_TASK_DIRECTORIES ),
        _profile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() ),
        _maxNumTilesInCache( 128 )
    {
	}
	
	void initialize( const std::string& referenceURI)
	{
        unsigned int numTilesWideAtLod0, numTilesHighAtLod0;
        _profile->getNumTiles(0, numTilesWideAtLod0, numTilesHighAtLod0);

        // validate dataset
        _url = _options.url().value();

        if ( !_url.empty() )
        {
			//If the path doesn't contain a server address, get the full path to the file.
			if (!osgDB::containsServerAddress(_url))
			{
				_url = osgEarth::getFullPath(referenceURI, _url);
			}
			
            osg::ref_ptr<osgDB::ReaderWriter::Options> localOptions = new osgDB::ReaderWriter::Options;
            localOptions->setPluginData("osgearth_vpb Plugin",(void*)(1));
            //_rootNode = osgDB::readNodeFile( _url, localOptions.get() );

            HTTPClient::ResultCode rc = HTTPClient::readNodeFile( _url, _rootNode, localOptions.get() );

            if ( rc == HTTPClient::RESULT_OK && _rootNode.valid() )
            {
                _baseNameToUse = _options.baseName().value();

                _path = osgDB::getFilePath(_url);
                if ( _baseNameToUse.empty() )
                    _baseNameToUse = osgDB::getStrippedName(_url);
                _extension = osgDB::getFileExtension(_url);
                
                OE_INFO<<"VPB: Loaded root "<<_url<<", path="<<_path<<" base_name="<<_baseNameToUse<<" extension="<<_extension<<std::endl;
                
                std::string srs = _profile->getSRS()->getInitString(); //.srs();
                
                osg::CoordinateSystemNode* csn = dynamic_cast<osg::CoordinateSystemNode*>(_rootNode.get());
                if (csn)
                {
                    OE_INFO<<"VPB: CordinateSystemNode found, coordinate system is : "<<csn->getCoordinateSystem()<<std::endl;
                    
                    srs = csn->getCoordinateSystem();
                }

                CollectTiles ct;
                _rootNode->accept(ct);

                    
                osgTerrain::Locator* locator = ct.getLocator();
                if (locator)
                {
                    double min_x, max_x, min_y, max_y;
                    ct.getRange(min_x, min_y, max_x, max_y);

                    OE_DEBUG<<"VPB: range("<<min_x<<", "<<min_y<<", "<<max_x<<", "<<max_y<< ")" <<std::endl;
					OE_DEBUG<<"VPB: range("<<osg::RadiansToDegrees(min_x)<<", "<<osg::RadiansToDegrees(min_y)<<", "
						<<osg::RadiansToDegrees(max_x)<<", "<<osg::RadiansToDegrees(max_y)<< ")" <<std::endl;

                    srs = locator->getCoordinateSystem();

                    double aspectRatio = (max_x-min_x)/(max_y-min_y);
                    
                    OE_DEBUG<<"VPB: aspectRatio = "<<aspectRatio<<std::endl;

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
                    
                    OE_DEBUG<<"VPB: computed numTilesWideAtLod0 = "<<numTilesWideAtLod0<<std::endl;
                    OE_DEBUG<<"VPB: computed numTilesHightAtLod0 = "<<numTilesHighAtLod0<<std::endl;
                    
                    //if ( _options.valid() )
                    {
                        if ( _options.numTilesWideAtLod0().isSet() )
                            numTilesWideAtLod0 = _options.numTilesWideAtLod0().value();

                        if ( _options.numTilesHighAtLod0().isSet() )
                            numTilesHighAtLod0 = _options.numTilesHighAtLod0().value();
                    }

                    OE_DEBUG<<"VPB: final numTilesWideAtLod0 = "<<numTilesWideAtLod0<<std::endl;
                    OE_DEBUG<<"VPB: final numTilesHightAtLod0 = "<<numTilesHighAtLod0<<std::endl;
                   
                    _profile = osgEarth::Profile::create( 
                        srs,
                        osg::RadiansToDegrees(min_x), 
                        osg::RadiansToDegrees(min_y), 
                        osg::RadiansToDegrees(max_x), 
                        osg::RadiansToDegrees(max_y),
                        "",
                        numTilesWideAtLod0,
                        numTilesHighAtLod0 );
                }
                
            }
            else
            {
                OE_WARN << "VPB: " << HTTPClient::getResultCodeString(rc) << ": " << _url << std::endl;
                _url = "";
            }
        }
        else 
        {
            OE_WARN<<"VPB: No data referenced "<<std::endl;
        }

    }
    
    std::string createTileName( int level, int tile_x, int tile_y )
    {
        std::stringstream buf;
        if ( _options.directoryStructure() == VPBOptions::DS_FLAT )
        {
             buf<<_path<<"/"<<_baseNameToUse<<"_L"<<level<<"_X"<<tile_x/2<<"_Y"<<tile_y/2<<"_subtile."<<_extension;
        }
        else
        {
            int psl = _options.primarySplitLevel().value();
            int ssl = _options.secondarySplitLevel().value();

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
            else if ( _options.directoryStructure() == VPBOptions::DS_TASK )
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
        OE_DEBUG<<"VPB: VPBDatabase::createTileName(), buf.str()=="<< bufStr <<std::endl;
        
		return bufStr;
    }
    
    osgTerrain::TerrainTile* getTerrainTile( const TileKey& key, ProgressCallback* progress )
    {
        int level = key.getLevelOfDetail();
        unsigned int tile_x, tile_y;
        key.getTileXY( tile_x, tile_y );
        
        //int max_x = (2 << level) - 1;
        int max_y = (1 << level) - 1;
        
        tile_y = max_y - tile_y;

        osgTerrain::TileID tileID(level, tile_x, tile_y);

        osg::ref_ptr<osgTerrain::TerrainTile> tile = findTile(tileID, false);
        if (tile.valid()) return tile.get();
        
        //OE_INFO<<"Max_x = "<<max_x<<std::endl;
        //OE_INFO<<"Max_y = "<<max_y<<std::endl;

        //OE_INFO<<"base_name = "<<base_name<<" psl="<<primary_split_level<<" ssl="<<secondary_split_level<<std::endl;
        //OE_INFO<<"level = "<<level<<", x = "<<tile_x<<", tile_y = "<<tile_y<<std::endl;
        //OE_INFO<<"tile_name "<<createTileName(level, tile_x, tile_y)<<std::endl;
        //OE_INFO<<"thread "<<OpenThreads::Thread::CurrentThread()<<std::endl;

        std::string filename = createTileName(level, tile_x, tile_y);
        
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_blacklistMutex);
            if (_blacklistedFilenames.count(filename)==1)
            {
                OE_DEBUG<<"VPB: file has been found in black list : "<<filename<<std::endl;
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
            //OE_INFO<<"Loaded model "<<filename<<std::endl;
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
                //OE_INFO<<"Black listing : "<< filename<< " (" << result << ")" << std::endl;
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_blacklistMutex);
                _blacklistedFilenames.insert(filename);
            }
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

                OE_DEBUG<<"VPB: Pruned tileID ("<<TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<")"<<std::endl;
            }

            OE_DEBUG<<"VPB: insertTile ("
                << TileKey::getLOD(tileID)<<", "<<tileID.x<<", "<<tileID.y<<") " 
                << " tileFIFO.size()=="<<_tileFIFO.size()<<std::endl;
        }
        else
        {
            OE_DEBUG<<"VPB: insertTile ("
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

    const VPBOptions _options;
    std::string _url;
    std::string _path;
    std::string _extension;

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
    VPBSource( VPBDatabase* vpbDatabase, const VPBOptions& in_options ) : //const VPBOptions* in_options) :  
        TileSource(in_options),
        _vpbDatabase(vpbDatabase),
        _options( in_options ),
        _referenceUri()
    {
        //nop
    }

    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
		_referenceUri = referenceURI;
		_vpbDatabase->initialize(referenceURI);
		if ( overrideProfile)
		{
			setProfile( overrideProfile );
		}
		else
		{
			setProfile(_vpbDatabase->_profile.get());
		}
    }
    
	osg::Image* createImage( const TileKey& key,
		ProgressCallback* progress)
	{
		osg::Image * ret = NULL;
		//TODO:  Make VPB driver use progress callback
		osg::ref_ptr<osgTerrain::TerrainTile> tile = _vpbDatabase->getTerrainTile(key, progress);                
		if (tile.valid())
		{        
			int layerNum = _options.layer().value();
			const optional<std::string> & layerSetName = _options.layerSetName();

			int numColorLayers = (int)tile->getNumColorLayers();
			if(layerNum > numColorLayers)
				layerNum = 0;
			if (layerNum < numColorLayers)
			{
				osgTerrain::Layer* layer = tile->getColorLayer(layerNum);

				osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>(layer);
				if (imageLayer)
				{
					OE_DEBUG<<"VPB: createImage(" << key.str() << " layerNum=" << layerNum << ") successful." <<std::endl;
					ret = new osg::Image( *imageLayer->getImage() );
				}
				else
				{
					osgTerrain::SwitchLayer* switchLayer = dynamic_cast<osgTerrain::SwitchLayer*>(layer);
					if (switchLayer && layerSetName.isSet())
					{
						for(unsigned int si=0; !imageLayer && si<switchLayer->getNumLayers(); ++si)
						{
							if(switchLayer->getSetName(si) == layerSetName.value())
							{
								imageLayer = dynamic_cast<osgTerrain::ImageLayer*>(switchLayer->getLayer(si));
							}
						}
					}
					if(imageLayer)
					{
						OE_DEBUG<<"VPB: createImage(" << key.str() << " layerSet=" << layerSetName.value() << ") successful." <<std::endl;
						ret = new osg::Image( *imageLayer->getImage() );
					}
				}
			}
			if(!ret)
			{
				OE_DEBUG<<"VPB: createImage(" << key.str() << " layerSet=" << layerSetName.value() << " layerNum=" << layerNum << "/" << numColorLayers << ") failed." <<std::endl;
			}
		}
		else
		{
			OE_DEBUG<<"VPB: createImage(" << key.str() << ") database retrieval failed." <<std::endl;
		}
		return ret;
	}

    osg::HeightField* createHeightField( const TileKey& key,
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
    const VPBOptions _options;
	std::string	_referenceUri;
};


class VPBSourceFactory : public TileSourceDriver
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

            VPBOptions vpbOptions( getTileSourceOptions(options) );

            std::string url = vpbOptions.url().value();
            if ( !url.empty() )
            {                
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(vpbDatabaseMapMutex);
                osg::observer_ptr<VPBDatabase>& db_ptr = vpbDatabaseMap[url]; //get or create
                
                if (!db_ptr) db_ptr = new VPBDatabase( vpbOptions );
                
                if (db_ptr.valid())
                    return new VPBSource( db_ptr.get(), vpbOptions );
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

