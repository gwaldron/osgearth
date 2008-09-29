#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarreTileBuilder>
#include <osgEarth/MapConfig>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>
#include <stdlib.h>
#include <algorithm>
#include <map>

using namespace osgEarth;

#define TO_LOWER( S ) std::transform( S.begin(), S.end(), S.begin(), ::tolower )


class ReaderWriterEarth : public osgDB::ReaderWriter
{
    private:
        typedef std::map<std::string, osg::ref_ptr<TileBuilder> > TileBuilderMap;
        TileBuilderMap tile_builders;

    public:
        ReaderWriterEarth() {}

        virtual const char* className()
        {
            return "OSG Earth ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "earth" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            return readNode( file_name, options );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            TileBuilder* tile_builder = NULL;
            PlateCarreCellKey key( "" );

            // try to strip off a cell key:
            unsigned int i = file_name.find_first_of( '.' );
            if ( i >= file_name.length() - 1 )
                return ReadResult::FILE_NOT_HANDLED;
            

            std::string earth_file = file_name.substr( i+1 );
            TileBuilderMap::const_iterator k = tile_builders.find( earth_file );
            
            if ( k == tile_builders.end() )
            {
                // new map - read the config file
                osg::ref_ptr<MapConfig> map = MapConfigReader::readXml( file_name );
                if ( map.valid() )
                {
                    tile_builder = TileBuilder::create( map.get(), file_name );
                    const_cast<ReaderWriterEarth*>(this)->tile_builders[ file_name ] = tile_builder;
                }
                else
                {
                    return ReadResult::FILE_NOT_FOUND;
                }                
            }
            else
            {
                tile_builder = k->second.get();
                key = PlateCarreCellKey( file_name.substr( 0, i ) );
            }

            osg::Node* node = tile_builder->createNode( key );
            return node? ReadResult( node ) : ReadResult::FILE_NOT_FOUND;
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
