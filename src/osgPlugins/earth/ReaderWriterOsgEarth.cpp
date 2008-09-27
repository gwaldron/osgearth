#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/PlateCarreTileBuilder>
#include <osgEarth/MapConfig>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>
#include <stdlib.h>
#include <algorithm>

using namespace osgEarth;

#define TO_LOWER( S ) std::transform( S.begin(), S.end(), S.begin(), ::tolower )


class ReaderWriterEarth : public osgDB::ReaderWriter
{
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

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            return readNode( file_name, opt );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            osg::Node* node = NULL;

            // The URL looks like "[KEY].[SOURCE].earth"
            //
            // where:
            //   KEY = the cell quadkey
            //   SOURCE = either a mapfile URL/filename (ending in ".map")
            //            or a direct driver name (like "msve")
            //
            // Map File Example:
            //   "01.msve_strm.map.earth"
            // 
            //   Cell Key = "01" (code for the exact tile to load)
            //   Map File = "msve_strm.map" (configuration file that defines the data sources)
            //
            // Direct-Driver Example:
            //   "01.msve.earth"
            //
            //   Cell Key = "01"
            //   Driver   = "msve" (send the cell key directly to the MSVE driver)


            // the first component is always the cell key:
            unsigned int i = file_name.find_first_of( '.' );
            if ( i >= file_name.length() - 1 )
                return ReadResult::FILE_NOT_HANDLED;

            PlateCarreCellKey key( file_name.substr( 0, i ) );

            std::string url_template = file_name.substr( i+1 );
            std::string target = osgDB::getNameLessExtension( url_template );
            std::string target_ext = osgDB::getFileExtension( target );
            TO_LOWER( target_ext );

            //osg::notify(osg::NOTICE)
            //    << "filename = " << file_name << ", "
            //    << "key = " << key.str() << ", "
            //    << "url_template = " << url_template << ", "
            //    << "target = " << target << ", "
            //    << "target_ext = " << target_ext << std::endl;

            osg::ref_ptr<TileBuilder> tile_builder;

            if ( target_ext == "map" )
            {
                // mapfile configuration:
                osg::ref_ptr<MapConfig> map = MapConfigReader::readXml( target );
                if ( map.valid() )
                {
                    tile_builder = TileBuilder::create( map.get(), url_template );
                }
                else
                {
                    return ReadResult::FILE_NOT_FOUND;
                }
            }
            else
            {
                // direct-to-driver:
                SourceConfig* source = new SourceConfig();
                source->setDriver( target );
                MapConfig* map = new MapConfig();
                map->getImageSources().push_back( source );
                tile_builder = TileBuilder::create( map, url_template );
            }

            // make the node for this key!
            if ( tile_builder.valid() )
            {
                node = tile_builder->createNode( key );
            }



            return node? ReadResult( node ) : ReadResult::FILE_NOT_FOUND;

        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
