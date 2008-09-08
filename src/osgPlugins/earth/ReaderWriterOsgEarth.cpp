#include <osgEarth/GeocentricTileBuilder>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>

using namespace osgEarth;

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

            // the filename looks like: "[KEY].[SOURCE].earth"

            unsigned int i = file_name.find_first_of( '.' );
            if ( i >= 0 && i < file_name.length()-1 )
            {
                // the first segment is the quad-key:
                PlateCarreQuadKey qk( file_name.substr( 0, i ) );

                // the rest is the source identifier:
                std::string image_source = file_name.substr( i+1 );
                std::string field_source = "wcs.earth";

                // todo: create a std::map, cache the source=>builder mapping so the builder
                // can maintain a state

                GeocentricTileBuilder builder(
                    new ReaderWriterPlateCarreTileSource( image_source ),
                    new ReaderWriterPlateCarreTileSource( field_source ) );

                node = builder.create( qk );
            }

            return node? ReadResult( node ) : ReadResult::FILE_NOT_FOUND;

        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
