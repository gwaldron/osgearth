#include <osgEarth/PlateCarre>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include "WCS11Source.h"
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

class ReaderWriterWCS : public osgDB::ReaderWriter
{
    public:
        ReaderWriterWCS() {}

        virtual const char* className()
        {
            return "WCS 1.1.0 Reader";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "wcs" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
        {
            return readNode( file_name, opt );
        }

        virtual ReadResult readNode(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
            //return ReadResult( "WCS: illegal usage (readNode); please use readImage/readHeightField" );
        }

        virtual ReadResult readImage(const std::string& file_name, const Options* opt) const
        {
            return ReadResult::FILE_NOT_HANDLED;
        }

        virtual ReadResult readHeightField(const std::string& file_name, const Options* opt) const
        {            
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            std::string keystr = file_name.substr( 0, file_name.find_first_of( '.' ) );
            osg::ref_ptr<TileKey> key = TileKeyFactory::createFromName( keystr );

            osg::ref_ptr<WCS11Source> source = new WCS11Source(); //TODO: config/cache it
            osg::HeightField* field = source->createHeightField( key.get() );
            return field? ReadResult( field ) : ReadResult( "Unable to load WCS height field" );
        }
};

REGISTER_OSGPLUGIN(wcs, ReaderWriterWCS)
