#include <osgEarth/TileSource>
#include <osgDB/ReadFile>

using namespace osgEarth;

ReaderWriterTileSource::ReaderWriterTileSource(const std::string& _extension,
                                               const osgDB::ReaderWriter::Options* _options )
: extension( _extension ),
  options( _options )
{
}

osg::Image*
ReaderWriterTileSource::createImage( const TileKey* key )
{
    std::string uri = key->getName() + "." + extension;
    osg::Image* image = NULL;

    image = osgDB::readImageFile( uri, options.get() );
    if ( !image )
    {
        osg::notify(osg::WARN) << "ReaderWriterTileSource: osgDB::readImageFile FAILED for \"" << uri << "\"" << std::endl;
    }
    return image;
}

osg::HeightField*
ReaderWriterTileSource::createHeightField( const TileKey* key )
{
    std::string uri = key->getName() + "." + extension;
    osg::HeightField* field = NULL;

    field = osgDB::readHeightFieldFile( uri, options.get() );
    if ( !field )
    {
        osg::notify(osg::WARN) << "ReaderWriterTileSource: osgDB::readHeightField FAILED for \"" << uri << "\"" << std::endl;
    }
    return field;
}
