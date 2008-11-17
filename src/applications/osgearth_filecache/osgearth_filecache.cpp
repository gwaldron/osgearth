#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <osg/CoordinateSystemNode>
#include <osg/io_utils>

#include <osgEarth/Common>
#include <osgEarth/MapConfig>
#include <osgEarth/Mercator>
#include <osgEarth/PlateCarre>
#include <osgEarth/TileBuilder>

#include <iostream>
#include <sstream>

using namespace osgEarth;


struct Bounds
{
    Bounds(double minx, double miny, double maxx, double maxy):
_min(minx, miny),
_max(maxx, maxy)
{
}

bool intersects(TileKey* key)
{
    double minlon, minlat, maxlon, maxlat;
    key->getGeoExtents(minlon, minlat, maxlon, maxlat);


    return  osg::maximum(_min.x(), minlon) <= osg::minimum(_max.x(),maxlon) &&
            osg::maximum(_min.y(), minlat) <= osg::minimum(_max.y(), maxlat);
}

osg::Vec2d _min;
osg::Vec2d _max;
};

void processKey(TileBuilder* tile_builder, TileKey *key, Bounds& bounds, const unsigned int &maxLevel)
{
    if (maxLevel > key->getLevelOfDetail())
    {
        osg::notify(osg::NOTICE) << "Processing " << key->str() << std::endl;

        if (key->getLevelOfDetail() != 0)
        {
            //Assumes the the TileSource will perform the caching for us when we call createImage
            for (TileSourceList::iterator itr = tile_builder->getImageSources().begin(); itr != tile_builder->getImageSources().end(); ++itr)
            {
                osg::ref_ptr<osg::Image> image = itr->get()->createImage(key);
            }

            for (TileSourceList::iterator itr = tile_builder->getHeightFieldSources().begin(); itr != tile_builder->getHeightFieldSources().end(); ++itr)
            {
                osg::ref_ptr<osg::HeightField> heightField = itr->get()->createHeightField(key);
            }
        }



        osg::ref_ptr<TileKey> k0 = key->getSubkey(0);
        osg::ref_ptr<TileKey> k1 = key->getSubkey(1);
        osg::ref_ptr<TileKey> k2;
        osg::ref_ptr<TileKey> k3;

        if (key->getLevelOfDetail() > 0 || dynamic_cast<MercatorTileKey*>(key))
        {
            k2 = key->getSubkey(2);
            k3 = key->getSubkey(3);
        }

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if ((k0.valid() && bounds.intersects(k0.get())) ||
            (k1.valid() && bounds.intersects(k1.get())) ||
            (k2.valid() && bounds.intersects(k2.get())) ||
            (k3.valid() && bounds.intersects(k3.get())))
        {
            if (k0.valid()) processKey(tile_builder, k0.get(), bounds, maxLevel); 
            if (k1.valid()) processKey(tile_builder, k1.get(), bounds, maxLevel); 
            if (k2.valid()) processKey(tile_builder, k2.get(), bounds, maxLevel); 
            if (k3.valid()) processKey(tile_builder, k3.get(), bounds, maxLevel); 
        }
    }
}

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser args(&argc,argv);

    args.getApplicationUsage()->setApplicationName(args.getApplicationName());

    std::string filename;
    //Find the input filename
    for(int pos=1;pos<args.argc();++pos)
    {
        if (!args.isOption(pos))
        {
            filename = args[pos];
        }
    } 


    if (filename.size() == 0 )
    {
        osg::notify(osg::NOTICE) << "Please specify a .earth file to cache" << std::endl;
        return 1;
    }

    unsigned int maxLevel = 5;

    Bounds bounds(-180, -90, 180, 90);
    while (args.read("--max-level", maxLevel));
    while (args.read("-l", maxLevel));
    while (args.read("--bounds", bounds._min.x(), bounds._min.y(), bounds._max.x(), bounds._max.y()));
    while (args.read("-b", bounds._min.x(), bounds._min.y(), bounds._max.x(), bounds._max.y()));

    std::string export_dir;
    while (args.read("--export", export_dir));
    while (args.read("-e", export_dir));

    bool doExport = !export_dir.empty();

    osg::notify(osg::NOTICE) << "Caching files within (" << bounds._min.x() << ", " << bounds._min.y() << ") to (" << bounds._max.x() << ", " << bounds._max.y() << ") to level " << maxLevel << std::endl;

    //Load the map file
    osg::ref_ptr<MapConfig> map = MapConfigReaderWriter::readXml( filename );
    if ( map.valid() )
    {
        //Modify the MapConfig so that it caches to the export directory
        if (doExport)
        {
            //Write out the destination map file
            std::string export_map_file = osgDB::concatPaths(export_dir, osgDB::getSimpleFileName( filename) );
            osg::notify(osg::NOTICE) << "Exporting file to " << export_map_file << std::endl;

            std::string path = osgDB::getFilePath( export_map_file );
            //If the path doesn't currently exist or we can't create the path, don't cache the file
            if (!osgDB::fileExists( path ) && !osgDB::makeDirectory( path ) )
            {
                osg::notify(osg::NOTICE) << "Couldn't create path " << path << std::endl;
                return 0;
            }

            //The caceh will be relative to the output location of the mapfile
            map->setCachePath(".");
            map->setOfflineHint(false);
            MapConfigReaderWriter::writeXml(map.get(), export_map_file); 
            //Set the filename of the map so relative paths will be relative to the location of the mapfile
            map->setFilename( osgDB::getRealPath( export_map_file ) );
        }

        osg::ref_ptr<TileKey> key;

        if ( map->getTileProjection() == MapConfig::PROJ_MERCATOR )
            key = new MercatorTileKey( "" );
        else
            key = new PlateCarreTileKey( "" );

        //Create a TileBuilder for the map
        osg::ref_ptr<TileBuilder> tile_builder = TileBuilder::create( map.get(), filename );


        processKey(tile_builder.get(), key.get(), bounds, maxLevel);



        return 0;
    }
    else
    {
        osg::notify(osg::NOTICE) << "Could not load earth file from " << filename << std::endl;
        return 1;
    }
}
