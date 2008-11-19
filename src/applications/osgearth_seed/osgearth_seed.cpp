#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <osg/io_utils>

#include <osgEarth/Common>
#include <osgEarth/MapConfig>
#include <osgEarth/CacheSeed>

#include <iostream>
#include <sstream>

using namespace osgEarth;

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser args(&argc,argv);

    args.getApplicationUsage()->setApplicationName(args.getApplicationName());
    args.getApplicationUsage()->setDescription(args.getApplicationName() + " is an application used to seed a cache for an osgEarth.");
    args.getApplicationUsage()->setCommandLineUsage(args.getApplicationName()+" [options] filename ...");
    args.getApplicationUsage()->addCommandLineOption("--max-level level","The maximum level to seed down to.");
    args.getApplicationUsage()->addCommandLineOption("-l","Shorthand for --max-level.");
    args.getApplicationUsage()->addCommandLineOption("--bounds minlon minlat maxlon maxlat","The geospatial extents to seed.");
    args.getApplicationUsage()->addCommandLineOption("-b","Shorthand for --bounds.");

    // if user request help write it out to cout.
    if (args.read("-h") || args.read("--help") || args.argc() <= 1)
    {
        args.getApplicationUsage()->write(std::cout);
        return 1;
    }

    std::string filename;
    //Find the input filename
    for(int pos=1;pos<args.argc();++pos)
    {
        if (!args.isOption(pos))
        {
            filename = args[pos];
        }
    } 

    //Make sure the user specified a file
    if ( filename.empty() )
    {
        osg::notify(osg::NOTICE) << "Please specify a .earth file to seed." << std::endl;
        return 1;
    }

    //Read the max level
    unsigned int maxLevel = 5;
    while (args.read("--max-level", maxLevel));
    while (args.read("-l", maxLevel));

    //Read the bounds
    Bounds bounds(-180, -90, 180, 90);
    while (args.read("--bounds", bounds._min.x(), bounds._min.y(), bounds._max.x(), bounds._max.y()));
    while (args.read("-b", bounds._min.x(), bounds._min.y(), bounds._max.x(), bounds._max.y()));

    //Load the map file
    osg::ref_ptr<MapConfig> map = MapConfigReaderWriter::readXml( filename );
    if ( map.valid() )
    {        
        //Make sure there is a cache_path specified
        if (map->getFullCachePath().empty())
        {
            osg::notify(osg::NOTICE) << "No cache_path defined for " << filename << ".  Please define a cache_path element or use the OSGEARTH_FILECACHE environment variable." << std::endl;
            return 1;
        }

        //Create the CacheSeed
        osg::ref_ptr<CacheSeed> seed = new CacheSeed();
        seed->setMaxLevel(maxLevel);
        seed->setBounds(bounds);
        seed->seed( map.get() );
        return 0;
    }
    else
    {
        osg::notify(osg::NOTICE) << "Could not load earth file from " << filename << std::endl;
        return 1;
    }
}
