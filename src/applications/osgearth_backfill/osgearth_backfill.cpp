/* osgEarth
 * Copyright 2016 Pelican Mapping
 * MIT License
 */

#include <osg/io_utils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>

#include <osgEarth/Common>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarthUtil/TMS>
#include <osgEarthUtil/TMSBackFiller>


#include <iostream>
#include <sstream>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::TMS;

/** Prints an error message, usage information, and returns -1. */
int
usage( const std::string& msg = "" )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgearth_backfill <tms.xml>" << std::endl
        << std::endl        
        << "            --bounds xmin ymin xmax ymax    : bounds to backfill in (in map coordinates; default=entire map)\n"
        << "            [--min-level <num>]             : The minimum level to stop backfilling to.  (default=0)\n"
        << "            [--max-level <num>]             : The level to start backfilling from(default=inf)\n"                
        << "            [--db-options]                : db options string to pass to the image writer in quotes (e.g., \"JPEG_QUALITY 60\")\n"
        << std::endl
        << "         [--quiet]               : suppress progress output" << std::endl;

    return -1;
}


/** Prints a message and returns a non-error return code. */
int
message( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl << std::endl;
    }
    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);    

    // verbosity?
    bool verbose = !args.read( "--quiet" );
     
    Bounds bounds;
    // restrict user-specified bounds.    
    double xmin=DBL_MAX, ymin=DBL_MAX, xmax=DBL_MIN, ymax=DBL_MIN;
    while (args.read("--bounds", xmin, ymin, xmax, ymax ))
    {                
        bounds.set( xmin, ymin, 0, xmax, ymax, 1 );        
    }        

    // min level to backfill to
    unsigned minLevel = 0;
    args.read( "--min-level", minLevel );

    // max level to which to generate
    unsigned maxLevel = ~0;
    args.read( "--max-level", maxLevel );  

    std::string dbOptions;
    args.read("--db-options", dbOptions);
    std::string::size_type n = 0;
    while ((n=dbOptions.find('"', n))!=dbOptions.npos)
    {
        dbOptions.erase(n,1);
    }

    osg::ref_ptr<osgDB::Options> options = new osgDB::Options(dbOptions);


    std::string tmsPath;

    //Get the first argument that is not an option
    for(int pos=1;pos<args.argc();++pos)
    {
        if (!args.isOption(pos))
        {
            tmsPath  = args[ pos ];
        }
    }

    if (tmsPath.empty())
    {
        return usage( "Please provide a path to a TMS TileMap" );
    }
    

    TMSBackFiller backfiller;
    backfiller.setMinLevel( minLevel );
    backfiller.setMaxLevel( maxLevel );
    backfiller.setBounds( bounds );
    backfiller.process( tmsPath, options.get() );
}
