/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#define LC "[osgearth_conv] "

#include <osgEarth/Notify>
#include <osgEarth/Profile>
#include <osgEarth/TileSource>
#include <osgEarth/TileHandler>
#include <osgEarth/TileVisitor>
#include <osg/ArgumentParser>
#include <iomanip>

using namespace osgEarth;


// TileHandler that copies images from one tilesource to another.
struct ImageTileCopier : public TileHandler
{
    ImageTileCopier(TileSource* source, ReadWriteTileSource* dest)
        : _source(source), _dest(dest)
    {
        //nop
    }

    bool handleTile(const TileKey& key)
    {
        bool ok = false;
        osg::ref_ptr<osg::Image> image = _source->createImage(key);
        if ( image.valid() )
            ok = _dest->storeImage(key, image.get(), 0L);
        return ok;
    }
    
    bool hasData(const TileKey& key) const
    {
        return _source->hasData(key);
    }

    TileSource*          _source;
    ReadWriteTileSource* _dest;
};


// Custom progress reporter
struct ProgressReporter : public osgEarth::ProgressCallback
{
    bool reportProgress(double             current, 
                        double             total, 
                        unsigned           currentStage,
                        unsigned           totalStages,
                        const std::string& msg )
    {
        float percentage = current/total*100.0f;
        std::cout 
            << std::fixed
            << std::setprecision(1) << "\r" 
            << (int)current << "/" << (int)total
            << " (" << percentage << "%)"
            << "                        "
            << std::flush;

        if ( percentage >= 100.0f )
            std::cout << std::endl;

        return false;
    }
};


/**
 * Command-line tool that copies the contents of one TileSource
 * to another. All arguments are Config name/value pairs, so you need
 * to look in the header file for each driver's Options structure for
 * options :)
 *
 * Example: copy a GDAL file to an MBTiles repo:
 *
 *   osgearth_conv
 *      --in driver gdal
 *      --in url world.tif
 *      --out driver mbtiles
 *      --out filename world.db
 *
 * The "in" properties come from the GDALOptions getConfig method. The
 * "out" properties come from the MBTilesOptions getConfig method.
 *
 * Of course, the output driver must support writing (by implementing
 * the ReadWriteTileSource interface).
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);

    typedef std::map<std::string,std::string> KeyValue;
    std::string key, value;

    // collect input configuration:
    Config inConf;
    while( args.read("--in", key, value) )
        inConf.set(key, value);

    TileSourceOptions inOptions(inConf);
    osg::ref_ptr<TileSource> input = TileSourceFactory::openReadOnly(inOptions);
    if ( !input.valid() )
    {
        OE_WARN << LC << "Failed to open input" << std::endl;
        return -1;
    }

    TileSource::Status inputStatus = input->startup(0L);
    if ( inputStatus.isError() )
    {
        OE_WARN << LC << "Error initializing input" << std::endl;
        return -1;
    }

    // collect output configuration:
    Config outConf;
    while( args.read("--out", key, value) )
        outConf.set(key, value);

    // copy source profile to output config:
    outConf.add("profile", input->getProfile()->toProfileOptions().getConfig());

    TileSourceOptions outOptions(outConf);
    osg::ref_ptr<ReadWriteTileSource> output = TileSourceFactory::openReadWrite(outOptions);
    if ( !output.valid() )
    {
        OE_WARN << LC << "Failed to open output" << std::endl;
        return -1;
    }

    TileSource::Status outputStatus = output->startup(0L);
    if ( outputStatus.isError() )
    {
        OE_WARN << LC << "Error initializing output" << std::endl;
        return -1;
    }

    // Dump out some stuff...
    OE_NOTICE << LC << "FROM:\n"
        << inConf.toJSON(true)
        << std::endl;

    OE_NOTICE << LC << "TO:\n"
        << outConf.toJSON(true)
        << std::endl;

    // Copy.
    TileVisitor visitor( new ImageTileCopier(input, output) );

    unsigned minLevel;
    if (args.read("--min-level", minLevel))
        visitor.setMinLevel( minLevel );

    unsigned maxLevel;
    if (args.read("--max-level", maxLevel))
        visitor.setMaxLevel( maxLevel );

    std::cout << "Working..." << std::endl;
    visitor.setProgressCallback( new ProgressReporter() );

    visitor.run( input->getProfile() );
}
