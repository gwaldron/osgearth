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

#include <osgEarth/Notify>
#include <osgEarthUtil/AtlasBuilder>
#include <osgEarthSymbology/ResourceLibrary>

#include <osg/ArgumentParser>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>

#define LC "[atlas] "

using namespace osgEarth;

int
usage(const char* msg, const char* name =0L)
{
    if ( msg )
    {
        OE_NOTICE << msg << std::endl;
    }

    if ( name )
    {
        OE_NOTICE
            << name << " will compile an osgEarth resource catalog into a texture atlas."
            << "\nUsage: " << name
            << "\n      --input catalog.xml      : Input resource catalog file name"
            << "\n      [--output outfile]       : Output image array file name"
            << std::endl;
    }

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // print usage info.
    if ( arguments.read("--help") )
        return usage(0L, argv[0]);

    // the input resource catalog XML file:
    std::string inCatalogFile;
    if ( !arguments.read("--input", inCatalogFile) )
        return usage("Missing required argument --input");

    // the output texture atlas image:
    std::string outImageFile;
    if ( !arguments.read("--output", outImageFile) )
        outImageFile = osgDB::getNameLessExtension(inCatalogFile) + "_atlas.osgb";

    // the output catalog file describing the texture atlas contents:
    std::string outCatalogFile;
    outCatalogFile = outImageFile + ".xml";

    // check that the input file exists:
    if ( !osgDB::fileExists(inCatalogFile) )
        return usage("Input file not found");

    // open the resource library.
    osg::ref_ptr<osgEarth::Symbology::ResourceLibrary> lib =
        new osgEarth::Symbology::ResourceLibrary("unnamed", inCatalogFile);

    if ( !lib->initialize(0L) )
        return usage("Error loading input catalog file");
    

    // build the atlas.
    osgEarth::Util::AtlasBuilder        builder;
    osgEarth::Util::AtlasBuilder::Atlas atlas;

    if ( !builder.build(lib.get(), atlas) )
        return usage("Failed to build atlas");

    osgDB::writeImageFile(*atlas._image.get(), outImageFile);

    OE_INFO << LC << "Wrote output image to \"" << outImageFile << "\"";
    return 0;
}
