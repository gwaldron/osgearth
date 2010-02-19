/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgEarth/EarthFile>

#include <iostream>

using namespace osgEarth;

/**
 * This is a TEST APP to test EARTH FILE WRITING.
 */
int main(int argc, char** argv)
{
    if ( argc != 3 ) {
        osg::notify(osg::NOTICE) << "Usage: osgearth_earthfile <inputfile> <outputfile>" << std::endl;
        return -1;
    }

    std::string infile( argv[1] );
    std::string outfile( argv[2] );

    // read in the earth file:
    EarthFile earthReader;
    if ( earthReader.readXML( infile ) )
    {
        osg::ref_ptr<Map> map = earthReader.getMap();
        MapEngineProperties engineProps = earthReader.getMapEngineProperties();

        // now write it back out
        EarthFile earthWriter;
        earthWriter.setMap( map.get() );
        earthWriter.setMapEngineProperties( engineProps );

        if ( !earthWriter.writeXML( outfile ) ) {
            osg::notify(osg::NOTICE) 
                << "ERROR: unable to write earth file to " << outfile << std::endl;
            return -1;
        }
    }
    else
    {
        osg::notify(osg::NOTICE)
            << "ERROR: unable to read earth file from " << infile << std::endl;
        return -1;
    }

    return 0;
}
