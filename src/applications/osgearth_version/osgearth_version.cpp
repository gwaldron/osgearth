/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include <iostream>
#include <osg/Notify>
#include <osg/ArgumentParser>
#include <osg/ApplicationUsage>
#include <osgEarth/Version>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

using namespace std;

int main( int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]");
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help",                   "Display this information");
    arguments.getApplicationUsage()->addCommandLineOption("--caps",                         "Print out system capabilities");
    arguments.getApplicationUsage()->addCommandLineOption("--version-number",               "Print out version number only");
    arguments.getApplicationUsage()->addCommandLineOption("--major-number",                 "Print out major version number only");
    arguments.getApplicationUsage()->addCommandLineOption("--minor-number",                 "Print out minor version number only");
    arguments.getApplicationUsage()->addCommandLineOption("--patch-number",                 "Print out patch version number only");
    arguments.getApplicationUsage()->addCommandLineOption("--so-number ",                   "Print out shared object version number only");    

    // if user request help write it out to cout.
    if (arguments.read("-h") || arguments.read("--help"))
    {
        cout << arguments.getApplicationUsage()->getCommandLineUsage() << endl;
        arguments.getApplicationUsage()->write(cout, arguments.getApplicationUsage()->getCommandLineOptions());
        return 1;
    }

    if (arguments.read("--version-number"))
    {
        cout << osgEarthGetVersion() << endl;
        return 0;
    }

    if (arguments.read("--major-number"))
    {
        cout << OSGEARTH_MAJOR_VERSION << endl;
        return 0;
    }

    if (arguments.read("--minor-number"))
    {
        cout << OSGEARTH_MINOR_VERSION << endl;
        return 0;
    }

    if (arguments.read("--patch-number"))
    {
        cout << OSGEARTH_PATCH_VERSION << endl;
        return 0;
    }

    if (arguments.read("--soversion-number") || arguments.read("--so-number") )
    {
        cout << osgEarthGetSOVersion() << endl;
        return 0;
    }    

    cout << osgEarthGetLibraryName() << " " << osgEarthGetVersion() << endl << endl;

    if ( arguments.read("--caps") )
    {
        osgEarth::setNotifyLevel( osg::INFO );
        osgEarth::Registry::instance()->getCapabilities();
    }

    return 0;
}
