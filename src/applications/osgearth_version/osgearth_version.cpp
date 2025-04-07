/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
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
    }

    else if (arguments.read("--version-number"))
    {
        cout << osgEarthGetVersion() << endl;
    }

    else if (arguments.read("--major-number"))
    {
        cout << OSGEARTH_MAJOR_VERSION << endl;
    }

    else if (arguments.read("--minor-number"))
    {
        cout << OSGEARTH_MINOR_VERSION << endl;
    }

    else if (arguments.read("--patch-number"))
    {
        cout << OSGEARTH_PATCH_VERSION << endl;
    }

    else if (arguments.read("--soversion-number") || arguments.read("--so-number") )
    {
        cout << osgEarthGetSOVersion() << endl;
    }    

    else if (arguments.read("--caps"))
    {
        osgEarth::setNotifyLevel(osg::INFO);
        osgEarth::initialize(arguments);
    }

    else
    {
        cout << osgEarthGetLibraryName() << " " << osgEarthGetVersion() << endl;
    }

    return 0;
}
