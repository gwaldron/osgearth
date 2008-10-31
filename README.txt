osgEarth - Dynamic terrain generator


REQUIREMENTS

- OpenSceneGraph (OSG) version 2.6 or newer
- EXPAT XML library (http://expat.sourceforge.net)
- cURL file transfer library (http://curl.haxx.se) (**)

(**) Also included in MEW's 3rd-party dependency set located here (http://mew.cx/osg)


BUILDING

- Use CMake 2.4.7 to build (same utility used to build OSG).

INSTALLING

- Copy "osgEarth" shared library (.dll/.so) to the OSG "bin" folder
- Copy all the plugin librarys (osgdb_*.dll/.do) to the OSG bin/osgPlugins-x.x.x folder

RUNNING

- Example earth files are located in the "tests" folder. Simply use osgviewer to load these files.

- You can edit the files to customize the cache folder and the proxy server if necessary.

Example: osgviewer msve-geocentric.earth

