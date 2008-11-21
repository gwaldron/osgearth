osgEarth - Dynamic terrain generator


--- REQUIREMENTS ---

- OpenSceneGraph (OSG) version 2.6 or newer
- EXPAT XML library (http://expat.sourceforge.net)
- Squish DDS compression library (http://code.google.com/p/libsquish)
- cURL file transfer library (http://curl.haxx.se) (**)

(**) Also included in MEW's 3rd-party dependency set located here (http://mew.cx/osg)


--- BUILDING ---

- Use CMake 2.4.7 to build (same utility used to build OSG).

--- INSTALLING ---

- Copy "osgEarth" shared library (.dll/.so) to the OSG "bin" folder
- Copy all the plugin librarys (osgdb_*.dll/.do) to the OSG bin/osgPlugins-x.x.x folder

--- RUNNING ---

- Example earth files are located in the "tests" folder. Simply use osgviewer to load these files.

- You can edit the files to customize the cache folder and the proxy server if necessary.

Example: osgviewer msve-geocentric.earth

--- EXPORTING ---

You can "export" a terrain using the osgearth_export utility. Exporting downloads all the tiles and stores them in a directory to you can run the dataset offline. It also generates a new .earth file to run the offline database.

Type osgearth_export --help for run instructions.

Example: osgviewer --export f:\temp\exported-data --max-level 5 --bounds -111 30 -65 50 input.earth
