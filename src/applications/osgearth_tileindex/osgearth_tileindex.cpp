/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osg/ArgumentParser>
#include <osgEarth/Notify>
#include <osg/Timer>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgEarth/Progress>
#include <osgEarthUtil/TileIndexBuilder>



using namespace osgDB;
using namespace osgEarth;
using namespace osgEarth::Util;

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    std::string indexFilename = "index.shp";
    while (arguments.read("--index", indexFilename));

    OE_NOTICE << "index name = " << indexFilename << std::endl;

    std::vector< std::string > filenames;

    //The rest of the arguments are filenames
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filenames.push_back( arguments[ pos ] );            
        }
    }    

	if (filenames.empty())
	{
		OE_NOTICE << "Please specify a directory or list of files to build an index for" << std::endl;
		return 1;
	}

    // Open or create the index file
    if (osgDB::fileExists( indexFilename ) )
    {
        OE_NOTICE << indexFilename << " exists, cannot update existing index" << std::endl;
        return 1;
    }

    osg::Timer_t start = osg::Timer::instance()->tick();

    TileIndexBuilder builder;
    builder.setProgressCallback( new ConsoleProgressCallback() );
    for (unsigned int i = 0; i < filenames.size(); i++)
    {
        builder.getFilenames().push_back( filenames[i] );
    }        
    builder.build( indexFilename );

    osg::Timer_t end = osg::Timer::instance()->tick();
    OE_NOTICE << "Built index " << indexFilename << " in " << osg::Timer::instance()->delta_s( start, end) << "s" << std::endl;


    return 0;
}
