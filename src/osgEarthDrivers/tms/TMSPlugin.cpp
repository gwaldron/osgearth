/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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
#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarthUtil/TMS>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

#include "TMSOptions"
#include "TMSTileSource"

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[TMS driver] "

namespace osgEarth { namespace Drivers { namespace TileMapService
{
    class TMSDriver : public TileSourceDriver
    {
    private:
        typedef std::map< std::string,osg::ref_ptr<TMS::TileMap> > TileMapCache;
        TileMapCache _tileMapCache;

    public:
        TMSDriver()
        {
            supportsExtension( "osgearth_tms", "Tile Map Service Driver" );
        }

        virtual const char* className() const
        {
            return "Tile Map Service Driver";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return new TMSTileSource( getTileSourceOptions(options) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_tms, TMSDriver)

} } } // namespace osgEarth::Drivers::TMS

