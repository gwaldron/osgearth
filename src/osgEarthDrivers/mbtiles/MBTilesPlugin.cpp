/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include "MBTilesOptions"
#include "MBTilesTileSource"

#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

using namespace osgEarth;

#define LC "[MBTilesPlugin] "

namespace osgEarth { namespace Drivers { namespace MBTiles
{
    class MBTilesPlugin : public TileSourceDriver
    {
    public:
        MBTilesPlugin()
        {
            supportsExtension( "osgearth_mbtiles", "MBTiles tile driver" );
        }

        virtual const char* className() const
        {
            return "MBTiles Driver";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            std::string iname = getInterfaceName(options);

            if ( iname == TileSource::INTERFACE_NAME )
                return new MBTilesTileSource( getTileSourceOptions(options) );

            return ReadResult::FILE_NOT_FOUND;
        }
    };

    REGISTER_OSGPLUGIN(osgearth_mbtiles, MBTilesPlugin)

} } } // namespace osgEarth::Drivers::MBTiles
