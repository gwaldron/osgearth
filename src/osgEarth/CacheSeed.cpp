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

#include <osgEarth/CacheSeed>
#include <osgEarth/ImageLayer>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace OpenThreads;

CacheTileHandler::CacheTileHandler( TerrainLayer* layer, const Map* map ):
_layer( layer ),
_map( map )
{
}

bool CacheTileHandler::handleTile(const TileKey& key, const TileVisitor& tv)
{        
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    // Just call createImage or createHeightField on the layer and the it will be cached!
    if (imageLayer)
    {                
        GeoImage image = imageLayer->createImage( key );
        if (image.valid())
        {                
            return true;
        }            
    }
    else if (elevationLayer )
    {
        GeoHeightField hf = elevationLayer->createHeightField(key, 0L);
        if (hf.valid())
        {                
            return true;
        }            
    }

    // If we didn't produce a result but the key isn't within range then we should continue to 
    // traverse the children b/c a min level was set.
    if (!_layer->isKeyInLegalRange(key))
    {
        return true;
    }

    return false;        
}   

bool CacheTileHandler::hasData( const TileKey& key ) const
{
    return _layer->mayHaveData(key);
}

std::string CacheTileHandler::getProcessString() const
{
    std::stringstream buf;
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    unsigned index = _map->getIndexOfLayer(_layer.get());
    if (index < _map->getNumLayers())
    {
        buf << "osgearth_cache --seed ";
        if (imageLayer)
        {
            buf << " --image " << index << " ";
        }
        else if (elevationLayer)
        {
            buf << " --elevation " << index << " ";
        }
    }
    return buf.str();
}



/***************************************************************************************/

CacheSeed::CacheSeed():
_visitor(new TileVisitor())
{
}

TileVisitor* CacheSeed::getVisitor() const
{
    return _visitor.get();
}

void CacheSeed::setVisitor(TileVisitor* visitor)
{
    _visitor = visitor;
}

void CacheSeed::run( TerrainLayer* layer, const Map* map )
{
    _visitor->setTileHandler( new CacheTileHandler( layer, map ) );
    _visitor->run( map->getProfile() );
}