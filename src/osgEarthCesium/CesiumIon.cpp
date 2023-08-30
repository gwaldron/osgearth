/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include "CesiumIon"

#include "Context"
#include "Settings"
#include <CesiumIonClient/Connection.h>

using namespace osgEarth::Cesium;
using namespace CesiumIonClient;

CesiumIon::CesiumIon()
{
    refresh();
}

void CesiumIon::refresh()
{
    Connection connection(Context::instance().asyncSystem, Context::instance().assetAccessor, getCesiumIonKey());

    bool loaded = false;

    connection.assets().thenInMainThread([&](Response<Assets>&& result) {
        assets.clear();

        loaded = true;

        if (result.value.has_value())
        {
            for (auto& a : result.value->items)
            {
                CesiumIonAsset asset;
                asset.attribution = a.attribution;
                asset.bytes = a.bytes;
                asset.dateAdded = a.dateAdded;
                asset.description = a.description;
                asset.id = a.id;
                asset.name = a.name;
                asset.percentComplete = a.percentComplete;
                asset.status = a.status;
                asset.type = a.type;
                assets.emplace_back(std::move(asset));
            }
        }
     });

    // Wait for the assets to be loaded.
    while (!loaded)
    {
        Context::instance().asyncSystem.dispatchMainThreadTasks();
    }    
}