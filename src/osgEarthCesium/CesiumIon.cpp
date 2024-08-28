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

CesiumIon::~CesiumIon()
{
    shutdown();
}

osgEarth::Cesium::CesiumIon& CesiumIon::instance()
{
    static CesiumIon s_cesiumIon;
    return s_cesiumIon;
}

void CesiumIon::shutdown()
{
    for (auto& c : contexts)
    {
        delete c.second;
    }
    contexts.clear();
}

Context* CesiumIon::getContext(const std::string& server)
{
    if (contexts.find(server) != contexts.end()) {
        return contexts[server];
    }
    else {
        Context* context = new Context();

        ApplicationData data;
        data.authenticationMode = AuthenticationMode::SingleUser;

        context->connection = std::unique_ptr<Connection>(new Connection(context->asyncSystem, context->assetAccessor, getCesiumIonKey(), data, server));

        contexts[server] = context;

        bool loaded = false;

        context->connection->assets().thenInMainThread([&](Response<Assets>&& result) {

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
            context->asyncSystem.dispatchMainThreadTasks();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return context;
    }
}

void CesiumIon::refresh()
{

}