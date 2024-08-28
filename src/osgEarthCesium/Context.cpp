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

#include "Context"
#include "Settings"

#include <CesiumIonClient/Connection.h>

using namespace osgEarth::Cesium;

Context::Context():
    taskProcessor(std::make_shared<TaskProcessor>()),
    asyncSystem(taskProcessor)
{
    Cesium3DTilesContent::registerAllTileContentTypes();
    assetAccessor = std::make_shared<AssetAccessor>();
    prepareRenderResources = std::make_shared< PrepareRendererResources >();
    logger = spdlog::default_logger();
    creditSystem = std::make_shared<CesiumUtility::CreditSystem>();
}

Context::~Context()
{
    shutdown();
}

void Context::shutdown()
{
    // Shutdown the task processor
    taskProcessor->shutdown();

    // Finish off any main thread jobs
    assetAccessor->tick();
    asyncSystem.dispatchMainThreadTasks();
}

