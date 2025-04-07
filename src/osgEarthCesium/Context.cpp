/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
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

