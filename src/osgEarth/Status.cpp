/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Status>

using namespace osgEarth;

#define LC "[Status] "

const osgEarth::Status osgEarth::STATUS_OK;

std::string osgEarth::Status::_codeText[6] = {
    "No error",
    "Resource unavailable",
    "Service unavailable",
    "Configuration error",
    "Assertion failure",
    "Error"
};
