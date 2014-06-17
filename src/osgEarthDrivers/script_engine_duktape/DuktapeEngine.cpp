/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "DuktapeEngine"

#define LC "[DuktapeEngine] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers::Duktape;


DuktapeEngine::DuktapeEngine(const ScriptEngineOptions& options) :
ScriptEngine(options)
{
    _ctx = duk_create_heap_default();
}

DuktapeEngine::~DuktapeEngine()
{
    duk_destroy_heap(_ctx);
}

ScriptResult
DuktapeEngine::run(Script*              script, 
                   Feature const*       feature,
                   FilterContext const* context)
{
    if (!script)
        return ScriptResult(EMPTY_STRING, false, "Script is null.");

    return run(script->getCode(), feature, context);
}

ScriptResult
DuktapeEngine::run(const std::string&   code,
                   Feature const*       feature,
                   FilterContext const* context)
{
    if (code.empty())
        return ScriptResult(EMPTY_STRING, false, "Script is empty.");

    if (feature)
    {
        JSStringRef featureStr = JSStringCreateWithUTF8CString("feature");
        JSObjectRef jsFeature = JSObjectMake(_ctx, JSFeature_class(_ctx), const_cast<osgEarth::Features::Feature*>(feature));
        JSObjectSetProperty(_ctx, JSContextGetGlobalObject(_ctx), featureStr, jsFeature, kJSPropertyAttributeNone, NULL);
    }

    // Compile and run the script
    ScriptResult result = executeScript(code);

    return result;
}

ScriptResult
JavaScriptCoreEngine::call(const std::string& function, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
    return ScriptResult("");
}
