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

namespace
{
    extern "C"
    {
        int oeduk_getFeatureAttr(duk_context* ctx)
        {
            Feature*    feature = reinterpret_cast<Feature*>(duk_require_pointer(ctx, 0));
            const char* key     = duk_require_string(ctx, 1);
            std::string value = feature->getString(key);
            duk_push_string(ctx, value.c_str());
            return 1;
        }
    }
}


DuktapeEngine::DuktapeEngine(const ScriptEngineOptions& options) :
ScriptEngine(options)
{
}

DuktapeEngine::~DuktapeEngine()
{
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
    
    // new allocation heap.
    duk_context* ctx = duk_create_heap_default();

    // new value stack:
    duk_push_global_object( ctx );

    // install C bindings.
    duk_push_c_function( ctx, oeduk_getFeatureAttr, 2/*numargs*/);
    duk_put_prop_string( ctx, -2, "osgEarthGetFeatureAttr" );

    if (feature)
    {
        // set a global object "feature" = native pointer
        duk_push_pointer(ctx, (void*)feature);
        duk_put_prop_string(ctx, -2, "feature");
    }

    // run the script:
    duk_eval_string(ctx, code.c_str());

    // read the result from the top of the stack:
    std::string resultString = duk_get_string(ctx, -1);

    // pop the value stack.
    duk_pop(ctx);
    
    // clean up.
    duk_destroy_heap(ctx);

    return ScriptResult(resultString);
}

ScriptResult
DuktapeEngine::call(const std::string& function, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
    return ScriptResult("", false);
}
