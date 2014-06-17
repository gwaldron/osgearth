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

#define MAXIMUM_ISOLATION

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers::Duktape;

//............................................................................

namespace
{
    extern "C"
    {
        int oeduk_get_feature_attr(duk_context* ctx)
        {
            Feature*    feature = reinterpret_cast<Feature*>(duk_require_pointer(ctx, 0));
            const char* key     = duk_require_string(ctx, 1);
            std::string value = feature->getString(key);
            duk_push_string(ctx, value.c_str());
            return 1;
        }
    }
}

//............................................................................

DuktapeEngine::Context::Context()
{
    _ctx = 0L;
}

void
DuktapeEngine::Context::initialize(const ScriptEngineOptions& options)
{
    if ( _ctx == 0L )
    {
        // new allocation heap.
        _ctx = duk_create_heap_default();

        // if there is a static script, evaluate it first
        if ( options.script().isSet() )
        {
            duk_eval_string_noresult(_ctx, options.script()->getCode().c_str());
        }

        // new value stack:
        duk_push_global_object( _ctx );

        // install C bindings.
        duk_push_c_function( _ctx, oeduk_get_feature_attr, 2/*numargs*/);
        duk_put_prop_string( _ctx, -2, "get_feature_attr" );
    }
}

DuktapeEngine::Context::~Context()
{
    if ( _ctx )
    {
        duk_destroy_heap(_ctx);
        _ctx = 0L;
    }
}

//............................................................................

DuktapeEngine::DuktapeEngine(const ScriptEngineOptions& options) :
ScriptEngine(options),
_options( options )
{
    //nop
}

DuktapeEngine::~DuktapeEngine()
{
    //nop
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

#ifdef MAXIMUM_ISOLATION
    // brand new context every time
    Context c;
    c.initialize( _options );
    duk_context* ctx = c._ctx;
#else
    // cache the Context on a per-thread basis
    Context& c = _contexts.get();
    c.initialize( _options );
    duk_context* ctx = c._ctx;
#endif

    if (feature)
    {
        // set a global object "feature" = native pointer
        duk_push_pointer(ctx, (void*)feature);
        duk_put_prop_string(ctx, -2, "feature");
    }

    // run the script:
    duk_eval_string(ctx, code.c_str());

    // convert the return value to a string and read it back:
    const char* resultVal = duk_to_string(ctx, -1);
    std::string resultString;
    if ( resultVal )
        resultString = resultVal;

    // pop the return value:
    duk_pop(ctx);

    // force a garbage collection to keep memory in check
    duk_gc(ctx, 0);

    return ScriptResult(resultString, resultVal? true : false);
}

ScriptResult
DuktapeEngine::call(const std::string& function, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
    return ScriptResult("", false);
}
