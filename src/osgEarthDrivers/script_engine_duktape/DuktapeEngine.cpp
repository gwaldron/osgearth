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
#include <sstream>

#define LC "[DuktapeEngine] "

// defining this will setup and tear down a complete duktape heap/context
// for each and every invocation. Good for testing memory usage until we
// complete the feature set.
#define MAXIMUM_ISOLATION

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers::Duktape;

//............................................................................

namespace
{
    // Creates the feature object for ECMA5.
    // TODO: for ECMA6 we might use a Proxy object and C callback? Maybe,
    // but this is pretty quick.
    // TODO: this will cause a Syntax Error for a property key or value that
    // has an invalid encoding -- need to fix that.
    std::string createFeatureScaffoldEcma5(Feature const* feature)
    {
        std::stringstream buf;
        buf << "var feature = {";

        const AttributeTable& attrs = feature->getAttrs();
        int n=0;
        for(AttributeTable::const_iterator a = attrs.begin(); a != attrs.end(); ++a)
        {
            if ( !a->first.empty() )
            {
                if ( n > 0 ) buf << ", ";
                buf << a->first << ":\"" << a->second.getString() << "\"";
                ++n;
            }
        }
        buf << "};";

        // This adds a meta-property "attributes" that allows you to use
        // the idiom: feature.attributes[prop]
        buf << "Object.defineProperty(feature, 'attributes', {get:function() {return feature;}});";

        std::string str = buf.str();
        return str;
    }

    extern "C"
    {
        /**
         * C binding for "get_feature_attr(feature, attrName)" JS method
         */
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
            bool ok = (duk_peval_string(_ctx, options.script()->getCode().c_str()) == 0);
            if ( !ok )
            {
                const char* err = duk_safe_to_string(_ctx, -1);
                OE_WARN << LC << err << std::endl;
            }
            duk_pop(_ctx);
        }

        // new value stack:
        duk_push_global_object( _ctx );

        // install C bindings.
        duk_push_c_function( _ctx, oeduk_get_feature_attr, 2/*numargs*/);
        duk_put_prop_string( _ctx, -2, "c_get_feature_attr" );
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

    std::string e = code;

    if (feature)
    {
        // set a global object "feature" = native pointer
        //duk_push_pointer(ctx, (void*)feature);
        //duk_put_prop_string(ctx, -2, "c_feature");
        
        e = createFeatureScaffoldEcma5(feature) + "\n" + e;
    }

    std::string resultString;

    // run the script. On error, the top of stack will hold the error
    // message instead of the return value.
    bool ok = (duk_peval_string(ctx, e.c_str()) == 0);
    const char* resultVal = duk_to_string(ctx, -1);
    if ( resultVal )
        resultString = resultVal;

    if ( !ok )
    {
        OE_WARN << LC << "Error: source =\n" << e << std::endl;
    }

    // pop the return value:
    duk_pop(ctx);

    // force a garbage collection to keep memory in check
    duk_gc(ctx, 0);

    return ok ?
        ScriptResult(resultString, true) :
        ScriptResult("", false, resultString);
}

ScriptResult
DuktapeEngine::call(const std::string& function, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
    return ScriptResult("", false);
}
