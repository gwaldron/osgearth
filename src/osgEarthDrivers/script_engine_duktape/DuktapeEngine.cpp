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
#include <osgEarth/JsonUtils>
#include <osgEarth/StringUtils>
#include <sstream>

#define LC "[DuktapeEngine] "

// defining this will setup and tear down a complete duktape heap/context
// for each and every invocation. Good for testing memory usage until we
// complete the feature set.
//#define MAXIMUM_ISOLATION

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers::Duktape;

//............................................................................

namespace
{
    // Updates the global feature object with new attributes.
    void updateFeature(duk_context* ctx, Feature const* feature)
    {
        duk_push_global_object(ctx);        // [ global ]
        duk_push_string(ctx, "feature");    // [ global, "feature" ]
        duk_get_prop(ctx, -2);              // [ global, feature ]

        // add a property for the Feature ID:
        std::string fid = Stringify() << feature->getFID();
        duk_push_string(ctx, "id");               // [global, feature, "id"]
        duk_push_int(ctx, feature->getFID());     // [global, feature, "id", fid]
        ::duk_put_prop(ctx, -3);                  // [global, feature]

        // add each property to the object:
        const AttributeTable& attrs = feature->getAttrs();
        for(AttributeTable::const_iterator a = attrs.begin(); a != attrs.end(); ++a)
        {
            if ( !a->first.empty() )
            {
                duk_push_string(ctx, a->second.getString().c_str()); // [global, feature, value]
                duk_put_prop_string(ctx, -2, a->first.c_str());      // [global, feature]
            }
        }

        duk_pop_2(ctx); // []
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
        // new heap + context.
        _ctx = duk_create_heap_default();

        // if there is a static script, evaluate it first. This will register
        // any functions or objects with the EcmaScript global object.
        if ( options.script().isSet() )
        {
            bool ok = (duk_peval_string(_ctx, options.script()->getCode().c_str()) == 0); // [ "result" ]
            if ( !ok )
            {
                const char* err = duk_safe_to_string(_ctx, -1);
                OE_WARN << LC << err << std::endl;
            }
            duk_pop(_ctx); // []
        }

        // Create the global feature object.
        {
            duk_push_global_object(_ctx);             // [ global ]         feature object's home
            duk_push_object(_ctx);                    // [ global feature ] empty object for starters
            duk_put_prop_string(_ctx, -2, "feature"); // [ global ]         name it and add it to the global
            duk_pop(_ctx);                            // []                 pop the global

            // support for the idiom: feature.attributes['attr']
            duk_eval_string_noresult(_ctx, "Object.defineProperty(feature, 'attributes', {get:function() {return feature;}});");
        }
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
ScriptEngine( options ),
_options    ( options )
{
    //nop
}

DuktapeEngine::~DuktapeEngine()
{
    //nop
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

	if(feature) {
		// encode the feature in the global object:
		updateFeature(ctx, feature);
	}


    // run the script. On error, the top of stack will hold the error
    // message instead of the return value.
    std::string resultString;
    bool ok = (duk_peval_string(ctx, code.c_str()) == 0); // [ "result" ]
    const char* resultVal = duk_to_string(ctx, -1);
    if ( resultVal )
        resultString = resultVal;

    if ( !ok )
    {
        OE_WARN << LC << "Error: source =\n" << code << std::endl;
    }

    // pop the return value:
    duk_pop(ctx); // []

    return ok ?
        ScriptResult(resultString, true) :
        ScriptResult("", false, resultString);
}
