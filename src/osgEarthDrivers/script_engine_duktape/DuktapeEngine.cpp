/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include "JSGeometry"
#include <osgEarth/JsonUtils>
#include <osgEarth/StringUtils>
#include <osgEarthFeatures/GeometryUtils>
#include <sstream>

#undef  LC
#define LC "[duktape] "

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
    // generic logging function.
    static duk_ret_t log( duk_context *ctx ) {
        duk_idx_t i, n;

        std::string msg;
        for( i = 0, n = duk_get_top( ctx ); i < n; i++ ) {
            if( i > 0 ) {
                msg += " ";
            }
            msg += duk_safe_to_string( ctx, i );
        }
        OE_WARN << LC << msg << std::endl;
        return 0;
    }

    static duk_ret_t oe_duk_save_feature(duk_context* ctx)
    {
        // stack: [ptr]

        // pull the feature ptr from argument #0
        Feature* feature = reinterpret_cast<Feature*>(duk_require_pointer(ctx, 0));

        // Fetch the feature data:
        duk_push_global_object(ctx);                    
        // [ptr, global]

        if ( !duk_get_prop_string(ctx, -1, "feature") || !duk_is_object(ctx, -1))
            return 0;

         // [ptr, global, feature]

        if ( duk_get_prop_string(ctx, -1, "properties") && duk_is_object(ctx, -1) )
        {
            // [ptr, global, feature, props]
            duk_enum(ctx, -1, 0);                       
        
            // [ptr, global, feature, props, enum]
            while( duk_next(ctx, -1, 1/*get_value=true*/) )
            {
                std::string key( duk_get_string(ctx, -2) );
                if (duk_is_string(ctx, -1))
                {
                    feature->set( key, std::string(duk_get_string(ctx, -1)) );
                }
                else if (duk_is_number(ctx, -1))
                {
                    feature->set( key, (double)duk_get_number(ctx, -1) );
                }
                else if (duk_is_boolean(ctx, -1))
                {
                    feature->set( key, duk_get_boolean(ctx, -1) );
                }
                else if( duk_is_null_or_undefined( ctx, -1 ) )
                {
                    feature->setNull( key );
                }
                 duk_pop_2(ctx);
            }

            duk_pop_2(ctx);
            // [ptr, global, feature]
        }
        else
        {   // [ptr, global, feature, undefined]
            duk_pop(ctx);
            // [ptr, global, feature]
        }

        // save the geometry, if set:
        if ( duk_get_prop_string(ctx, -1, "geometry")&& duk_is_object(ctx, -1) )
        {
            // [ptr, global, feature, geometry]
            std::string json( duk_json_encode(ctx, -1) ); // [ptr, global, feature, json]
            Geometry* newGeom = GeometryUtils::geometryFromGeoJSON(json);
            if ( newGeom )
            {
                feature->setGeometry( newGeom );
            }
            duk_pop(ctx);
            // [ptr, global, feature]
        }
        else
        {
            // [ptr, global, feature, undefined]
        }
        
        // [ptr, global, feature]
        duk_pop_2(ctx);     // [ptr] (as we found it)
        return 0;           // no return values.
    }
}

//............................................................................

namespace
{
    // Create a "feature" object in the global namespace.
    void setFeature(duk_context* ctx, Feature const* feature)
    {
        std::string geojson = feature->getGeoJSON();
        
        duk_push_global_object(ctx);                         // [global]
        duk_push_string(ctx, geojson.c_str());               // [global, json]
        duk_json_decode(ctx, -1);                            // [global, feature]
        duk_push_pointer(ctx, (void*)feature);               // [global, feature, ptr]
        duk_put_prop_string(ctx, -2, "__ptr");               // [global, feature]
        duk_put_prop_string(ctx, -2, "feature");             // [global]

        // add the save() function and the "attributes" alias.
        duk_eval_string_noresult(ctx,
            "feature.save = function() {"
            "    oe_duk_save_feature(this.__ptr);"
            "} ");

        duk_eval_string_noresult(ctx,
            "Object.defineProperty(feature, 'attributes', {get:function() {return feature.properties;}});");

        GeometryAPI::bindToFeature(ctx);

        duk_pop(ctx); 
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

        duk_push_global_object( _ctx );

        // Add global log function.
        duk_push_c_function( _ctx, log, DUK_VARARGS );
        duk_put_prop_string( _ctx, -2, "log" );

        // feature.save() callback
        duk_push_c_function(_ctx, oe_duk_save_feature, 1/*numargs*/); // [global, function]
        duk_put_prop_string(_ctx, -2, "oe_duk_save_feature");         // [global]

        GeometryAPI::install(_ctx);

        duk_pop(_ctx); // []
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
		// encode the feature in the global object and push a
        // native pointer:
		setFeature(ctx, feature);
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
