/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/GeometryUtils>
#include <osgEarth/Metrics>
#include <sstream>

#undef  LC
#define LC "[JavaScript] "

using namespace osgEarth;
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
                    feature->set( key, duk_get_boolean(ctx, -1) != 0 );
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
        if ( duk_get_prop_string(ctx, -1, "geometry") )
        {
            if (duk_is_object(ctx, -1))
            {
                // [ptr, global, feature, geometry]
                std::string json( duk_json_encode(ctx, -1) ); // [ptr, global, feature, json]
                Geometry* newGeom = GeometryUtils::geometryFromGeoJSON(json);
                if ( newGeom )
                {
                    feature->setGeometry( newGeom );
                }
                duk_pop(ctx); // [ptr, global, feature]
            }
            else
            {
                feature->setGeometry(nullptr);
            }
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
    void setFeature(duk_context* ctx, Feature const* feature, bool complete)
    {
        if (!feature)
            return;

        OE_PROFILING_ZONE;

        duk_push_global_object(ctx); // [global]

        // Complete profile: properties, geometry, and API bindings.
        if ( complete )
        {
            std::string geojson = feature->getGeoJSON();
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
        }

        // Minimal profile: ID and properties only. MUCH faster!
        else
        {
            duk_idx_t feature_i = duk_push_object(ctx);     // [global] [feature]
            {
                duk_push_number(ctx, feature->getFID());       // [global] [feature] [id]
                duk_put_prop_string(ctx, feature_i, "id");  // [global] [feature]

                duk_idx_t props_i = duk_push_object(ctx);   // [global] [feature] [properties]
                {
                    const AttributeTable& attrs = feature->getAttrs();
                    for(AttributeTable::const_iterator a = attrs.begin(); a != attrs.end(); ++a)
                    {
                        AttributeType type = a->second.type;
                        switch(type) {
                        case ATTRTYPE_DOUBLE: duk_push_number (ctx, a->second.getDouble()); break;          // [global] [feature] [properties] [name]
                        case ATTRTYPE_INT:    duk_push_number(ctx, (double)a->second.getInt()); break;             // [global] [feature] [properties] [name]
                        case ATTRTYPE_BOOL:   duk_push_boolean(ctx, a->second.getBool()?1:0); break;            // [global] [feature] [properties] [name]
#if 0
                        case ATTRTYPE_DOUBLEARRAY: break;
#endif
                        case ATTRTYPE_STRING:
                        default:              duk_push_string (ctx, a->second.getString().c_str()); break;  // [global] [feature] [properties] [name]
                        }
                        duk_put_prop_string(ctx, props_i, a->first.c_str()); // [global] [feature] [properties]
                    }
                }
                duk_put_prop_string(ctx, feature_i, "properties"); // [global] [feature]

                duk_idx_t geometry_i = duk_push_object(ctx);  // [global] [feature] [geometry]
                {
                    duk_push_string(ctx, Geometry::toString(feature->getGeometry()->getComponentType()).c_str()); // [global] [feature] [geometry] [type]
                    duk_put_prop_string(ctx, geometry_i, "type"); // [global] [feature] [geometry]
                }
                duk_put_prop_string(ctx, feature_i, "geometry");
            }
            duk_put_prop_string(ctx, -2, "feature"); // [global] [feature]
        }

        duk_pop(ctx);
    }

}

//............................................................................

DuktapeEngine::Context::Context()
{
    _ctx = nullptr;
    _bytecode = nullptr;
    _errorCount = 0u;
}

void
DuktapeEngine::Context::initialize(const ScriptEngineOptions& options, bool complete)
{
    if ( _ctx == nullptr)
    {
        // new heap + context.
        _ctx = duk_create_heap_default();

        // if there is a static script, evaluate it first. This will register
        // any functions or objects with the EcmaScript global object.
        if ( options.script().isSet() )
        {
            std::string temp(options.script()->getCode());
            bool ok = (duk_peval_string(_ctx, temp.c_str()) == 0); // [ "result" ]
            if ( !ok )
            {
                const char* err = duk_safe_to_string(_ctx, -1);
                OE_WARN << LC << err << std::endl;
            }
            duk_pop(_ctx); // []
        }

        duk_push_global_object( _ctx );

        // Add global log function.
        duk_push_c_function( _ctx, log, DUK_VARARGS ); // [global, function]
        duk_put_prop_string( _ctx, -2, "log" );        // [global]

        if ( complete )
        {
            // feature.save() callback
            duk_push_c_function(_ctx, oe_duk_save_feature, 1/*numargs*/); // [global, function]
            duk_put_prop_string(_ctx, -2, "oe_duk_save_feature");         // [global]

            GeometryAPI::install(_ctx);
        }

        duk_pop(_ctx); // []
    }
}

DuktapeEngine::Context::~Context()
{
    if ( _ctx )
    {
        duk_destroy_heap(_ctx);
        _ctx = nullptr;
    }
}

//............................................................................

DuktapeEngine::DuktapeEngine(const ScriptEngineOptions& options) :
    ScriptEngine(options),
    _options(options)
{
    //nop
}

DuktapeEngine::~DuktapeEngine()
{
    //nop
}

bool
DuktapeEngine::compile(
    Context& c,
    const std::string& code,
    ScriptResult& result)
{
    //OE_PROFILING_ZONE;

    duk_context* ctx = c._ctx;

    if (code != c._bytecodeSource)
    {
        c._bytecodeSource = code;
        c._errorCount = 0u;

        if (duk_pcompile_string(ctx, 0, code.c_str()) != 0) // [function|error]
        {
            std::string resultString = duk_safe_to_string(ctx, -1);
            OE_WARN << LC << "Compile error: " << resultString << std::endl;
            c._errorCount++;
            duk_pop(ctx); // []
            result = ScriptResult("", false, resultString); // return error.
            return false;
        }

        duk_dump_function(ctx); // [buffer]
        duk_size_t size;
        void* bytecode = duk_get_buffer(ctx, 0, &size);
        if (bytecode == nullptr)
        {
            // error. remove the buffer and return.
            duk_pop(ctx); // []
            OE_WARN << LC << "Allocation error; cannot continue" << std::endl;
            result = ScriptResult("", false, "Allocation error"); // return error
            c._errorCount++;
            return false;
        }

        if (c._bytecode) delete[] c._bytecode;
        c._bytecode = new unsigned char[size];
        ::memcpy(c._bytecode, (unsigned char*)bytecode, size);
        c._bytecodeSize = size;
    }
    else if (c._errorCount == 0u)
    {
        void* buf = duk_push_buffer(ctx, (duk_size_t)c._bytecodeSize, (duk_bool_t)0); // [buffer]
        ::memcpy(buf, c._bytecode, c._bytecodeSize);
    }
    else
    {
        // this code caused a previous compile error, so bail out.
        return false;
    }

    // convert bytecode to a function object:
    duk_load_function(ctx); // [function]

    return true;
}

bool
DuktapeEngine::run(
    const std::string& code,
    const FeatureList& features,
    std::vector<ScriptResult>& results,
    FilterContext const* context)
{
    if (code.empty())
    {
        for (auto& f : features)
            results.emplace_back(EMPTY_STRING, false, "Script is empty.");
        return false;
    }

    OE_PROFILING_ZONE;

    const bool complete = false;

    // cache the Context on a per-thread basis
    Context& c = _contexts.get();
    c.initialize(_options, complete);
    duk_context* ctx = c._ctx;

    std::string resultString;
    ScriptResult result;

    if (!compile(c, code, result)) // [function | null]
    {
        for (auto& f : features)
            results.push_back(result);
        return false;
    }

    for (auto& feature : features)
    {
        // Load the next feature into the global object:
        setFeature(c._ctx, feature.get(), complete);

        // Duplicate the function on the top since we'll be calling it multiple times
        duk_dup_top(ctx); // [function function]

        // Run the script:
        duk_int_t rc = duk_pcall(ctx, 0); // [function result]
        resultString = duk_safe_to_string(ctx, -1);
        duk_pop(ctx); // [function]

        if (rc != DUK_EXEC_SUCCESS)
        {
            OE_WARN << LC << "Runtime error: " << resultString << std::endl;
            c._errorCount++;
            results.emplace_back(EMPTY_STRING, false, resultString); // error
        }
        else
        {
            results.emplace_back(resultString, true);
        }
    }

    // Pop the function, clearing the stack
    duk_pop(ctx); // []

    return true;
}

ScriptResult
DuktapeEngine::run(
    const std::string& code,
    Feature const* feature,
    FilterContext const* context)
{
    if (code.empty())
        return ScriptResult(EMPTY_STRING, false, "Script is empty");

    if (!feature)
        return ScriptResult(EMPTY_STRING, false, "Feature is null");

    OE_PROFILING_ZONE;

    const bool complete = false;

    // cache the Context on a per-thread basis
    Context& c = _contexts.get();
    c.initialize( _options, complete );
    duk_context* ctx = c._ctx;

    // compile the function:
    ScriptResult result;
    if (!compile(c, code, result)) // [function]
    {
        return result;
    }

    // load the feature into the global namespace:
	if ( feature != c._feature.get() )
    {
		setFeature(ctx, feature, complete);
        c._feature = feature;
	}

    std::string resultString;

    duk_int_t rc = duk_pcall(ctx, 0); // [result]
    resultString = duk_safe_to_string(ctx, -1);
    duk_pop(ctx); // []

    if (rc != DUK_EXEC_SUCCESS)
    {
        OE_WARN << LC << "Runtime error: " << resultString << std::endl;
        c._errorCount++;
        return ScriptResult(EMPTY_STRING, false, resultString); // error
    }

    return ScriptResult(resultString, true);
}
