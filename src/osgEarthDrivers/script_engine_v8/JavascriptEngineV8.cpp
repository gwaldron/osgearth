/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

#include <osgEarthDrivers/script_engine_v8/JavascriptEngineV8>
#include <osgEarthDrivers/script_engine_v8/JSWrappers>
#include <osgEarthDrivers/script_engine_v8/V8Util>

#include <osgEarthFeatures/Script>
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarth/Notify>
#include <osgEarth/StringUtils>

#include <v8.h>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[JavascriptEngineV8] "


//----------------------------------------------------------------------------

JavascriptEngineV8::JavascriptEngineV8(const ScriptEngineOptions& options)
: ScriptEngine(options)
{
  v8:: HandleScope handle_scope;

  v8::Handle<v8::ObjectTemplate> global = createGlobalObjectTemplate();
  _globalContext = v8::Context::New(NULL, global);

  if (options.script().isSet() && !options.script()->getCode().empty())
  {
    // Create a nested handle scope
    v8::HandleScope local_handle_scope;

    // Enter the global context
    v8::Context::Scope context_scope(_globalContext);

    // Compile and run the script
    ScriptResult result = executeScript(v8::String::New(options.script()->getCode().c_str(), options.script()->getCode().length()));
    if (!result.success())
      OE_WARN << LC << "Error reading javascript: " << result.message() << std::endl;
  }

  _globalTemplate = v8::Persistent<v8::ObjectTemplate>::New(global);
}

JavascriptEngineV8::~JavascriptEngineV8()
{
  _globalTemplate.Dispose();
  _globalContext.Dispose();
}

v8::Local<v8::ObjectTemplate>
JavascriptEngineV8::createGlobalObjectTemplate()
{
  v8:: HandleScope handle_scope;

  v8::Handle<v8::ObjectTemplate> global = v8::ObjectTemplate::New();

  // add callback for global logging
  global->Set(v8::String::New("log"), v8::FunctionTemplate::New(logCallback));

  // add constructor callbacks for native objects
  //global->Set(v8::String::New("Feature"), v8::FunctionTemplate::New(constructFeatureCallback));
  global->Set(v8::String::New("Bounds"), v8::FunctionTemplate::New(constructBoundsCallback));
  global->Set(v8::String::New("Vec3d"), v8::FunctionTemplate::New(constructVec3dCallback));
  global->Set(v8::String::New("GeoExtent"), v8::FunctionTemplate::New(constructGeoExtentCallback));
  global->Set(v8::String::New("SpatialReference"), v8::FunctionTemplate::New(constructSpatialReferenceCallback));
  //global->Set(v8::String::New("Geometry"), v8::FunctionTemplate::New(constructSymbologyGeometryCallback));
  
  return handle_scope.Close(global);
}

v8::Handle<v8::Value>
JavascriptEngineV8::logCallback(const v8::Arguments& args)
{
  if (args.Length() < 1) return v8::Undefined();
  v8::HandleScope scope;
  v8::Handle<v8::Value> arg = args[0];
  v8::String::AsciiValue value(arg);
  
  OE_WARN << LC << "javascript message: " << (*value) << std::endl;

  return v8::Undefined();
}

ScriptResult
JavascriptEngineV8::executeScript(v8::Handle<v8::String> script)
{
  // Handle scope for temporary handles.
  v8::HandleScope handle_scope;

  // TryCatch for any script errors
  v8::TryCatch try_catch;

  // Compile the script
  v8::Handle<v8::Script> compiled_script = v8::Script::Compile(script);
  if (compiled_script.IsEmpty())
  {
    v8::String::AsciiValue error(try_catch.Exception());
    return ScriptResult(EMPTY_STRING, false, std::string("Script compile error: ") + std::string(*error));
  }

  // Run the script
  v8::Handle<v8::Value> result = compiled_script->Run();
  if (result.IsEmpty())
  {
    v8::String::AsciiValue error(try_catch.Exception());
    return ScriptResult(EMPTY_STRING, false, std::string("Script result was empty: ") + std::string(*error));
  }

  v8::String::AsciiValue ascii(result);
  return ScriptResult(std::string(*ascii));
}

ScriptResult
JavascriptEngineV8::run(Script* script, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
  if (!script)
    return ScriptResult(EMPTY_STRING, false, "Script is null.");

  return run(script->getCode(), feature, context);
}

ScriptResult
JavascriptEngineV8::run(const std::string& code, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
  if (code.empty())
    return ScriptResult(EMPTY_STRING, false, "Script is empty.");

  v8::Locker locker;
  v8::HandleScope handle_scope;

  //Create a separate context
  //v8::Persistent<v8::Context> context = v8::Context::New(NULL, _globalTemplate);
  //v8::Context::Scope context_scope(context);
  v8::Context::Scope context_scope(_globalContext);

  v8::Handle<v8::Object> fObj = JSFeature::WrapFeature(const_cast<Feature*>(feature));
  _globalContext->Global()->Set(v8::String::New("feature"), fObj);

  v8::Handle<v8::Object> cObj = JSFilterContext::WrapFilterContext(const_cast<FilterContext*>(context));
  _globalContext->Global()->Set(v8::String::New("context"), cObj);

  // Compile and run the script
  ScriptResult result = executeScript(v8::String::New(code.c_str(), code.length()));

  //context.Dispose();

  return result;
}

ScriptResult
JavascriptEngineV8::call(const std::string& function, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
  if (function.empty())
    return ScriptResult(EMPTY_STRING, false, "Empty function name parameter.");

  // Lock for V8 multithreaded uses
  v8::Locker locker;

  v8::HandleScope handle_scope;

  v8::Context::Scope context_scope(_globalContext);

  // Attempt to fetch the function from the global object.
  v8::Handle<v8::String> func_name = v8::String::New(function.c_str(), function.length());
  v8::Handle<v8::Value> func_val = _globalContext->Global()->Get(func_name);

  // If there is no function, or if it is not a function, bail out
  if (!func_val->IsFunction())
    return ScriptResult(EMPTY_STRING, false, "Function not found in script.");

  v8::Handle<v8::Function> func_func = v8::Handle<v8::Function>::Cast(func_val);

  v8::Handle<v8::Object> fObj = JSFeature::WrapFeature(const_cast<Feature*>(feature));
  _globalContext->Global()->Set(v8::String::New("feature"), fObj);

  v8::Handle<v8::Object> cObj = JSFilterContext::WrapFilterContext(const_cast<FilterContext*>(context));
  _globalContext->Global()->Set(v8::String::New("context"), cObj);

  // Set up an exception handler before calling the Eval function
  v8::TryCatch try_catch;

  // Invoke the specifed function
  //const int argc = 1;
  //v8::Handle<v8::Value> argv[argc] = { fObj };
  //v8::Handle<v8::Value> result = func_func->Call(_globalContext->Global(), argc, argv);
  v8::Handle<v8::Value> result = func_func->Call(_globalContext->Global(), 0, NULL);

  if (result.IsEmpty())
  {
    return ScriptResult(EMPTY_STRING, false, "Function result was empty.");
  }
  else
  {
    v8::String::AsciiValue ascii(result);
    return ScriptResult(std::string(*ascii));
  }
}

//----------------------------------------------------------------------------
// Constructor callbacks for constructing native objects in javascript

//v8::Handle<v8::Value>
//JavascriptEngineV8::constructFeatureCallback(const v8::Arguments &args)
//{
//  if (!args.IsConstructCall()) 
//    return v8::ThrowException(v8::String::New("Cannot call constructor as function"));
// 
//	v8::HandleScope handle_scope;
// 
//  Feature* feature;
//  if (args.Length() == 0)
//  {
//    feature = new Feature();
//  }
//  else if (args.Length() == 1 && args[0]->IsUint32())
//  {
//    feature = new Feature(args[0]->Uint32Value());
//  }
//  else
//  {
//    //TODO: add support for other Feature constructors
//  }
//
//  if (feature)
//    return JSFeature::WrapFeature(feature, true);
//
//  return v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
//}

v8::Handle<v8::Value>
JavascriptEngineV8::constructBoundsCallback(const v8::Arguments &args)
{
  if (!args.IsConstructCall()) 
    return v8::ThrowException(v8::String::New("Cannot call constructor as function"));
 
	v8::HandleScope handle_scope;
 
  osgEarth::Bounds* bounds;
  //if (args.Length() == 0)
  //  bounds = new osgEarth::Bounds();
  //else
  if (args.Length() == 4)
    bounds = new osgEarth::Bounds(args[0]->NumberValue(), args[1]->NumberValue(), args[2]->NumberValue(), args[3]->NumberValue());

  if (bounds)
    return JSBounds::WrapBounds(bounds, true);

  return v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
}

v8::Handle<v8::Value>
JavascriptEngineV8::constructVec3dCallback(const v8::Arguments &args)
{
  if (!args.IsConstructCall()) 
    return v8::ThrowException(v8::String::New("Cannot call constructor as function"));
 
	v8::HandleScope handle_scope;

  osg::Vec3d* vec;
  if (args.Length() == 0)
    vec = new osg::Vec3d();
  else if (args.Length() == 1 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* rhs = V8Util::UnwrapObject<osg::Vec3d>(obj);
      vec = new osg::Vec3d(*rhs);
    }
  }
  else if (args.Length() == 3)
    vec = new osg::Vec3d(args[0]->NumberValue(), args[1]->NumberValue(), args[2]->NumberValue());

  if (vec)
    return JSVec3d::WrapVec3d(vec, true);

  return v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
}

v8::Handle<v8::Value>
JavascriptEngineV8::constructGeoExtentCallback(const v8::Arguments &args)
{
  if (!args.IsConstructCall()) 
    return v8::ThrowException(v8::String::New("Cannot call constructor as function"));
 
	v8::HandleScope handle_scope;

  osgEarth::GeoExtent* extent;// = new osgEarth::GeoExtent(
  //if (args.Length() == 0)
  //  extent = new osgEarth::GeoExtent();
  //else
  if (args.Length() == 1 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    //if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
    //{
    //  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
    //  extent = new osgEarth::GeoExtent(srs);
    //}
    //else
    if (V8Util::CheckObjectType(obj, JSGeoExtent::GetObjectType()))
    {
      osgEarth::GeoExtent* rhs = V8Util::UnwrapObject<osgEarth::GeoExtent>(obj);
      extent = new osgEarth::GeoExtent(*rhs);
    }
  }
  else if (args.Length() == 2 && args[0]->IsObject() && args[1]->IsObject())
  {
    v8::Local<v8::Object> obj0( v8::Object::Cast(*args[0]) );
    v8::Local<v8::Object> obj1( v8::Object::Cast(*args[1]) );

    if (V8Util::CheckObjectType(obj0, JSSpatialReference::GetObjectType()) && V8Util::CheckObjectType(obj1, JSBounds::GetObjectType()))
    {
      osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj0);
      osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(obj1);
      extent = new osgEarth::GeoExtent(srs, *bounds);
    }
  }
  else if (args.Length() == 5 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
    {
      osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
      extent = new osgEarth::GeoExtent(srs, args[1]->NumberValue(), args[2]->NumberValue(), args[3]->NumberValue(), args[4]->NumberValue());
    }
  }

  if (extent)
    return JSGeoExtent::WrapGeoExtent(extent, true);

  return v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
}

v8::Handle<v8::Value>
JavascriptEngineV8::constructSpatialReferenceCallback(const v8::Arguments &args)
{
  if (!args.IsConstructCall()) 
    return v8::ThrowException(v8::String::New("Cannot call constructor as function"));

	v8::HandleScope handle_scope;
 
  osgEarth::SpatialReference* srs;
  if (args.Length() == 1 && args[0]->IsString())
  {
    v8::String::Utf8Value utf8_value(args[0]->ToString());
    std::string init(*utf8_value);
    srs = osgEarth::SpatialReference::create(init);
  }

  if (srs)
    return JSSpatialReference::WrapSpatialReference(srs, true);

  return v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
}

//v8::Handle<v8::Value>
//JavascriptEngineV8::constructSymbologyGeometryCallback(const v8::Arguments &args)
//{
//	v8::HandleScope handle_scope;
// 
//  osgEarth::Symbology::Geometry* geom;
//  if (args.Length() == 2)
//    geom = new osgEarth::Symbology::Geometry::create(
//
//  if (geom)
//    return JSBounds::WrapBounds(bounds, true);
//
//  //return v8::ThrowException(v8::String::New("Unsupported arguments"));
//  return v8::Handle<v8::Value>();
//}