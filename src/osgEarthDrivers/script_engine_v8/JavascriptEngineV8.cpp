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
  _isolate = v8::Isolate::New();

  v8::Locker locker(_isolate);
  v8::Isolate::Scope isolate_scope(_isolate);

  v8::HandleScope handle_scope(_isolate);

  _globalContext.Reset(_isolate, createGlobalContext());
  if (options.script().isSet() && !options.script()->getCode().empty())
  {
    // Create a nested handle scope
    v8::HandleScope local_handle_scope(_isolate);

    // Enter the global context
    v8::Local<v8::Context> globalContext = *reinterpret_cast<v8::Local<v8::Context>*>(&_globalContext);
    v8::Context::Scope context_scope(globalContext);

    // Compile and run the script
    ScriptResult result = executeScript(v8::String::New(options.script()->getCode().c_str(), options.script()->getCode().length()));
    if (!result.success())
      OE_WARN << LC << "Error reading javascript: " << result.message() << std::endl;
  }
}

JavascriptEngineV8::~JavascriptEngineV8()
{
  {
    v8::Locker locker(_isolate);
    v8::Isolate::Scope isolate_scope(_isolate);

    _globalContext.Dispose();
  }

  _isolate->Dispose();
}

v8::Handle<v8::Context>
JavascriptEngineV8::createGlobalContext()
{
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
  
  return v8::Context::New(_isolate, NULL, global);
}

void
JavascriptEngineV8::logCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  if (info.Length() < 1) return;
  v8::HandleScope scope(v8::Isolate::GetCurrent());
  v8::Handle<v8::Value> arg = info[0];
  v8::String::AsciiValue value(arg);
  
  OE_WARN << LC << "javascript message: " << (*value) << std::endl;
}

ScriptResult
JavascriptEngineV8::executeScript(v8::Handle<v8::String> script)
{
  v8::String::Utf8Value utf8_value(script);
  std::string scriptStr(*utf8_value);

  // Handle scope for temporary handles.
  v8::HandleScope handle_scope(_isolate);

  // TryCatch for any script errors
  v8::TryCatch try_catch;

  // Compile the script
  v8::Handle<v8::Script> compiled_script = v8::Script::Compile(script);
  if (compiled_script.IsEmpty())
  {
    v8::String::AsciiValue error(try_catch.Exception());
	v8::Handle<v8::Message> message = try_catch.Message();
	if(!message.IsEmpty()) {
		//v8::String::AsciiValue filename(message->GetScriptResourceName());
		int linenum = message->GetLineNumber();
		std::ostringstream str;
		str << linenum << ":[" << message->GetStartColumn() << "-" << message->GetEndColumn() << "]:" << std::string(*error) << std::endl;
		v8::String::AsciiValue sourceline(message->GetSourceLine());
		str << std::string(*sourceline) << std::endl;
		/*
		v8::String::Utf8Value stack_trace(try_catch.StackTrace());
		if (stack_trace.length() > 0) {
			str <<  std::string(*stack_trace) << std::endl;
		}
		*/
	    return ScriptResult(EMPTY_STRING, false, std::string("Script compile error: ") + str.str());
	}
	else {
	    return ScriptResult(EMPTY_STRING, false, std::string("Script compile error: ") + std::string(*error));
	}
  }

  // Run the script
  v8::Handle<v8::Value> result = compiled_script->Run();
  if (result.IsEmpty())
  {
    v8::String::AsciiValue error(try_catch.Exception());
    return ScriptResult(EMPTY_STRING, false, std::string("Script result was empty: ") + std::string(*error));
  }

  if (result->IsUndefined())
    return ScriptResult(EMPTY_STRING, false, "Script result was undefined");

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

  v8::Locker locker(_isolate);
  v8::Isolate::Scope isolate_scope(_isolate);

  v8::HandleScope handle_scope(_isolate);

  v8::Local<v8::Context> globalContext = *reinterpret_cast<v8::Local<v8::Context>*>(&_globalContext);

  v8::Context::Scope context_scope(globalContext);

  if (feature)
  {
    v8::Handle<v8::Object> fObj = JSFeature::WrapFeature(_isolate, const_cast<Feature*>(feature));
    if (!fObj.IsEmpty())
      globalContext->Global()->Set(v8::String::New("feature"), fObj);
  }

  if (context)
  {
    v8::Handle<v8::Object> cObj = JSFilterContext::WrapFilterContext(_isolate, const_cast<FilterContext*>(context));
    if (!cObj.IsEmpty())
      globalContext->Global()->Set(v8::String::New("context"), cObj);
  }

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
  v8::Locker locker(_isolate);
  v8::Isolate::Scope isolate_scope(_isolate);

  v8::HandleScope handle_scope(_isolate);

  v8::Local<v8::Context> globalContext = *reinterpret_cast<v8::Local<v8::Context>*>(&_globalContext);

  v8::Context::Scope context_scope(globalContext);

  // Attempt to fetch the function from the global object.
  v8::Handle<v8::String> func_name = v8::String::New(function.c_str(), function.length());
  v8::Handle<v8::Value> func_val = globalContext->Global()->Get(func_name);

  // If there is no function, or if it is not a function, bail out
  if (!func_val->IsFunction())
    return ScriptResult(EMPTY_STRING, false, "Function not found in script.");

  v8::Handle<v8::Function> func_func = v8::Handle<v8::Function>::Cast(func_val);

  if (feature)
  {
    v8::Handle<v8::Object> fObj = JSFeature::WrapFeature(_isolate, const_cast<Feature*>(feature));
    if (!fObj.IsEmpty())
      globalContext->Global()->Set(v8::String::New("feature"), fObj);
  }

  if (context)
  {
    v8::Handle<v8::Object> cObj = JSFilterContext::WrapFilterContext(_isolate, const_cast<FilterContext*>(context));
    if (!cObj.IsEmpty())
      globalContext->Global()->Set(v8::String::New("context"), cObj);
  }

  // Set up an exception handler before calling the Eval function
  v8::TryCatch try_catch;

  // Invoke the specifed function
  //const int argc = 1;
  //v8::Handle<v8::Value> argv[argc] = { fObj };
  //v8::Handle<v8::Value> result = func_func->Call(_globalContext->Global(), argc, argv);
  v8::Handle<v8::Value> result = func_func->Call(globalContext->Global(), 0, NULL);

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

//void
//JavascriptEngineV8::constructFeatureCallback(const v8::FunctionCallbackInfo<v8::Value> &info)
//{
//  if (!info.IsConstructCall()) 
//    return v8::ThrowException(v8::String::New("Cannot call constructor as function"));
// 
//	v8::HandleScope handle_scope(_isolate);
// 
//  Feature* feature;
//  if (info.Length() == 0)
//  {
//    feature = new Feature();
//  }
//  else if (info.Length() == 1 && info[0]->IsUint32())
//  {
//    feature = new Feature(info[0]->Uint32Value());
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

void
JavascriptEngineV8::constructBoundsCallback(const v8::FunctionCallbackInfo<v8::Value> &info)
{
  if (!info.IsConstructCall())
  {
    v8::ThrowException(v8::String::New("Cannot call constructor as function"));
    return;
  }
 
	v8::HandleScope handle_scope(v8::Isolate::GetCurrent());
 
  osgEarth::Bounds* bounds;
  //if (info.Length() == 0)
  //  bounds = new osgEarth::Bounds();
  //else
  if (info.Length() == 4)
    bounds = new osgEarth::Bounds(info[0]->NumberValue(), info[1]->NumberValue(), info[2]->NumberValue(), info[3]->NumberValue());

  if (!bounds)
  {
    v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
    return;
  }

  info.GetReturnValue().Set(JSBounds::WrapBounds(v8::Isolate::GetCurrent(), bounds, true));
}

void
JavascriptEngineV8::constructVec3dCallback(const v8::FunctionCallbackInfo<v8::Value> &info)
{
  if (!info.IsConstructCall()) 
  {
    v8::ThrowException(v8::String::New("Cannot call constructor as function"));
    return;
  }
 
  v8::HandleScope handle_scope(v8::Isolate::GetCurrent());

  osg::Vec3d* vec;
  if (info.Length() == 0)
    vec = new osg::Vec3d();
  else if (info.Length() == 1 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* rhs = V8Util::UnwrapObject<osg::Vec3d>(obj);
      vec = new osg::Vec3d(*rhs);
    }
  }
  else if (info.Length() == 3)
    vec = new osg::Vec3d(info[0]->NumberValue(), info[1]->NumberValue(), info[2]->NumberValue());

  if (!vec)
  {
    v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
    return;
  }
  
  info.GetReturnValue().Set(JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), vec, true));

}

void
JavascriptEngineV8::constructGeoExtentCallback(const v8::FunctionCallbackInfo<v8::Value> &info)
{
  if (!info.IsConstructCall())
  {
    v8::ThrowException(v8::String::New("Cannot call constructor as function"));
    return;
  }

  v8::HandleScope handle_scope(v8::Isolate::GetCurrent());
  osgEarth::GeoExtent* extent = 0L;// = new osgEarth::GeoExtent(

  //if (info.Length() == 0)
  //  extent = new osgEarth::GeoExtent();
  //else
  if (info.Length() == 1 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

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
  else if (info.Length() == 2 && info[0]->IsObject() && info[1]->IsObject())
  {
    v8::Local<v8::Object> obj0( v8::Local<v8::Object>::Cast(info[0]) );
    v8::Local<v8::Object> obj1( v8::Local<v8::Object>::Cast(info[1]) );

    if (V8Util::CheckObjectType(obj0, JSSpatialReference::GetObjectType()) && V8Util::CheckObjectType(obj1, JSBounds::GetObjectType()))
    {
      osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj0);
      osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(obj1);
      extent = new osgEarth::GeoExtent(srs, *bounds);
    }
  }
  else if (info.Length() == 5 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
    {
      osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
      extent = new osgEarth::GeoExtent(srs, info[1]->NumberValue(), info[2]->NumberValue(), info[3]->NumberValue(), info[4]->NumberValue());
    }
  }

  if (!extent)
  {
    v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
    return;
  }

  info.GetReturnValue().Set(JSGeoExtent::WrapGeoExtent(v8::Isolate::GetCurrent(), extent, true));
}

void
JavascriptEngineV8::constructSpatialReferenceCallback(const v8::FunctionCallbackInfo<v8::Value> &info)
{
  if (!info.IsConstructCall())
  {
    v8::ThrowException(v8::String::New("Cannot call constructor as function"));
    return;
  }

	v8::HandleScope handle_scope(v8::Isolate::GetCurrent());
 
  osgEarth::SpatialReference* srs;
  if (info.Length() == 1 && info[0]->IsString())
  {
    v8::String::Utf8Value utf8_value(info[0]->ToString());
    std::string init(*utf8_value);
    srs = osgEarth::SpatialReference::create(init);
  }

  if (!srs)
  {
    v8::ThrowException(v8::String::New("Unsupported arguments in constructor call"));
    return;
  }

  info.GetReturnValue().Set(JSSpatialReference::WrapSpatialReference(v8::Isolate::GetCurrent(), srs, true));
}

//void
//JavascriptEngineV8::constructSymbologyGeometryCallback(const v8::FunctionCallbackInfo<v8::Value> &info)
//{
//	v8::HandleScope handle_scope(_isolate);
// 
//  osgEarth::Symbology::Geometry* geom;
//  if (info.Length() == 2)
//    geom = new osgEarth::Symbology::Geometry::create(
//
//  if (geom)
//    info.GetReturnValue().Set(JSBounds::WrapBounds(bounds, true));
//
//  //return v8::ThrowException(v8::String::New("Unsupported arguments"));
//}