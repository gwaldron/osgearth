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

#include "JavaScriptCoreEngine"
#include "JSWrappers"

#include <osgEarthFeatures/Script>
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarth/Notify>
#include <osgEarth/StringUtils>

#include <JavaScriptCore/JavaScriptCore.h>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[JavaScriptCoreEngine] "


//----------------------------------------------------------------------------

JavaScriptCoreEngine::JavaScriptCoreEngine(const ScriptEngineOptions& options)
: ScriptEngine(options)
{
    // Create JavaScript execution context.
    _ctx = JSGlobalContextCreate(NULL);
    
  if (options.script().isSet() && !options.script()->getCode().empty())
  {
    // Compile and run the script
    ScriptResult result = executeScript(options.script()->getCode());
    if (!result.success())
      OE_WARN << LC << "Error reading javascript: " << result.message() << std::endl;
  }
}

JavaScriptCoreEngine::~JavaScriptCoreEngine()
{
    // Release JavaScript execution context.
    JSGlobalContextRelease(_ctx);
}

ScriptResult
JavaScriptCoreEngine::executeScript(const std::string& script)
 {   
    // Evaluate script.
    JSStringRef scriptJS = JSStringCreateWithUTF8CString(script.c_str());
    JSValueRef result = JSEvaluateScript(_ctx, scriptJS, NULL, NULL, 0, NULL);
    JSStringRelease(scriptJS);
    
    // Convert result to string, unless result is NULL.
    char* buf = 0L;
    if (result) {
        JSStringRef resultStringJS = JSValueToStringCopy(_ctx, result, NULL);
        buf = JSUtils::JSStringRef_to_CharArray(resultStringJS);
        JSStringRelease(resultStringJS);
    }
    
    if (buf)
    {
        std::string resultStr(buf);
        delete [] buf;

        return ScriptResult(resultStr);
    }

  return ScriptResult("");
}

ScriptResult
JavaScriptCoreEngine::run(Script* script, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
  if (!script)
    return ScriptResult(EMPTY_STRING, false, "Script is null.");

  return run(script->getCode(), feature, context);
}

ScriptResult
JavaScriptCoreEngine::run(const std::string& code, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
  if (code.empty())
    return ScriptResult(EMPTY_STRING, false, "Script is empty.");


  if (feature)
  {
      JSStringRef featureStr = JSStringCreateWithUTF8CString("feature");
      JSObjectRef jsFeature = JSObjectMake(_ctx, JSFeature_class(_ctx), const_cast<osgEarth::Features::Feature*>(feature));
      JSObjectSetProperty(_ctx, JSContextGetGlobalObject(_ctx), featureStr, jsFeature, kJSPropertyAttributeNone, NULL);
  }

    
  //TODO: Wrap FilterContext and set as global property
  //if (context)
  //{
  //}

    
  // Compile and run the script
  ScriptResult result = executeScript(code);

  return result;
}

ScriptResult
JavaScriptCoreEngine::call(const std::string& function, osgEarth::Features::Feature const* feature, osgEarth::Features::FilterContext const* context)
{
    return ScriptResult("");
}
