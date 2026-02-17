/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "QuickJSNGEngine"
#include <osgEarth/StringUtils>

#undef  LC
#define LC "[JavaScript] "

using namespace osgEarth;
using namespace osgEarth::Drivers::QJS;

//............................................................................

namespace
{
    static JSValue log(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv)
    {
        if (argc < 1)
            return JS_EXCEPTION;

        const char* str = JS_ToCString(ctx, argv[0]);
        if (!str)
            return JS_EXCEPTION;

        OE_NOTICE << "[JS] " << str << std::endl;

        JS_FreeCString(ctx, str);
        return JS_UNDEFINED;
    }

    static void saveFeature()
    {
        // TODO
    }
}

//............................................................................

namespace
{

    // Create a "feature" object in the global namespace.
    void setFeature(JSContext* c, JSValue& global, const Feature* feature)
    {
        if (!feature)
            return;

        auto obj = JS_NewObject(c);
        JS_SetPropertyStr(c, obj, "id", JS_NewString(c, std::to_string(feature->getFID()).c_str()));

        auto properties = JS_NewObject(c);
        JS_SetPropertyStr(c, obj, "properties", properties);

        auto& attrs = feature->getAttrs();
        for (auto& a : attrs)
        {
            auto type = a.second.getType();
            auto value =
                type == ATTRTYPE_DOUBLE ? JS_NewFloat64(c, a.second.getDouble()) :
                type == ATTRTYPE_INT ? JS_NewInt32(c, a.second.getInt()) :
                type == ATTRTYPE_BOOL ? JS_NewBool(c, a.second.getBool()) :
                JS_NewString(c, a.second.getString().c_str());

            if (a.first[0] == '.')
                JS_SetPropertyStr(c, obj, a.first.substr(1).c_str(), value);
            else
                JS_SetPropertyStr(c, properties, a.first.c_str(), value);
        }

        JS_SetPropertyStr(c, obj, "type", JS_NewString(c, Geometry::toString(feature->getGeometry()->getComponentType()).c_str()));

        JS_SetPropertyStr(c, global, "feature", obj);
    }

}

//............................................................................

void
QuickJSNGEngine::Context::initialize(const ScriptEngineOptions& options)
{
    if (_jscontext == nullptr)
    {
        // new heap + context.
        _jsruntime = JS_NewRuntime();
        _jscontext = JS_NewContext(_jsruntime);

        // Compile and evaluate any static module:
        if (options.script().isSet())
        {
            auto v = JS_Eval(_jscontext,
                options.script()->getCode().c_str(),
                options.script()->getCode().size(),
                "<static>",
                JS_EVAL_TYPE_GLOBAL);

            if (JS_IsException(v))
            {
                JSValue ex = JS_GetException(_jscontext);
                const char* msg = JS_ToCString(_jscontext, ex);
                OE_WARN << LC << "Error evaluating static script module: " << msg << std::endl;
                JS_FreeCString(_jscontext, msg);
                JS_FreeValue(_jscontext, ex);
            }
        }

        // Log function:
        auto global = JS_GetGlobalObject(_jscontext);

        JS_SetPropertyStr(_jscontext, global, "log", JS_NewCFunction(_jscontext, log, "log", 1));
    }
}

QuickJSNGEngine::Context::~Context()
{
    if ( _jscontext )
        JS_FreeContext(_jscontext);

    if (_jsruntime)
        JS_FreeRuntime(_jsruntime);
}

bool
QuickJSNGEngine::Context::compile(const std::string& code)
{
    // precompile the bytecode if it's new
    if (code != _bytecodeSource)
    {
        if (JS_IsFunction(_jscontext, _bytecode))
            JS_FreeValue(_jscontext, _bytecode);

        _bytecode = JS_Eval(_jscontext, code.c_str(), code.size(), "<input>",
            JS_EVAL_TYPE_GLOBAL | JS_EVAL_FLAG_COMPILE_ONLY);

        if (JS_IsException(_bytecode))
        {
            JSValue ex = JS_GetException(_jscontext);
            const char* msg = JS_ToCString(_jscontext, ex);
            OE_WARN << LC << "Compile error: " << msg << std::endl;
            JS_FreeCString(_jscontext, msg);
            JS_FreeValue(_jscontext, ex);
            _errorCount++;
            return false;
        }

        _bytecodeSource = code;
        _errorCount = 0u;
    }

    if (_errorCount != 0u)
    {
        // this code caused a previous compile error, so bail out.
        return false;
    }

    return true;
}

//............................................................................

QuickJSNGEngine::QuickJSNGEngine(const ScriptEngineOptions& options) :
    ScriptEngine(options),
    _options(options)
{
    //nop
}

bool
QuickJSNGEngine::run(const std::string& code, const FeatureList& features, std::vector<ScriptResult>& results, FilterContext const* context)
{
    if (code.empty())
    {
        for (auto& f : features)
            results.emplace_back(EMPTY_STRING, false, "Script is empty.");
        return false;
    }

    // cache the Context on a per-thread basis
    Context& c = _contexts.get();
    c.initialize(_options);

    ScriptResult result;

    if (!c.compile(code))
    {
        //for (auto& f : features)
        //    results.push_back(result);
        return false;
    }

    auto global = JS_GetGlobalObject(c._jscontext); // global context

    for (auto& feature : features)
    {
        // Load the next feature into the global object:
        setFeature(c._jscontext, global, feature.get());

        // run the pre-compiled script:
        auto bytecode = JS_DupValue(c._jscontext, c._bytecode);
        auto r = JS_EvalFunction(c._jscontext, bytecode);

        if (!JS_IsException(r))
        {
            auto str = JS_ToCString(c._jscontext, r);
            results.emplace_back(str, true);
            JS_FreeCString(c._jscontext, str);
        }
        else
        {
            JSValue ex = JS_GetException(c._jscontext);
            const char* msg = JS_ToCString(c._jscontext, ex);
            OE_WARN << LC << "Runtime error: " << msg << std::endl;
            JS_FreeCString(c._jscontext, msg);
            JS_FreeValue(c._jscontext, ex);
        }

        JS_FreeValue(c._jscontext, r); // unref
    }

    JS_FreeValue(c._jscontext, global); // unref

    return true;
}

ScriptResult
QuickJSNGEngine::run(const std::string& code, Feature* feature, FilterContext const* context)
{
    if (code.empty())
        return ScriptResult(EMPTY_STRING, false, "Feature is empty");

    // cache the Context on a per-thread basis
    Context& c = _contexts.get();
    c.initialize(_options);

    if (!c.compile(code))
    {
        return ScriptResult(EMPTY_STRING, false, "Compile error");
    }

    auto global = JS_GetGlobalObject(c._jscontext); // global context

    // Load the next feature into the global object:
    setFeature(c._jscontext, global, feature);

    // run the pre-compiled script:
    auto bytecode = JS_DupValue(c._jscontext, c._bytecode);
    auto r = JS_EvalFunction(c._jscontext, bytecode);

    ScriptResult result;

    if (!JS_IsException(r))
    {
        auto str = JS_ToCString(c._jscontext, r);
        result = ScriptResult(str, true);
        JS_FreeCString(c._jscontext, str);
    }
    else
    {
        JSValue ex = JS_GetException(c._jscontext);
        const char* msg = JS_ToCString(c._jscontext, ex);
        OE_WARN << LC << "Runtime error: " << msg << std::endl;
        result = ScriptResult(EMPTY_STRING, false, msg);
        JS_FreeCString(c._jscontext, msg);
        JS_FreeValue(c._jscontext, ex);
    }

    JS_FreeValue(c._jscontext, r); // unref
    JS_FreeValue(c._jscontext, global); // unref

    return result;
}
