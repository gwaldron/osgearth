//
//  JSWrappers.cpp
//  OSGEARTH
//
//  Created by Jeff Smith on 5/22/13.
//
//

#include "JSWrappers"
#include <osgEarthFeatures/Feature>
#include <osgEarth/Notify>


char* JSUtils::JSStringRef_to_CharArray(JSStringRef jsString)
{
    char* buf = 0L;
    int len = JSStringGetLength(jsString) + 1;
    buf = (char*) malloc(len*sizeof(char));
    JSStringGetUTF8CString(jsString, buf, len);
    
    return buf;
}

// --------------------------------------------------------------------------
// JSFeature

static void JSFeature_initialize(JSContextRef ctx, JSObjectRef object)
{
    //NOP
}

static void JSFeature_finalize(JSObjectRef object)
{
    //NOP
}

static JSValueRef JSFeature_getProperty(JSContextRef ctx, JSObjectRef object, JSStringRef propertyName, JSValueRef* exception)
{
    osgEarth::Features::Feature *feature = static_cast<osgEarth::Features::Feature*>(JSObjectGetPrivate(object));

    char* attrBuf = JSUtils::JSStringRef_to_CharArray(propertyName);
    if (attrBuf)
    {
        std::string attr(attrBuf);
        delete [] attrBuf;
        
        if (attr == "attributes" || attr == "attrs")
        {
            return object;
        }
        
        osgEarth::Features::AttributeTable::const_iterator it = feature->getAttrs().find(attr);
        if (it != feature->getAttrs().end())
        {
            osgEarth::Features::AttributeType atype = (*it).second.first;
            switch (atype)
            {
                case osgEarth::Features::ATTRTYPE_BOOL:
                    return JSValueMakeBoolean(ctx, (*it).second.getBool());
                case osgEarth::Features::ATTRTYPE_DOUBLE:
                    return JSValueMakeNumber(ctx, (*it).second.getDouble());
                case osgEarth::Features::ATTRTYPE_INT:
                    return JSValueMakeNumber(ctx, (*it).second.getInt());
                default:
                    return JSValueMakeString(ctx, JSStringCreateWithUTF8CString((*it).second.getString().c_str()));
            }
        }
    }

    return JSValueMakeNull(ctx);
}

//Caches and returns the JSShape class.
JSClassRef JSFeature_class(JSContextRef ctx)
{
    static JSClassRef jsClass;
    if (!jsClass) {
        JSClassDefinition classDefinition = kJSClassDefinitionEmpty;
        classDefinition.initialize = JSFeature_initialize;
        classDefinition.finalize = JSFeature_finalize;
        classDefinition.getProperty = JSFeature_getProperty;
        
        jsClass = JSClassCreate(&classDefinition);
    }
    
    return jsClass;
}