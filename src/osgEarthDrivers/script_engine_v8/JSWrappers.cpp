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

#include <osgEarthDrivers/script_engine_v8/JSWrappers>
#include <osgEarthDrivers/script_engine_v8/V8Util>
#include <osgEarthFeatures/FilterContext>
#include <osgEarth/StringUtils>
#include <v8.h>

using namespace osgEarth::Features;

// ---------------------------------------------------------------------------
const std::string JSFeature::_objectType = "JSFeature";

v8::Handle<v8::Object>
JSFeature::WrapFeature(v8::Isolate* isolate, osgEarth::Features::Feature* feature, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  if (!feature)
  {
    v8::Handle<v8::Object> obj;
    return handle_scope.Close(obj);
  }

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, feature, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(feature, &FreeFeatureCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSFeature::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> feat_instance = v8::ObjectTemplate::New();

  feat_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  feat_instance->SetInternalFieldCount(1);
  feat_instance->SetNamedPropertyHandler(PropertyCallback);

  return handle_scope.Close(feat_instance);
}

v8::Handle<v8::ObjectTemplate>
JSFeature::GetAttributesObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> attr_instance = v8::ObjectTemplate::New();

  attr_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New("JSFeature_Attrs"));
  attr_instance->SetInternalFieldCount(1);
  attr_instance->SetNamedPropertyHandler(AttrPropertyCallback);

  return handle_scope.Close(attr_instance);
}

v8::Handle<v8::Value>
JSFeature::GetFeatureAttr(const std::string& attr, Feature const* feature)
{
  AttributeTable::const_iterator it = feature->getAttrs().find(attr);

  // If the key is not present return an empty handle as signal
  if (it == feature->getAttrs().end())
    return v8::Handle<v8::Value>();

  // Otherwise fetch the value and wrap it in a JavaScript string
  osgEarth::Features::AttributeType atype = (*it).second.first;
  switch (atype)
  {
    case osgEarth::Features::ATTRTYPE_BOOL:
      return v8::Boolean::New((*it).second.second.set ? (*it).second.getBool() : false);
    case osgEarth::Features::ATTRTYPE_DOUBLE:
      if ((*it).second.second.set)
        return v8::Number::New((*it).second.getDouble());
      else
        return v8::Undefined();
    case osgEarth::Features::ATTRTYPE_INT:
      if ((*it).second.second.set)
        return v8::Integer::New((*it).second.getInt());
      else
        return v8::Undefined();
    default:
      std::string val = (*it).second.second.set ? (*it).second.getString() : "";
      return v8::String::New(val.c_str(), val.length());
  }
}

void
JSFeature::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  Feature* feature = V8Util::UnwrapObject<Feature>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!feature || prop.empty())
    return;

  v8::Local<v8::Value> value;
  if (prop == "fid")
    value = v8::Uint32::New(feature->getFID());
  else if (prop == "attrs" || prop == "attributes")
    value = V8Util::WrapObject(v8::Isolate::GetCurrent(), feature, GetAttributesObjectTemplate(v8::Isolate::GetCurrent()));
  else if (prop == "geometry")
    value = JSSymbologyGeometry::WrapGeometry(v8::Isolate::GetCurrent(), feature->getGeometry());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFeature::AttrPropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  Feature* feature = V8Util::UnwrapObject<Feature>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string attr(*utf8_value);

  if (!feature || attr.empty())
    return;

  v8::Local<v8::Value> value = GetFeatureAttr(attr, feature);
  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFeature::FreeFeatureCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::Features::Feature* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSSymbologyGeometry::_objectType = "JSSymbologyGeometry";

v8::Handle<v8::Object>
JSSymbologyGeometry::WrapGeometry(v8::Isolate* isolate, osgEarth::Symbology::Geometry* geometry, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, geometry, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(geometry, &FreeGeometryCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSSymbologyGeometry::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);
  template_instance->SetIndexedPropertyHandler(IndexedPropertyCallback);

  return handle_scope.Close(template_instance);
}

void
JSSymbologyGeometry::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osgEarth::Symbology::Geometry* geom = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!geom || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "totalPointCount")
    value = v8::Integer::New(geom->getTotalPointCount());
  else if (prop == "numComponents")
    value = v8::Uint32::New(geom->getNumComponents());
  else if (prop == "bounds")
  {
    osgEarth::Bounds bounds = geom->getBounds();
    osgEarth::Bounds* newBounds = new osgEarth::Bounds();
    newBounds->set(bounds.xMin(), bounds.yMin(), bounds.zMin(), bounds.xMax(), bounds.yMax(), bounds.zMax());
    value = JSBounds::WrapBounds(v8::Isolate::GetCurrent(), newBounds, true);
  }
  else if (prop == "type")
    value = v8::String::New(osgEarth::Symbology::Geometry::toString(geom->getType()).c_str());
  else if (prop == "componentType")
    value = v8::String::New(osgEarth::Symbology::Geometry::toString(geom->getComponentType()).c_str());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSSymbologyGeometry::IndexedPropertyCallback(uint32_t index, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osgEarth::Symbology::Geometry* geom = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(info.Holder());

  v8::Local<v8::Value> value;
  
  if (geom)
    value = JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), &((*geom)[index]));

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSSymbologyGeometry::FreeGeometryCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::Symbology::Geometry* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSBounds::_objectType = "JSBounds";

v8::Handle<v8::Object>
JSBounds::WrapBounds(v8::Isolate* isolate, osgEarth::Bounds* bounds, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, bounds, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(bounds, &FreeBoundsCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSBounds::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

  template_instance->Set(v8::String::New("contains"), v8::FunctionTemplate::New(ContainsCallback));
  template_instance->Set(v8::String::New("unionWith"), v8::FunctionTemplate::New(UnionCallback));
  template_instance->Set(v8::String::New("intersectionWith"), v8::FunctionTemplate::New(IntersectionCallback));

  return handle_scope.Close(template_instance);
}

void
JSBounds::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!bounds || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "valid")
    value = v8::Boolean::New(bounds->valid());
  else if (prop == "xMin")
    value = v8::Number::New(bounds->xMin());
  else if (prop == "xMax")
    value = v8::Number::New(bounds->xMax());
  else if (prop == "yMin")
    value = v8::Number::New(bounds->yMin());
  else if (prop == "yMax")
    value = v8::Number::New(bounds->yMax());
  else if (prop == "zMin")
    value = v8::Number::New(bounds->zMin());
  else if (prop == "zMax")
    value = v8::Number::New(bounds->zMax());
  else if (prop == "center")
  {
    osg::Vec3d* vec = new osg::Vec3d(bounds->center());
    value = JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), vec, true);
  }
  else if (prop == "radius")
    value = v8::Number::New(bounds->radius());
  else if (prop == "width")
    value = v8::Number::New(bounds->width());
  else if (prop == "height")
    value = v8::Number::New(bounds->height());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSBounds::ContainsCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(info.Holder());

  if (bounds)
  {
    v8::Local<v8::Value> value;

    if (info.Length() == 1 && info[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );
      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* rhs = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        value = v8::Boolean::New(bounds->contains(*rhs));
      }
    }
    else if (info.Length() == 2)
    {
      value = v8::Boolean::New(bounds->contains(info[0]->NumberValue(), info[1]->NumberValue()));
    }

    if (!value.IsEmpty())
      info.GetReturnValue().Set(value);
  }
}

void
JSBounds::UnionCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(info.Holder());

  if (bounds)
  {
    v8::Local<v8::Value> value;

    if (info.Length() == 1 && info[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* rhs = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        osgEarth::Bounds* outBounds = new osgEarth::Bounds();
        outBounds->expandBy(bounds->unionWith(*rhs));
        value = WrapBounds(v8::Isolate::GetCurrent(), outBounds, true);
      }
    }

    if (!value.IsEmpty())
      info.GetReturnValue().Set(value);
  }
}

void
JSBounds::IntersectionCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(info.Holder());

  if (bounds)
  {
    v8::Local<v8::Value> value;

    if (info.Length() == 1 && info[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* rhs = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        osgEarth::Bounds* outBounds = new osgEarth::Bounds();
        outBounds->expandBy(bounds->intersectionWith(*rhs));
        value = WrapBounds(v8::Isolate::GetCurrent(), outBounds, true);
      }
    }

    if (!value.IsEmpty())
      info.GetReturnValue().Set(value);
  }
}

void
JSBounds::FreeBoundsCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::Bounds* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSVec3d::_objectType = "JSVec3d";

v8::Handle<v8::Object>
JSVec3d::WrapVec3d(v8::Isolate* isolate, osg::Vec3d* vec, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, vec, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(vec, &FreeVecCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSVec3d::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);
  template_instance->SetIndexedPropertyHandler(IndexedPropertyCallback);

  return handle_scope.Close(template_instance);
}

void
JSVec3d::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osg::Vec3d* v = V8Util::UnwrapObject<osg::Vec3d>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!v || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "x")
    value = v8::Number::New(v->x());
  if (prop == "y")
    value = v8::Number::New(v->y());
  if (prop == "z")
    value = v8::Number::New(v->z());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSVec3d::IndexedPropertyCallback(uint32_t index, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osg::Vec3d* v = V8Util::UnwrapObject<osg::Vec3d>(info.Holder());

  if (!v || index > 2)
    return;

  info.GetReturnValue().Set(v8::Number::New((*v)[index]));
}

void JSVec3d::FreeVecCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osg::Vec3d* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSFilterContext::_objectType = "JSFilterContext";

v8::Handle<v8::Object>
JSFilterContext::WrapFilterContext(v8::Isolate* isolate, osgEarth::Features::FilterContext* context, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  if (!context)
  {
    v8::Handle<v8::Object> obj;
    return handle_scope.Close(obj);
  }

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, context, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(context, &FreeContextCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSFilterContext::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

#if 0
  template_instance->Set(v8::String::New("toLocal"), v8::FunctionTemplate::New(ToLocalCallback));
  template_instance->Set(v8::String::New("toWorld"), v8::FunctionTemplate::New(ToWorldCallback));
  template_instance->Set(v8::String::New("toMap"), v8::FunctionTemplate::New(ToMapCallback));
  template_instance->Set(v8::String::New("fromMap"), v8::FunctionTemplate::New(FromMapCallback));
#endif

  return handle_scope.Close(template_instance);
}

void
JSFilterContext::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!context || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "session")
    value = JSSession::WrapSession(v8::Isolate::GetCurrent(), const_cast<Session*>(context->getSession()));
  if (prop == "profile")
    value = JSFeatureProfile::WrapFeatureProfile(v8::Isolate::GetCurrent(), const_cast<FeatureProfile*>(context->profile().get()));
  if (prop == "extent" && context->extent().isSet())
    value = JSGeoExtent::WrapGeoExtent(v8::Isolate::GetCurrent(), const_cast<osgEarth::GeoExtent*>(&context->extent().get()));
  //if (prop == "geocentric")
  //  value = v8::Boolean::New(context->isGeocentric());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFilterContext::ToLocalCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(info.Holder());

  v8::Local<v8::Value> value;

  if (context && info.Length() == 1 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    /*if (V8Util::CheckObjectType(obj, JSGeometry::GetObjectType()))  // Geometry
    {
      osgEarth::Symbology::Geometry* geometry = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(obj);

      return 
    }*/
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* local = new osg::Vec3d(context->toLocal(*vec));
      value = JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), local, true);
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFilterContext::ToWorldCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(info.Holder());

  v8::Local<v8::Value> value;

  if (context && info.Length() == 1 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    /*if (V8Util::CheckObjectType(obj, JSGeometry::GetObjectType()))  // Geometry
    {
      osgEarth::Symbology::Geometry* geometry = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(obj);

      return 
    }*/
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* world = new osg::Vec3d(context->toWorld(*vec));
      value = JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), world, true);
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFilterContext::ToMapCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(info.Holder());

  v8::Local<v8::Value> value;

  if (context && info.Length() == 1 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* map = new osg::Vec3d(context->toMap(*vec));
      value = JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), map, true);
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFilterContext::FromMapCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(info.Holder());

  v8::Local<v8::Value> value;

  if (context && info.Length() == 1 && info[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* map = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* local = new osg::Vec3d(context->fromMap(*map));
      value = JSVec3d::WrapVec3d(v8::Isolate::GetCurrent(), local, true);
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFilterContext::FreeContextCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::Features::FilterContext* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSSession::_objectType = "JSSession";

v8::Handle<v8::Object>
JSSession::WrapSession(v8::Isolate* isolate, osgEarth::Features::Session* session, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, session, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(session, &FreeSessionCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSSession::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

#if 0
  template_instance->Set(v8::String::New("resolveURI"), v8::FunctionTemplate::New(ResolveUriCallback));
#endif

  return handle_scope.Close(template_instance);
}

void
JSSession::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  Session* session = V8Util::UnwrapObject<Session>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!session || prop.empty())
    return;

  if (prop == "mapInfo")
    info.GetReturnValue().Set(JSMapInfo::WrapMapInfo(v8::Isolate::GetCurrent(), const_cast<osgEarth::MapInfo*>(&session->getMapInfo())));
}

#if 0
void
JSSession::ResolveUriCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  Session* session = V8Util::UnwrapObject<Session>(info.Holder());

  v8::Local<v8::Value> value;

  if (session && info.Length() == 1 && info[0]->IsString())
  {
    v8::String::Utf8Value utf8_value(info[0]->ToString());
    std::string uri(*utf8_value);
    value = v8::String::New(session->resolveURI(uri).c_str());
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}
#endif

void
JSSession::FreeSessionCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::Features::Session* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSMapInfo::_objectType = "JSMapInfo";

v8::Handle<v8::Object>
JSMapInfo::WrapMapInfo(v8::Isolate* isolate, osgEarth::MapInfo* mapInfo, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, mapInfo, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(mapInfo, &FreeMapInfoCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSMapInfo::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

#if 0
  template_instance->Set(v8::String::New("toMapPoint"), v8::FunctionTemplate::New(ToMapCallback));
  template_instance->Set(v8::String::New("mapPointToWorldPoint"), v8::FunctionTemplate::New(MapToWorldCallback));
  template_instance->Set(v8::String::New("worldPointToMapPoint"), v8::FunctionTemplate::New(WorldToMapCallback));
#endif

  return handle_scope.Close(template_instance);
}

void
JSMapInfo::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!mapInfo || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "geocentric")
    value = v8::Boolean::New(mapInfo->isGeocentric());
  if (prop == "cube")
    value = v8::Boolean::New(mapInfo->isCube());
  if (prop == "plateCarre" || prop == "platecarre")
    value = v8::Boolean::New(mapInfo->isPlateCarre());
  if (prop == "projectedSRS")
    value = v8::Boolean::New(mapInfo->isProjectedSRS());
  if (prop == "geographicSRS")
    value = v8::Boolean::New(mapInfo->isGeographicSRS());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

#if 0
void
JSMapInfo::ToMapCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(info.Holder());

  v8::Local<v8::Value> value;

  if (mapInfo && info.Length() == 2 && info[0]->IsObject() && info[1]->IsObject()) // Vec3d & SpatialReference
  {
    v8::Local<v8::Object> obj0( v8::Object::Cast(*info[0]) );
    v8::Local<v8::Object> obj1( v8::Object::Cast(*info[1]) );

    if (V8Util::CheckObjectType(obj0, JSVec3d::GetObjectType()) && V8Util::CheckObjectType(obj1, JSSpatialReference::GetObjectType()))
    {
      osg::Vec3d* input = V8Util::UnwrapObject<osg::Vec3d>(obj0);
      osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj1);

      osg::Vec3d* out = new osg::Vec3d();

      GeoPoint mapPoint;
      mapPoint.fromWorld( srs, *input );
      out->set( mapPoint.x(), mapPoint.y(), mapPoint.z() );

      value = JSVec3d::WrapVec3d(out, true);

      //if (mapInfo->toMapPoint(*input, srs, *out))
      //  value = JSVec3d::WrapVec3d(out, true);

      delete out;
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSMapInfo::MapToWorldCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(info.Holder());
  
  v8::Local<v8::Value> value;

  if (mapInfo && info.Length() == 1 && info[0]->IsObject()) // Vec3d
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* input = V8Util::UnwrapObject<osg::Vec3d>(obj);

      osg::Vec3d* out = new osg::Vec3d();
      if (mapInfo->mapPointToWorldPoint(*input, *out))
        value = JSVec3d::WrapVec3d(out, true);

      delete out;
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSMapInfo::WorldToMapCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(info.Holder());

  v8::Local<v8::Value> value;

  if (mapInfo && info.Length() == 1 && info[0]->IsObject()) // Vec3d
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* input = V8Util::UnwrapObject<osg::Vec3d>(obj);

      osg::Vec3d* out = new osg::Vec3d();
      if (mapInfo->worldPointToMapPoint(*input, *out))
        value = JSVec3d::WrapVec3d(out, true);

      delete out;
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}
#endif

void
JSMapInfo::FreeMapInfoCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::MapInfo* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSFeatureProfile::_objectType = "JSFeatureProfile";

v8::Handle<v8::Object>
JSFeatureProfile::WrapFeatureProfile(v8::Isolate* isolate, osgEarth::Features::FeatureProfile* profile, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, profile, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(profile, &FreeProfileCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSFeatureProfile::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

  //template_instance->Set(v8::String::New("toLocal"),

  return handle_scope.Close(template_instance);
}

void
JSFeatureProfile::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  FeatureProfile* profile = V8Util::UnwrapObject<FeatureProfile>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!profile || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "extent")
    value = JSGeoExtent::WrapGeoExtent(v8::Isolate::GetCurrent(), const_cast<osgEarth::GeoExtent*>(&profile->getExtent()));
  if (prop == "srs")
    value = JSSpatialReference::WrapSpatialReference(v8::Isolate::GetCurrent(), const_cast<osgEarth::SpatialReference*>(profile->getSRS()));

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSFeatureProfile::FreeProfileCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::Features::FeatureProfile* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSGeoExtent::_objectType = "JSGeoExtent";

v8::Handle<v8::Object>
JSGeoExtent::WrapGeoExtent(v8::Isolate* isolate, osgEarth::GeoExtent* extent, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, extent, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(extent, &FreeGeoExtentCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSGeoExtent::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

  template_instance->Set(v8::String::New("contains"), v8::FunctionTemplate::New(ContainsCallback));
  template_instance->Set(v8::String::New("intersects"), v8::FunctionTemplate::New(IntersectsCallback));

  return handle_scope.Close(template_instance);
}

void
JSGeoExtent::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osgEarth::GeoExtent* extent = V8Util::UnwrapObject<osgEarth::GeoExtent>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!extent || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "srs")
    value = JSSpatialReference::WrapSpatialReference(v8::Isolate::GetCurrent(), const_cast<osgEarth::SpatialReference*>(extent->getSRS()));
  if (prop == "xMin")
    value = v8::Number::New(extent->xMin());
  if (prop == "xMax")
    value = v8::Number::New(extent->xMax());
  if (prop == "yMin")
    value = v8::Number::New(extent->yMin());
  if (prop == "yMax")
    value = v8::Number::New(extent->yMax());
  if (prop == "width")
    value = v8::Number::New(extent->width());
  if (prop == "height")
    value = v8::Number::New(extent->height());
  if (prop == "crossesAntimeridian")
    value = v8::Boolean::New(extent->crossesAntimeridian());
  if (prop == "valid")
    value = v8::Boolean::New(extent->isValid());
  if (prop == "defined")
    value = v8::Boolean::New(extent->defined());
  if (prop == "area")
    value = v8::Number::New(extent->area());
  //if (prop == "toString")
  //  value = v8::String::New(extent->toString().c_str());

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSGeoExtent::ContainsCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::GeoExtent* extent = V8Util::UnwrapObject<osgEarth::GeoExtent>(info.Holder());
  v8::Local<v8::Value> value;

  if (extent)
  {
    if (info.Length() == 1 && info[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        value = v8::Boolean::New(extent->contains(*bounds));
      }
    }
    else if (info.Length() == 2 /*&& info[0]->IsNumber() && info[1]->IsNumber()*/)  // x and y
    {
      value = v8::Boolean::New(extent->contains(info[0]->NumberValue(), info[1]->NumberValue()));
    }
    else if (info.Length() == 3 && /*info[0]->IsNumber() && info[1]->IsNumber() &&*/ info[2]->IsObject())  // x, y, and SpatialReference
    {
      v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[2]) );

      if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
      {
        osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
        value = v8::Boolean::New(extent->contains(info[0]->NumberValue(), info[1]->NumberValue(), srs));
      }
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSGeoExtent::IntersectsCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::GeoExtent* extent = V8Util::UnwrapObject<osgEarth::GeoExtent>(info.Holder());

  if (!extent)
    return;

  v8::Local<v8::Value> value;

  if (info.Length() == 1 && info[0]->IsObject())  // GeoExtent
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    if (V8Util::CheckObjectType(obj, JSGeoExtent::GetObjectType()))
    {
      osgEarth::GeoExtent* rhs = V8Util::UnwrapObject<osgEarth::GeoExtent>(obj);
      value = v8::Boolean::New(extent->intersects(*rhs));
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSGeoExtent::FreeGeoExtentCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::GeoExtent* parameter)
{
  delete parameter;
  handle->Dispose();
}

// ---------------------------------------------------------------------------

const std::string JSSpatialReference::_objectType = "JSSpatialReference";

v8::Handle<v8::Object>
JSSpatialReference::WrapSpatialReference(v8::Isolate* isolate, osgEarth::SpatialReference* srs, bool freeObject)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::Object> obj = V8Util::WrapObject(isolate, srs, GetObjectTemplate(isolate));

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef;
    weakRef.Reset(v8::Isolate::GetCurrent(), obj);
    weakRef.MakeWeak(srs, &FreeSpatialReferenceCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSSpatialReference::GetObjectTemplate(v8::Isolate* isolate)
{
  v8::HandleScope handle_scope(isolate);

  v8::Handle<v8::ObjectTemplate> template_instance = v8::ObjectTemplate::New();

  template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
  template_instance->SetInternalFieldCount(1);
  template_instance->SetNamedPropertyHandler(PropertyCallback);

  template_instance->Set(v8::String::New("isEquivalentTo"), v8::FunctionTemplate::New(EquivalenceCallback));
  template_instance->Set(v8::String::New("createTangentPlaneSRS"), v8::FunctionTemplate::New(TangentPlaneCallback));
  //template_instance->Set(v8::String::New("createTransMercFromLongitude"), v8::FunctionTemplate::New(equivalenceCallback));
  //template_instance->Set(v8::String::New("createUTMFromLongitude"), v8::FunctionTemplate::New(equivalenceCallback));

  return handle_scope.Close(template_instance);
}

void
JSSpatialReference::PropertyCallback(v8::Local<v8::String> name, const v8::PropertyCallbackInfo<v8::Value>& info)
{
  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);
  
  if (!srs || prop.empty())
    return;

  v8::Local<v8::Value> value;

  if (prop == "geographic")
    value = v8::Boolean::New(srs->isGeographic());
  if (prop == "projected")
    value = v8::Boolean::New(srs->isProjected());
  if (prop == "mercator")
    value = v8::Boolean::New(srs->isMercator());
  if (prop == "sphericalMercator")
    value = v8::Boolean::New(srs->isSphericalMercator());
  if (prop == "northPolar")
    value = v8::Boolean::New(srs->isNorthPolar());
  if (prop == "southPolar")
    value = v8::Boolean::New(srs->isSouthPolar());
  if (prop == "userDefined")
    value = v8::Boolean::New(srs->isUserDefined());
  if (prop == "contiguous")
    value = v8::Boolean::New(srs->isContiguous());
  if (prop == "cube")
    value = v8::Boolean::New(srs->isCube());
  if (prop == "LTP" || prop == "ltp")
    value = v8::Boolean::New(srs->isLTP());
  if (prop == "name")
    value = v8::String::New(srs->getName().c_str());
  if (prop == "WKT" || prop == "wkt")
    value = v8::String::New(srs->getWKT().c_str());
  if (prop == "initType")
    value = v8::String::New(srs->getInitType().c_str());
  if (prop == "horizInitString")
    value = v8::String::New(srs->getHorizInitString().c_str());
  if (prop == "vertInitString")
    value = v8::String::New(srs->getVertInitString().c_str());
  if (prop == "datumName")
    value = v8::String::New(srs->getDatumName().c_str());
  if (prop == "geographicSRS")
    value = JSSpatialReference::WrapSpatialReference(v8::Isolate::GetCurrent(), const_cast<osgEarth::SpatialReference*>(srs->getGeographicSRS()));

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSSpatialReference::EquivalenceCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(info.Holder());

  if (!srs)
    return;

  v8::Local<v8::Value> value;

  if (info.Length() == 1 && info[0]->IsObject())  // SpatialReference
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
    {
      osgEarth::SpatialReference* rhs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
      value = v8::Boolean::New(srs->isEquivalentTo(rhs));
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSSpatialReference::TangentPlaneCallback(const v8::FunctionCallbackInfo<v8::Value>& info)
{
  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(info.Holder());

  if (!srs)
    return;

  v8::Local<v8::Value> value;

  if (info.Length() == 1 && info[0]->IsObject())  // Vec3d
  {
    v8::Local<v8::Object> obj( v8::Local<v8::Object>::Cast(info[0]) );

    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      value = JSSpatialReference::WrapSpatialReference(v8::Isolate::GetCurrent(), const_cast<SpatialReference*>(srs->createTangentPlaneSRS(*vec)), true);
    }
  }

  if (!value.IsEmpty())
    info.GetReturnValue().Set(value);
}

void
JSSpatialReference::FreeSpatialReferenceCallback(v8::Isolate* isolate, v8::Persistent<v8::Object>* handle, osgEarth::SpatialReference* parameter)
{
  osg::ref_ptr<osgEarth::SpatialReference> srs = parameter;
  handle->Dispose();
}