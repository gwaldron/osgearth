/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
JSFeature::WrapFeature(osgEarth::Features::Feature* feature, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(feature, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(feature, FreeFeatureCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSFeature::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> feat_instance;

  if (feat_instance.IsEmpty())
  {
    feat_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    feat_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    feat_instance->SetInternalFieldCount(1);
    feat_instance->SetNamedPropertyHandler(PropertyCallback);
  }

  return feat_instance;
}

v8::Handle<v8::ObjectTemplate>
JSFeature::GetAttributesObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> attr_instance;

  if (attr_instance.IsEmpty())
  {
    attr_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    attr_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New("JSFeature_Attrs"));
    attr_instance->SetInternalFieldCount(1);
    attr_instance->SetNamedPropertyHandler(AttrPropertyCallback);
  }

  return attr_instance;
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
      return v8::Boolean::New((*it).second.getBool());
    case osgEarth::Features::ATTRTYPE_DOUBLE:
      return v8::Number::New((*it).second.getDouble());
    case osgEarth::Features::ATTRTYPE_INT:
      return v8::Integer::New((*it).second.getInt());
    default:
      std::string val = (*it).second.getString();
      return v8::String::New(val.c_str(), val.length());
  }
}

v8::Handle<v8::Value>
JSFeature::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  Feature* feature = V8Util::UnwrapObject<Feature>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!feature || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "fid")
    return v8::Uint32::New(feature->getFID());
  else if (prop == "attrs" || prop == "attributes")
    return V8Util::WrapObject(feature, GetAttributesObjectTemplate());
  else if (prop == "geometry")
    return JSSymbologyGeometry::WrapGeometry(feature->getGeometry());

  //return GetFeatureAttr(prop, feature);
  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSFeature::AttrPropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  Feature* feature = V8Util::UnwrapObject<Feature>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string attr(*utf8_value);

  if (!feature || attr.empty())
    return v8::Handle<v8::Value>();

  return GetFeatureAttr(attr, feature);
}

void
JSFeature::FreeFeatureCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  Feature* feature = static_cast<Feature*>(parameter);
  delete feature;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSSymbologyGeometry::_objectType = "JSSymbologyGeometry";

v8::Handle<v8::Object>
JSSymbologyGeometry::WrapGeometry(osgEarth::Symbology::Geometry* geometry, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(geometry, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(geometry, FreeGeometryCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSSymbologyGeometry::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);
    template_instance->SetIndexedPropertyHandler(IndexedPropertyCallback);
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSSymbologyGeometry::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  osgEarth::Symbology::Geometry* geom = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!geom || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "totalPointCount")
    return v8::Integer::New(geom->getTotalPointCount());
  else if (prop == "numComponents")
    return v8::Uint32::New(geom->getNumComponents());
  else if (prop == "bounds")
  {
    osgEarth::Bounds bounds = geom->getBounds();
    osgEarth::Bounds* newBounds = new osgEarth::Bounds();
    newBounds->set(bounds.xMin(), bounds.yMin(), bounds.zMin(), bounds.xMax(), bounds.yMax(), bounds.zMax());
    return JSBounds::WrapBounds(newBounds, true);
  }
  else if (prop == "type")
    return v8::String::New(osgEarth::Symbology::Geometry::toString(geom->getType()).c_str());
  else if (prop == "componentType")
    return v8::String::New(osgEarth::Symbology::Geometry::toString(geom->getComponentType()).c_str());

  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSSymbologyGeometry::IndexedPropertyCallback(uint32_t index, const v8::AccessorInfo& info)
{
  osgEarth::Symbology::Geometry* geom = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(info.Holder());

  if (!geom)
    return v8::Handle<v8::Value>();

  return JSVec3d::WrapVec3d(&((*geom)[index]));
}

void
JSSymbologyGeometry::FreeGeometryCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  osgEarth::Symbology::Geometry* geometry = static_cast<osgEarth::Symbology::Geometry*>(parameter);
  delete geometry;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSBounds::_objectType = "JSBounds";

v8::Handle<v8::Object>
JSBounds::WrapBounds(osgEarth::Bounds* bounds, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(bounds, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(bounds, FreeBoundsCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSBounds::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;
  
  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

    template_instance->Set(v8::String::New("contains"), v8::FunctionTemplate::New(ContainsCallback));
    template_instance->Set(v8::String::New("unionWith"), v8::FunctionTemplate::New(UnionCallback));
    template_instance->Set(v8::String::New("intersectionWith"), v8::FunctionTemplate::New(IntersectionCallback));
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSBounds::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!bounds || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "valid")
    return v8::Boolean::New(bounds->valid());
  else if (prop == "xMin")
    return v8::Number::New(bounds->xMin());
  else if (prop == "xMax")
    return v8::Number::New(bounds->xMax());
  else if (prop == "yMin")
    return v8::Number::New(bounds->yMin());
  else if (prop == "yMax")
    return v8::Number::New(bounds->yMax());
  else if (prop == "zMin")
    return v8::Number::New(bounds->zMin());
  else if (prop == "zMax")
    return v8::Number::New(bounds->zMax());
  else if (prop == "center")
  {
    osg::Vec3d* vec = new osg::Vec3d(bounds->center());
    return JSVec3d::WrapVec3d(vec, true);
  }
  else if (prop == "radius")
    return v8::Number::New(bounds->radius());
  else if (prop == "width")
    return v8::Number::New(bounds->width());
  else if (prop == "height")
    return v8::Number::New(bounds->height());

  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSBounds::ContainsCallback(const v8::Arguments& args)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(args.Holder());

  if (bounds)
  {
    if (args.Length() == 1 && args[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* rhs = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        return v8::Boolean::New(bounds->contains(*rhs));
      }
    }
    else if (args.Length() == 2)
    {
      return v8::Boolean::New(bounds->contains(args[0]->NumberValue(), args[1]->NumberValue()));
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSBounds::UnionCallback(const v8::Arguments& args)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(args.Holder());

  if (bounds)
  {
    if (args.Length() == 1 && args[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* rhs = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        osgEarth::Bounds* outBounds = new osgEarth::Bounds();
        outBounds->expandBy(bounds->unionWith(*rhs));
        return WrapBounds(outBounds, true);
      }
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSBounds::IntersectionCallback(const v8::Arguments& args)
{
  osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(args.Holder());

  if (bounds)
  {
    if (args.Length() == 1 && args[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* rhs = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        osgEarth::Bounds* outBounds = new osgEarth::Bounds();
        outBounds->expandBy(bounds->intersectionWith(*rhs));
        return WrapBounds(outBounds, true);
      }
    }
  }

  return v8::Undefined();
}

void
JSBounds::FreeBoundsCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  Bounds* bounds = static_cast<Bounds*>(parameter);
  delete bounds;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSVec3d::_objectType = "JSVec3d";

v8::Handle<v8::Object>
JSVec3d::WrapVec3d(osg::Vec3d* vec, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(vec, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(vec, FreeVecCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSVec3d::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);
    template_instance->SetIndexedPropertyHandler(IndexedPropertyCallback);
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSVec3d::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  osg::Vec3d* v = V8Util::UnwrapObject<osg::Vec3d>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!v || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "x")
    return v8::Number::New(v->x());
  if (prop == "y")
    return v8::Number::New(v->y());
  if (prop == "z")
    return v8::Number::New(v->z());

  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSVec3d::IndexedPropertyCallback(uint32_t index, const v8::AccessorInfo& info)
{
  osg::Vec3d* v = V8Util::UnwrapObject<osg::Vec3d>(info.Holder());

  if (!v || index > 2)
    return v8::Handle<v8::Value>();

  return v8::Number::New((*v)[index]);
}

void
JSVec3d::FreeVecCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  osg::Vec3d* v = static_cast<osg::Vec3d*>(parameter);
  delete v;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSFilterContext::_objectType = "JSFilterContext";

v8::Handle<v8::Object>
JSFilterContext::WrapFilterContext(osgEarth::Features::FilterContext* context, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(context, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(context, FreeContextCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSFilterContext::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

#if 0
    template_instance->Set(v8::String::New("toLocal"), v8::FunctionTemplate::New(ToLocalCallback));
    template_instance->Set(v8::String::New("toWorld"), v8::FunctionTemplate::New(ToWorldCallback));
    template_instance->Set(v8::String::New("toMap"), v8::FunctionTemplate::New(ToMapCallback));
    template_instance->Set(v8::String::New("fromMap"), v8::FunctionTemplate::New(FromMapCallback));
#endif
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSFilterContext::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!context || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "session")
    return JSSession::WrapSession(const_cast<Session*>(context->getSession()));
  if (prop == "profile")
    return JSFeatureProfile::WrapFeatureProfile(const_cast<FeatureProfile*>(context->profile().get()));
  if (prop == "extent" && context->extent().isSet())
    return JSGeoExtent::WrapGeoExtent(const_cast<osgEarth::GeoExtent*>(&context->extent().get()));
  //if (prop == "geocentric")
  //  return v8::Boolean::New(context->isGeocentric());

  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSFilterContext::ToLocalCallback(const v8::Arguments& args)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(args.Holder());

  if (context && args.Length() == 1 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    /*if (V8Util::CheckObjectType(obj, JSGeometry::GetObjectType()))  // Geometry
    {
      osgEarth::Symbology::Geometry* geometry = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(obj);

      return 
    }*/
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* local = new osg::Vec3d(context->toLocal(*vec));
      return JSVec3d::WrapVec3d(local, true);
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSFilterContext::ToWorldCallback(const v8::Arguments& args)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(args.Holder());

  if (context && args.Length() == 1 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    /*if (V8Util::CheckObjectType(obj, JSGeometry::GetObjectType()))  // Geometry
    {
      osgEarth::Symbology::Geometry* geometry = V8Util::UnwrapObject<osgEarth::Symbology::Geometry>(obj);

      return 
    }*/
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* world = new osg::Vec3d(context->toWorld(*vec));
      return JSVec3d::WrapVec3d(world, true);
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSFilterContext::ToMapCallback(const v8::Arguments& args)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(args.Holder());

  if (context && args.Length() == 1 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* map = new osg::Vec3d(context->toMap(*vec));
      return JSVec3d::WrapVec3d(map, true);
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSFilterContext::FromMapCallback(const v8::Arguments& args)
{
  FilterContext* context = V8Util::UnwrapObject<FilterContext>(args.Holder());

  if (context && args.Length() == 1 && args[0]->IsObject())
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );
    
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))  // Vec3d
    {
      osg::Vec3d* map = V8Util::UnwrapObject<osg::Vec3d>(obj);
      osg::Vec3d* local = new osg::Vec3d(context->fromMap(*map));
      return JSVec3d::WrapVec3d(local, true);
    }
  }

  return v8::Undefined();
}

void
JSFilterContext::FreeContextCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  FilterContext* context = static_cast<FilterContext*>(parameter);
  delete context;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSSession::_objectType = "JSSession";

v8::Handle<v8::Object>
JSSession::WrapSession(osgEarth::Features::Session* session, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(session, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(session, FreeSessionCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSSession::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

#if 0
    template_instance->Set(v8::String::New("resolveURI"), v8::FunctionTemplate::New(ResolveUriCallback));
#endif
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSSession::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  Session* session = V8Util::UnwrapObject<Session>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!session || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "mapInfo")
    return JSMapInfo::WrapMapInfo(const_cast<osgEarth::MapInfo*>(&session->getMapInfo()));

  return v8::Handle<v8::Value>();
}

#if 0
v8::Handle<v8::Value>
JSSession::ResolveUriCallback(const v8::Arguments& args)
{
  Session* session = V8Util::UnwrapObject<Session>(args.Holder());

  if (session && args.Length() == 1 && args[0]->IsString())
  {
    v8::String::Utf8Value utf8_value(args[0]->ToString());
    std::string uri(*utf8_value);
    return v8::String::New(session->resolveURI(uri).c_str());
  }

  return v8::Undefined();
}
#endif

void
JSSession::FreeSessionCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  Session* session = static_cast<Session*>(parameter);
  delete session;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSMapInfo::_objectType = "JSMapInfo";

v8::Handle<v8::Object>
JSMapInfo::WrapMapInfo(osgEarth::MapInfo* mapInfo, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(mapInfo, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(mapInfo, FreeMapInfoCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSMapInfo::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

#if 0
    template_instance->Set(v8::String::New("toMapPoint"), v8::FunctionTemplate::New(ToMapCallback));
    template_instance->Set(v8::String::New("mapPointToWorldPoint"), v8::FunctionTemplate::New(MapToWorldCallback));
    template_instance->Set(v8::String::New("worldPointToMapPoint"), v8::FunctionTemplate::New(WorldToMapCallback));
#endif
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSMapInfo::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!mapInfo || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "geocentric")
    return v8::Boolean::New(mapInfo->isGeocentric());
  if (prop == "cube")
    return v8::Boolean::New(mapInfo->isCube());
  if (prop == "plateCarre" || prop == "platecarre")
    return v8::Boolean::New(mapInfo->isPlateCarre());
  if (prop == "projectedSRS")
    return v8::Boolean::New(mapInfo->isProjectedSRS());
  if (prop == "geographicSRS")
    return v8::Boolean::New(mapInfo->isGeographicSRS());

  return v8::Handle<v8::Value>();
}

#if 0
v8::Handle<v8::Value>
JSMapInfo::ToMapCallback(const v8::Arguments& args)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(args.Holder());

  if (mapInfo && args.Length() == 2 && args[0]->IsObject() && args[1]->IsObject()) // Vec3d & SpatialReference
  {
    v8::Local<v8::Object> obj0( v8::Object::Cast(*args[0]) );
    v8::Local<v8::Object> obj1( v8::Object::Cast(*args[1]) );

    if (V8Util::CheckObjectType(obj0, JSVec3d::GetObjectType()) && V8Util::CheckObjectType(obj1, JSSpatialReference::GetObjectType()))
    {
      osg::Vec3d* input = V8Util::UnwrapObject<osg::Vec3d>(obj0);
      osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj1);

      osg::Vec3d* out = new osg::Vec3d();
      if (mapInfo->toMapPoint(*input, srs, *out))
        return JSVec3d::WrapVec3d(out, true);

      delete out;
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSMapInfo::MapToWorldCallback(const v8::Arguments& args)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(args.Holder());

  if (mapInfo && args.Length() == 1 && args[0]->IsObject()) // Vec3d
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* input = V8Util::UnwrapObject<osg::Vec3d>(obj);

      osg::Vec3d* out = new osg::Vec3d();
      if (mapInfo->mapPointToWorldPoint(*input, *out))
        return JSVec3d::WrapVec3d(out, true);

      delete out;
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSMapInfo::WorldToMapCallback(const v8::Arguments& args)
{
  osgEarth::MapInfo* mapInfo = V8Util::UnwrapObject<osgEarth::MapInfo>(args.Holder());

  if (mapInfo && args.Length() == 1 && args[0]->IsObject()) // Vec3d
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );
    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* input = V8Util::UnwrapObject<osg::Vec3d>(obj);

      osg::Vec3d* out = new osg::Vec3d();
      if (mapInfo->worldPointToMapPoint(*input, *out))
        return JSVec3d::WrapVec3d(out, true);

      delete out;
    }
  }

  return v8::Undefined();
}
#endif

void
JSMapInfo::FreeMapInfoCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  Session* session = static_cast<Session*>(parameter);
  delete session;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSFeatureProfile::_objectType = "JSFeatureProfile";

v8::Handle<v8::Object>
JSFeatureProfile::WrapFeatureProfile(osgEarth::Features::FeatureProfile* profile, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(profile, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(profile, FreeProfileCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSFeatureProfile::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

    //template_instance->Set(v8::String::New("toLocal"),
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSFeatureProfile::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  FeatureProfile* profile = V8Util::UnwrapObject<FeatureProfile>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!profile || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "extent")
    return JSGeoExtent::WrapGeoExtent(const_cast<osgEarth::GeoExtent*>(&profile->getExtent()));
  if (prop == "srs")
    return JSSpatialReference::WrapSpatialReference(const_cast<osgEarth::SpatialReference*>(profile->getSRS()));

  return v8::Handle<v8::Value>();
}

void
JSFeatureProfile::FreeProfileCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  FeatureProfile* profile = static_cast<FeatureProfile*>(parameter);
  delete profile;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSGeoExtent::_objectType = "JSGeoExtent";

v8::Handle<v8::Object>
JSGeoExtent::WrapGeoExtent(osgEarth::GeoExtent* extent, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(extent, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(extent, FreeGeoExtentCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSGeoExtent::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

    template_instance->Set(v8::String::New("contains"), v8::FunctionTemplate::New(ContainsCallback));
    template_instance->Set(v8::String::New("intersects"), v8::FunctionTemplate::New(IntersectsCallback));
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSGeoExtent::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  osgEarth::GeoExtent* extent = V8Util::UnwrapObject<osgEarth::GeoExtent>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);

  if (!extent || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "srs")
    return JSSpatialReference::WrapSpatialReference(const_cast<osgEarth::SpatialReference*>(extent->getSRS()));
  if (prop == "xMin")
    return v8::Number::New(extent->xMin());
  if (prop == "xMax")
    return v8::Number::New(extent->xMax());
  if (prop == "yMin")
    return v8::Number::New(extent->yMin());
  if (prop == "yMax")
    return v8::Number::New(extent->yMax());
  if (prop == "width")
    return v8::Number::New(extent->width());
  if (prop == "height")
    return v8::Number::New(extent->height());
  if (prop == "crossesAntimeridian")
    return v8::Boolean::New(extent->crossesAntimeridian());
  if (prop == "valid")
    return v8::Boolean::New(extent->isValid());
  if (prop == "defined")
    return v8::Boolean::New(extent->defined());
  if (prop == "area")
    return v8::Number::New(extent->area());
  //if (prop == "toString")
  //  return v8::String::New(extent->toString().c_str());

  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSGeoExtent::ContainsCallback(const v8::Arguments& args)
{
  osgEarth::GeoExtent* extent = V8Util::UnwrapObject<osgEarth::GeoExtent>(args.Holder());

  if (extent)
  {
    if (args.Length() == 1 && args[0]->IsObject())  // Bounds
    {
      v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

      if (V8Util::CheckObjectType(obj, JSBounds::GetObjectType()))
      {
        osgEarth::Bounds* bounds = V8Util::UnwrapObject<osgEarth::Bounds>(obj);
        return v8::Boolean::New(extent->contains(*bounds));
      }
    }
    else if (args.Length() == 2 /*&& args[0]->IsNumber() && args[1]->IsNumber()*/)  // x and y
    {
      return v8::Boolean::New(extent->contains(args[0]->NumberValue(), args[1]->NumberValue()));
    }
    else if (args.Length() == 3 && /*args[0]->IsNumber() && args[1]->IsNumber() &&*/ args[2]->IsObject())  // x, y, and SpatialReference
    {
      v8::Local<v8::Object> obj( v8::Object::Cast(*args[2]) );

      if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
      {
        osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
        return v8::Boolean::New(extent->contains(args[0]->NumberValue(), args[1]->NumberValue(), srs));
      }
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSGeoExtent::IntersectsCallback(const v8::Arguments& args)
{
  osgEarth::GeoExtent* extent = V8Util::UnwrapObject<osgEarth::GeoExtent>(args.Holder());

  if (!extent)
    return v8::Undefined();

  if (args.Length() == 1 && args[0]->IsObject())  // GeoExtent
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    if (V8Util::CheckObjectType(obj, JSGeoExtent::GetObjectType()))
    {
      osgEarth::GeoExtent* rhs = V8Util::UnwrapObject<osgEarth::GeoExtent>(obj);
      return v8::Boolean::New(extent->intersects(*rhs));
    }
  }

  return v8::Undefined();
}

void
JSGeoExtent::FreeGeoExtentCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  osgEarth::GeoExtent* extent = static_cast<osgEarth::GeoExtent*>(parameter);
  delete extent;

  object.Dispose();
  object.Clear();
}

// ---------------------------------------------------------------------------

const std::string JSSpatialReference::_objectType = "JSSpatialReference";

v8::Handle<v8::Object>
JSSpatialReference::WrapSpatialReference(osgEarth::SpatialReference* srs, bool freeObject)
{
  v8::HandleScope handle_scope;

  v8::Handle<v8::Object> obj = V8Util::WrapObject(srs, GetObjectTemplate());

  if (freeObject)
  {
    v8::Persistent<v8::Object> weakRef = v8::Persistent<v8::Object>::New(obj);
    weakRef.MakeWeak(srs, FreeSpatialReferenceCallback);
  }

  return handle_scope.Close(obj);
}

v8::Handle<v8::ObjectTemplate>
JSSpatialReference::GetObjectTemplate()
{
  v8::HandleScope handle_scope;

  static v8::Persistent<v8::ObjectTemplate> template_instance;

  if (template_instance.IsEmpty())
  {
    template_instance = v8::Persistent<v8::ObjectTemplate>::New(v8::ObjectTemplate::New());
    template_instance->Set(v8::String::New(V8_OBJECT_TYPE_PROPERTY), v8::String::New(GetObjectType().c_str()));
    template_instance->SetInternalFieldCount(1);
    template_instance->SetNamedPropertyHandler(PropertyCallback);

    template_instance->Set(v8::String::New("isEquivalentTo"), v8::FunctionTemplate::New(EquivalenceCallback));
    template_instance->Set(v8::String::New("createTangentPlaneSRS"), v8::FunctionTemplate::New(TangentPlaneCallback));
    //template_instance->Set(v8::String::New("createTransMercFromLongitude"), v8::FunctionTemplate::New(equivalenceCallback));
    //template_instance->Set(v8::String::New("createUTMFromLongitude"), v8::FunctionTemplate::New(equivalenceCallback));
  }

  return template_instance;
}

v8::Handle<v8::Value>
JSSpatialReference::PropertyCallback(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(info.Holder());

  v8::String::Utf8Value utf8_value(name);
  std::string prop(*utf8_value);
  
  if (!srs || prop.empty())
    return v8::Handle<v8::Value>();

  if (prop == "geographic")
    return v8::Boolean::New(srs->isGeographic());
  if (prop == "projected")
    return v8::Boolean::New(srs->isProjected());
  if (prop == "mercator")
    return v8::Boolean::New(srs->isMercator());
  if (prop == "sphericalMercator")
    return v8::Boolean::New(srs->isSphericalMercator());
  if (prop == "northPolar")
    return v8::Boolean::New(srs->isNorthPolar());
  if (prop == "southPolar")
    return v8::Boolean::New(srs->isSouthPolar());
  if (prop == "userDefined")
    return v8::Boolean::New(srs->isUserDefined());
  if (prop == "contiguous")
    return v8::Boolean::New(srs->isContiguous());
  if (prop == "cube")
    return v8::Boolean::New(srs->isCube());
  if (prop == "LTP" || prop == "ltp")
    return v8::Boolean::New(srs->isLTP());
  if (prop == "name")
    return v8::String::New(srs->getName().c_str());
  if (prop == "WKT" || prop == "wkt")
    return v8::String::New(srs->getWKT().c_str());
  if (prop == "initType")
    return v8::String::New(srs->getInitType().c_str());
  if (prop == "horizInitString")
    return v8::String::New(srs->getHorizInitString().c_str());
  if (prop == "vertInitString")
    return v8::String::New(srs->getVertInitString().c_str());
  if (prop == "datumName")
    return v8::String::New(srs->getDatumName().c_str());
  if (prop == "geographicSRS")
    return JSSpatialReference::WrapSpatialReference(const_cast<osgEarth::SpatialReference*>(srs->getGeographicSRS()));

  return v8::Handle<v8::Value>();
}

v8::Handle<v8::Value>
JSSpatialReference::EquivalenceCallback(const v8::Arguments& args)
{
  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(args.Holder());

  if (!srs)
    return v8::Undefined();

  if (args.Length() == 1 && args[0]->IsObject())  // SpatialReference
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    if (V8Util::CheckObjectType(obj, JSSpatialReference::GetObjectType()))
    {
      osgEarth::SpatialReference* rhs = V8Util::UnwrapObject<osgEarth::SpatialReference>(obj);
      return v8::Boolean::New(srs->isEquivalentTo(rhs));
    }
  }

  return v8::Undefined();
}

v8::Handle<v8::Value>
JSSpatialReference::TangentPlaneCallback(const v8::Arguments& args)
{
  osgEarth::SpatialReference* srs = V8Util::UnwrapObject<osgEarth::SpatialReference>(args.Holder());

  if (!srs)
    return v8::Undefined();

  if (args.Length() == 1 && args[0]->IsObject())  // Vec3d
  {
    v8::Local<v8::Object> obj( v8::Object::Cast(*args[0]) );

    if (V8Util::CheckObjectType(obj, JSVec3d::GetObjectType()))
    {
      osg::Vec3d* vec = V8Util::UnwrapObject<osg::Vec3d>(obj);
      return JSSpatialReference::WrapSpatialReference(const_cast<SpatialReference*>(srs->createTangentPlaneSRS(*vec)), true);
    }
  }

  return v8::Undefined();
}

void
JSSpatialReference::FreeSpatialReferenceCallback(v8::Persistent<v8::Value> object, void *parameter)
{
  //osgEarth::SpatialReference* srs = static_cast<osgEarth::SpatialReference*>(parameter);
  //delete srs;
  osg::ref_ptr<osgEarth::SpatialReference> srs = static_cast<osgEarth::SpatialReference*>(parameter);

  object.Dispose();
  object.Clear();
}