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
#include <osgEarth/LandCover>
#include <osgEarth/XmlUtils>

#define LC "[LandCover] "

using namespace osgEarth;

LandCoverClass::LandCoverClass() :
osg::Object(),
_value(0)
{
    //nop
}

LandCoverClass::LandCoverClass(const std::string& name, int value) :
osg::Object()
{
    setName(name);
    setValue(value);
}

LandCoverClass::LandCoverClass(const Config& conf) :
osg::Object(),
_value(0)
{
    fromConfig(conf);
}

LandCoverClass::LandCoverClass(const LandCoverClass& rhs, const osg::CopyOp& op) :
osg::Object(rhs, op),
_value(rhs._value)
{
    //nop
}

void
LandCoverClass::fromConfig(const Config& conf)
{
    setName(conf.value("name"));
}

Config
LandCoverClass::getConfig() const
{
    Config conf("class");
    conf.set("name", getName());
    return conf;
}

//............................................................................

#undef  LC
#define LC "[LandCoverDictionary] "

REGISTER_OSGEARTH_LAYER(landcoverdictionary, LandCoverDictionary);
REGISTER_OSGEARTH_LAYER(land_cover_dictionary, LandCoverDictionary);

void
LandCoverDictionary::Options::fromConfig(const Config& conf)
{
    const Config* classes = conf.child_ptr("classes");
    if (classes)
    {
        int value = 0;
        for(ConfigSet::const_iterator i = classes->children().begin();
            i != classes->children().end();
            ++i)
        {
            osg::ref_ptr<LandCoverClass> lcc = new LandCoverClass(*i);
            if (!lcc->getName().empty())
            {
                lcc->setValue( value++ );
                _landCoverClasses.push_back(lcc.get());
            }
        }
    }
    OE_INFO << LC << _landCoverClasses.size() << " classes defined.\n";
}

Config
LandCoverDictionary::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();

    if (!classes().empty())
    {
        Config classes("classes");
        conf.add(classes);
        for (LandCoverClassVector::const_iterator i = _landCoverClasses.begin();
            i != _landCoverClasses.end();
            ++i)
        {
            const LandCoverClass* lcc = i->get();
            if (lcc && !lcc->getName().empty())
            {
                classes.add(lcc->getConfig());
            }
        }
    }
    return conf;
}

bool
LandCoverDictionary::Options::loadFromXML(const URI& uri)
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load(uri);
    if (xml.valid())
    {
        Config c = xml->getConfig();
        if (c.hasChild("landcoverdictionary"))
            _conf = c.child("landcoverdictionary");
        else
            _conf = c.child("land_cover_dictionary"); // back-compay
        fromConfig(_conf);
        return true;
    }
    else
    {
        return false;
    }
}

bool
LandCoverDictionary::loadFromXML(const URI& uri)
{
    return options().loadFromXML(uri);
}

void
LandCoverDictionary::addClass(const std::string& name, int value)
{
    if (value == INT_MAX)
        value = options().classes().size();

    options().classes().push_back(new LandCoverClass(name, value));
}

const LandCoverClass*
LandCoverDictionary::getClassByName(const std::string& name) const
{
    for (LandCoverClassVector::const_iterator i = options().classes().begin();
        i != options().classes().end();
        ++i)
    {
        if (i->get()->getName() == name)
            return i->get();
    }
    return 0L;
}


const LandCoverClass*
LandCoverDictionary::getClassByValue(int value) const
{
    for (LandCoverClassVector::const_iterator i = options().classes().begin();
        i != options().classes().end();
        ++i)
    {
        if (i->get()->getValue() == value)
            return i->get();
    }
    return 0L;
}

const LandCoverClass*
LandCoverDictionary::getClassByUV(const GeoImage& tile, double u, double v) const
{
    if (getStatus().isError())
        return 0L;

    if (!tile.valid())
        return 0L;

    ImageUtils::PixelReader read(tile.getImage());
    read.setBilinear(false); // nearest neighbor only!
    read.setDenormalize(false);
    float value = read(u, v).r();

    return getClassByValue((int)value);
}

//...........................................................................

#undef  LC
#define LC "[LandCoverValueMapping] "

LandCoverValueMapping::LandCoverValueMapping() :
osg::Object()
{
    //nop
}

LandCoverValueMapping::LandCoverValueMapping(const Config& conf) :
osg::Object()
{
    fromConfig(conf);
}

LandCoverValueMapping::LandCoverValueMapping(const LandCoverValueMapping& rhs, const osg::CopyOp& op) :
osg::Object(rhs, op)
{
    _value = rhs._value;
    _lcClassName = rhs._lcClassName;
}

LandCoverValueMapping::LandCoverValueMapping(int value, const std::string& className) :
osg::Object(),
_value(value),
_lcClassName(className)
{
    //nop
}

void
LandCoverValueMapping::fromConfig(const Config& conf)
{
    conf.get("value", _value);
    conf.get("class", _lcClassName);
}

Config
LandCoverValueMapping::getConfig() const
{
    Config conf("mapping");
    conf.set("value", _value);
    conf.set("class", _lcClassName);
    return conf;
}

//...........................................................................

#undef  LC
#define LC "[LandCoverCoverageLayer] "

void
LandCoverCoverageLayer::Options::fromConfig(const Config& conf)
{
    warp().init(0.0f);

    ConfigSet mappingsConf = conf.child("land_cover_mappings").children("mapping");
    for (ConfigSet::const_iterator i = mappingsConf.begin(); i != mappingsConf.end(); ++i)
    {
        osg::ref_ptr<LandCoverValueMapping> mapping = new LandCoverValueMapping(*i);
        mappings().push_back(mapping.get());
    }

    conf.get("warp", warp());

    for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        osg::ref_ptr<Layer> temp = Layer::create(*i);
        if (temp.valid())
        {
            layer() = ImageLayer::Options(*i);
            break;
        }
    }
}

Config
LandCoverCoverageLayer::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();
    conf.key() = "coverage";
    if (conf.hasChild("land_cover_mappings") == false)
    {   
        Config mappingConf("land_cover_mappings");
        conf.add(mappingConf); //.update(mappings);
        for(LandCoverValueMappingVector::const_iterator i = mappings().begin();
            i != mappings().end();
            ++i)
        {
            LandCoverValueMapping* mapping = i->get();
            if (mapping)
                mappingConf.add(mapping->getConfig());
        }
    }
    conf.set("warp", warp());

    if (layer().isSet())
        conf.set(layer()->getConfig());

    return conf;
}

bool
LandCoverCoverageLayer::Options::loadMappingsFromXML(const URI& uri)
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load(uri);
    if (xml.valid())
    {
        fromConfig(xml->getConfig());
        return true;
    }
    else return false;
}

void
LandCoverCoverageLayer::Options::map(int value, const std::string& lcClass)
{
    mappings().push_back(new LandCoverValueMapping(value, lcClass));
}

//...................................................................

OE_LAYER_PROPERTY_IMPL(LandCoverCoverageLayer, float, Warp, warp);

Status
LandCoverCoverageLayer::openImplementation()
{
    Status parent = Layer::openImplementation();
    if (parent.isError())
        return parent;

    if (!_imageLayer.valid())
    {
        if (options().layer().isSet())
        {
            osg::ref_ptr<Layer> layer = Layer::create(options().layer().get());
            _imageLayer = dynamic_cast<ImageLayer*>(layer.get());
        }
    }

    Status status;

    if (_imageLayer.valid())
    {
        // pass along the read options
        _imageLayer->setReadOptions(getReadOptions());

        // disable caching for coverage layers so they don't inherit the wrong bin
        // (they are always used in composited form)
        _imageLayer->setCachePolicy(CachePolicy::NO_CACHE);

        // force the "coverage" flag on to generate unnormalized and unfiltered data
        _imageLayer->setCoverage(true);

        status = _imageLayer->open();
    }
    else
    {
        status = Status(Status::ConfigurationError, "No image layer");
    }

    return status;

}

void
LandCoverCoverageLayer::addedToMap(const Map* map)
{
    Layer::addedToMap(map);

    if (_imageLayer.valid())
        _imageLayer->addedToMap(map);
}

void
LandCoverCoverageLayer::removedFromMap(const Map* map)
{
    Layer::removedFromMap(map);

    if (_imageLayer.valid())
        _imageLayer->removedFromMap(map);
}

void
LandCoverCoverageLayer::setImageLayer(ImageLayer* value)
{
    _imageLayer = value;
    if (_imageLayer.valid())
        options().layer() = _imageLayer->getConfig();
}
