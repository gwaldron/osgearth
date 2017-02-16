/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/Registry>

#define LC "[LandCover] "

using namespace osgEarth;

LandCoverClass::LandCoverClass() :
osg::Object(),
_value(0)
{
    //nop
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

LandCoverDictionary::LandCoverDictionary() :
osg::Object()
{
    //nop
}

LandCoverDictionary::LandCoverDictionary(const Config& conf) :
osg::Object()
{
    fromConfig(conf);
}

LandCoverDictionary::LandCoverDictionary(const LandCoverDictionary& rhs, const osg::CopyOp& op) :
osg::Object(rhs, op),
_landCoverClasses(rhs._landCoverClasses)
{
    //nop
}

const LandCoverClass*
LandCoverDictionary::getClass(const std::string& name) const
{
    for (LandCoverClassVector::const_iterator i = _landCoverClasses.begin();
        i != _landCoverClasses.end();
        ++i)
    {
        if (i->get()->getName() == name)
            return i->get();
    }
    return 0L;
}

void
LandCoverDictionary::fromConfig(const Config& conf)
{
    setName(conf.value("name"));

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
LandCoverDictionary::getConfig() const
{
    Config conf("land_cover_dictionary");
    if (!_landCoverClasses.empty())
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

void
LandCoverValueMapping::fromConfig(const Config& conf)
{
    conf.getIfSet("value", _value);
    conf.getIfSet("class", _lcClassName);
}

Config
LandCoverValueMapping::getConfig() const
{
    Config conf("mapping");
    conf.addIfSet("value", _value);
    conf.addIfSet("class", _lcClassName);
    return conf;
}

//...........................................................................

#undef  LC
#define LC "[LandCoverDataSource] "

LandCoverDataSource::LandCoverDataSource() :
osg::Object()
{
    //nop
}

LandCoverDataSource::LandCoverDataSource(const Config& conf) :
osg::Object()
{
    fromConfig(conf);
}

LandCoverDataSource::LandCoverDataSource(const LandCoverDataSource& rhs, const osg::CopyOp& op) :
osg::Object(rhs, op)
{
    _valueMappings = rhs._valueMappings;
}

void
LandCoverDataSource::fromConfig(const Config& conf)
{
    setName(conf.value("name"));

    const Config* mappings = conf.child_ptr("mappings");
    if (mappings)
    {
        for (ConfigSet::const_iterator i = mappings->children().begin();
            i != mappings->children().end();
            ++i)
        {
            osg::ref_ptr<LandCoverValueMapping> mapping = new LandCoverValueMapping(*i);
            _valueMappings.push_back(mapping.get());
        }
    }

    OE_INFO << LC << _valueMappings.size() << " mappings loaded for " << getName() << "\n";
}

Config
LandCoverDataSource::getConfig() const
{
    Config conf("land_cover_data_source");
    Config mappings("mappings");
    conf.add(mappings);
    for(LandCoverValueMappingVector::const_iterator i = _valueMappings.begin();
        i != _valueMappings.end();
        ++i)
    {
        LandCoverValueMapping* mapping = i->get();
        if (mapping)
            mappings.add(mapping->getConfig());
    }
    return conf;
}
