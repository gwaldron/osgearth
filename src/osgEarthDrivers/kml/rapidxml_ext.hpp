#ifndef RAPIDXML_EXT_HPP_INCLUDED 
#define RAPIDXML_EXT_HPP_INCLUDED 1

#include "rapidxml.hpp"
#include <string>

#include <osgEarth/StringUtils>

using namespace rapidxml;

/**
 * Gets the value of an attribute on an xml_node.  Key is case insensitive.
 */
inline std::string getAttr(xml_node<>* node, const std::string& key)
{
	for (xml_attribute<> *attr = node->first_attribute(); attr; attr = attr->next_attribute())
	{
		if (osgEarth::ciEquals(attr->name(), key))
		{
			return attr->value();
		}
	}
	return "";
}

/**
 * Gets the value of a child element.  Key is case insensitive.
 */
inline std::string getChildValue(xml_node<>* node, const std::string& key)
{
    std::string result;
	if (node)
	{
		xml_node<>* child = node->first_node(key.c_str(), 0, false);
		if (child)
		{
            if (child->value_size() > 0)
            {
                result = child->value();
            }
            else //Try to read CDATA node
            {
                child = child->first_node();
                if (child)
                {
                    result = child->value();
                }
            }
		}                

    }
    if (!result.empty())
        osgEarth::trim2(result);

	return result;
}

/*
 * Gets a value from an xml_node by first looking at the attribtes and then looking at the child elements
 */
inline std::string getValue(xml_node<>* node, const std::string &key)
{
    std::string value = "";
	if (node)
	{
		// First look through the attributes
		value = getAttr(node, key);
		if (value.empty())
		{
			value = getChildValue(node, key);
        }
	}
	return value;
}


#endif
