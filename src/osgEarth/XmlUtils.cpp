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

#include <osgEarth/XmlUtils>
#include <osgEarth/StringUtils>
#include <osg/Notify>

#include "tinyxml.h"
#include <algorithm>
#include <sstream>
#include <iomanip>


using namespace osgEarth;

static std::string EMPTY_VALUE = "";

XmlNode::XmlNode()
{
    //NOP
}

XmlElement::XmlElement( const std::string& _name )
{
    name = _name;
}

XmlElement::XmlElement( const std::string& _name, const XmlAttributes& _attrs )
{
    name = _name;
    attrs = _attrs;
}

XmlElement::XmlElement( const Config& conf )
{
    name = conf.key();

    if ( !conf.value().empty() )
    {
        children.push_back( new XmlText(conf.value()) );
    }

    for( ConfigSet::const_iterator j = conf.children().begin(); j != conf.children().end(); j++ )
    {
        if ( j->isSimple() )
        {
            attrs[j->key()] = j->value();
        }
        else if ( j->children().size() > 0 )
        {
            children.push_back( new XmlElement(*j) );
        }
    }
}

const std::string&
XmlElement::getName() const
{
    return name;
}

void
XmlElement::setName( const std::string& name)
{
    this->name = name;
}

XmlAttributes&
XmlElement::getAttrs()
{
    return attrs;
}

const XmlAttributes&
XmlElement::getAttrs() const
{
    return attrs;
}

const std::string&
XmlElement::getAttr( const std::string& key ) const
{
    XmlAttributes::const_iterator i = attrs.find( key );
    return i != attrs.end()? i->second : EMPTY_VALUE;
}

XmlNodeList&
XmlElement::getChildren()
{
    return children;
}

const XmlNodeList&
XmlElement::getChildren() const
{
    return children;
}
        
XmlElement*
XmlElement::getSubElement( const std::string& name ) const
{
    std::string name_lower = name;
    std::transform( name_lower.begin(), name_lower.end(), name_lower.begin(), tolower );

    for( XmlNodeList::const_iterator i = getChildren().begin(); i != getChildren().end(); i++ )
    {
        if ( i->get()->isElement() )
        {
            XmlElement* e = (XmlElement*)i->get();
            std::string name = e->getName();
            std::transform( name.begin(), name.end(), name.begin(), tolower );
            if ( name == name_lower )
                return e;
        }
    }
    return NULL;
}


std::string
XmlElement::getText() const
{
    std::stringstream builder;

    for( XmlNodeList::const_iterator i = getChildren().begin(); i != getChildren().end(); i++ )
    {
        if ( i->get()->isText() )
        {
            builder << ( static_cast<XmlText*>( i->get() ) )->getValue();
        }
    }

 	 std::string builderStr;
	 builderStr = builder.str();
    std::string result = trim( builderStr );
    return result;
}


std::string
XmlElement::getSubElementText( const std::string& name ) const
{
    XmlElement* e = getSubElement( name );
    return e? e->getText() : EMPTY_VALUE;
}


XmlNodeList 
XmlElement::getSubElements( const std::string& name ) const
{
    XmlNodeList results;

    std::string name_lower = name;
    std::transform( name_lower.begin(), name_lower.end(), name_lower.begin(), tolower );

    for( XmlNodeList::const_iterator i = getChildren().begin(); i != getChildren().end(); i++ )
    {
        if ( i->get()->isElement() )
        {
            XmlElement* e = (XmlElement*)i->get();
            std::string name = e->getName();
            std::transform( name.begin(), name.end(), name.begin(), tolower );
            if ( name == name_lower )
                results.push_back( e );
        }
    }

    return results;
}

void
XmlElement::addSubElement(const std::string& tag, const std::string& text)
{
    XmlElement* ele = new XmlElement(tag);
    ele->getChildren().push_back(new XmlText(text));
    children.push_back(ele);
}

void
XmlElement::addSubElement(const std::string& tag, const Properties& attrs, const std::string& text)
{
    XmlElement* ele = new XmlElement(tag);
    for( Properties::const_iterator i = attrs.begin(); i != attrs.end(); i++ )
        ele->attrs[i->first] = i->second;
    ele->getChildren().push_back(new XmlText(text));
    children.push_back(ele);
}

Config
XmlElement::getConfig() const
{
    Config conf( name );

    for( XmlAttributes::const_iterator a = attrs.begin(); a != attrs.end(); a++ )
    {
        conf.set( a->first, a->second );
    }

    for( XmlNodeList::const_iterator c = children.begin(); c != children.end(); c++ )
    {
        XmlNode* n = c->get();
        if ( n->isElement() )
            conf.add( static_cast<const XmlElement*>(n)->getConfig() );
    }

    conf.value() = getText();
        //else 
        //    conf.value() = trim( static_cast<const XmlText*>(n)->getValue() );
    return conf;
}

XmlText::XmlText( const std::string& _value )
{
    value = _value;
}

const std::string&
XmlText::getValue() const
{
    return value;
}

XmlDocument::XmlDocument() :
XmlElement( "Document" )
{
    //nop
}

XmlDocument::XmlDocument( const Config& conf ) :
XmlElement( conf )
{
    //NOP
}

XmlDocument::~XmlDocument()
{
    //NOP
}

namespace
{
    XmlAttributes
    getAttributes( const char** attrs )
    {
        XmlAttributes map;
        const char** ptr = attrs;
        while( *ptr != NULL )
        {
            std::string name = *ptr++;
            std::string value = *ptr++;
            std::transform( name.begin(), name.end(), name.begin(), tolower );
            map[name] = value;
        }
        return map;
    }

    void processNode(XmlElement* parent, TiXmlNode* node)
    {
        XmlElement* new_element = 0;
        switch (node->Type())
        {
        case TiXmlNode::TINYXML_ELEMENT:
            {
                TiXmlElement* element = node->ToElement();
                std::string tag = element->Value();
                std::transform( tag.begin(), tag.end(), tag.begin(), tolower);

                //Get all the attributes
                XmlAttributes attrs;
                TiXmlAttribute* attr = element->FirstAttribute();
                while (attr)
                {
                    std::string name  = attr->Name();
                    std::string value = attr->Value();
                    std::transform( name.begin(), name.end(), name.begin(), tolower);
                    attrs[name] = value;
                    attr = attr->Next();
                }

                //All the element to the stack
                new_element = new XmlElement( tag, attrs );
                parent->getChildren().push_back( new_element );
            }
            break;
        case TiXmlNode::TINYXML_TEXT:
            {
                TiXmlText* text = node->ToText();
                std::string data( text->Value());
                parent->getChildren().push_back( new XmlText( data ) );
            }
            break;
        }    

        XmlElement* new_parent = new_element ? new_element : parent;
        TiXmlNode* child;
        for (child = node->FirstChild(); child != 0; child = child->NextSibling())
        {    
            processNode( new_parent, child );
        }
    }

    void
    removeDocType(std::string &xmlStr)
    {
        //TinyXML has an issue with parsing DTDs.  See http://www.grinninglizard.com/tinyxmldocs/index.html
        //We need to remove any !DOCTYPE block that appears in the XML before parsing to avoid errors.
        std::string::size_type startIndex = xmlStr.find("<!DOCTYPE");
        if (startIndex == xmlStr.npos) return;

        std::string::size_type endIndex = startIndex;
        int numChildElements = 0;
        //We've found the first index of the <!DOCTYPE, now find the index of the closing >
        while (endIndex < xmlStr.size())
        {
            endIndex+=1;
            if (xmlStr[endIndex] == '<')
            {
                numChildElements++;
            }
            else if (xmlStr[endIndex] == '>')
            {
                if (numChildElements == 0)
                {
                    break;
                }
                else
                {
                    numChildElements--;
                }
            }
        }

        //Now, replace the <!DOCTYPE> element with whitespace
        xmlStr.erase(startIndex, endIndex - startIndex + 1);
    }
}


XmlDocument*
XmlDocument::load( const std::string& location, const osgDB::Options* dbOptions )
{
    return load( URI(location), dbOptions );
}

XmlDocument*
XmlDocument::load( const URI& uri, const osgDB::Options* dbOptions )
{
    XmlDocument* result = 0L;

    ReadResult r = uri.readString( dbOptions );
    if ( r.succeeded() )
    {
        std::stringstream buf( r.getString() );
        result = load( buf );
        if ( result )
            result->_sourceURI = uri;
    }

    return result;
}

XmlDocument*
XmlDocument::load( std::istream& in, const URIContext& uriContext )
{
    TiXmlDocument xmlDoc;

    //Read the entire document into a string
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string xmlStr;
    xmlStr = buffer.str();

    removeDocType( xmlStr );
    //OE_NOTICE << xmlStr;

    XmlDocument* doc = NULL;
    xmlDoc.Parse(xmlStr.c_str());    

    if ( xmlDoc.Error() )
    {
        std::stringstream buf;
        buf << xmlDoc.ErrorDesc() << " (row " << xmlDoc.ErrorRow() << ", col " << xmlDoc.ErrorCol() << ")";
        std::string str;
        str = buf.str();
        OE_WARN << "Error in XML document: " << str << std::endl;
        if ( !uriContext.referrer().empty() )
            OE_WARN << uriContext.referrer() << std::endl;
    }

    if ( !xmlDoc.Error() && xmlDoc.RootElement() )
    {
        doc = new XmlDocument();
        processNode( doc,  xmlDoc.RootElement() );
        doc->_sourceURI = URI("", uriContext);
    }
    return doc;    
}

Config
XmlDocument::getConfig() const
{
    Config conf = XmlElement::getConfig();
    conf.setReferrer( _sourceURI.full() );
    return conf;
}

static void
storeNode( const XmlNode* node, TiXmlNode* parent)
{
    if (node->isElement())
    {
        XmlElement* e = (XmlElement*)node;
        TiXmlElement* element = new TiXmlElement( e->getName().c_str() );
        //Write out all the attributes
        for( XmlAttributes::iterator a = e->getAttrs().begin(); a != e->getAttrs().end(); a++ )
        {
            element->SetAttribute(a->first.c_str(), a->second.c_str() );            
        }

        //Write out all the child nodes
        for( XmlNodeList::iterator i = e->getChildren().begin(); i != e->getChildren().end(); i++ )
        {
            storeNode( i->get(), element );
        }
        parent->LinkEndChild( element );
    }
    else if (node->isText())
    {
        XmlText* t = (XmlText*)node;
        parent->LinkEndChild( new TiXmlText( t->getValue().c_str() ) );
    }
}

void
XmlDocument::store( std::ostream& out ) const
{    
    TiXmlDocument doc;
    doc.LinkEndChild( new TiXmlDeclaration( "1.0", "", ""));
    storeNode( this, &doc );    


    //Use TiXmlPrinter to do pretty printing.
    TiXmlPrinter printer;
    printer.SetIndent("  ");
    doc.Accept(&printer);
    out << printer.CStr();

    //out << doc;    
}
