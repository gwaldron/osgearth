/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osg/Notify>
#include <algorithm>
#include <sstream>
#include <expat.h>

using namespace osgEarth;


static std::string EMPTY_VALUE = "";

std::string
osgEarth::trim( const std::string& in )
{
    std::string whitespace (" \t\f\v\n\r");
    // by Rodrigo C F Dias
    // http://www.codeproject.com/KB/stl/stdstringtrim.aspx
    std::string str = in;
    std::string::size_type pos = str.find_last_not_of( whitespace );
    if(pos != std::string::npos) {
        str.erase(pos + 1);
        pos = str.find_first_not_of( whitespace );
        if(pos != std::string::npos) str.erase(0, pos);
    }
    else str.erase(str.begin(), str.end());
    return str;
}


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
    name = conf.name();
    for( Properties::const_iterator i = conf.attrs().begin(); i != conf.attrs().end(); i++ )
        attrs[i->first] = i->second;
    for( ConfigSet::const_iterator j = conf.children().begin(); j != conf.children().end(); j++ )
    {
        if (!j->children().empty())
        {
            children.push_back( new XmlElement( *j ) );
        }
        else
        {
            addSubElement(j->name(), j->value());
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
    XmlElement *ele = new XmlElement(tag);
    ele->getChildren().push_back(new XmlText(text));
    children.push_back(ele);
}

Config
XmlElement::toConfig() const
{
    Config conf( name );
    for( XmlAttributes::const_iterator a = attrs.begin(); a != attrs.end(); a++ )
        conf.attr( a->first ) = a->second;
    for( XmlNodeList::const_iterator c = children.begin(); c != children.end(); c++ )
    {
        XmlNode* n = c->get();
        if ( n->isElement() )
            conf.children().push_back( static_cast<const XmlElement*>(n)->toConfig() );
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


XmlDocument::XmlDocument( const std::string& _source_uri ) :
XmlElement( "Document" ),
source_uri( _source_uri )
{
    //NOP
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


static XmlAttributes
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


static void XMLCALL
startElement( void* user_data, const XML_Char* c_tag, const XML_Char** c_attrs )
{
    XmlElementNoRefStack& stack = *(XmlElementNoRefStack*)user_data;
    XmlElement* top = stack.top();

    std::string tag( c_tag );
    std::transform( tag.begin(), tag.end(), tag.begin(), tolower );
    XmlAttributes attrs = getAttributes( c_attrs );

    XmlElement* new_element = new XmlElement( tag, attrs );
    top->getChildren().push_back( new_element );
    stack.push( new_element );
}

static void XMLCALL
endElement( void* user_data, const XML_Char* c_tag )
{
    XmlElementNoRefStack& stack = *(XmlElementNoRefStack*)user_data;
    XmlElement* top = stack.top();
    stack.pop();
} 

static void XMLCALL
handleCharData( void* user_data, const XML_Char* c_data, int len )
{
    if ( len > 0 )
    {
        XmlElementNoRefStack& stack = *(XmlElementNoRefStack*)user_data;
        XmlElement* top = stack.top();
        std::string data( c_data, len );
        top->getChildren().push_back( new XmlText( data ) );
    }
}

XmlDocument*
XmlDocument::load( std::istream& in )
{
    XmlElementNoRefStack tree;

    XmlDocument* doc = new XmlDocument();
    tree.push( doc );

#define BUFSIZE 1024
    char buf[BUFSIZE];
    XML_Parser parser = XML_ParserCreate( NULL );
    bool done = false;
    XML_SetUserData( parser, &tree );
    XML_SetElementHandler( parser, startElement, endElement );
    XML_SetCharacterDataHandler( parser, (XML_CharacterDataHandler)handleCharData );
    while( !in.eof() )
    {
        in.read( buf, BUFSIZE );
        int bytes_read = in.gcount();
        if ( bytes_read > 0 )
        {
            if ( XML_Parse( parser, buf, bytes_read, in.eof() ) == XML_STATUS_ERROR )
            {
                osg::notify( osg::WARN ) 
                    << XML_ErrorString( XML_GetErrorCode( parser ) )
                    << ", "
                    << XML_GetCurrentLineNumber( parser ) 
                    << std::endl;

                XML_ParserFree( parser );
                return NULL;
            }
        }
    }
    XML_ParserFree( parser );
    return doc;
}

#define INDENT 4

static void
storeNode( const XmlNode* node, int depth, std::ostream& out )
{
    for( int k=0; k<depth*INDENT; k++ ) out << " ";

    if ( node->isElement() )
    {
        XmlElement* e = (XmlElement*)node;
        out << "<" << e->getName();
        for( XmlAttributes::iterator a = e->getAttrs().begin(); a != e->getAttrs().end(); a++ )
        {
            out << " " << a->first << "=" << "\"" << a->second << "\"";
        }
        out << ">" << std::endl;
        for( XmlNodeList::iterator i = e->getChildren().begin(); i != e->getChildren().end(); i++ )
        {
            storeNode( i->get(), depth+1, out );
        }
        for( int k=0; k<depth*INDENT; k++ ) out << " ";
        out << "</" << e->getName() << ">" << std::endl;
    }
    else if ( node->isText() )
    {
        XmlText* t = (XmlText*)node;
        out << t->getValue() << std::endl;
    }
}

void
XmlDocument::store( std::ostream& out ) const
{
    out << "<?xml version=\"1.0\"?>" << std::endl;
    storeNode( this, 0, out);
    /*for( XmlNodeList::const_iterator i = getChildren().begin(); i != getChildren().end(); i++ )
    {
        storeNode( i->get(), 0, out );
    }*/
}