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
    if (!conf.externalRef().empty())
    {
        attrs["href"] = conf.externalRef();
        name = "xi:include";
    }
    else
    {

        name = conf.key();

        if ( !conf.value().empty() )
        {
            children.push_back( new XmlText(conf.value()) );
        }

        for( ConfigSet::const_iterator j = conf.children().begin(); j != conf.children().end(); j++ )
        {
            //if ( j->isSimple() )
            //{
            //    attrs[j->key()] = j->value();
            //}

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
    for( XmlNodeList::const_iterator i = getChildren().begin(); i != getChildren().end(); i++ )
    {
        if ( i->get()->isElement() )
        {
            XmlElement* e = (XmlElement*)i->get();
            if (osgEarth::ciEquals(name, e->getName()))
                return e;
        }
    }
    return NULL;
}

const XmlElement*
XmlElement::findElement(const std::string& name) const
{
    const XmlElement* result = 0L;

    if ( this->getName() == name )
    {
        result = this;
    }
    else
    {
        // first check the subelements (breadth first search)
        for(XmlNodeList::const_iterator i = getChildren().begin();
            i != getChildren().end() && result == 0L;
            i++ )
        {
            if ( i->get()->isElement() )
            {
                XmlElement* e = (XmlElement*)i->get();
                if (osgEarth::ciEquals(name, e->getName()))
                {
                    result = e;
                }
            }
        }

        // not found? traverse the subelements.
        if ( result == 0L )
        {
            for(XmlNodeList::const_iterator i = getChildren().begin();
                i != getChildren().end() && result == 0L;
                i++ )
            {
                XmlElement* e = (XmlElement*)i->get();
                result = e->findElement( name );
            }
        }
    }

    return result;
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

    for( XmlNodeList::const_iterator i = getChildren().begin(); i != getChildren().end(); i++ )
    {
        if ( i->get()->isElement() )
        {
            XmlElement* e = (XmlElement*)i->get();
            if ( osgEarth::ciEquals(name, e->getName()) )
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
XmlElement::getConfig(const std::string& referrer) const
{
	if (isInclude())
	{
		std::string href = getAttr("href");

        if (href.empty())
        {
            OE_WARN << "Missing href with xi:include" << std::endl;
            return Config();
        }

        URIContext uriContext(referrer);
        URI uri(href, uriContext);
        std::string fullURI = uri.full();
        OE_INFO << "Loading href from " << fullURI << std::endl;

        osg::ref_ptr< XmlDocument > doc = XmlDocument::load(fullURI);
        if (doc && doc->getChildren().size() > 0)
        {
            Config conf = static_cast<XmlElement*>(doc->children.front().get())->getConfig( fullURI );
            conf.setExternalRef( href ); //fullURI );
            conf.setReferrer( fullURI );
            return conf;
        }
        else
        {
            OE_WARN << "Failed to load xi:include from " << fullURI << std::endl;
            return Config();
        }        
	}
	else
	{
		Config conf( name );
        conf.setReferrer( referrer );

		for( XmlAttributes::const_iterator a = attrs.begin(); a != attrs.end(); a++ )
		{
			conf.set( a->first, a->second );
		}

		for( XmlNodeList::const_iterator c = children.begin(); c != children.end(); c++ )
		{
			XmlNode* n = c->get();
			if ( n->isElement() )
				conf.add( static_cast<const XmlElement*>(n)->getConfig(referrer) );
		}

		conf.value() = getText();
		//else 
		//    conf.value() = trim( static_cast<const XmlText*>(n)->getValue() );
		return conf;
	}
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
            map[osgEarth::toLower(name)] = value;
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
                std::string tag = osgEarth::toLower(element->Value());

                //Get all the attributes
                XmlAttributes attrs;
                TiXmlAttribute* attr = element->FirstAttribute();
                while (attr)
                {
                    std::string name  = osgEarth::toLower(attr->Name());
                    std::string value = attr->Value();
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
        URIContext context( uri.full() );
        result = load( buf, context );
        if ( result )
        {
            result->_sourceURI = uri;
        }
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
    Config conf = XmlElement::getConfig(_sourceURI.full());
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
