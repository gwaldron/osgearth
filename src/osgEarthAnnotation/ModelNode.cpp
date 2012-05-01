/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include <osgEarthAnnotation/ModelNode>
#include <osgEarthAnnotation/AnnotationRegistry>

#define LC "[ModelNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;


ModelNode::ModelNode(MapNode*              mapNode,
                     const URI&            uri,
                     const Style&          style,
                     const osgDB::Options* dbOptions,
                     const CachePolicy&    cachePolicy ) :
LocalizedNode( mapNode ),
_dbOptions   ( dbOptions ),
_cachePolicy ( cachePolicy )
{
    if ( !uri.empty() )
        _uri = uri;
    
    if ( !style.empty() )
        _style = style;

    init();
}


void
ModelNode::init()
{
    this->setHorizonCulling(false);

    if ( _uri.isSet() )
    {
        osg::Node* node = _uri->getNode( _dbOptions.get(), *_cachePolicy );
        if ( node )
        {
            getTransform()->addChild( node );
            this->addChild( getTransform() );
            
            applyStyle( *_style, false );
        }
        else
        {
            OE_WARN << LC << "Failed to load node from " << _uri->full() << std::endl;
        }
    }
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( model, osgEarth::Annotation::ModelNode );


ModelNode::ModelNode(MapNode* mapNode, const Config& conf) :
LocalizedNode( mapNode )
{
    conf.getIfSet   ( "url",          _uri );
    conf.getObjIfSet( "style",        _style );
    conf.getObjIfSet( "cache_policy", _cachePolicy );

    init();

    if ( conf.hasChild( "position" ) )
        setPosition( GeoPoint(conf.child("position")) );
}

Config
ModelNode::getConfig() const
{
    Config conf("model");

    conf.updateIfSet   ( "url",          _uri );
    conf.updateObjIfSet( "style",        _style );
    conf.updateObjIfSet( "cache_policy", _cachePolicy );
    conf.updateObj     ( "position",     getPosition() );

    return conf;
}
