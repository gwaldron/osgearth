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
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/InstanceSymbol>
#include <osgEarth/Registry>

#define LC "[ModelNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


//------------------------------------------------------------------------


ModelNode::ModelNode(MapNode*              mapNode,
                     const Style&          style,
                     const osgDB::Options* dbOptions ) :
LocalizedNode( mapNode )
{
    _style = style;
    init( dbOptions );
}


void
ModelNode::init(const osgDB::Options* dbOptions)
{
    this->setHorizonCulling(false);

    osg::ref_ptr<const ModelSymbol> sym = _style->get<ModelSymbol>();
    
    // backwards-compatibility: support for MarkerSymbol (deprecated)
    if ( !sym.valid() && _style->has<MarkerSymbol>() )
    {
        osg::ref_ptr<InstanceSymbol> temp = _style->get<MarkerSymbol>()->convertToInstanceSymbol();
        sym = dynamic_cast<const ModelSymbol*>( temp.get() );
    }

    if ( sym.valid() )
    {
        if ( sym->url().isSet() )
        {
            URI uri( sym->url()->eval(), sym->url()->uriContext() );
            osg::Node* node = 0L;
            
            if ( sym->uriAliasMap()->empty() )
            {
                node = uri.getNode( dbOptions );
            }
            else
            {
                // install an alias map if there's one in the symbology.
                osg::ref_ptr<osgDB::Options> tempOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
                tempOptions->setReadFileCallback( new URIAliasMapReadCallback(*sym->uriAliasMap(), uri.full()) );
                node = uri.getNode( tempOptions.get() );
            }

            if ( node )
            {
                getTransform()->addChild( node );
                this->addChild( getTransform() );

                if ( sym->scale().isSet() )
                {
                    double s = sym->scale()->eval();
                    this->setScale( osg::Vec3f(s, s, s) );
                }

                if (sym && (sym->heading().isSet() || sym->pitch().isSet() || sym->roll().isSet()) )
                {
                    osg::Matrix rot;
                    double heading = sym->heading().isSet() ? sym->heading()->eval() : 0.0;
                    double pitch   = sym->pitch().isSet()   ? sym->pitch()->eval()   : 0.0;
                    double roll    = sym->roll().isSet()    ? sym->roll()->eval()    : 0.0;
                    rot.makeRotate( 
                        osg::DegreesToRadians(heading), osg::Vec3(1,0,0),
                        osg::DegreesToRadians(pitch),   osg::Vec3(0,0,1),
                        osg::DegreesToRadians(roll),    osg::Vec3(0,1,0) );
                    this->setLocalRotation( rot.getRotate() );
                }

                applyStyle( *_style );
            }
            else
            {
                OE_WARN << LC << "Failed to load data from " << uri.full() << std::endl;
            }
        }
        else
        {
            OE_WARN << LC << "Symbology: no URI" << std::endl;
        }
    }
    else
    {
        OE_WARN << LC << "Insufficient symbology" << std::endl;
    }
}

//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( model, osgEarth::Annotation::ModelNode );


ModelNode::ModelNode(MapNode* mapNode, const Config& conf, const osgDB::Options* dbOptions) :
LocalizedNode( mapNode )
{
    conf.getObjIfSet( "style", _style );

    std::string uri = conf.value("url");
    if ( !uri.empty() )
        _style->getOrCreate<ModelSymbol>()->url() = StringExpression(uri);

    init( dbOptions );

    if ( conf.hasChild( "position" ) )
        setPosition( GeoPoint(conf.child("position")) );
}

Config
ModelNode::getConfig() const
{
    Config conf("model");

    conf.updateObjIfSet( "style",    _style );
    conf.updateObj     ( "position", getPosition() );

    return conf;
}
