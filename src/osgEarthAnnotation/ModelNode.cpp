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

#include <osgEarthAnnotation/ModelNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/InstanceSymbol>
#include <osgEarth/AutoScale>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/VirtualProgram>

#define LC "[ModelNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


//------------------------------------------------------------------------


ModelNode::ModelNode(MapNode*              mapNode,
                     const Style&          style,
                     const osgDB::Options* dbOptions ) :
LocalizedNode( mapNode ),
_style       ( style ),
_dbOptions   ( dbOptions )
{
    _xform = new osg::MatrixTransform();
    init();
}


void
ModelNode::setStyle(const Style& style)
{
    _style = style;
    init();
}


void
ModelNode::init()
{
    // reset.
    this->clearDecoration();
    osgEarth::clearChildren( this );
    osgEarth::clearChildren( _xform.get() );
    this->addChild( _xform.get() );

    this->setHorizonCulling(false);

    osg::ref_ptr<const ModelSymbol> sym = _style.get<ModelSymbol>();
    
    // backwards-compatibility: support for MarkerSymbol (deprecated)
    if ( !sym.valid() && _style.has<MarkerSymbol>() )
    {
        osg::ref_ptr<InstanceSymbol> temp = _style.get<MarkerSymbol>()->convertToInstanceSymbol();
        sym = dynamic_cast<const ModelSymbol*>( temp.get() );
    }

    if ( sym.valid() )
    {
        if ( ( sym->url().isSet() ) || (sym->getModel() != NULL) )
        {
            // Try to get a model from symbol
            osg::ref_ptr<osg::Node> node = sym->getModel();

            // Try to get a model from URI
            if (node.valid() == false)
            {
                URI uri( sym->url()->eval(), sym->url()->uriContext() );

                if ( sym->uriAliasMap()->empty() )
                {
                    node = uri.getNode( _dbOptions.get() );
                }
                else
                {
                    // install an alias map if there's one in the symbology.
                    osg::ref_ptr<osgDB::Options> tempOptions = Registry::instance()->cloneOrCreateOptions(_dbOptions.get());
                    tempOptions->setReadFileCallback( new URIAliasMapReadCallback(*sym->uriAliasMap(), uri.full()) );
                    node = uri.getNode( tempOptions.get() );
                }

                if (node.valid() == false)
                {
                    OE_WARN << LC << "No model and failed to load data from " << uri.full() << std::endl;
                }
            }

            if (node.valid() == true)
            {
                if ( Registry::capabilities().supportsGLSL() )
                {
                    // generate shader code for the loaded model:
                    ShaderGenerator gen;
                    gen.setProgramName( "osgEarth.ModelNode" );
                    gen.run( node, Registry::stateSetCache() );

                    // do we really need this? perhaps
#if 0
                    // better: put your modelnode under the mapnode?
                    node->addCullCallback( new UpdateLightingUniformsHelper() );
#endif
                }

                // attach to the transform:
                _xform->addChild( node );

                // insert a clamping agent if necessary:
                replaceChild( _xform.get(), applyAltitudePolicy(_xform.get(), _style) );

                if ( sym->scale().isSet() )
                {
                    double s = sym->scale()->eval();
                    this->setScale( osg::Vec3f(s, s, s) );
                }

                // auto scaling?
                if ( sym->autoScale() == true )
                {
                    this->getOrCreateStateSet()->setRenderBinDetails(0, osgEarth::AUTO_SCALE_BIN );
                }

                // rotational offsets?
                if (sym && (sym->heading().isSet() || sym->pitch().isSet() || sym->roll().isSet()) )
                {
                    osg::Matrix rot;
                    double heading = sym->heading().isSet() ? sym->heading()->eval() : 0.0;
                    double pitch   = sym->pitch().isSet()   ? sym->pitch()->eval()   : 0.0;
                    double roll    = sym->roll().isSet()    ? sym->roll()->eval()    : 0.0;
                    rot.makeRotate( 
                        osg::DegreesToRadians(heading), osg::Vec3(0,0,1),
                        osg::DegreesToRadians(pitch),   osg::Vec3(1,0,0),
                        osg::DegreesToRadians(roll),    osg::Vec3(0,1,0) );
                    this->setLocalRotation( rot.getRotate() );
                }

                this->applyGeneralSymbology( _style );
            }
            else
            {
                OE_WARN << LC << "No model" << std::endl;
            }
        }
        else
        {
            OE_WARN << LC << "Symbology: no URI or model" << std::endl;
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
LocalizedNode( mapNode, conf ),
_dbOptions   ( dbOptions )
{
    _xform = new osg::MatrixTransform();

    conf.getObjIfSet( "style", _style );

    std::string uri = conf.value("url");
    if ( !uri.empty() )
        _style.getOrCreate<ModelSymbol>()->url() = StringExpression(uri);

    init();
}

Config
ModelNode::getConfig() const
{
    Config conf = LocalizedNode::getConfig();
    conf.key() = "model";

    if ( !_style.empty() )
        conf.addObj( "style", _style );

    return conf;
}
