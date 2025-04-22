/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/ModelNode>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/GeoPositionNodeAutoScaler>
#include <osgEarth/Style>
#include <osgEarth/InstanceSymbol>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/NodeUtils>

#define LC "[ModelNode] "

using namespace osgEarth;


//------------------------------------------------------------------------


ModelNode::ModelNode(MapNode*              mapNode,
                     const Style&          style,
                     const osgDB::Options* readOptions ) :
GeoPositionNode(),
_style( style ),
_readOptions( readOptions )
{
    construct();
    setMapNode(mapNode);
    compileModel();
}

void
ModelNode::construct()
{
    // By default, generate shaders on loaded models.
    _shaderPolicy.init(SHADERPOLICY_GENERATE);
}

void
ModelNode::setStyle(const Style& style)
{
    _style = style;
    compileModel();
}

void
ModelNode::compileModel()
{
    osgEarth::clearChildren( getPositionAttitudeTransform() );

    osg::ref_ptr<const ModelSymbol> sym = _style.get<ModelSymbol>();

    if ( sym.valid() )
    {
        if ( ( sym->url().isSet() ) || (sym->getModel() != NULL) )
        {
            // Try to get a model from symbol
            osg::ref_ptr<osg::Node> node = sym->getModel();

            // Try to get a model from URI
            if ( !node.valid() )
            {
                URI uri = sym->url()->evalURI();

                if ( sym->uriAliasMap()->empty() )
                {
                    node = uri.getNode( _readOptions.get() );
                }
                else
                {
                    // install an alias map if there's one in the symbology.
                    osg::ref_ptr<osgDB::Options> tempOptions = Registry::instance()->cloneOrCreateOptions(_readOptions.get());
                    tempOptions->setReadFileCallback( new URIAliasMapReadCallback(*sym->uriAliasMap(), uri.full()) );
                    node = uri.getNode( tempOptions.get() );
                }

                if ( !node.valid() )
                {
                    OE_WARN << LC << "No model and failed to load data from " << uri.full() << std::endl;
                }
            }

            if ( node.valid() )
            {
                if (_shaderPolicy == SHADERPOLICY_GENERATE)
                {
                    // generate shader code for the loaded model:
                    Registry::shaderGenerator().run(
                        node.get(),
                        "osgEarth.ModelNode",
                        Registry::stateSetCache());
                }
                else if (_shaderPolicy == SHADERPOLICY_DISABLE)
                {
                    node->getOrCreateStateSet()->setAttributeAndModes(
                        new osg::Program(),
                        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
                }

                // install clamping/draping if necessary
                node = AnnotationUtils::installOverlayParent( node.get(), _style );

                getPositionAttitudeTransform()->addChild( node.get() );

                osg::Vec3d scale(1, 1, 1);

                if ( sym->scale().isSet() )
                {
                    double s = sym->scale()->eval();
                    scale.set(s, s, s);
                }
                if ( sym->scaleX().isSet() )
                    scale.x() = sym->scaleX()->eval();

                if ( sym->scaleY().isSet() )
                    scale.y() = sym->scaleY()->eval();

                if ( sym->scaleZ().isSet() )
                    scale.z() = sym->scaleZ()->eval();

                if (scale != osg::Vec3d(1,1,1))
                {
                    getPositionAttitudeTransform()->setScale( scale );
                }

                // auto scaling?
                if ( sym->autoScale() == true )
                {
                    this->setCullingActive(false);
                    this->addCullCallback( new GeoPositionNodeAutoScaler( osg::Vec3d(1,1,1), sym->minAutoScale().value(), sym->maxAutoScale().value() ));
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

                    getPositionAttitudeTransform()->setAttitude( rot.getRotate() );
                }

                this->applyRenderSymbology( _style );
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

OSGEARTH_REGISTER_ANNOTATION( model, osgEarth::ModelNode );


ModelNode::ModelNode(const Config& conf, const osgDB::Options* readOptions) :
GeoPositionNode(conf, readOptions),
_readOptions(readOptions)
{
    construct();

    conf.get( "style", _style );

    std::string uri = conf.value("url");
    if ( !uri.empty() )
        _style.getOrCreate<ModelSymbol>()->url() = StringExpression(uri);

    conf.get("shader_policy", "disable", _shaderPolicy, SHADERPOLICY_DISABLE);
    conf.get("shader_policy", "inherit", _shaderPolicy, SHADERPOLICY_INHERIT);
    conf.get("shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE);

    compileModel();
}

Config
ModelNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "model";

    if ( !_style.empty() )
        conf.set( "style", _style );

    conf.set("shader_policy", "disable", _shaderPolicy, SHADERPOLICY_DISABLE);
    conf.set("shader_policy", "inherit", _shaderPolicy, SHADERPOLICY_INHERIT);
    conf.set("shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE);

    return conf;
}
