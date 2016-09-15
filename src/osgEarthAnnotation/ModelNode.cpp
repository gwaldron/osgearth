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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarthAnnotation/ModelNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/GeoPositionNodeAutoScaler>
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
GeoPositionNode    ( mapNode, GeoPoint() ),
_style       ( style ),
_dbOptions   ( dbOptions )
{
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
    osgEarth::clearChildren( getPositionAttitudeTransform() );

    osg::ref_ptr<const ModelSymbol> sym = _style.get<ModelSymbol>();
    
    // backwards-compatibility: support for MarkerSymbol (deprecated)
    if ( !sym.valid() && _style.has<MarkerSymbol>() )
    {
        OE_WARN << LC << "MarkerSymbol is deprecated, please remove it\n";
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
            if ( !node.valid() )
            {
                URI uri = sym->url()->evalURI();

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

                if ( !node.valid() )
                {
                    OE_WARN << LC << "No model and failed to load data from " << uri.full() << std::endl;
                }
            }

            if ( node.valid() )
            {
                if ( Registry::capabilities().supportsGLSL() )
                {
                    // generate shader code for the loaded model:
                    Registry::shaderGenerator().run(
                        node.get(),
                        "osgEarth.ModelNode",
                        Registry::stateSetCache() );
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

                getPositionAttitudeTransform()->setScale( scale );

                // auto scaling?
                if ( sym->autoScale() == true )
                {
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

OSGEARTH_REGISTER_ANNOTATION( model, osgEarth::Annotation::ModelNode );


ModelNode::ModelNode(MapNode* mapNode, const Config& conf, const osgDB::Options* dbOptions) :
GeoPositionNode    ( mapNode, conf ),
_dbOptions   ( dbOptions )
{
    conf.getObjIfSet( "style", _style );

    std::string uri = conf.value("url");
    if ( !uri.empty() )
        _style.getOrCreate<ModelSymbol>()->url() = StringExpression(uri);

    init();
}

Config
ModelNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "model";

    if ( !_style.empty() )
        conf.addObj( "style", _style );

    return conf;
}
