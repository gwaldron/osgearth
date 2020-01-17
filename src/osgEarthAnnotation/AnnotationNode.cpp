/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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

#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/AnnotationSettings>
#include <osgEarthAnnotation/AnnotationUtils>

#include <osgEarth/DepthOffset>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/ShaderUtils>
#include <osgEarth/GLUtils>
#include <osgEarth/CullingUtils>

#include <osg/PolygonOffset>
#include <osg/Depth>

#ifndef GL_CLIP_DISTANCE0
#define GL_CLIP_DISTANCE0 0x3000
#endif

using namespace osgEarth;
using namespace osgEarth::Annotation;

#define LC "[AnnotationNode] "

//-------------------------------------------------------------------

Style AnnotationNode::s_emptyStyle;

//-------------------------------------------------------------------

AnnotationNode::AnnotationNode()
{
    construct();
}

AnnotationNode::AnnotationNode(const Config& conf, const osgDB::Options*)
{
    construct();

    setName(conf.value("name"));
}

void
AnnotationNode::construct()
{
    _dynamic = false;
    _depthAdj = false;
    _priority = false;

    this->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );

    // always draw after the terrain.
    this->getOrCreateStateSet()->setRenderBinDetails( 1, "DepthSortedBin" );

    _altCallback = new AltitudeCullCallback();
    this->addCullCallback(_altCallback);

    _horizonCuller = new HorizonCullCallback();
    this->addCullCallback( _horizonCuller.get() );

    _mapNodeRequired = true;
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}

AnnotationNode::~AnnotationNode()
{
    setMapNode( 0L );
}

void
AnnotationNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        // MapNode auto discovery.
        if (_mapNodeRequired)
        {
            if (getMapNode() == 0L)
            {
                MapNode* mapNode = osgEarth::findInNodePath<MapNode>(nv);
                if (mapNode)
                {
                    setMapNode(mapNode);
                }
            }

            if (getMapNode() != 0L)
            {
                _mapNodeRequired = false;
                ADJUST_UPDATE_TRAV_COUNT(this, -1);
            }
        }
    }
    osg::Group::traverse(nv);
}

void
AnnotationNode::setDefaultLighting( bool lighting )
{
    GLUtils::setLighting(
        getOrCreateStateSet(),
        lighting ? osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
                   osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
}

void
AnnotationNode::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        _mapNode = mapNode;
        
        if ( mapNode )
        {
            if ( mapNode->isGeocentric() )
                _horizonCuller->setHorizon( new Horizon(mapNode->getMapSRS()) );
            else
                _horizonCuller->setEnabled( false );

            static_cast<AltitudeCullCallback*>(_altCallback)->srs() = mapNode->getMapSRS();
        }

		applyStyle( this->getStyle() );
    }
}

bool
AnnotationNode::getHorizonCulling() const
{
    return _horizonCuller->getEnabled();
}

void
AnnotationNode::setHorizonCulling(bool value)
{
    _horizonCuller->setEnabled( value );
}

void
AnnotationNode::setDynamic( bool value )
{
    _dynamic = value;
}

void
AnnotationNode::setPriority(float priority)
{
    _priority = priority;
}

void
AnnotationNode::setDepthAdjustment( bool enable )
{
    if ( enable )
    {
        _doAdapter.setGraph(this);
        _doAdapter.recalculate();
    }
    else
    {
        _doAdapter.setGraph(0L);
    }
    _depthAdj = enable;
}

void
AnnotationNode::applyStyle( const Style& style)
{
    applyRenderSymbology(style);

    // default priority if available.
    const TextSymbol* ts = style.get<TextSymbol>();
    if (ts)
    {
        if (ts->priority().isSet())
        {
            _priority = ts->priority()->eval();
        }
    }
}

void
AnnotationNode::applyRenderSymbology(const Style& style)
{
    const RenderSymbol* render = style.get<RenderSymbol>();
    if ( render )
    {
        if ( render->depthTest().isSet() )
        {
            getOrCreateStateSet()->setMode(
                GL_DEPTH_TEST,
                (render->depthTest() == true? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

        if ( render->lighting().isSet() )
        {
            GLUtils::setLighting(
                getOrCreateStateSet(),
                (render->lighting() == true? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

        if ( render->depthOffset().isSet() )
        {
            _doAdapter.setDepthOffsetOptions( *render->depthOffset() );
            setDepthAdjustment( true );
        }

        if ( render->backfaceCulling().isSet() )
        {
            getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                (render->backfaceCulling() == true? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

#if !( defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE) )
        if ( render->clipPlane().isSet() )
        {
            GLenum mode = GL_CLIP_DISTANCE0 + render->clipPlane().value();
            getOrCreateStateSet()->setMode(mode, 1);
        }
#endif

        if ( supportsRenderBinDetails() && (render->order().isSet() || render->renderBin().isSet()) )
        {
            osg::StateSet* ss = getOrCreateStateSet();
            int binNumber = render->order().isSet() ? (int)render->order()->eval() : ss->getBinNumber();
            std::string binName =
                render->renderBin().isSet() ? render->renderBin().get() :
                ss->useRenderBinDetails() ? ss->getBinName() : "DepthSortedBin";
            ss->setRenderBinDetails(binNumber, binName);
        }

        if ( render->minAlpha().isSet() )
        {
            DiscardAlphaFragments().install( getOrCreateStateSet(), render->minAlpha().value() );
        }
        

        if ( render->transparent() == true )
        {
            osg::StateSet* ss = getOrCreateStateSet();
            ss->setRenderingHint( ss->TRANSPARENT_BIN );
        }
        
        if (render->decal() == true)
        {
            getOrCreateStateSet()->setAttributeAndModes(
                new osg::PolygonOffset(-1,-1), 1);

            getOrCreateStateSet()->setAttributeAndModes(
                new osg::Depth(osg::Depth::LEQUAL, 0, 1, false));
        }

        if (render->maxAltitude().isSet())
        {
            AltitudeCullCallback* cc = static_cast<AltitudeCullCallback*>(_altCallback);
            cc->maxAltitude() = render->maxAltitude()->as(Units::METERS);
        }
        else
        {
            AltitudeCullCallback* cc = static_cast<AltitudeCullCallback*>(_altCallback);
            cc->maxAltitude().unset();
        }
    }
}
