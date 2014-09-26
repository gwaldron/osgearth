/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/AnnotationSettings>
#include <osgEarthAnnotation/AnnotationUtils>

#include <osgEarth/DepthOffset>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Annotation;

//-------------------------------------------------------------------

namespace osgEarth { namespace Annotation
{
    struct AutoClampCallback : public TerrainCallback
    {
        AutoClampCallback( AnnotationNode* annotation):
        _annotation( annotation )
        {
        }

        void onTileAdded( const TileKey& key, osg::Node* tile, TerrainCallbackContext& context )
        {
            _annotation->reclamp( key, tile, context.getTerrain() );
        }

        AnnotationNode* _annotation;
    };
}  }

//-------------------------------------------------------------------

Style AnnotationNode::s_emptyStyle;

//-------------------------------------------------------------------

AnnotationNode::AnnotationNode(MapNode* mapNode) :
_mapNode    ( mapNode ),
_dynamic    ( false ),
_autoclamp  ( false ),
_depthAdj   ( false ),
_activeDs   ( 0L )
{
    //Note: Cannot call setMapNode() here because it's a virtual function.
    //      Each subclass will be separately responsible at ctor time.

    // always blend.
    this->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
}

AnnotationNode::AnnotationNode(MapNode* mapNode, const Config& conf) :
_mapNode    ( mapNode ),
_dynamic    ( false ),
_autoclamp  ( false ),
_depthAdj   ( false ),
_activeDs   ( 0L )
{
    if ( conf.hasValue("lighting") )
    {
        bool lighting = conf.value<bool>("lighting", false);
        setLightingIfNotSet( lighting );
    }

    if ( conf.hasValue("backface_culling") )
    {
        bool culling = conf.value<bool>("backface_culling", false);
        getOrCreateStateSet()->setMode( GL_CULL_FACE, (culling?1:0) | osg::StateAttribute::OVERRIDE );
    }

    if ( conf.hasValue("blending") )
    {
        bool blending = conf.value<bool>("blending", false);
        getOrCreateStateSet()->setMode( GL_BLEND, (blending?1:0) | osg::StateAttribute::OVERRIDE );
    }
    else
    {
        // blend by default.
        this->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    }

}

AnnotationNode::~AnnotationNode()
{
    setMapNode( 0L );
}

void
AnnotationNode::setLightingIfNotSet( bool lighting )
{
    osg::StateSet* ss = this->getOrCreateStateSet();

    if ( ss->getMode(GL_LIGHTING) == osg::StateAttribute::INHERIT )
    {
        this->getOrCreateStateSet()->setMode(
            GL_LIGHTING,
            (lighting ? 1 : 0) | osg::StateAttribute::OVERRIDE );
    }
}

void
AnnotationNode::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        // relocate the auto-clamping callback, if there is one:
        osg::ref_ptr<MapNode> oldMapNode = _mapNode.get();
        if ( oldMapNode.valid() )
        {
            if ( _autoClampCallback )
            {
                oldMapNode->getTerrain()->removeTerrainCallback( _autoClampCallback.get() );
                if ( mapNode )
                    mapNode->getTerrain()->addTerrainCallback( _autoClampCallback.get() );
            }
        }		

        _mapNode = mapNode;

		applyStyle( this->getStyle() );
    }
}

void
AnnotationNode::setAnnotationData( AnnotationData* data )
{
    _annoData = data;
}

AnnotationData*
AnnotationNode::getOrCreateAnnotationData()
{
    if ( !_annoData.valid() )
    {
        setAnnotationData( new AnnotationData() );
    }
    return _annoData.get();
}

void
AnnotationNode::setDynamic( bool value )
{
    _dynamic = value;
}

void
AnnotationNode::setCPUAutoClamping( bool value )
{
    if ( getMapNode() )
    {
        if ( !_autoclamp && value )
        {
            setDynamic( true );

            if ( AnnotationSettings::getContinuousClamping() )
            {
                _autoClampCallback = new AutoClampCallback( this );
                getMapNode()->getTerrain()->addTerrainCallback( _autoClampCallback.get() );
            }
        }
        else if ( _autoclamp && !value && _autoClampCallback.valid())
        {
            getMapNode()->getTerrain()->removeTerrainCallback( _autoClampCallback );
            _autoClampCallback = 0;
        }

        _autoclamp = value;
        
        if ( _autoclamp && AnnotationSettings::getApplyDepthOffsetToClampedLines() )
        {
            if ( !_depthAdj )
            {
                // verify that the geometry if polygon-less:
                bool wantDepthAdjustment = false;
                PrimitiveSetTypeCounter counter;
                this->accept(counter);
                if ( counter._polygon == 0 && (counter._line > 0 || counter._point > 0) )
                {
                    wantDepthAdjustment = true;
                }

                setDepthAdjustment( wantDepthAdjustment );
            }
            else
            {
                // update depth adjustment calculation
                //getOrCreateStateSet()->addUniform( DepthOffsetUtils::createMinOffsetUniform(this) );
            }
        }
    }
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

bool
AnnotationNode::makeAbsolute( GeoPoint& mapPoint, osg::Node* patch ) const
{
    // in terrain-clamping mode, force it to HAT=0:
    if ( _altitude.valid() && (
        _altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN || 
        _altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN) )
    {
        mapPoint.altitudeMode() = ALTMODE_RELATIVE;
        //If we're clamping to the terrain
        if (_altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN)
        {
            mapPoint.z() = 0.0;
        }
    }

    // if the point's already absolute and we're not clamping it, nop.
    if ( mapPoint.altitudeMode() == ALTMODE_ABSOLUTE )
    {
        return true;
    }

    // calculate the absolute Z of the map point.
    if ( getMapNode() )
    {
        // find the terrain height at the map point:
        double hamsl;
        if (getMapNode()->getTerrain()->getHeight(patch, mapPoint.getSRS(), mapPoint.x(), mapPoint.y(), &hamsl, 0L))
        {
            // apply any scale/offset in the symbology:
            if ( _altitude.valid() )
            {
                if ( _altitude->verticalScale().isSet() )
                    hamsl *= _altitude->verticalScale()->eval();
                if ( _altitude->verticalOffset().isSet() )
                    hamsl += _altitude->verticalOffset()->eval();
            }
            mapPoint.z() += hamsl;
        }
        mapPoint.altitudeMode() = ALTMODE_ABSOLUTE;
        return true;
    }

    return false;
}


void
AnnotationNode::installDecoration( const std::string& name, Decoration* ds )
{
    if ( _activeDs )
    {
        clearDecoration();
    }

    if ( ds == 0L )
    {
        _dsMap.erase( name );
    }
    else
    {
        _dsMap[name] = ds->copyOrClone();
    }
}

void
AnnotationNode::uninstallDecoration( const std::string& name )
{
    clearDecoration();
    _dsMap.erase( name );
}

const std::string&
AnnotationNode::getDecoration() const
{
    return _activeDsName;
}

void
AnnotationNode::setDecoration( const std::string& name )
{
    // already active?
    if ( _activeDs && _activeDsName == name )
        return;

    // is a different one active? if so kill it
    if ( _activeDs )
        clearDecoration();

    // try to find and enable the new one
    DecorationMap::iterator i = _dsMap.find(name);
    if ( i != _dsMap.end() )
    {
        Decoration* ds = i->second.get();
        if ( ds )
        {
            if ( this->accept(ds, true) ) 
            {
                _activeDs = ds;
                _activeDsName = name;
            }
        }
    }
}

void
AnnotationNode::clearDecoration()
{
    if ( _activeDs )
    {
        this->accept(_activeDs, false);
        _activeDs = 0L;
        _activeDsName = "";
    }
}

bool
AnnotationNode::hasDecoration( const std::string& name ) const
{
    return _dsMap.find(name) != _dsMap.end();
}

osg::Group*
AnnotationNode::getChildAttachPoint()
{
    osg::Transform* t = osgEarth::findTopMostNodeOfType<osg::Transform>(this);
    return t ? (osg::Group*)t : (osg::Group*)this;
}

bool
AnnotationNode::supportsAutoClamping( const Style& style ) const
{
    return
        !style.has<ExtrusionSymbol>()  &&
        !style.has<InstanceSymbol>()   &&
        !style.has<MarkerSymbol>()     &&  // backwards-compability
        style.has<AltitudeSymbol>()    &&
        (style.get<AltitudeSymbol>()->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN ||
         style.get<AltitudeSymbol>()->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN);
}

void
AnnotationNode::configureForAltitudeMode( const AltitudeMode& mode )
{
    setCPUAutoClamping(
        mode == ALTMODE_RELATIVE ||
        (_altitude.valid() && _altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN) );
}

void
AnnotationNode::applyStyle( const Style& style)
{
    if ( supportsAutoClamping(style) )
    {
        _altitude = style.get<AltitudeSymbol>();
        setCPUAutoClamping( true );
    }
    applyGeneralSymbology(style);
}

void
AnnotationNode::applyGeneralSymbology(const Style& style)
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
            getOrCreateStateSet()->setMode(
                GL_LIGHTING,
                (render->lighting() == true? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

        if ( render->depthOffset().isSet() ) // && !_depthAdj )
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

        if ( render->clipPlane().isSet() )
        {
            GLenum mode = GL_CLIP_PLANE0 + render->clipPlane().value();
            getOrCreateStateSet()->setMode(mode, 1);
        }

        if ( render->minAlpha().isSet() )
        {
            DiscardAlphaFragments().install( getOrCreateStateSet(), render->minAlpha().value() );
        }
    }
}
