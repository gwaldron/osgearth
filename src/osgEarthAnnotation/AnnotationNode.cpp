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
        void onTileAdded( const TileKey& key, osg::Node* tile, TerrainCallbackContext& context )
        {
            AnnotationNode* anno = static_cast<AnnotationNode*>(context.getClientData());
            anno->reclamp( key, tile, context.getTerrain() );
        }
    };
}  }

//-------------------------------------------------------------------

AnnotationNode::AnnotationNode(MapNode* mapNode) :
_mapNode    ( mapNode ),
_dynamic    ( false ),
_autoclamp  ( false ),
_depthAdj   ( false ),
_activeDs   ( 0L )
{
    //nop
}

AnnotationNode::~AnnotationNode()
{
    osg::ref_ptr<MapNode> mapNodeSafe = _mapNode.get();
    if ( mapNodeSafe.get() )
    {
        mapNodeSafe->getTerrain()->removeTerrainCallbacksWithClientData(this);
    }
}

void
AnnotationNode::setAnnotationData( AnnotationData* data )
{
    _annoData = data;
}

void
AnnotationNode::setDynamic( bool value )
{
    _dynamic = value;
}

void
AnnotationNode::setAutoClamp( bool value )
{
    osg::ref_ptr<MapNode> mapNode_safe = _mapNode.get();
    if ( mapNode_safe.valid() )
    {
        if ( !_autoclamp && value )
        {
            setDynamic( true );

            if ( AnnotationSettings::getContinuousClamping() )
            {
                mapNode_safe->getTerrain()->addTerrainCallback(new AutoClampCallback(), this);
            }
        }
        else if ( _autoclamp && !value )
        {
            mapNode_safe->getTerrain()->removeTerrainCallbacksWithClientData(this);
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
                getOrCreateStateSet()->addUniform( DepthOffsetUtils::createMinOffsetUniform(this) );
            }
        }
    }
}

void
AnnotationNode::setDepthAdjustment( bool enable )
{
    if ( enable )
    {
        osg::StateSet* s = this->getOrCreateStateSet();
        osg::Program* daProgram = DepthOffsetUtils::getOrCreateProgram(); // cached, not a leak.
        osg::Program* p = dynamic_cast<osg::Program*>( s->getAttribute(osg::StateAttribute::PROGRAM) );
        if ( !p || p != daProgram )
            s->setAttributeAndModes( daProgram, osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE );

        s->addUniform( DepthOffsetUtils::createMinOffsetUniform(this) );
        s->addUniform( DepthOffsetUtils::getIsNotTextUniform() );
    }
    else if ( this->getStateSet() )
    {
        this->getStateSet()->removeAttribute(osg::StateAttribute::PROGRAM);
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
    osg::ref_ptr<MapNode> mapNode_safe = _mapNode.get();
    if ( mapNode_safe.valid() )
    {
        // find the terrain height at the map point:
        double hamsl;
        if (mapNode_safe->getTerrain()->getHeight(mapPoint.x(), mapPoint.y(), &hamsl, 0L, patch))
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

osgEarth::MapNode*
AnnotationNode::getMapNode() const
{
    return _mapNode.get();
}

bool
AnnotationNode::supportsAutoClamping( const Style& style ) const
{
    return
        !style.has<ExtrusionSymbol>()  &&
        !style.has<MarkerSymbol>()     &&
        style.has<AltitudeSymbol>()    &&
        (style.get<AltitudeSymbol>()->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN ||
         style.get<AltitudeSymbol>()->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN);
}

void
AnnotationNode::configureForAltitudeMode( const AltitudeMode& mode )
{
    setAutoClamp(
        mode == ALTMODE_RELATIVE ||
        (_altitude.valid() && _altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN) );
}

void
AnnotationNode::applyStyle( const Style& style, bool noClampHint )
{
    if ( !noClampHint && supportsAutoClamping(style) )
    {
        _altitude = style.get<AltitudeSymbol>();
        setAutoClamp( true );
    }
}
