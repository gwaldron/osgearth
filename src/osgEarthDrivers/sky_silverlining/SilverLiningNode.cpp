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

#include "SilverLiningNode"
#include "SilverLiningContext"
#include "SilverLiningSkyDrawable"
#include "SilverLiningCloudsDrawable"

#include <osg/Light>
#include <osg/LightSource>
#include <osgEarth/CullingUtils>
#include <SilverLining.h>

#define LC "[SilverLiningNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::SilverLining;

SilverLiningNode::SilverLiningNode(const Map*                 map,
                                   const SilverLiningOptions& options) :
_options     (options),
_lastAltitude(DBL_MAX)
{
    // Create a new Light for the Sun.
    _light = new osg::Light();
    _light->setLightNum( 0 );
    _light->setDiffuse( osg::Vec4(1,1,1,1) );
    _light->setAmbient( osg::Vec4(0.2f, 0.2f, 0.2f, 1) );
    _light->setPosition( osg::Vec4(1, 0, 0, 0) ); // w=0 means infinity
    _light->setDirection( osg::Vec3(-1,0,0) );

    osg::LightSource* source = new osg::LightSource();
    source->setLight( _light.get() );
    source->setReferenceFrame(osg::LightSource::RELATIVE_RF);
    this->addChild( source );

    // The main silver lining data:
    _SL = new SilverLiningContext( options );
    _SL->setLight( _light.get() );
    _SL->setSRS  ( map->getSRS() );

    // Geode to hold each of the SL drawables:
    _geode = new osg::Geode();
    _geode->setCullingActive( false );
    this->addChild( _geode );

    // Draws the sky:
    _skyDrawable = new SkyDrawable( _SL.get() );
    //_skyDrawable->getOrCreateStateSet()->setRenderBinDetails( 98, "RenderBin" );
    _geode->addDrawable( _skyDrawable );

    // Clouds
    _cloudsDrawable = new CloudsDrawable( _SL.get() );
    //_cloudsDrawable->getOrCreateStateSet()->setRenderBinDetails( 99, "RenderBin" );
    _geode->addDrawable( _cloudsDrawable.get() );

    // scene lighting
    osg::StateSet* stateset = this->getOrCreateStateSet();
    _lighting = new PhongLightingEffect();
    _lighting->setCreateLightingUniform( false );
    _lighting->attach( stateset );

    // ensure it's depth sorted and draws after the terrain
    //stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //getOrCreateStateSet()->setRenderBinDetails( 100, "RenderBin" );

    // SL requires an update pass.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);

    // initialize date/time
    onSetDateTime();
}


SilverLiningNode::~SilverLiningNode()
{
    if ( _lighting.valid() )
        _lighting->detach();
}

void
SilverLiningNode::attach(osg::View* view, int lightNum)
{
    _light->setLightNum( lightNum );
    view->setLight( _light.get() );
    view->setLightingMode( osg::View::SKY_LIGHT );
}

void
SilverLiningNode::onSetDateTime()
{
    // set the SL local time to UTC/epoch.
    ::SilverLining::LocalTime utcTime;
    utcTime.SetFromEpochSeconds( getDateTime().asTimeStamp() );
    _SL->getAtmosphere()->GetConditions()->SetTime( utcTime );
}


void
SilverLiningNode::traverse(osg::NodeVisitor& nv)
{
    if ( _SL && _SL->ready() )
    {
        if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
        {
			int frameNumber = nv.getFrameStamp()->getFrameNumber();
            _SL->updateLocation();
            _SL->updateLight();
            _SL->getAtmosphere()->UpdateSkyAndClouds();
            _skyDrawable->dirtyBound();

            if( _cloudsDrawable )
            {
                if (_lastAltitude <= *_options.cloudsMaxAltitude() )
                {
                    if ( _cloudsDrawable->getNumParents() == 0 )
                        _geode->addDrawable( _cloudsDrawable.get() );
                    
                    _cloudsDrawable->dirtyBound();
                }
                else
                {
                    if ( _cloudsDrawable->getNumParents() > 0 )
                        _geode->removeDrawable( _cloudsDrawable.get() );
                }
            }
        }
        else if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            // TODO: make this multi-camera safe
            _SL->setCameraPosition( nv.getEyePoint() );
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            _SL->getAtmosphere()->SetCameraMatrix( cv->getModelViewMatrix()->ptr() );
            _SL->getAtmosphere()->SetProjectionMatrix( cv->getProjectionMatrix()->ptr() );

			_lastAltitude = _SL->getSRS()->isGeographic() ?
				cv->getEyePoint().length() - _SL->getSRS()->getEllipsoid()->getRadiusEquator() :
				cv->getEyePoint().z();

			if (_lastAltitude <= *_options.cloudsMaxAltitude() )
			{
				_SL->getAtmosphere()->CullObjects();
			}
        }
    }
    osgEarth::Util::SkyNode::traverse( nv );
}
