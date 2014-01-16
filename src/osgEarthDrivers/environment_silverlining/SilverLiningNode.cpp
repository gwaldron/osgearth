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

#include "SilverLiningNode"
#include "SilverLiningContext"
#include "SilverLiningSkyDrawable"

#include <osg/Light>
#include <osg/LightSource>
#include <osgEarth/CullingUtils>
#include <SilverLining.h>

#define LC "[SilverLiningNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::SilverLining;

SilverLiningNode::SilverLiningNode(const Map*                 map,
                                   const SilverLiningOptions& options)
{
    // Create a new Light for the Sun.
    osg::Light* light = new osg::Light();
    light->setLightNum( 0 );
    light->setDiffuse( osg::Vec4(1,1,1,1) );
    light->setAmbient( osg::Vec4(0.2f, 0.2f, 0.2f, 1) );
    light->setPosition( osg::Vec4(1, 0, 0, 0) ); // w=0 means infinity
    light->setDirection( osg::Vec3(-1,0,0) );
    
    osg::LightSource* source = new osg::LightSource();
    source->setLight( light );
    source->setReferenceFrame(osg::LightSource::RELATIVE_RF);
    this->addChild( source );

    // The main silver lining data:
    _SL = new SilverLiningContext( options );
    _SL->setLight( light );
    _SL->setSRS  ( map->getSRS() );

    // Geode to hold each of the SL drawables:
    osg::Geode* geode = new osg::Geode();
    geode->setCullingActive( false );
    this->addChild( geode );

    // Draws the sky:
    _skyDrawable = new SkyDrawable( _SL.get() );
    geode->addDrawable( _skyDrawable );

    // SL requires an update pass.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}


SilverLiningNode::~SilverLiningNode()
{
    //nop
}


void
SilverLiningNode::onSetDateTime()
{
    // set the SL local time to UTC/epoch.
    ::SilverLining::LocalTime localTime;
    localTime.SetFromEpochSeconds( getDateTime().asTimeStamp() );
    _SL->getAtmosphere()->GetConditions()->SetTime( localTime );
}


void
SilverLiningNode::traverse(osg::NodeVisitor& nv)
{
    if ( _SL->ready() )
    {
        if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
        {
            _SL->updateLocation();
            _SL->getAtmosphere()->UpdateSkyAndClouds();
            _SL->updateLight();
            _skyDrawable->dirtyBound();
        }
        else if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            // TODO: make this multi-camera safe
            _SL->setCameraPosition( nv.getEyePoint() );
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            _SL->getAtmosphere()->SetCameraMatrix( cv->getModelViewMatrix()->ptr() );
            _SL->getAtmosphere()->SetProjectionMatrix( cv->getProjectionMatrix()->ptr() );
            _SL->getAtmosphere()->CullObjects();
        }
    }
    osgEarth::Util::EnvironmentNode::traverse( nv );
}
