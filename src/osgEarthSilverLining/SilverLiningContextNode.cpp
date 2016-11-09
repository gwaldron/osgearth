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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <SilverLining.h>

#include "SilverLiningContextNode"
#include "SilverLiningNode"
#include "SilverLiningContext"
#include "SilverLiningSkyDrawable"
#include "SilverLiningCloudsDrawable"

#include <osg/Light>
#include <osg/LightSource>
#include <osgEarth/CullingUtils>

#undef  LC
#define LC "[SilverLiningContextNode] "

using namespace osgEarth::SilverLining;

SilverLiningContextNode::SilverLiningContextNode(SilverLiningNode* node,
								   osg::Camera*               camera,
                                   osg::Light*                light,
	                               const osgEarth::SpatialReference*    mapSRS,
                                   const SilverLiningOptions& options,
                                   Callback*                  callback) :
_silverLiningNode (node),
_camera           (camera),
_options          (options),
_lastAltitude(DBL_MAX)
{
    // The main silver lining data:
    _SL = new SilverLiningContext( options );
    _SL->setLight( light);
    _SL->setSRS  ( mapSRS );
    _SL->setCallback( callback );
    _SL->setMinimumAmbient( node->getMinimumAmbient() );

    // Geode to hold each of the SL drawables:
    _geode = new osg::Geode();
    _geode->setCullingActive( false );

    // Draws the sky before everything else
    _skyDrawable = new SkyDrawable(this);
    _skyDrawable->getOrCreateStateSet()->setRenderBinDetails( -99, "RenderBin" );
    _geode->addDrawable(_skyDrawable.get());

    // Clouds draw after everything else
    _cloudsDrawable = new CloudsDrawable(this);
    _cloudsDrawable->getOrCreateStateSet()->setRenderBinDetails( 99, "DepthSortedBin" );
    _geode->addDrawable(_cloudsDrawable.get());

    // SL requires an update pass.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);

    // initialize date/time
    onSetDateTime();
}


SilverLiningContextNode::~SilverLiningContextNode()
{

}

void
SilverLiningContextNode::onSetDateTime()
{
    // set the SL local time to UTC/epoch.
    ::SilverLining::LocalTime utcTime;
    utcTime.SetFromEpochSeconds( _silverLiningNode->getDateTime().asTimeStamp() );
    _SL->getAtmosphere()->GetConditions()->SetTime( utcTime );
}

void
SilverLiningContextNode::onSetMinimumAmbient()
{
    _SL->setMinimumAmbient( _silverLiningNode->getMinimumAmbient() );
}

void
SilverLiningContextNode::traverse(osg::NodeVisitor& nv)
{
    if ( _SL && _SL->ready() )
    {
        if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
        {
			int frameNumber = nv.getFrameStamp()->getFrameNumber();
            _skyDrawable->dirtyBound();

            if( _cloudsDrawable )
            {
                if ( _lastAltitude <= *_options.cloudsMaxAltitude() )
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
			osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
			osg::Camera* camera  = cv->getCurrentCamera();
			if ( camera )
			{
#ifndef SL_USE_CULL_MASK
				//Check if this is the target camera for this context
				if(getTargetCamera() == camera)
#endif
     			{

					// TODO: make this multi-camera safe
					_SL->setCameraPosition( nv.getEyePoint() );
					osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
					_SL->getAtmosphere()->SetCameraMatrix( cv->getModelViewMatrix()->ptr() );
					_SL->getAtmosphere()->SetProjectionMatrix( cv->getProjectionMatrix()->ptr() );

					_lastAltitude = _SL->getSRS()->isGeographic() ?
						cv->getEyePoint().length() - _SL->getSRS()->getEllipsoid()->getRadiusEquator() :
					cv->getEyePoint().z();

					_SL->updateLocation();
					_SL->updateLight();
					//_SL->getAtmosphere()->UpdateSkyAndClouds();
					//_SL->getAtmosphere()->CullObjects();
				}
			}
        }
    }

    if ( _geode.valid() )
    {
        _geode->accept(nv);
    }
}