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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <SilverLining.h>
#include "SilverLiningCloudsDrawable"
#include "SilverLiningContext"
#include "SilverLiningContextNode"
#include <osgEarth/SpatialReference>

#ifndef SILVERLINING_MAJOR_VERSION
#include <Version.h>
#endif

#undef  LC
#define LC "[SilverLining:SkyDrawable] "

using namespace osgEarth::SilverLining;


CloudsDrawable::CloudsDrawable(SilverLiningContextNode* contexNode) :
_SL(contexNode->getSLContext()),
_contextNode(contexNode)
{
    // call this to ensure draw() gets called every frame.
    setSupportsDisplayList( false );
    
    // not MT-safe (camera updates, etc)
    this->setDataVariance(osg::Object::DYNAMIC);
    
    setName("SilverLining::CloudsDrawable");
}

void
CloudsDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	osg::Camera* camera = renderInfo.getCurrentCamera();
#ifndef SL_USE_CULL_MASK
	if(_contextNode->getTargetCamera() == camera)
#endif
	{
	if ( _SL->ready())
	{
	    osg::State* state = renderInfo.getState();

        // adapt the SL shaders so they can accept OSG uniforms:
        osgEarth::NativeProgramAdapterCollection& adapters = _adapters[ state->getContextID() ]; // thread safe.
        if ( adapters.empty() )
        {
            adapters.push_back( new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetSkyShader(), NULL, "SkyShader"));
            adapters.push_back( new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetBillboardShader(), NULL, "BillboardShader"));
            adapters.push_back( new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetStarShader(), NULL, "StarShader"));
            adapters.push_back( new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetPrecipitationShader(), NULL, "PrecipitationShader"));
            //adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetAtmosphericLimbShader(), NULL, "AtmosphericLimbShader"));

            SL_VECTOR(unsigned) handles = _SL->getAtmosphere()->GetActivePlanarCloudShaders();
            for(int i=0; i<handles.size(); ++i)          
                adapters.push_back( new osgEarth::NativeProgramAdapter(state, handles[i], NULL, "PlanarCloudShader"));
        }
        adapters.apply( state );

        // invoke the user callback if it exists
        if (_SL->getCallback())
            _SL->getCallback()->onDrawClouds(_SL->getAtmosphereWrapper());

        renderInfo.getState()->disableAllVertexArrays();

#if ((SILVERLINING_MAJOR_VERSION >= 5) && (SILVERLINING_MINOR_VERSION >= 30))
        _SL->getAtmosphere()->DrawObjects(true, true, true, 0.0f, false, 0, true, true, true, _SL->getSRS()->isGeographic());
#else
        _SL->getAtmosphere()->DrawObjects(true, true, true);
#endif

        // Restore the GL state to where it was before.
        state->dirtyAllVertexArrays();
        state->dirtyAllAttributes();

        state->apply();
    }
	}
}

osg::BoundingBox
CloudsDrawable::computeBoundingBox() const
{
    osg::BoundingBox cloudBoundBox;
    if ( !_SL->ready() )
        return cloudBoundBox;
    
    double minX, minY, minZ, maxX, maxY, maxZ;
    _SL->getAtmosphere()->GetCloudBounds( minX, minY, minZ, maxX, maxY, maxZ );
    cloudBoundBox.set( osg::Vec3d(minX, minY, minZ), osg::Vec3d(maxX, maxY, maxZ) );
    return cloudBoundBox;
}
