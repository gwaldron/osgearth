/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <SilverLining.h>
#include "SilverLiningCloudsDrawable"
#include "SilverLiningContext"
#include "SilverLiningContextNode"
#include <osgEarth/SpatialReference>
#include <osgEarth/GLUtils>

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
    OE_GL_ZONE;

	osg::Camera* camera = renderInfo.getCurrentCamera();
#ifndef SL_USE_CULL_MASK
	if(_contextNode->getTargetCamera() == camera)
#endif
	{
	if ( _SL->ready())
	{
	    osg::State* state = renderInfo.getState();

        // adapt the SL shaders so they can accept OSG uniforms:
        auto cid = GLUtils::getSharedContextID(*state);
        osgEarth::NativeProgramAdapterCollection& adapters = _adapters[cid]; // thread safe.
        if (adapters.empty())
        {
            adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetSkyShader(), {}, "SkyShader"));
            adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetBillboardShader(), {}, "BillboardShader"));
            adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetStarShader(), {}, "StarShader"));
            adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetPrecipitationShader(), {}, "PrecipitationShader"));

            if (_SL->getAtmosphere()->GetAtmosphericLimbShader())
                adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetAtmosphericLimbShader(), {}, "AtmosphericLimbShader"));

#if ((SILVERLINING_MAJOR_VERSION > 6) || (SILVERLINING_MAJOR_VERSION == 6 && SILVERLINING_MINOR_VERSION >= 24))
            adapters.push_back(new osgEarth::NativeProgramAdapter(state, _SL->getAtmosphere()->GetBillboardShaderInstanced(), {}, "BillboardShaderInstanced"));
#endif

            for (auto& handle : _SL->getAtmosphere()->GetActivePlanarCloudShaders())
                adapters.push_back(new osgEarth::NativeProgramAdapter(state, handle, {}, Stringify() << "PlanarCloudShader" << handle));

#if (SILVERLINING_MAJOR_VERSION >= 6)
            for (auto& handle : _SL->getAtmosphere()->GetUserShaders())
                adapters.push_back(new osgEarth::NativeProgramAdapter(state, handle, {}, Stringify() << "UserShader" << handle));
#endif
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
