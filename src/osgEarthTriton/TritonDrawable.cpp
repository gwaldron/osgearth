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
#include "TritonContext"
#include "TritonDrawable"
#include "TritonHeightMap"
#include <Version.h>
#include <osg/MatrixTransform>
#include <osg/FrameBufferObject>
#include <osg/Depth>

#include <osgEarth/SpatialReference>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Random>

#undef  LC
#define LC "[TritonDrawable] "

//#define DEBUG_HEIGHTMAP

using namespace osgEarth::Triton;


TritonDrawable::TritonDrawable(TritonContext* TRITON) :
_TRITON(TRITON)
{
    // call this to ensure draw() gets called every frame.
    setSupportsDisplayList( false );
    setUseVertexBufferObjects( false );

    // dynamic variance prevents update/cull overlap when drawing this
    setDataVariance( osg::Object::DYNAMIC );
}

TritonDrawable::~TritonDrawable()
{
    //nop
}

void
TritonDrawable::setMaskLayer(const osgEarth::ImageLayer* layer)
{
    _maskLayer = layer;
}

void
TritonDrawable::setHeightMapGenerator(TritonHeightMap* value)
{
    _heightMapGenerator = value;
}

void
TritonDrawable::setPlanarReflectionMap(osg::Texture2D* map)
{ 
    _planarReflectionMap = map;
}

void
TritonDrawable::setPlanarReflectionProjection(osg::RefMatrix* proj)
{
    _planarReflectionProjection = proj;
}

osg::BoundingBox
TritonDrawable::computeBoundingBox() const
{
    return osg::BoundingBox();
}

namespace {

// Wrapper around Ocean->GetShaderObject() to account for API changes from Triton 3.x to 4.x
GLint
getOceanShader(::Triton::Ocean* ocean, ::Triton::Shaders shaderProgram, void* context, const ::Triton::Camera* camera)
{
#if (TRITON_MAJOR_VERSION >= 4)
  return (GLint)ocean->GetShaderObject( shaderProgram, context, camera );
#else
  return (GLint)ocean->GetShaderObject( shaderProgram );
#endif
}

}

void
TritonDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State* state = renderInfo.getState();

    state->disableAllVertexArrays();

    _TRITON->initialize( renderInfo );
    if ( !_TRITON->ready() )
        return;

    // Configure the height map generator.
    // If configuration fails, attempt to continue without a heightmap.
    if (_heightMapGenerator.valid())
    {
        bool configOK = _heightMapGenerator->configure(_TRITON->getHeightMapSize(), *state);
        if (configOK == false)
        {
            _heightMapGenerator = 0L;
            OE_WARN << LC << "Failed to establish a legal FBO configuration; disabling height map generator!" << std::endl;
        }
    }

    ::Triton::Environment* environment = _TRITON->getEnvironment();

    // Find or create the Triton camera for this OSG camera:
    CameraLocal& local = _local.get(renderInfo.getCurrentCamera());
    if (local._tritonCam == 0L)
    {
        local._tritonCam = environment->CreateCamera();
        local._tritonCam->SetName(renderInfo.getCurrentCamera()->getName().c_str());
    }
    ::Triton::Camera* tritonCam = local._tritonCam;

    osgEarth::NativeProgramAdapterCollection& adapters = _adapters[ state->getContextID() ];
    if ( adapters.empty() )
    {
        OE_INFO << LC << "Initializing Triton program adapters" << std::endl;
        const char* prefix = "oe_"; // because, don't forget osg_*
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, getOceanShader(_TRITON->getOcean(), ::Triton::WATER_SURFACE, 0L, tritonCam), prefix, "WATER_SURFACE"));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, getOceanShader(_TRITON->getOcean(), ::Triton::WATER_SURFACE_PATCH, 0L, tritonCam), prefix, "WATER_SURFACE_PATCH"));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, getOceanShader(_TRITON->getOcean(), ::Triton::GOD_RAYS, 0L, tritonCam), prefix, "GOD_RAYS"));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, getOceanShader(_TRITON->getOcean(), ::Triton::SPRAY_PARTICLES, 0L, tritonCam), prefix, "SPRAY_PARTICLES"));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, getOceanShader(_TRITON->getOcean(), ::Triton::WAKE_SPRAY_PARTICLES, 0L, tritonCam), prefix, "WAKE_SPRAY_PARTICLES"));
#if 0
        // In older Triton (3.91), this line causes problems in Core profile and prevents the ocean from drawing.  In newer Triton (4.01),
        // this line causes a crash because there is no context passed in to GetShaderObject(), resulting in multiple NULL references.
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, getOceanShader(_TRITON->getOcean(), ::Triton::WATER_DECALS, 0L, tritonCam), prefix, "WATER_DECALS"));
#endif
    }
    adapters.apply( state );


    // Pass the final view and projection matrices into Triton.
    if ( environment )
    {
        tritonCam->SetCameraMatrix(state->getModelViewMatrix().ptr());
        tritonCam->SetProjectionMatrix(state->getProjectionMatrix().ptr());
    }

    if (_heightMapGenerator.valid())
    {
        GLint texName;
        osg::Matrix hMM;
        if (_heightMapGenerator->getTextureAndMatrix(renderInfo, texName, hMM))
        {
            // copy the OSG matrix to a Triton matrix:
            ::Triton::Matrix4 texMat(
                hMM(0, 0), hMM(0, 1), hMM(0, 2), hMM(0, 3),
                hMM(1, 0), hMM(1, 1), hMM(1, 2), hMM(1, 3),
                hMM(2, 0), hMM(2, 1), hMM(2, 2), hMM(2, 3),
                hMM(3, 0), hMM(3, 1), hMM(3, 2), hMM(3, 3));

            environment->SetHeightMap((::Triton::TextureHandle)texName, texMat, 0L, tritonCam);
                
            OE_DEBUG << LC << "Updating height map, FN=" << renderInfo.getState()->getFrameStamp()->getFrameNumber() << std::endl;
        }
    }

    state->dirtyAllVertexArrays();

    // Now light and draw the ocean:
    if ( environment )
    {
        // User pre-draw callback:
        if (_TRITON->getCallback())
        {
            _TRITON->getCallback()->onDrawOcean(
                _TRITON->getEnvironmentWrapper(),
                _TRITON->getOceanWrapper());
        }

        // The sun position is roughly where it is in our skybox texture:

        // Since this is a simple example we will just assume that Sun is the light from View light source
        // TODO: fix this...
        osg::Light* light = renderInfo.getView() ? renderInfo.getView()->getLight() : NULL;

        // This is the light attached to View so there are no transformations above..
        // But in general case you would need to accumulate all transforms above the light into this matrix
        osg::Matrix lightLocalToWorldMatrix = osg::Matrix::identity();

        // If you don't know where the sun lightsource is attached and don't know its local to world matrix you may use
        // following elaborate scheme to grab the light source while drawing Triton ocean:
        // - Install cull callback to catch CullVisitor and record pointer to its associated RenderStage
        //   I was hoping RenderStage can be found from renderInfo in drawImplementation but I didn't figure how ...
        // - When TritonDrawable::drawImplementation is called all lights will be already applied to OpenGL
        //   then just find proper infinite directional light by scanning renderStage->PositionalStateContainer.
        // - Note that we canot scan for the lights inside cull because they may not be traversed before Triton drawable
        // - When you found interesting ligt source that can work as Sun, read its modelview matrix and lighting params
        //   Multiply light position by ( modelview * inverse camera view ) and pass this to Triton with lighting colors

        if ( light && light->getPosition().w() == 0 )
        {
            osg::Vec4 ambient = light->getAmbient();
            osg::Vec4 diffuse = light->getDiffuse();
            osg::Vec4 position = light->getPosition();

            // Compute light position/direction in the world
            position = position * lightLocalToWorldMatrix;

            // Diffuse direction and color
            environment->SetDirectionalLight(
                ::Triton::Vector3( position[0], position[1], position[2] ),
                ::Triton::Vector3( diffuse[0],  diffuse[1],  diffuse[2] ) );

            // Sun-based ambient value:
            osg::Vec3d up = osg::Vec3d(0,0,0) * renderInfo.getCurrentCamera()->getInverseViewMatrix();
            up.normalize();
            osg::Vec3d pos3 = osg::Vec3d(position.x(), position.y(), position.z());
            pos3.normalize();
            float dot = osg::clampAbove(up*pos3, 0.0); dot*=dot;
            float sunAmbient = (float)osg::clampBetween( dot, 0.0f, 0.88f );
            float fa = osg::maximum(sunAmbient, ambient[0]);

            // Ambient color based on the zenith color in the cube map
            environment->SetAmbientLight( ::Triton::Vector3(fa, fa, fa) );
        }

        else
        {
            environment->SetDirectionalLight( ::Triton::Vector3(0,0,1), ::Triton::Vector3(1,1,1) );
            environment->SetAmbientLight( ::Triton::Vector3(0.88f, 0.88f, 0.88f) );
        }

        if ( _cubeMap.valid() )
        {
            // Build transform from our cube map orientation space to native Triton orientation
            // See worldToCubeMap function used in SkyBox to orient sky texture so that sky is up and earth is down
            osg::Matrix m = osg::Matrix::rotate( osg::PI_2, osg::X_AXIS ); // = worldToCubeMap

            ::Triton::Matrix3 transformFromYUpToZUpCubeMapCoords(
                m(0,0), m(0,1), m(0,2),
                m(1,0), m(1,1), m(1,2),
                m(2,0), m(2,1), m(2,2) );

            // Grab the cube map from our sky box and give it to Triton to use as an _environment map
            // GLenum texture = renderInfo.getState()->getLastAppliedTextureAttribute( _stage, osg::StateAttribute::TEXTURE );
            environment->SetEnvironmentMap(
                (::Triton::TextureHandle)_cubeMap->getTextureObject( state->getContextID() )->id(),
                transformFromYUpToZUpCubeMapCoords );

            if( _planarReflectionMap.valid() && _planarReflectionProjection.valid() )
            {
                osg::Matrix & p = *_planarReflectionProjection;

                ::Triton::Matrix3 planarProjection(
                    p(0,0), p(0,1), p(0,2),
                    p(1,0), p(1,1), p(1,2),
                    p(2,0), p(2,1), p(2,2) );

                environment->SetPlanarReflectionMap(
                    (::Triton::TextureHandle)_planarReflectionMap->getTextureObject( state->getContextID() )->id(),
                    planarProjection,
                    0.125 );
            }
        }

        // Draw the ocean for the current time sample
        if ( _TRITON->getOcean() )
        {
            osg::GLExtensions* ext = osg::GLExtensions::Get(state->getContextID(), true);

            bool writeDepth = true;
            const osg::Depth* depth = static_cast<const osg::Depth*>(state->getLastAppliedAttribute(osg::StateAttribute::DEPTH));
            if (depth)
                writeDepth = depth->getWriteMask();

            double simTime = renderInfo.getView()->getFrameStamp()->getSimulationTime();
            simTime = fmod(simTime, 86400.0);

            _TRITON->getOcean()->Draw(
                simTime,
                writeDepth, // depth writes
                true, // draw water
                true, // draw particles
                NULL, // optional context
                tritonCam);

        }
    }
            
    // Put GL back in a state that won't confuse the OSG state tracking:
    state->dirtyAllVertexArrays();
    state->dirtyAllAttributes();
    state->dirtyAllModes();    

#ifndef OSG_GL_FIXED_FUNCTION_AVAILABLE
    // Keep OSG from reapplying GL_LIGHTING on next state change after dirtyAllModes().
    state->setModeValidity(GL_LIGHTING, false);
#endif

    // Keep an eye on this.
    // I had to remove something similar in another module (Rex engine) because it was causing
    // positional attributes (like clip planes) to re-apply with an incorrect MVM. -gw
    state->apply();    
}
