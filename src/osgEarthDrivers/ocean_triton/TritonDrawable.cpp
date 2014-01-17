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
#include "TritonDrawable"
#include "TritonContext"
#include <osgEarth/SpatialReference>
#include <Triton.h>

#define LC "[TritonDrawable] "

using namespace osgEarth;
using namespace osgEarth::Drivers::Triton;

TritonDrawable::TritonDrawable(TritonContext* TRITON) :
_TRITON(TRITON)
{
    // call this to ensure draw() gets called every frame.
    setSupportsDisplayList( false );
    setUseVertexBufferObjects( false );

    // dynamic variance prevents update/cull overlap when drawing this
    setDataVariance( osg::Object::DYNAMIC );
}

void
TritonDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State& state = *renderInfo.getState();

    state.disableAllVertexArrays();

    _TRITON->initialize( renderInfo );
    if ( !_TRITON->ready() )
        return;

    ::Triton::Environment* environment = _TRITON->getEnvironment();

    // Pass the final view and projection matrices into Triton.
    if ( environment )
    {
        environment->SetCameraMatrix( state.getModelViewMatrix().ptr() );
        environment->SetProjectionMatrix( state.getProjectionMatrix().ptr() );
    }

    state.dirtyAllVertexArrays();

    // Now light and draw the ocean:
    if ( environment )
    {
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

            // Ambient color based on the zenith color in the cube map
            environment->SetAmbientLight(
                ::Triton::Vector3( ambient[0], ambient[1], ambient[2] ) );
        }

        // Build transform from our cube map orientation space to native Triton orientation
        // See worldToCubeMap function used in SkyBox to orient sky texture so that sky is up and earth is down
        osg::Matrix m = osg::Matrix::rotate( osg::PI_2, osg::X_AXIS ); // = worldToCubeMap

        ::Triton::Matrix3 transformFromYUpToZUpCubeMapCoords(
            m(0,0), m(0,1), m(0,2),
            m(1,0), m(1,1), m(1,2),
            m(2,0), m(2,1), m(2,2) );

        // Grab the cube map from our sky box and give it to Triton to use as an _environment map
        // GLenum texture = renderInfo.getState()->getLastAppliedTextureAttribute( _stage, osg::StateAttribute::TEXTURE );
        if ( _cubeMap.valid() )
        {
            environment->SetEnvironmentMap(
                (::Triton::TextureHandle)_cubeMap->getTextureObject( state.getContextID() )->id(), transformFromYUpToZUpCubeMapCoords );
        }

        // Draw the ocean for the current time sample
        if ( _TRITON->getOcean() )
        {
            _TRITON->getOcean()->Draw( renderInfo.getView()->getFrameStamp()->getSimulationTime() );
        }
    }

    state.dirtyAllVertexArrays();
}
