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
#include <Triton.h>

#include "TritonDrawable"
#include "TritonContext"
#include <osg/MatrixTransform>
#include <osg/FrameBufferObject>

#include <osgEarth/SpatialReference>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>

#undef  LC
#define LC "[TritonDrawable] "

//#define DEBUG_HEIGHTMAP

using namespace osgEarth::Triton;

namespace
{

/** Default size of _contextDirty */
static const size_t NUM_CONTEXTS = 64;

#ifdef DEBUG_HEIGHTMAP
    osg::Node*
    makeFrustumFromCamera( osg::Camera* camera )
    {
        osg::Matrixd proj;
        osg::Matrixd mv;
        if (camera)
        {
            proj = camera->getProjectionMatrix();
            mv = camera->getViewMatrix();
        }
        else
        {
            proj.makePerspective( 30., 1., 1., 10. );
            // leave mv as identity
        }

        double near=0.0, far=0.0;
        double left=0.0, right=0.0, bottom=0.0, top=0.0;
        double fovy=0.0, aspectRatio=0.0;
        double nLeft=0.0, nRight=0.0, nBottom=0.0, nTop=0.0;
        double fLeft=0.0, fRight=0.0, fBottom=0.0, fTop=0.0;

        osg::Geode* geode = new osg::Geode;

        if( proj.getPerspective(fovy, aspectRatio, near, far) )
        {
            near = proj(3,2) / (proj(2,2)-1.0);
            far = proj(3,2) / (1.0+proj(2,2));

            nLeft = near * (proj(2,0)-1.0) / proj(0,0);
            nRight = near * (1.0+proj(2,0)) / proj(0,0);
            nTop = near * (1.0+proj(2,1)) / proj(1,1);
            nBottom = near * (proj(2,1)-1.0) / proj(1,1);

            fLeft = far * (proj(2,0)-1.0) / proj(0,0);
            fRight = far * (1.0+proj(2,0)) / proj(0,0);
            fTop = far * (1.0+proj(2,1)) / proj(1,1);
            fBottom = far * (proj(2,1)-1.0) / proj(1,1);

            osg::Vec3Array* v = new osg::Vec3Array;
            v->resize( 9 );
            (*v)[0].set( 0., 0., 0. );

            (*v)[1].set( nLeft, nBottom, -near );
            (*v)[2].set( nRight, nBottom, -near );
            (*v)[3].set( nRight, nTop, -near );
            (*v)[4].set( nLeft, nTop, -near );
            (*v)[5].set( fLeft, fBottom, -far );
            (*v)[6].set( fRight, fBottom, -far );
            (*v)[7].set( fRight, fTop, -far );
            (*v)[8].set( fLeft, fTop, -far );

            osg::Geometry* geom = new osg::Geometry;
            geom->setVertexArray( v );

            GLushort idxLines[8] = {
                0, 5, 0, 6, 0, 7, 0, 8 };
            GLushort idxLoops0[4] = {
                1, 2, 3, 4 };
            GLushort idxLoops1[4] = {
                5, 6, 7, 8 };
            geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
            geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
            geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

            geode->addDrawable( geom );
        }
        else if(proj.getOrtho( left, right, bottom, top, near, far) )
        {
            nLeft = left;//near * (proj(2,0)-1.0) / proj(0,0);
            nRight = right;//near * (1.0+proj(2,0)) / proj(0,0);
            nTop = top;//near * (1.0+proj(2,1)) / proj(1,1);
            nBottom = bottom;//near * (proj(2,1)-1.0) / proj(1,1);

            fLeft = left;//far * (proj(2,0)-1.0) / proj(0,0);
            fRight = right;//far * (1.0+proj(2,0)) / proj(0,0);
            fTop = top;//far * (1.0+proj(2,1)) / proj(1,1);
            fBottom = bottom;//far * (proj(2,1)-1.0) / proj(1,1);

            osg::Vec3Array* v = new osg::Vec3Array;
            v->resize( 9 );
            (*v)[0].set( 0., 0., 0. );

            (*v)[1].set( nLeft, nBottom, -near );
            (*v)[2].set( nRight, nBottom, -near );
            (*v)[3].set( nRight, nTop, -near );
            (*v)[4].set( nLeft, nTop, -near );
            (*v)[5].set( fLeft, fBottom, -far );
            (*v)[6].set( fRight, fBottom, -far );
            (*v)[7].set( fRight, fTop, -far );
            (*v)[8].set( fLeft, fTop, -far );

            osg::Geometry* geom = new osg::Geometry;
            geom->setVertexArray( v );

            GLushort idxLines[8] = {
                1, 5, 2, 6, 3, 7, 4, 8 };
            GLushort idxLoops0[4] = {
                1, 2, 3, 4 };
            GLushort idxLoops1[4] = {
                5, 6, 7, 8 };
            geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
            geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
            geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

            geode->addDrawable( geom );
        }

        geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );



        osg::MatrixTransform* mt = new osg::MatrixTransform;
        osg::Matrixd mvi = osg::Matrixd::inverse( mv );
        mt->setMatrix( mvi );
        mt->addChild( geode );

        return mt;
    }

    osg::Camera *
    CreateTextureQuadOverlay( osg::Texture * texture, float x, float y, float w, float h )
    {
        osg::Camera * camera = new osg::Camera;

        // set up the background color and clear mask.
        camera->setClearMask( GL_DEPTH_BUFFER_BIT );

        // set viewport
        camera->setProjectionMatrixAsOrtho2D( 0, 1, 0, 1 );
        camera->setViewMatrix( osg::Matrix::identity() );
        camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );

        osg::Geode * geode = new osg::Geode();
        camera->addChild( geode );
    /*
        osg::Geometry * background = osg::createTexturedQuadGeometry( osg::Vec3( x,y,0 ),osg::Vec3( w,0,0 ), osg::Vec3( 0,h,0 ) );
        geode->addDrawable( background );

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back( osg::Vec4( 1,1,0,0.3 ) );
        background->setColorArray( colors );
        background->setColorBinding( osg::Geometry::BIND_OVERALL );
        background->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
        background->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );
    */
        osg::Geometry * quad = osg::createTexturedQuadGeometry( osg::Vec3( x,y,0 ),osg::Vec3( w,0,0 ), osg::Vec3( 0,h,0 ) );
        geode->addDrawable( quad );
        quad->getOrCreateStateSet()->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON );
        quad->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
        quad->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
        quad->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );

        camera->getOrCreateStateSet()->setAttribute(new osg::Program());


        // set the camera to render after the main camera.
        camera->setRenderOrder(osg::Camera::POST_RENDER);

        return camera;
    }
#endif /* DEBUG_HEIGHTMAP */


    struct PassHeightMapToTritonCallback : public osg::Camera::DrawCallback
    {
        PassHeightMapToTritonCallback(TritonContext* triton) : _TRITON(triton), _enable(false), _id(0) { _ptrEnable = const_cast<bool*> (&_enable); };
        virtual void operator()( osg::RenderInfo& renderInfo ) const
        {
            if( _enable )
            {
                *_ptrEnable = false; // cannot directly change _enable because of const

                //osg::notify( osg::ALWAYS ) << "passing heightmap to Triton" << std::endl;
                if(_TRITON->ready())
                    _TRITON->getEnvironment()->SetHeightMap((::Triton::TextureHandle)_id,_heightMapMatrix);
            }
        }

        unsigned int _frameNumber;
        bool _enable;
        bool* _ptrEnable;
        int _id;
        ::Triton::Matrix4 _heightMapMatrix;

    private:
        osg::observer_ptr<TritonContext>  _TRITON;
    };


    /**
     * Responds to tile-added events by telling the Triton drawable that it needs to update
     * its height map.
     */
    class OceanTerrainChangedCallback : public osgEarth::TerrainCallback
    {
    public:
        OceanTerrainChangedCallback(TritonDrawable* drawable)
            : _drawable(drawable) { }

    public: // osgEarth::TerrainCallback

        // Called when the terrain engine loads a new tile (or new tile heightfield data).
        // When this happens we need to update the Triton height map.
        void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, osgEarth::TerrainCallbackContext& context)
        {
            osg::ref_ptr<TritonDrawable> drawable;
            if ( _drawable.lock(drawable) )
                drawable->dirtyAllContexts();
        }

    private:
        osg::observer_ptr<TritonDrawable> _drawable;
    };


    const char* vertexShader =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "// terrain SDK:\n"
        "float oe_terrain_getElevation(); \n"

        "varying float oe_triton_elev;\n"

        "void setupContour(inout vec4 VertexModel) \n"
        "{ \n"
        "    oe_triton_elev = oe_terrain_getElevation(); \n"
        "} \n";

    // The fragment shader simply takes the texture index that we generated
    // in the vertex shader and does a texture lookup. In this case we're
    // just wholesale replacing the color, so if the map had any existing
    // imagery, this will overwrite it.

    const char* fragmentShader =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "varying float oe_triton_elev;\n"

        "void colorContour( inout vec4 color ) \n"
        "{ \n"
#ifdef DEBUG_HEIGHTMAP
          // Map to black = -500m, white = +500m
          "   float nHeight = clamp(oe_triton_elev / 1000.0 + 0.5, 0.0, 1.0);\n"
#else
          "   float nHeight = oe_triton_elev;\n"
#endif
        "    gl_FragColor = vec4( nHeight, 0.0, 0.0, 1.0 ); \n"
        "} \n";
}


TritonDrawable::TritonDrawable(osgEarth::MapNode* mapNode, TritonContext* TRITON) :
_TRITON(TRITON),
_mapNode(mapNode)
{
    // call this to ensure draw() gets called every frame.
    setSupportsDisplayList( false );
    setUseVertexBufferObjects( false );

    // dynamic variance prevents update/cull overlap when drawing this
    setDataVariance( osg::Object::DYNAMIC );

    _contextDirty.resize(NUM_CONTEXTS);
    dirtyAllContexts();
}

TritonDrawable::~TritonDrawable()
{
    osg::ref_ptr<MapNode> mapNode;
    osg::ref_ptr<osgEarth::TerrainCallback> callback;
    if ( _mapNode.lock(mapNode) && _terrainChangedCallback.lock(callback) && mapNode->getTerrain() )
    {
        _mapNode->getTerrain()->removeTerrainCallback( callback );
    }
}

void
TritonDrawable::dirtyAllContexts()
{
    _contextDirty.setAllElementsTo(1);
}

void
TritonDrawable::updateHeightMap(osg::RenderInfo& renderInfo) const
{
    if ( !_TRITON->ready() )
        return;

    osg::ref_ptr<MapNode> mapNode;
    if (!_mapNode.lock(mapNode))
        return;

    const osg::Matrix& viewMatrix = renderInfo.getCurrentCamera()->getViewMatrix();
    const osg::Matrix& projectionMatrix = renderInfo.getCurrentCamera()->getProjectionMatrix();

    osg::Vec3d eye, center, up;
    viewMatrix.getLookAt(eye, center, up);
    double fovyDEG=0.0, aspectRatio=0.0, zNear=0.0, zFar=0.0;
    projectionMatrix.getPerspective(fovyDEG, aspectRatio,zNear,zFar);

    // aspect_ratio = tan( HFOV/2 ) / tan( VFOV/2 )
    // tan( HFOV/2 ) = tan( VFOV/2 ) * aspect_ratio
    // HFOV/2 = atan( tan( VFOV/2 ) * aspect_ratio )
    // HFOV = 2.0 * atan( tan( VFOV/2 ) * aspect_ratio )
    double fovxDEG = osg::RadiansToDegrees( 2.0 * atan( tan(osg::DegreesToRadians(fovyDEG))/2.0 * aspectRatio ));

    double eyeLat=0.0, eyeLon=0.0, eyeHeight=0.0;
    mapNode->getMap()->getSRS()->getEllipsoid()->convertXYZToLatLongHeight(eye.x(), eye.y(), eye.z(), eyeLat, eyeLon, eyeHeight);
    double clampedEyeX=0.0, clampedEyeY=0.0,clampedEyeZ=0.0;
    mapNode->getMap()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(eyeLat, eyeLon, 0.0, clampedEyeX, clampedEyeY, clampedEyeZ);
    osg::Vec3 mslEye(clampedEyeX,clampedEyeY,clampedEyeZ);
    double lookAtLat=0.0, lookAtLon=0.0, lookAtHeight=0.0;
    mapNode->getMap()->getSRS()->getEllipsoid()->convertXYZToLatLongHeight(center.x(), center.y(), center.z(), lookAtLat, lookAtLon, lookAtHeight);

    // Calculate the distance to the horizon from the eyepoint
    double eyeLen = eye.length();
    double radE = mslEye.length();
    double hmax = radE + 8848.0;
    double hmin = radE - 12262.0;
    double hasl = osg::maximum(0.1, eyeLen - radE);
    double radius = eyeLen - hasl;
    double horizonDistance = osg::minimum(radE, sqrt( 2.0*radius*hasl + hasl*hasl ));

    osg::Vec3d heightCamEye(eye);

    double near = osg::maximum(1.0, heightCamEye.length() - hmax);
    double far = osg::maximum(10.0, heightCamEye.length() - hmin + radE);
    //osg::notify( osg::ALWAYS ) << "near = " << near << "; far = " << far << std::endl;

    //horizonDistance *= 0.25;

    _heightCamera->setProjectionMatrix(osg::Matrix::ortho(-horizonDistance,horizonDistance,-horizonDistance,horizonDistance,near,far) );
    _heightCamera->setViewMatrixAsLookAt( heightCamEye, osg::Vec3d(0.0,0.0,0.0), osg::Vec3d(0.0,0.0,1.0));

    const osg::Matrixd bias(0.5, 0.0, 0.0, 0.0,
        0.0, 0.5, 0.0, 0.0,
        0.0, 0.0, 0.5, 0.0,
        0.5, 0.5, 0.5, 1.0);

    osg::Matrix hMM = _heightCamera->getViewMatrix() * _heightCamera->getProjectionMatrix() * bias;
    ::Triton::Matrix4 heightMapMatrix(hMM(0,0),hMM(0,1),hMM(0,2),hMM(0,3),
        hMM(1,0),hMM(1,1),hMM(1,2),hMM(1,3),
        hMM(2,0),hMM(2,1),hMM(2,2),hMM(2,3),
        hMM(3,0),hMM(3,1),hMM(3,2),hMM(3,3));

    osg::Texture::TextureObject* texObj = _heightMap->getTextureObject(renderInfo.getContextID());

    if(texObj)
    {
        PassHeightMapToTritonCallback* cb = dynamic_cast<PassHeightMapToTritonCallback*>(_heightCamera->getFinalDrawCallback());
        if( cb )
        {
            cb->_enable = true;
            cb->_id = texObj->id();
            cb->_heightMapMatrix = heightMapMatrix;
        }
    }
    else
    {
        // may happen on the first frame; ignore
        //OE_WARN << LC << "Texture object is NULL (Internal error)" << std::endl;
    }

#ifdef DEBUG_HEIGHTMAP
    mapNode->getParent(0)->removeChild(0, 1);
    mapNode->getParent(0)->insertChild(0, makeFrustumFromCamera(_heightCam));
#endif /* DEBUG_HEIGHTMAP */
}

void
TritonDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::State* state = renderInfo.getState();

    state->disableAllVertexArrays();

    _TRITON->initialize( renderInfo );
    if ( !_TRITON->ready() )
        return;

    if ( _TRITON->passHeightMapToTriton() && !_terrainChangedCallback.valid() )
    {
        const_cast<TritonDrawable*>(this)->setupHeightMap(*state);
    }

    ::Triton::Environment* environment = _TRITON->getEnvironment();

    osgEarth::NativeProgramAdapterCollection& adapters = _adapters[ state->getContextID() ];
    if ( adapters.empty() )
    {
        const char* prefix = "oe_";
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, (GLint)_TRITON->getOcean()->GetShaderObject(::Triton::GOD_RAYS), prefix));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, (GLint)_TRITON->getOcean()->GetShaderObject(::Triton::SPRAY_PARTICLES), prefix));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, (GLint)_TRITON->getOcean()->GetShaderObject(::Triton::WAKE_SPRAY_PARTICLES), prefix));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, (GLint)_TRITON->getOcean()->GetShaderObject(::Triton::WATER_DECALS), prefix));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, (GLint)_TRITON->getOcean()->GetShaderObject(::Triton::WATER_SURFACE_PATCH), prefix));
        adapters.push_back( new osgEarth::NativeProgramAdapter(state, (GLint)_TRITON->getOcean()->GetShaderObject(::Triton::WATER_SURFACE), prefix));
    }
    adapters.apply( state );

    // Pass the final view and projection matrices into Triton.
    if ( environment )
    {
        environment->SetCameraMatrix( state->getModelViewMatrix().ptr() );
        environment->SetProjectionMatrix( state->getProjectionMatrix().ptr() );
    }

    if ( _TRITON->passHeightMapToTriton() )
    {
        unsigned cid = renderInfo.getContextID();
        const bool validCid = (cid < _contextDirty.size());

        bool dirty =
            ( validCid && _contextDirty[cid] ) ||
            ( renderInfo.getView()->getCamera()->getViewMatrix()       != _viewMatrix ) ||
            ( renderInfo.getView()->getCamera()->getProjectionMatrix() != _projMatrix );

        if ( dirty )
        {
            updateHeightMap( renderInfo );
            if ( validCid )
                _contextDirty[renderInfo.getContextID()] = 0;
            _viewMatrix = renderInfo.getView()->getCamera()->getViewMatrix();
            _projMatrix = renderInfo.getView()->getCamera()->getProjectionMatrix();
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
            float fa = std::max(sunAmbient, ambient[0]);

            // Ambient color based on the zenith color in the cube map
            environment->SetAmbientLight( ::Triton::Vector3(fa, fa, fa) );
            //::Triton::Vector3( ambient[0], ambient[1], ambient[2] ) );
        }

        else
        {
            environment->SetDirectionalLight( ::Triton::Vector3(0,0,1), ::Triton::Vector3(1,1,1) );
            environment->SetAmbientLight( ::Triton::Vector3(0.88f, 0.88f, 0.88f) );
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
                (::Triton::TextureHandle)_cubeMap->getTextureObject( state->getContextID() )->id(), transformFromYUpToZUpCubeMapCoords );

            if( _planarReflectionMap.valid() && _planarReflectionProjection.valid() )
            {
                osg::Matrix & p = *_planarReflectionProjection;

                ::Triton::Matrix3 planarProjection( p(0,0), p(0,1), p(0,2),
                                                    p(1,0), p(1,1), p(1,2),
                                                    p(2,0), p(2,1), p(2,2) );

                environment->SetPlanarReflectionMap( (::Triton::TextureHandle)
                                                      _planarReflectionMap->getTextureObject( state->getContextID() )->id(),
                                                      planarProjection, 0.125  );
            }
        }

        // Draw the ocean for the current time sample
        if ( _TRITON->getOcean() )
        {
            _TRITON->getOcean()->Draw( renderInfo.getView()->getFrameStamp()->getSimulationTime() );
        }
    }

    // Put GL back in a state that won't confuse the OSG state tracking:
    state->dirtyAllVertexArrays();
    state->dirtyAllAttributes();
    //osg::GL2Extensions* api = osg::GL2Extensions::Get(state->getContextID(), true);
    //api->glUseProgram((GLuint)0);
    //state->setLastAppliedProgramObject( 0L );
    state->apply();
}

namespace
{
#ifdef GL_LUMINANCE_FLOAT16_ATI
#   define GL_LUMINANCE_FLOAT16_ATI 0x881E
#endif
    /** Choose a supported FBO format by testing suitable ones until we find one that works. */
    bool getBestFBOConfiguration(GLint* out_internalFormat, GLenum* out_sourceFormat, osg::State& state)
    {
#define NUM_FORMATS 7

        struct Format {
            GLint internalFormat;
            GLenum sourceFormat;
            std::string name;
        };

        const Format formats[NUM_FORMATS] = {
            { GL_LUMINANCE16F_ARB,      GL_LUMINANCE, "GL_LUMINANCE16F_ARB" },
            { GL_LUMINANCE_FLOAT16_ATI, GL_LUMINANCE, "GL_LUMINANCE_FLOAT16_ATI" },
            { GL_LUMINANCE32F_ARB,      GL_LUMINANCE, "GL_LUMINANCE32F_ARB" },
            { GL_RGB16F_ARB,            GL_RGB,       "GL_RGB16F_ARB" },
            { GL_RGBA16F_ARB,           GL_RGBA,      "GL_RGBA16F_ARB" },
            { GL_RGB32F_ARB,            GL_RGB,       "GL_RGB32F_ARB" },
            { GL_RGBA32F_ARB,           GL_RGBA,      "GL_RGBA32F_ARB" }
        };

#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)
        osg::GLExtensions* ext = osg::GLExtensions::Get(state.getContextID(), true);
#else
        osg::FBOExtensions* ext = osg::FBOExtensions::instance(state.getContextID(), true);
#endif

        osg::State::CheckForGLErrors check = state.getCheckForGLErrors();
        state.setCheckForGLErrors(state.NEVER_CHECK_GL_ERRORS);

        bool found = false;

        for(int i=0; i<NUM_FORMATS && !found; ++i)
        {
            const Format& format = formats[i];

            osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D();
            tex->setTextureSize(1, 1);
            tex->setInternalFormat( format.internalFormat );
            tex->setSourceFormat  ( format.sourceFormat );

            osg::ref_ptr<osg::FrameBufferObject> fbo = new osg::FrameBufferObject();
            fbo->setAttachment( osg::Camera::COLOR_BUFFER, osg::FrameBufferAttachment(tex.get()) );

            fbo->apply( state );

            GLenum status = ext->glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);

            fbo->releaseGLObjects( &state );
            tex->releaseGLObjects( &state );

            if ( status == GL_FRAMEBUFFER_COMPLETE_EXT )
            {
                if ( out_internalFormat) *out_internalFormat = format.internalFormat;
                if ( out_sourceFormat )  *out_sourceFormat   = format.sourceFormat;
                OE_INFO << LC << "Height map format => " << format.name << std::endl;
                found = true;
            }
        }

        state.setCheckForGLErrors(check);

        return found;
    }
}

void TritonDrawable::setupHeightMap(osg::State& state)
{
    osg::ref_ptr<MapNode> mapNode;
    if (!_mapNode.lock(mapNode))
        return;

    int textureUnit = 0;
    int textureSize = _TRITON->getHeightMapSize();

    // Discover a suitable FBO format for floating point values:
    GLint internalFormat;
    GLenum sourceFormat;
    if ( !getBestFBOConfiguration(&internalFormat, &sourceFormat, state) )
    {
        OE_WARN << LC << "No supported FBO format available for height map; height map disabled!\n";
        return;
    }

    // Create our height map texture
    _heightMap = new osg::Texture2D;
    _heightMap->setTextureSize(textureSize, textureSize);
    _heightMap->setInternalFormat( internalFormat );
    _heightMap->setSourceFormat( sourceFormat );
    _heightMap->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
    _heightMap->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);

    // Create its camera and render to it
    _heightCamera = new osg::Camera;
    _heightCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    _heightCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    _heightCamera->setClearColor(osg::Vec4(-1000.0, -1000.0, -1000.0, 1.0f));
    _heightCamera->setViewport(0, 0, textureSize, textureSize);
    _heightCamera->setRenderOrder(osg::Camera::PRE_RENDER);
    _heightCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    _heightCamera->setImplicitBufferAttachmentMask(0, 0);
    _heightCamera->attach(osg::Camera::COLOR_BUFFER, _heightMap);
    _heightCamera->setCullMask( ~TRITON_OCEAN_MASK );
    _heightCamera->setAllowEventFocus(false);
    _heightCamera->setFinalDrawCallback(new PassHeightMapToTritonCallback(_TRITON.get()));

    // Install the shaders. We also bind osgEarth's elevation data attribute, which the
    // terrain engine automatically generates at the specified location.
    osg::StateSet* stateSet = _heightCamera->getOrCreateStateSet();
    osgEarth::VirtualProgram* heightProgram = osgEarth::VirtualProgram::getOrCreate(stateSet);
    heightProgram->setFunction( "setupContour", vertexShader,   osgEarth::ShaderComp::LOCATION_VERTEX_MODEL);
    heightProgram->setFunction( "colorContour", fragmentShader, osgEarth::ShaderComp::LOCATION_FRAGMENT_OUTPUT);

    _heightCamera->addChild( mapNode->getTerrainEngine() );
    _terrainChangedCallback = new OceanTerrainChangedCallback(this);
    if ( mapNode->getTerrain() )
        mapNode->getTerrain()->addTerrainCallback( _terrainChangedCallback.get() );

    osg::Group* root = osgEarth::findTopMostNodeOfType<osg::Group>(mapNode);
    root->addChild(_heightCamera.get());

#ifdef DEBUG_HEIGHTMAP
    mapNode->getParent(0)->addChild(CreateTextureQuadOverlay(_heightMap, 0.65, 0.05, 0.3, 0.3));
    mapNode->getParent(0)->insertChild(0, makeFrustumFromCamera(_heightCamera));
#endif /* DEBUG_HEIGHTMAP */
}
