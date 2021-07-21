/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include "WindLayer"
#include <osgEarth/Shaders>
#include <osgEarth/StringUtils>
#include <osgEarth/GeoTransform>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/GLUtils>
#include <osgEarth/TerrainEngineNode>
#include <osg/BindImageTexture>
#include <osg/Texture3D>
#include <osg/Program>
#include <osgUtil/CullVisitor>

#ifdef OE_HAVE_BINDIMAGETEXTURE
#include <osg/BindImageTexture>
#endif

using namespace osgEarth;

#define LC "[WindLayer] "

REGISTER_OSGEARTH_LAYER(wind, WindLayer);

// texture size (3D) -- in view space. A larger Z dimension (which cooresponds
// to your camera look vector) seesm to help with aliasing.
#define WIND_DIM_X 8
#define WIND_DIM_Y 8
#define WIND_DIM_Z 16

//........................................................................

namespace 
{
    struct WindData // keep me 16-byte aligned
    {
        GLfloat position[4];
        GLfloat direction[3];
        GLfloat speed;
    };

    // GL data that must be stored per-graphics-context
    struct DrawState
    {
        DrawState() :
            _bufferSize(0),
            _glBufferStorage(nullptr) { }

        GLBuffer::Ptr _buffer;
        GLuint _bufferSize;

        // pre-OSG 3.6 support
        void (GL_APIENTRY * _glBufferStorage)(GLenum, GLuint, const void*, GLenum);
    };

    // Data stored per-camera
    struct CameraState
    {
        WindData* _windData;
        unsigned _numWindsAllocated;

        osg::ref_ptr<osg::StateSet> _computeStateSet;
        osg::ref_ptr<osg::Uniform> _viewToTexMatrix;

        osg::ref_ptr<osg::StateSet> _sharedStateSet;
        osg::ref_ptr<osg::Uniform> _texToViewMatrix;

        CameraState() :
            _windData(nullptr),
            _numWindsAllocated(0) { }

        ~CameraState() {
            if (_windData)
                delete[] _windData;
            releaseGLObjects(nullptr);
        }

        void releaseGLObjects(osg::State* state) const {
            if (_computeStateSet.valid())
                _computeStateSet->releaseGLObjects(state);
            if (_sharedStateSet.valid())
                _sharedStateSet->releaseGLObjects(state);
        }

        void resizeGLObjectBuffers(unsigned size) {
            if (_computeStateSet.valid())
                _computeStateSet->resizeGLObjectBuffers(size);
            if (_sharedStateSet.valid())
                _sharedStateSet->resizeGLObjectBuffers(size);
        }
    };

    // mutexed per-camera map of CameraState objects
    struct CameraStates : public Mutexed<std::unordered_map<const osg::Camera*, CameraState>>
    {
        CameraState& get(const osg::Camera* camera)
        {
            ScopedMutexLock lock(mutex());
            return (*this)[camera];
        }

        void for_each(const std::function<void(CameraState&)>& func) {
            ScopedMutexLock lock(mutex());
            for (auto& i : *this) {
                func(i.second);
            }
        }

        void for_each(const std::function<void(const CameraState&)>& func) const {
            ScopedMutexLock lock(mutex());
            for (auto& i : *this) {
                func(i.second);
            }
        }
    };

    struct WindDrawable;

    // custom cull callback for the wind drawable
    struct WindStateCuller : public osg::Drawable::CullCallback
    {
        bool cull(osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo) const;
    };

    //! Drawable that runs the compute shader to populate our wind texture.
    struct WindDrawable : public osg::Drawable
    {
    public:
        WindDrawable(const osgDB::Options* readOptions);

        void setupPerCameraState(const osg::Camera* camera);
        void compileGLObjects(osg::RenderInfo& ri, DrawState& ds) const;
        void updateBuffers(CameraState&, const osg::Camera*);

        void drawImplementation(osg::RenderInfo& ri) const override;
        void releaseGLObjects(osg::State* state) const override;
        void resizeGLObjectBuffers(unsigned maxSize) override;

        osg::ref_ptr<const osgDB::Options> _readOptions;
        osg::ref_ptr<osg::Program> _computeProgram;
        std::vector<osg::ref_ptr<Wind> > _winds;
        mutable osg::buffered_object<DrawState> _ds;
        mutable CameraStates _cameraState;
        TextureImageUnitReservation _unitReservation;
    };

    bool WindStateCuller::cull(osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo) const
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
        const osg::Camera* camera = cv->getCurrentCamera();
        WindDrawable* wd = static_cast<WindDrawable*>(drawable);
        CameraState& cs = wd->_cameraState.get(camera); // should exist by now
        if (cs._computeStateSet.valid())
        {
            cv->pushStateSet(cs._computeStateSet.get());
            cv->addDrawableAndDepth(drawable, cv->getModelViewMatrix(), 0.0f);
            cv->popStateSet();
        }
        return true;
    }

    WindDrawable::WindDrawable(const osgDB::Options* readOptions)
    {
        _cameraState.setName(OE_MUTEX_NAME);

        // Always run the shader.
        setCullingActive(false);

        // So we can update the wind instances synchronously
        setDataVariance(osg::Object::DYNAMIC);

        // Prevent some variations of OSG from building an old-school display list
        // and then never calling drawImplementation again. Cry.
        setUseDisplayList(false);

        // Set up our compute shader
        Shaders shaders;
        std::string source = ShaderLoader::load(shaders.WindComputer, shaders, readOptions);
        osg::Shader* computeShader = new osg::Shader(osg::Shader::COMPUTE, source);
        _computeProgram = new osg::Program();
        _computeProgram->addShader(computeShader);

        setCullCallback(new WindStateCuller());
    }

    void WindDrawable::setupPerCameraState(const osg::Camera* camera)
    {
        // wind texture
        osg::Texture3D* tex = new osg::Texture3D();
        tex->setTextureSize(WIND_DIM_X, WIND_DIM_Y, WIND_DIM_Z);
        tex->setSourceFormat(GL_RGBA);
        tex->setInternalFormat(GL_RGBA8);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
        tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

        // state set for the compute program that populates the wind texture
        CameraState& cs = _cameraState.get(camera);
        cs._computeStateSet = new osg::StateSet();
        cs._computeStateSet->setAttribute(_computeProgram.get(), 1);

        // Binding so the compute shader can write to the texture.
        // Note, setting "layered=GL_TRUE" is required to bind the entire 3D texture.
        cs._computeStateSet->addUniform(new osg::Uniform("oe_wind_tex", 0));
        cs._computeStateSet->setAttribute(new osg::BindImageTexture(0, tex, osg::BindImageTexture::WRITE_ONLY, GL_RGBA8, 0, GL_TRUE));

        cs._texToViewMatrix = new osg::Uniform("oe_wind_texToViewMatrix", osg::Matrixf::identity());
        cs._computeStateSet->addUniform(cs._texToViewMatrix.get());

        cs._computeStateSet->setRenderBinDetails(-90210, "RenderBin"); // necessary? probably not


        // Shared stateset using during the cull traversal
        // that gives the rest of the Map access to the computed wind texture.
        cs._sharedStateSet = new osg::StateSet();

        cs._viewToTexMatrix = new osg::Uniform("oe_wind_matrix", osg::Matrixf::identity());
        cs._sharedStateSet->addUniform(cs._viewToTexMatrix.get());
        cs._sharedStateSet->setDefine("OE_WIND_TEX_MATRIX", "oe_wind_matrix");

        cs._sharedStateSet->addUniform(new osg::Uniform("oe_wind_tex", _unitReservation.unit()));
        cs._sharedStateSet->setTextureAttribute(_unitReservation.unit(), tex, osg::StateAttribute::ON);
        cs._sharedStateSet->setDefine("OE_WIND_TEX", "oe_wind_tex");
    }

    void WindDrawable::compileGLObjects(osg::RenderInfo& ri, DrawState& ds) const
    {
        osg::State* state = ri.getState();
        if (state)
        {
            osg::GLExtensions* ext = state->get<osg::GLExtensions>();

            CameraState& cs = _cameraState.get(ri.getCurrentCamera());

            GLuint requiredBufferSize = sizeof(WindData) * (_winds.size()+1);

            if (ds._buffer == nullptr || ds._bufferSize < requiredBufferSize)
            {
                ds._buffer = GLBuffer::create(GL_SHADER_STORAGE_BUFFER, *state, "oe.wind");
                ds._bufferSize = requiredBufferSize;

                ds._buffer->bind();

                GLFunctions::get(*state).
                    glBufferStorage(GL_SHADER_STORAGE_BUFFER, ds._bufferSize, nullptr, GL_DYNAMIC_STORAGE_BIT);
            }
            else
            {
                ds._buffer->bind();
            }

            // download to GPU
            ext->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, ds._bufferSize, cs._windData);
        }
    }

    void WindDrawable::updateBuffers(CameraState& cs, const osg::Camera* camera)
    {
        if (cs._numWindsAllocated < _winds.size()+1)
        {
            if (cs._windData != nullptr)
                delete [] cs._windData;

            // add one for the terminator.
            cs._windData = new WindData[_winds.size()+1];
            ::memset(cs._windData, 0, sizeof(WindData)*(_winds.size()+1));

            cs._numWindsAllocated = _winds.size()+1;
        }

        size_t i;
        for(i=0; i<_winds.size(); ++i)
        {
            const Wind* wind = _winds[i].get();

            if (wind->type() == Wind::TYPE_POINT)
            {
                // transform from world to camera-view space
                osg::Vec3d posView =  wind->getPointWorld() * camera->getViewMatrix();

                cs._windData[i].position[0] = posView.x();
                cs._windData[i].position[1] = posView.y();
                cs._windData[i].position[2] = posView.z();
                cs._windData[i].position[3] = 1.0;
            }
            else // TYPE_DIRECTIONAL
            {
                // transform from world to camera-view space
                osg::Vec3f dir;
                dir.x() = wind->direction()->x();
                dir.y() = wind->direction()->y();
                dir = osg::Matrixf::transform3x3(dir, camera->getViewMatrix());
                dir.normalize();

                cs._windData[i].direction[0] = dir.x();
                cs._windData[i].direction[1] = dir.y();
                cs._windData[i].direction[2] = dir.z();
                cs._windData[i].position[3] = 0.0f;
            }

            cs._windData[i].speed = wind->speed()->as(Units::KNOTS);
        }

        // terminator record: set power to negative.
        cs._windData[i].speed = -1.0f;
    }

    void WindDrawable::releaseGLObjects(osg::State* state) const
    {
        if (state)
        {
            DrawState& ds = _ds[state->getContextID()];
            ds._buffer = nullptr;
        }
        else
        {
            _ds.clear();
        }

        _cameraState.for_each([&](const CameraState& cs)
            {
                cs.releaseGLObjects(state);
            });

        osg::Drawable::releaseGLObjects(state);
    }

    void WindDrawable::resizeGLObjectBuffers(unsigned maxSize)
    {
        _ds.resize(maxSize);

        _cameraState.for_each([&](CameraState& cs)
            {
                cs.resizeGLObjectBuffers(maxSize);
            });

        osg::Drawable::resizeGLObjectBuffers(maxSize);
    }

    void WindDrawable::drawImplementation(osg::RenderInfo& ri) const
    {
        if (ri.getCurrentCamera() == nullptr)
            return;

        DrawState& ds = _ds[ri.getState()->getContextID()];
        osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

        // update buffer with wind data
        compileGLObjects(ri, ds);

        // activate layout() binding point:
        ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ds._buffer->name());

        // run it
        ext->glDispatchCompute(WIND_DIM_X, WIND_DIM_Y, WIND_DIM_Z);

        // sync the output
        ext->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }
}

//........................................................................

// default: gentle west-to-east breeze
Wind::Wind() :
    _type(TYPE_DIRECTIONAL),
    _direction(osg::Vec2f(1,0)),
    _speed(Speed(3.0, Units::KNOTS)),
    _pointWorld(osg::Vec3d(0,0,0))
{
    //nop
}

void
Wind::setPoint(const GeoPoint& point)
{
    _point = point;
    point.toWorld(_pointWorld);
}

Wind::Wind(const Config& conf)
{
    conf.get("type", "point", type(), TYPE_POINT);
    conf.get("type", "directional", type(), TYPE_DIRECTIONAL);
    conf.get("point", point());
    conf.get("direction", direction());
    conf.get("speed", speed());
}

Config
Wind::getConfig() const
{
    Config conf("wind");
    conf.set("type", "point", type(), TYPE_POINT);
    conf.set("type", "directional", type(), TYPE_DIRECTIONAL);
    conf.set("point", point());
    conf.set("direction", direction());
    conf.set("speed", speed());
    return conf;
}

//........................................................................

Config
WindLayer::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();
    conf.set("ortho", ortho());
    conf.set("radius", radius());
    if (!winds().empty())
    {
        Config windsConf("winds");
        for(std::vector<osg::ref_ptr<Wind> >::const_iterator i = winds().begin();
            i != winds().end();
            ++i)
        {
            windsConf.add("wind", i->get()->getConfig());
        }
        conf.add(windsConf);
    }
    return conf;
}

void
WindLayer::Options::fromConfig(const Config& conf)
{
    ortho().setDefault(true);
    radius().setDefault(Distance(75.0, Units::METERS));
    winds().clear();

    conf.get("ortho", ortho());
    conf.get("radius", radius());
    const ConfigSet windsConf = conf.child("winds").children();
    for(ConfigSet::const_iterator i = windsConf.begin(); i != windsConf.end(); ++i)
    {
        Wind* wind = new Wind(*i);
        winds().push_back(wind);
    }
}

//........................................................................

void
WindLayer::addWind(Wind* wind)
{
    WindDrawable* wd = dynamic_cast<WindDrawable*>(_drawable.get()); // also checks for nullptr
    if (wd)
    {
        wd->_winds.push_back(wind);
    }
}

void
WindLayer::removeWind(Wind* wind)
{
    WindDrawable* wd = dynamic_cast<WindDrawable*>(_drawable.get()); // also checks for nullptr
    if (wd)
    {
        for (std::vector<osg::ref_ptr<Wind>>::iterator i = wd->_winds.begin();
            i != wd->_winds.end();
            ++i)
        {
            if (i->get() == wind)
            {
                wd->_winds.erase(i);
                break;
            }
        }
    }
}

void
WindLayer::init()
{
    Layer::init();

    // Never cache
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

Status
WindLayer::openImplementation()
{
    // GL version requirement
    if (Registry::capabilities().getGLSLVersion() < 4.3f)
    {
        return Status(Status::ResourceUnavailable, "WindLayer requires GL 4.3+");
    }

    return Layer::openImplementation();
}

Status
WindLayer::closeImplementation()
{
    return Layer::closeImplementation();
}

osg::Node*
WindLayer::getNode() const
{
    return _drawable.get();
}

void
WindLayer::prepareForRendering(TerrainEngine* engine)
{
    Layer::prepareForRendering(engine);

    // Create the wind drawable that will provide a wind texture
    WindDrawable* wd = new WindDrawable(getReadOptions());
    _drawable = wd;

    // texture image unit for the shared wind LUT
    engine->getResources()->reserveTextureImageUnit(wd->_unitReservation, "WindLayer");

#if 0 // TESTING
    Wind* wind = new Wind();
    wind->type() = Wind::TYPE_DIRECTIONAL;
    wind->direction()->set(-1.0f, 0.0f);
    wind->speed()->set(10, Units::KNOTS);
    addWind(wind);

    Wind* wind2 = new Wind();
    wind2->type() = Wind::TYPE_DIRECTIONAL;
    wind2->direction()->set(0.0f, -1.0f);
    wind2->speed()->set(10, Units::KNOTS);
    addWind(wind2);
#endif

    for(int i=0; i<options().winds().size(); ++i)
    {
        addWind(options().winds()[i].get());
    }
}

osg::StateSet*
WindLayer::getSharedStateSet(osg::NodeVisitor* nv) const
{
    if (!isOpen())
        return nullptr;

    OE_SOFT_ASSERT_AND_RETURN(_drawable.valid(), nullptr);

    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

    WindDrawable* windDrawable = static_cast<WindDrawable*>(_drawable.get());

    const osg::Camera* camera = cv->getCurrentCamera();
    CameraState& cs = windDrawable->_cameraState.get(camera);

    if (!cs._computeStateSet.valid())
    {
        windDrawable->setupPerCameraState(camera);
    }

    // this xforms from clip [-1..1] to texture [0..1] space
    static osg::Matrix clipToTexture = 
        osg::Matrix::translate(1.0,1.0,1.0) * 
        osg::Matrix::scale(0.5,0.5,0.5);

    osg::Matrix rttProjection;



    double R = options().radius()->as(Units::METERS);

    if (options().ortho() == true)
    {
        rttProjection = osg::Matrix::ortho(-R, R, -R, R, 0, R);
    }
    else
    { 
        double y,a,n,f;
        camera->getProjectionMatrix().getPerspective(y,a,n,f);

        // pushing the NEAR out reduces jitter a lot
        rttProjection.makePerspective(y, a, 5.0, R);
    }

    // view to texture:
    osg::Matrix camViewToTexture = rttProjection * clipToTexture;
    cs._viewToTexMatrix->set(camViewToTexture);

    // texture to view (for the compute shader):
    osg::Matrix textureToCamView;
    textureToCamView.invert(camViewToTexture);
    cs._texToViewMatrix->set(textureToCamView);

    windDrawable->updateBuffers(cs, camera);

    return cs._sharedStateSet.get();
}

void
WindLayer::releaseGLObjects(osg::State* state) const
{
    if (_drawable.valid())
        _drawable->releaseGLObjects(state);
}

void
WindLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_drawable.valid())
        _drawable->resizeGLObjectBuffers(maxSize);
}

