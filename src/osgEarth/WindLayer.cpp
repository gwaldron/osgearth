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
#include <osg/Texture3D>
#include <osg/Program>
#include <osg/BindImageTexture>
#include <osgUtil/CullVisitor>

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
            _buffer(INT_MAX),
            _bufferSize(0) { }

        GLuint _buffer;
        osg::GLintptr _bufferSize;
    };

    // Data stored per-camera
    struct CameraState
    {
        CameraState() :
            _windData(NULL),
            _numWindsAllocated(0) { }

        ~CameraState();

        WindData* _windData;
        unsigned _numWindsAllocated;

        osg::ref_ptr<osg::StateSet> _computeStateSet;
        osg::ref_ptr<osg::Uniform> _viewToTexMatrix;

        osg::ref_ptr<osg::StateSet> _sharedStateSet;
        osg::ref_ptr<osg::Uniform> _texToViewMatrix;
    };

    typedef PerObjectFastMap<const osg::Camera*, CameraState> CameraStates;

    struct CameraState_ReleaseGLObjects : public CameraStates::Functor
    {
        osg::State* _state;
        CameraState_ReleaseGLObjects(osg::State* state) : _state(state) { }
        void operator()(CameraState& cs)
        {
            if (cs._computeStateSet.valid())
                cs._computeStateSet->releaseGLObjects(_state);
            if (cs._sharedStateSet.valid())
                cs._sharedStateSet->releaseGLObjects(_state);
        }
    };

    CameraState::~CameraState()
    {
        if (_windData)
            delete[] _windData;

        CameraState_ReleaseGLObjects r(NULL);
        r(*this);
    }


    struct WindDrawable;


    // Helpers that picks a stateset based on the traversing camera. 
    struct StateSwitcherCullCallback : public osg::NodeCallback
    {
        WindDrawable* _d;
        StateSwitcherCullCallback(WindDrawable* d) : _d(d) { }
        void operator()(osg::Node* node, osg::NodeVisitor* nv);
    };
    struct StateSwitcher : public osg::Group
    {
        StateSwitcher(WindDrawable* d)
        {
            setCullingActive(false);
            setCullCallback(new StateSwitcherCullCallback(d));
        }
    };

    //! Drawable that runs the compute shader to populate our wind texture.
    struct WindDrawable : public osg::Drawable
    {
    public:
        WindDrawable(const osgDB::Options* readOptions);

        void setupPerCameraState(const osg::Camera* camera, int textureImageUnit);
        void drawImplementation(osg::RenderInfo& ri) const;
        void compileGLObjects(osg::RenderInfo& ri, DrawState& ds) const;
        void releaseGLObjects(osg::State* state) const;
        void resizeGLObjectBuffers(unsigned maxSize);
        void updateBuffers(CameraState&, const osg::Camera*);

        osg::ref_ptr<const osgDB::Options> _readOptions;
        osg::ref_ptr<osg::Program> _computeProgram;
        std::vector<osg::ref_ptr<Wind> > _winds;
        mutable osg::buffered_object<DrawState> _ds;
        mutable CameraStates _cameraState;
    };

    void StateSwitcherCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
        const osg::Camera* camera = cv->getCurrentCamera();
        CameraState& cs = _d->_cameraState.get(camera); // should exist by now
        cv->pushStateSet(cs._computeStateSet.get());
        traverse(node, nv);
        cv->popStateSet();
    }

    WindDrawable::WindDrawable(const osgDB::Options* readOptions)
    {
        setCullingActive(false);
        setDataVariance(osg::Object::DYNAMIC); // so we can update the wind instances synchronously

        Shaders shaders;
        std::string source = ShaderLoader::load(shaders.WindComputer, shaders, readOptions);
        osg::Shader* computeShader = new osg::Shader(osg::Shader::COMPUTE, source);
        _computeProgram = new osg::Program();
        _computeProgram->addShader(computeShader);
    }

    void WindDrawable::setupPerCameraState(const osg::Camera* camera, int textureImageUnit)
    {
        // wind texture
        osg::Texture3D* tex = new osg::Texture3D();
        tex->setTextureSize(WIND_DIM_X, WIND_DIM_Y, WIND_DIM_Z);
        tex->setInternalFormat(GL_RGBA8);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);

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

        cs._sharedStateSet->addUniform(new osg::Uniform("oe_wind_tex", textureImageUnit));
        cs._sharedStateSet->setTextureAttribute(textureImageUnit, tex, osg::StateAttribute::ON);
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

            if (ds._buffer == INT_MAX || ds._bufferSize < requiredBufferSize)
            {
                if (ds._buffer != INT_MAX)
                    ext->glDeleteBuffers(1, &ds._buffer);

                ext->glGenBuffers(1, &ds._buffer);

                ds._bufferSize = requiredBufferSize;

                ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, ds._buffer);
                ext->glBufferStorage(GL_SHADER_STORAGE_BUFFER, ds._bufferSize, NULL, GL_DYNAMIC_STORAGE_BIT);
            }

            // download to GPU
            ext->glBindBuffer(GL_SHADER_STORAGE_BUFFER, ds._buffer);
            ext->glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, ds._bufferSize, cs._windData);
        }
    }

    void WindDrawable::updateBuffers(CameraState& cs, const osg::Camera* camera)
    {
        if (cs._numWindsAllocated < _winds.size()+1)
        {
            if (cs._windData != NULL)
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
            if (ds._buffer != INT_MAX)
            {
                osg::GLExtensions* ext = state->get<osg::GLExtensions>();
                ext->glDeleteBuffers(1, &ds._buffer);
            }

            CameraState_ReleaseGLObjects r(state);
            _cameraState.forEach(r);
        }
        else
        {
            _cameraState.clear();
        }
    }

    void WindDrawable::resizeGLObjectBuffers(unsigned maxSize)
    {
        _ds.resize(maxSize);
        _cameraState.clear();
    }

    void WindDrawable::drawImplementation(osg::RenderInfo& ri) const
    {
        DrawState& ds = _ds[ri.getState()->getContextID()];
        osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

        // update buffer with wind data
        compileGLObjects(ri, ds);

        // activate layout() binding point:
        ext->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ds._buffer);

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
    radius().setDefault(75.0f);
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
    WindDrawable* wd = static_cast<WindDrawable*>(_drawable.get());
    wd->_winds.push_back(wind);
}

void
WindLayer::removeWind(Wind* wind)
{
    WindDrawable* wd = static_cast<WindDrawable*>(_drawable.get());
    for(std::vector<osg::ref_ptr<Wind> >::iterator i = wd->_winds.begin();
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

void
WindLayer::init()
{
    Layer::init();

    // Never cache decals
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

osg::Node*
WindLayer::getNode() const
{
    return _node.get();
}

void
WindLayer::setTerrainResources(TerrainResources* res)
{
    res->reserveTextureImageUnit(_unit, "WindLayer");

    // Create the wind drawable that will provide a wind texture
    WindDrawable* wd = new WindDrawable(getReadOptions());
    _drawable = wd;

    // chooses the right per-camera stateset for the compute shader
    _node = new StateSwitcher(wd);
    _node->addChild(wd);

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
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

    WindDrawable* windDrawable = static_cast<WindDrawable*>(_drawable.get());

    const osg::Camera* camera = cv->getCurrentCamera();
    CameraState& cs = windDrawable->_cameraState.get(camera);

    if (!cs._computeStateSet.valid())
    {
        windDrawable->setupPerCameraState(camera, _unit.unit());
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
