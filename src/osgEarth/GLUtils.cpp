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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/GLUtils>
#include <osgEarth/Lighting>
#include <osgEarth/StringUtils>
#include <osgEarth/Math>
#include <osgEarth/Utils>

#include <osg/LineStipple>
#include <osg/GraphicsContext>
#include <osgUtil/IncrementalCompileOperation>
#include <osgViewer/GraphicsWindow>

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
#include <osg/LineWidth>
#include <osg/Point>
#endif

using namespace osgEarth;

#define LC "[GLUtils] "

#ifndef GL_LINE_SMOOTH
#define GL_LINE_SMOOTH 0x0B20
#endif

#ifndef GL_POINT_SIZE
#define GL_POINT_SIZE 0x0B11
#endif

#ifndef GL_NORMALIZE
#define GL_NORMALIZE 0x0BA1
#endif


void
GLUtils::setGlobalDefaults(osg::StateSet* stateSet)
{
    // anything that uses a uniform.
    setLineWidth(stateSet, 1.0f, osg::StateAttribute::ON);
    setLineStipple(stateSet, 1, 0xffff, osg::StateAttribute::ON);
    setPointSize(stateSet, 1, osg::StateAttribute::ON);
}

void
GLUtils::setLighting(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setMode(GL_LIGHTING, ov);
#endif

    stateSet->setDefine(OE_LIGHTING_DEFINE, ov);
}

void
GLUtils::setLineWidth(osg::StateSet* stateSet, float value, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setAttributeAndModes(new osg::LineWidth(value), ov);
#endif

    stateSet->addUniform(new osg::Uniform("oe_GL_LineWidth", value), ov);
}

void
GLUtils::setLineStipple(osg::StateSet* stateSet, int factor, unsigned short pattern, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setAttributeAndModes(new osg::LineStipple(factor, pattern), ov);
#endif

    stateSet->addUniform(new osg::Uniform("oe_GL_LineStippleFactor", (int)factor), ov);
    stateSet->addUniform(new osg::Uniform("oe_GL_LineStipplePattern", (int)pattern), ov);
}

void
GLUtils::setLineSmooth(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setMode(GL_LINE_SMOOTH, ov);
#endif

    stateSet->setDefine("OE_LINE_SMOOTH", ov);
}

void
GLUtils::setPointSize(osg::StateSet* stateSet, float value, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setAttributeAndModes(new osg::Point(value), ov);
#endif

    stateSet->addUniform(new osg::Uniform("oe_GL_PointSize", value), ov);
}

void
GLUtils::setPointSmooth(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setMode(GL_POINT_SMOOTH, ov);
#endif

    stateSet->setDefine("OE_POINT_SMOOTH", ov);
}

void
GLUtils::remove(osg::StateSet* stateSet, GLenum cap)
{
    if (!stateSet)
        return;

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    switch(cap)
    {
    case GL_LIGHTING:
        stateSet->removeMode(GL_LIGHTING);
        break;

    case GL_LINE_WIDTH:
        stateSet->removeAttribute(osg::StateAttribute::LINEWIDTH);
        break;

    case GL_LINE_STIPPLE:
        stateSet->removeAttribute(osg::StateAttribute::LINESTIPPLE);
        break;

    case GL_LINE_SMOOTH:
        stateSet->removeMode(GL_LINE_SMOOTH);
        break;

    case GL_POINT_SIZE:
        stateSet->removeAttribute(osg::StateAttribute::POINT);
        break;

    default:
        stateSet->removeMode(cap);
        break;
    }
#endif

    switch(cap)
    {   
    case GL_LIGHTING:
        stateSet->removeDefine(OE_LIGHTING_DEFINE);
        break;

    case GL_LINE_WIDTH:
        stateSet->removeUniform("oe_GL_LineWidth");
        break;

    case GL_LINE_STIPPLE:
        stateSet->removeUniform("oe_GL_LineStippleFactor");
        stateSet->removeUniform("oe_GL_LineStipplePattern");
        break;

    case GL_LINE_SMOOTH:
        stateSet->removeDefine("OE_LINE_SMOOTH");
        break;

    case GL_POINT_SIZE:
        stateSet->removeUniform("oe_GL_PointSize");
        break;
    }
}

void
CustomRealizeOperation::setSyncToVBlank(bool value)
{
    _vsync = value;
}

void
CustomRealizeOperation::operator()(osg::Object* object)
{
    if (_vsync.isSet())
    {
        osgViewer::GraphicsWindow* win = dynamic_cast<osgViewer::GraphicsWindow*>(object);
        if (win)
        {
            win->setSyncToVBlank(_vsync.get());
        }
    }
}

void
GL3RealizeOperation::operator()(osg::Object* object)
{
    osg::GraphicsContext* gc = dynamic_cast<osg::GraphicsContext*>(object);
    if (gc)
    {
        osg::State* state = gc->getState();

        // force NVIDIA-style vertex attribute aliasing, since osgEarth
        // makes use of some specific attribute registers. Later we can
        // perhaps create a reservation system for this.
        state->resetVertexAttributeAlias(false);

#ifdef OSG_GL3_AVAILABLE
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);
#endif

#ifndef OSG_GL_FIXED_FUNCTION_AVAILABLE
        state->setModeValidity(GL_LIGHTING, false);
        state->setModeValidity(GL_NORMALIZE, false);
        state->setModeValidity(GL_RESCALE_NORMAL, false);
        state->setModeValidity(GL_LINE_STIPPLE, false);
        state->setModeValidity(GL_LINE_SMOOTH, false);
#endif
    }

    CustomRealizeOperation::operator()(object);
}



GLBufferReleaser::GLBufferReleaser(GLBuffer* buffer) : 
    osg::GraphicsOperation("osgEarth::GLBufferReleaser", true),
    _buffer(buffer),
    _handle(buffer->_handle)
{
    //nop
}

void
GLBufferReleaser::operator () (osg::GraphicsContext* context)
{
    if (!_buffer.valid() && _handle != (GLuint)~0 && context && context->getState())
    {
        OE_DEBUG << "Note: glDeleteBuffers(1, " << _handle << ")" << std::endl;
        osg::GLExtensions* ext = context->getState()->get<osg::GLExtensions>();
        ext->glDeleteBuffers(1, &_handle);
        _handle = (GLuint)~0;
        setKeep(false);
    }
}

#undef LC
#define LC "[GPUJobArena] "

#if 0
GPUJobArena<bool>::Result
GPUJobArena::dispatchOnAllContexts(
    GPUJobArena::Function& function)
{
    if (_gcs.size() == 1)
    {
        return dispatch(function);
    }

    else
    {
        Job<bool>::dispatch(
            [...](Cancelable* progress)
            {
                std::queue<Future<bool>> futures;

                for (auto& gc : _gcs)
                {
                    Future<bool> f = arena(gc).dispatch(...);
                    futures.emplace_back(f);
                }

                while (futures.empty() == false)
                {
                    futures.front().get(progress);
                    futures.pop_front();
                }
            }
        );
    }
}
#endif


//static defs
osg::ref_ptr<GPUJobArena> GPUJobArena::_arena_pool;
Mutex GPUJobArena::_arena_pool_mutex;

GPUJobArena&
GPUJobArena::arena()
{
    ScopedMutexLock lock(_arena_pool_mutex);

    if (_arena_pool.valid() == false)
    {
        _arena_pool = new GPUJobArena();
    }

    return *_arena_pool.get();
}

GPUJobArena::GPUJobArena() :
    osg::GraphicsOperation("oe.GPUJobArena", true),
    _timeSlice(2), // default time slice (milliseconds)
    _done(false)
{
    const char* value = ::getenv("OSGEARTH_GPU_TIME_SLICE_MS");
    if (value)
    {
        _timeSlice = std::chrono::milliseconds(clamp(atoi(value), 1, 1000));
    }
}

GPUJobArena::~GPUJobArena()
{
    setGraphicsContext(nullptr);
}

void
GPUJobArena::setGraphicsContext(osg::GraphicsContext* gc)
{
    if (gc != _gc.get() || gc == nullptr)
    {
        osg::ref_ptr<osg::GraphicsContext> old_gc(_gc);
        if (old_gc.valid())
        {
            old_gc->remove(this);
        }

        _gc = nullptr;

        if (gc)
        {
            _gc = gc;
            gc->add(this);
            OE_INFO << LC << getName() << " attached to GC " << std::hex << gc << std::dec << std::endl;
        }
    }
}

osg::ref_ptr<osg::GraphicsContext>
GPUJobArena::getGraphicsContext()
{
    return osg::ref_ptr<osg::GraphicsContext>(_gc);
}

void
GPUJobArena::setTimeSlice(const std::chrono::milliseconds& value)
{
    _timeSlice = value;
}

const std::chrono::milliseconds&
GPUJobArena::getTimeSlice() const
{
    return _timeSlice;
}

void
GPUJobArena::dispatch(Delegate& del)
{
    std::unique_lock<Mutex> lock(_queue_mutex);
    _queue.emplace_back(del);
}

void
GPUJobArena::operator()(osg::GraphicsContext* gc)
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::milliseconds ms;

    // always run at least one job.
    Clock::time_point start = Clock::now();
    while (!_done)
    {
        Delegate next;

        _queue_mutex.lock();
        if (!_queue.empty() && !_done)
        {
            next = std::move(_queue.front());
            _queue.pop_front();
        }
        _queue_mutex.unlock();

        if (next != nullptr)
        {
            // run the job
            next(gc->getState());

            // check the time slice:
            auto timeElapsed = std::chrono::duration_cast<ms>(Clock::now() - start);
            if (timeElapsed >= _timeSlice)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
}

GPUJobArenaConnector::GPUJobArenaConnector() :
    osg::Drawable()
{
    // ensure it doesn't get culled out
    setCullingActive(false);

    // ensure the draw runs synchronously:
    setDataVariance(DYNAMIC);

    // force the draw to run every frame:
    setUseDisplayList(false);
}

GPUJobArenaConnector::~GPUJobArenaConnector()
{
    GPUJobArena& arena = GPUJobArena::arena();
    arena.setGraphicsContext(nullptr);
}

void
GPUJobArenaConnector::drawImplementation(osg::RenderInfo& ri) const
{
    GPUJobArena& arena = GPUJobArena::arena();

    if (arena.getGraphicsContext().valid() == false)
    {
        arena.setGraphicsContext(ri.getState()->getGraphicsContext());
    }
}


namespace
{
    using ICO = osgUtil::IncrementalCompileOperation;

    struct ICOCallback : public ICO::CompileCompletedCallback
    {
        Promise<bool> _promise;

        bool compileCompleted(ICO::CompileSet* compileSet) override
        {
            _promise.resolve(true);
            return true;
        }
    };
}

Future<bool>
GLObjectsCompiler::compileAsync(
    osg::Node* node,
    const osgDB::Options* options,
    Cancelable* progress) const
{
    Future<bool> result;
    if (node)
    {
        // if there is an ICO available, schedule the GPU compilation
        osg::ref_ptr<ICO> ico;
        if (OptionsData<ICO>::get(options, ico))
        {
            auto compileSet = new osgUtil::IncrementalCompileOperation::CompileSet(node);

            ICOCallback* callback = new ICOCallback();
            result = callback->_promise.getFuture();
            compileSet->_compileCompletedCallback = callback;

            ico->add(compileSet);
        }
    }
    return result;
}


void
GLObjectsCompiler::compileNow(
    osg::Node* node,
    const osgDB::Options* options,
    Cancelable* progress) const
{
    if (node)
    {
        Future<bool> result = compileAsync(node, options, progress);
        result.join(progress);
    }
}



#ifndef OE_HAVE_BINDIMAGETEXTURE
using namespace osg;

int BindImageTexture::compare(const osg::StateAttribute &sa) const
{
    COMPARE_StateAttribute_Types(BindImageTexture,sa)
        // Compare each parameter in turn against the rhs.
        COMPARE_StateAttribute_Parameter(_target)
        COMPARE_StateAttribute_Parameter(_imageunit)
        COMPARE_StateAttribute_Parameter(_access)
        COMPARE_StateAttribute_Parameter(_format)
        COMPARE_StateAttribute_Parameter(_layered)
        COMPARE_StateAttribute_Parameter(_level)
        return 0;
}

void BindImageTexture::apply(osg::State& state) const
{
    if(_target.valid())
    {
        unsigned int contextID = state.getContextID();
        osg::Texture::TextureObject *to = _target->getTextureObject( contextID );
        if( !to ) //|| _target->isDirty( contextID ))
        {
            // _target never been applied yet or is dirty
            state.applyTextureAttribute( state.getActiveTextureUnit(), _target.get());
            to = _target->getTextureObject( contextID );
        }

        state.get<osg::GLExtensions>()->glBindImageTexture(_imageunit, to->id(), _level, _layered, _layer, _access, _format);
    }
}

#endif
