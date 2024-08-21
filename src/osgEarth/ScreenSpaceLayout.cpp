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
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/Utils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Extension>
#include <osgEarth/Text>
#include <osgEarth/LineDrawable>
#include <osgEarth/GLUtils>
#include "ScreenSpaceLayoutDeclutter"
#include "ScreenSpaceLayoutCallout"

#include <stdlib.h> // getenv

#define LC "[ScreenSpaceLayout] "

using namespace osgEarth;
using namespace osgEarth::Internal;

//----------------------------------------------------------------------------

bool osgEarth::ScreenSpaceLayout::globallyEnabled = true;


//----------------------------------------------------------------------------

void
ScreenSpaceLayoutOptions::fromConfig( const Config& conf )
{
    conf.get( "min_animation_scale", _minAnimScale );
    conf.get( "min_animation_alpha", _minAnimAlpha );
    conf.get( "in_animation_time",   _inAnimTime );
    conf.get( "out_animation_time",  _outAnimTime );
    conf.get( "sort_by_priority",    _sortByPriority );
    conf.get( "sort_by_distance",    _sortByDistance);
    conf.get( "snap_to_pixel",       _snapToPixel );
    conf.get( "max_objects",         _maxObjects );
    conf.get( "render_order",        _renderBinNumber );
    conf.get( "technique", "labels", _technique, TECHNIQUE_LABELS );
    conf.get( "technique", "callouts", _technique, TECHNIQUE_CALLOUTS );
    conf.get( "max_leader_length", _leaderLineMaxLen ); // backwards compat
    conf.get( "leader_line_max_length", _leaderLineMaxLen );
    conf.get( "leader_line_color", _leaderLineColor );
    conf.get( "leader_line_width", _leaderLineWidth );
}

Config
ScreenSpaceLayoutOptions::getConfig() const
{
    Config conf;
    conf.set( "min_animation_scale", _minAnimScale );
    conf.set( "min_animation_alpha", _minAnimAlpha );
    conf.set( "in_animation_time",   _inAnimTime );
    conf.set( "out_animation_time",  _outAnimTime );
    conf.set( "sort_by_priority",    _sortByPriority );
    conf.set( "sort_by_distance",    _sortByDistance);
    conf.set( "snap_to_pixel",       _snapToPixel );
    conf.set( "max_objects",         _maxObjects );
    conf.set( "render_order",        _renderBinNumber );
    conf.set( "technique", "labels", _technique, TECHNIQUE_LABELS);
    conf.set( "technique", "callouts", _technique, TECHNIQUE_CALLOUTS);
    conf.set( "leader_line_max_length", _leaderLineMaxLen );
    conf.set( "leader_line_color", _leaderLineColor );
    conf.set( "leader_line_width", _leaderLineWidth );
    return conf;
}


namespace
{
    /**
    * The actual custom render bin
    * This wants to be in the global scope for the dynamic registration to work,
    * hence the annoyinging long class name
    */
    class osgEarthScreenSpaceLayoutRenderBin : public osgUtil::RenderBin
    {
    public:
        osgEarthScreenSpaceLayoutRenderBin()
        {
            _vpInstalled = false;

            this->setName( OSGEARTH_SCREEN_SPACE_LAYOUT_BIN );
            _context = new ScreenSpaceLayoutContext();
            clearSortingFunctor();

            // needs its own state set for special magic.
            osg::StateSet* stateSet = new osg::StateSet();
            this->setStateSet( stateSet );
        }

        osgEarthScreenSpaceLayoutRenderBin(const osgEarthScreenSpaceLayoutRenderBin& rhs, const osg::CopyOp& copy)
            : osgUtil::RenderBin(rhs, copy),
            _f(rhs._f.get()),
            _context(rhs._context.get())
        {
            // Set up a VP to do fading. Do it here so it doesn't happen until the first time
            // we clone the render bin. This play nicely with static initialization.
            if (!_vpInstalled)
            {
                std::lock_guard<std::mutex> lock(_vpMutex);
                if (!_vpInstalled)
                {
                    VirtualProgram* vp = VirtualProgram::getOrCreate(getStateSet());
                    vp->setName(typeid(*this).name());
                    vp->setFunction( "oe_declutter_apply_fade", s_faderFS, VirtualProgram::LOCATION_FRAGMENT_COLORING, 0.5f );
                    _vpInstalled = true;
                    //OE_INFO << LC << "Decluttering VP installed\n";
                }
            }
        }

        virtual osg::Object* clone(const osg::CopyOp& copyop) const
        {
            return new osgEarthScreenSpaceLayoutRenderBin(*this, copyop);
        }

        void setSortingFunctor( DeclutterSortFunctor* f )
        {
            _f = f;

            if (_context->_options.technique() == ScreenSpaceLayoutOptions::TECHNIQUE_LABELS)
            {
                setSortCallback(new DeclutterImplementation(_context.get(), f));
                setDrawCallback(new DeclutterDraw(_context.get()));
            }
            else
            {
                CalloutImplementation* impl = new CalloutImplementation(_context.get(), f);
                setSortCallback(impl);
                CalloutDraw* draw = new CalloutDraw(_context.get());
                draw->_sortCallback = impl;
                setDrawCallback(draw);
            }
        }

        void clearSortingFunctor()
        {
            setSortingFunctor(NULL);
        }

        void refresh()
        {
            setSortingFunctor(_f.get());
        }

        osg::ref_ptr<DeclutterSortFunctor> _f;
        osg::ref_ptr<ScreenSpaceLayoutContext> _context;
        static std::mutex _vpMutex;
        static bool _vpInstalled;
    };

    std::mutex osgEarthScreenSpaceLayoutRenderBin::_vpMutex;
    bool osgEarthScreenSpaceLayoutRenderBin::_vpInstalled = false;
}

//----------------------------------------------------------------------------

void
ScreenSpaceLayout::activate(osg::StateSet* stateSet) //, int binNum)
{
    if ( stateSet )
    {
        int binNum = getOptions().renderOrder().get();

        // the OVERRIDE prevents subsequent statesets from disabling the layout bin
        stateSet->setRenderBinDetails(
            binNum,
            OSGEARTH_SCREEN_SPACE_LAYOUT_BIN,
            osg::StateSet::OVERRIDE_PROTECTED_RENDERBIN_DETAILS);

        // Force a single shared layout bin per render stage
        stateSet->setNestRenderBins( false );

        // Range opacity is not supported for screen-space rendering
        stateSet->setDefine("OE_DISABLE_RANGE_OPACITY");
    }
}

void
ScreenSpaceLayout::deactivate(osg::StateSet* stateSet)
{
    if (stateSet)
    {
        stateSet->setRenderBinToInherit();
        stateSet->setNestRenderBins(true);
    }
}

void
ScreenSpaceLayout::setDeclutteringEnabled(bool enabled)
{
    globallyEnabled = enabled;
}

void
ScreenSpaceLayout::setSortFunctor( DeclutterSortFunctor* functor )
{
    // pull our prototype
    osgEarthScreenSpaceLayoutRenderBin* bin = dynamic_cast<osgEarthScreenSpaceLayoutRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_SCREEN_SPACE_LAYOUT_BIN ) );

    if ( bin )
    {
        bin->setSortingFunctor( functor );
    }
}

void
ScreenSpaceLayout::clearSortFunctor()
{
    // pull our prototype
    osgEarthScreenSpaceLayoutRenderBin* bin = dynamic_cast<osgEarthScreenSpaceLayoutRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_SCREEN_SPACE_LAYOUT_BIN ) );

    if ( bin )
    {
        bin->clearSortingFunctor();
    }
}

void
ScreenSpaceLayout::setOptions( const ScreenSpaceLayoutOptions& options )
{
    // pull our prototype
    osgEarthScreenSpaceLayoutRenderBin* bin = dynamic_cast<osgEarthScreenSpaceLayoutRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_SCREEN_SPACE_LAYOUT_BIN ) );

    if ( bin )
    {
        // activate priority-sorting through the options.
        if ( options.sortByPriority().isSetTo( true ) &&
            bin->_context->_options.sortByPriority() == false )
        {
            ScreenSpaceLayout::setSortFunctor(new SortByPriorityPreservingGeodeTraversalOrder());
        }

        // communicate the new options on the shared context.
        bin->_context->_options = options;

        bin->refresh();
    }
}

const ScreenSpaceLayoutOptions&
ScreenSpaceLayout::getOptions()
{
    static ScreenSpaceLayoutOptions s_defaultOptions;

    // pull our prototype
    osgEarthScreenSpaceLayoutRenderBin* bin = dynamic_cast<osgEarthScreenSpaceLayoutRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_SCREEN_SPACE_LAYOUT_BIN ) );

    if ( bin )
    {
        return bin->_context->_options;
    }
    else
    {
        return s_defaultOptions;
    }
}

//----------------------------------------------------------------------------

namespace
{
    /** the actual registration. */
    extern "C" void osgEarth_declutter(void) {}
    static osgEarthRegisterRenderBinProxy<osgEarthScreenSpaceLayoutRenderBin> s_regbin(OSGEARTH_SCREEN_SPACE_LAYOUT_BIN);
}


//----------------------------------------------------------------------------

// Extension for configuring the decluterring/SSL options from an Earth file.
namespace osgEarth
{
    class ScreenSpaceLayoutExtension : public Extension,
        public ScreenSpaceLayoutOptions
    {
    public:
        META_OE_Extension(osgEarth, ScreenSpaceLayoutExtension, screen_space_layout);

        ScreenSpaceLayoutExtension() { }

        ScreenSpaceLayoutExtension(const ConfigOptions& co) : ScreenSpaceLayoutOptions(co)
        {
            // sets the global default options.
            ScreenSpaceLayout::setOptions(*this);
        }

        const ConfigOptions& getConfigOptions() const { return *this; }
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_screen_space_layout, ScreenSpaceLayoutExtension);
    REGISTER_OSGEARTH_EXTENSION(osgearth_screenspacelayout,   ScreenSpaceLayoutExtension);
    REGISTER_OSGEARTH_EXTENSION(osgearth_decluttering,        ScreenSpaceLayoutExtension);
}

