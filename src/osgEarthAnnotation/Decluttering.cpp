/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationData>
#include <osgEarth/Utils>
#include <osgEarth/ThreadingUtils>
#include <osgUtil/RenderBin>
#include <osgUtil/StateGraph>
#include <osgText/Text>
#include <set>
#include <algorithm>

#define LC "[Declutter] "

using namespace osgEarth;
using namespace osgEarth::Annotation;

//----------------------------------------------------------------------------

namespace
{
    // Sort wrapper to satisfy the template processor.
    struct SortContainer
    {
        SortContainer( DeclutterSortFunctor& f ) : _f(f) { }
        const DeclutterSortFunctor& _f;
        bool operator()( const osgUtil::RenderLeaf* lhs, const osgUtil::RenderLeaf* rhs ) const 
        {
            return _f(lhs, rhs);
        }
    };

    // Data structure shared across entire decluttering system.
    struct DeclutterContext : public osg::Referenced
    {
        DeclutteringOptions _options;
    };

    // records information about each drawable.
    // TODO: a way to clear out this list when drawables go away
    struct DrawableInfo
    {
        DrawableInfo() : _lastAlpha(1.0), _lastScale(1.0) { }
        float _lastAlpha, _lastScale;
    };

    typedef std::map<const osg::Drawable*, DrawableInfo> DrawableMemory;
    
    typedef std::pair<const osg::Node*, osg::BoundingBox> RenderLeafBox;

    // Data structure stored one-per-View.
    struct PerViewInfo
    {
        PerViewInfo() : _lastTimeStamp(0.0) { }

        // remembers the state of each drawable from the previous pass
        DrawableMemory _memory;
        
        // re-usable structures (to avoid unnecessary re-allocation)
        osgUtil::RenderBin::RenderLeafList _passed;
        osgUtil::RenderBin::RenderLeafList _failed;
        std::vector<RenderLeafBox>         _used;

        // time stamp of the previous pass, for calculating animation speed
        double _lastTimeStamp;
    };
}

//----------------------------------------------------------------------------

void
DeclutteringOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "min_animation_scale", _minAnimScale );
    conf.getIfSet( "min_animation_alpha", _minAnimAlpha );
    conf.getIfSet( "in_animation_time",   _inAnimTime );
    conf.getIfSet( "out_animation_time",  _outAnimTime );
    conf.getIfSet( "sort_by_priority",    _sortByPriority );
}

Config
DeclutteringOptions::getConfig() const
{
    Config conf;
    conf.addIfSet( "min_animation_scale", _minAnimScale );
    conf.addIfSet( "min_animation_alpha", _minAnimAlpha );
    conf.addIfSet( "in_animation_time",   _inAnimTime );
    conf.addIfSet( "out_animation_time",  _outAnimTime );
    conf.addIfSet( "sort_by_priority",    _sortByPriority );
    return conf;
}

//----------------------------------------------------------------------------

/**
 * A custom RenderLeaf sorting algorithm for decluttering objects.
 *
 * First we sort the leaves front-to-back so that objects closer to the camera
 * get higher priority. If you have installed a custom sorting functor,
 * this is used instead.
 *
 * Next, we go though all the drawables and remove any that try to occupy
 * already-occupied real estate in the 2D viewport. Objects that fail the test
 * go on a "failed" list and are either completely removed from the display
 * or transitioned to a secondary visual state (scaled down, alpha'd down)
 * dependeing on the options setup.
 *
 * Drawables with the same parent (i.e., Geode) are treated as a group. As
 * soon as one passes the occlusion test, all its siblings will automatically
 * pass as well.
 */
struct /*internal*/ DeclutterSort : public osgUtil::RenderBin::SortCallback
{
    DeclutterSortFunctor* _customSortFunctor;
    DeclutterContext*     _context;

    Threading::PerObjectMap<osg::View*, PerViewInfo> _perView;

    /**
     * Constructs the new sorter.
     * @param f Custom declutter sorting predicate. Pass NULL to use the 
     *          default sorter (sort by distance-to-camera).
     */
    DeclutterSort( DeclutterContext* context, DeclutterSortFunctor* f = 0L )
        : _context(context), _customSortFunctor(f)
    {
        //nop
    }

    // override.
    // Sorts the bin. This runs in the CULL thread after the CULL traversal has completed.
    void sortImplementation(osgUtil::RenderBin* bin)
    {
        osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();

        // first, sort the leaves:
        if ( _customSortFunctor )
        {
            // if there's a custom sorting function installed
            bin->copyLeavesFromStateGraphListToRenderLeafList();
            std::sort( leaves.begin(), leaves.end(), SortContainer( *_customSortFunctor ) );
        }
        else
        {
            // default behavior is to sort by depth.
            bin->sortFrontToBack();
        }

        // nothing to sort? bail out
        if ( leaves.size() == 0 )
            return;

        // access the view-specific persistent data:
        osg::View*   view  = bin->getStage()->getCamera()->getView();
        PerViewInfo& local = _perView.get( bin->getStage()->getCamera()->getView() );   
        
        // calculate the elapsed time since the previous pass; we'll use this for
        // the animations
        double now = view->getFrameStamp()->getReferenceTime();
        float elapsedSeconds = float(now - local._lastTimeStamp);
        local._lastTimeStamp = now;

        // Reset the local re-usable containers
        local._passed.clear();          // drawables that pass occlusion test
        local._failed.clear();          // drawables that fail occlusion test
        local._used.clear();            // list of occupied bounding boxes in screen space

        // compute a window matrix so we can do window-space culling:
        const osg::Viewport* vp = bin->getStage()->getCamera()->getViewport();
        osg::Matrix windowMatrix = vp->computeWindowMatrix();

        // Track the parent nodes of drawables that are obscured (and culled). Drawables
        // with the same parent node (typically a Geode) are considered to be grouped and
        // will be culled as a group.
        std::set<const osg::Node*> culledParents;

        const DeclutteringOptions& options = _context->_options;

        // Go through each leaf and test for visibility.
        for( osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin(); i != leaves.end(); ++i )
        {
            bool visible = true;

            osgUtil::RenderLeaf* leaf = *i;
            const osg::Drawable* drawable = leaf->getDrawable();
            const osg::Node*     drawableParent = drawable->getParent(0);

            // transform the bounding box of the drawable into window-space.
            osg::BoundingBox box = drawable->getBound();
            static osg::Vec4d s_zero_w(0,0,0,1);
            osg::Vec4d clip = s_zero_w * (*leaf->_modelview.get()) * (*leaf->_projection.get());
            osg::Vec3d clip_ndc( clip.x()/clip.w(), clip.y()/clip.w(), clip.z()/clip.w() );
            osg::Vec3f winPos = clip_ndc * windowMatrix;
            osg::Vec2f offset( -box.xMin(), -box.yMin() );
            box.set(
                winPos.x() + box.xMin(),
                winPos.y() + box.yMin(),
                winPos.z(),
                winPos.x() + box.xMax(),
                winPos.y() + box.yMax(),
                winPos.z() );

            // if this leaf is already in a culled group, skip it.
            if ( culledParents.find(drawableParent) != culledParents.end() )
            {
                visible = false;
            }
            else
            {
                // weed out any drawables that are obscured by closer drawables.
                // TODO: think about a more efficient algorithm - right now we are just using
                // brute force to compare all bbox's
                for( std::vector<RenderLeafBox>::const_iterator j = local._used.begin(); j != local._used.end(); ++j )
                {
                    // only need a 2D test since we're in clip space
                    bool isClear =
                        box.xMin() > j->second.xMax() ||
                        box.xMax() < j->second.xMin() ||
                        box.yMin() > j->second.yMax() ||
                        box.yMax() < j->second.yMin();

                    // if there's an overlap (and the conflict isn't from the same drawable
                    // parent, which is acceptable), then the leaf is culled.
                    if ( !isClear && drawableParent != j->first )
                    {
                        visible = false;
                        break;
                    }
                }
            }

            if ( visible )
            {
                // passed the test, so add the leaf's bbox to the "used" list, and add the leaf
                // to the final draw list.
                local._used.push_back( std::make_pair(drawableParent, box) );
                local._passed.push_back( leaf );
            }

            else
            {
                // culled, so put the parent in the parents list so that any future leaves
                // with the same parent will be trivially rejected
                culledParents.insert( drawable->getParent(0) );
                local._failed.push_back( leaf );
            }

            // modify the leaf's modelview matrix to correctly position it in the 2D ortho
            // projection when it's drawn later. (Note: we need a new RefMatrix since the
            // original might be shared ... potential optimization here)
            leaf->_modelview = new osg::RefMatrix( osg::Matrix::translate(
                box.xMin() + offset.x(),
                box.yMin() + offset.y(), 
                0) );
        }

        // copy the final draw list back into the bin, rejecting any leaves whose parents
        // are in the cull list.

        leaves.clear();
        for( osgUtil::RenderBin::RenderLeafList::const_iterator i=local._passed.begin(); i != local._passed.end(); ++i )
        {
            osgUtil::RenderLeaf* leaf     = *i;
            const osg::Drawable* drawable = leaf->getDrawable();

            if ( culledParents.find( drawable->getParent(0) ) == culledParents.end() )
            {
                DrawableInfo& info = local._memory[drawable];

                bool fullyIn = true;

                // scale in until at full scale:
                if ( info._lastScale != 1.0f )
                {
                    fullyIn = false;
                    info._lastScale += elapsedSeconds / std::max(*options.inAnimationTime(), 0.001f);
                    if ( info._lastScale > 1.0f )
                        info._lastScale = 1.0f;
                }

                if ( info._lastScale != 1.0f )
                    leaf->_modelview->preMult( osg::Matrix::scale(info._lastScale,info._lastScale,1) );
                
                // fade in until at full alpha:
                if ( info._lastAlpha != 1.0f )
                {
                    fullyIn = false;
                    info._lastAlpha += elapsedSeconds / std::max(*options.inAnimationTime(), 0.001f);
                    if ( info._lastAlpha > 1.0f )
                        info._lastAlpha = 1.0f;
                }

                leaf->_depth = info._lastAlpha;
                leaves.push_back( leaf );
            }
            else
            {
                local._failed.push_back(leaf);
            }
        }

        // next, go through the FAILED list and sort them into failure bins so we can draw
        // them using a different technique if necessary.
        for( osgUtil::RenderBin::RenderLeafList::const_iterator i=local._failed.begin(); i != local._failed.end(); ++i )
        {
            osgUtil::RenderLeaf* leaf =     *i;
            const osg::Drawable* drawable = leaf->getDrawable();

            DrawableInfo& info = local._memory[drawable];

            bool isText = dynamic_cast<const osgText::Text*>(drawable) != 0L;
            bool fullyOut = true;

            if ( info._lastScale != *options.minAnimationScale() )
            {
                fullyOut = false;
                info._lastScale -= elapsedSeconds / std::max(*options.outAnimationTime(), 0.001f);
                if ( info._lastScale < *options.minAnimationScale() )
                    info._lastScale = *options.minAnimationScale();
            }

            if ( info._lastAlpha != *options.minAnimationAlpha() )
            {
                fullyOut = false;
                info._lastAlpha -= elapsedSeconds / std::max(*options.outAnimationTime(), 0.001f);
                if ( info._lastAlpha < *options.minAnimationAlpha() )
                    info._lastAlpha = *options.minAnimationAlpha();
            }

            leaf->_depth = info._lastAlpha;

            if ( !isText || !fullyOut )
            {
                if ( info._lastAlpha > 0.01f && info._lastScale >= 0.0f )
                {
                    leaves.push_back( leaf );

                    // scale it:
                    if ( info._lastScale != 1.0f )
                        leaf->_modelview->preMult( osg::Matrix::scale(info._lastScale,info._lastScale,1) );
                }
            }

        }
    }
};

/**
 * Custom draw routine for our declutter render bin.
 */
struct DeclutterDraw : public osgUtil::RenderBin::DrawCallback
{
    DeclutterContext*                                    _context;
    Threading::PerThread< osg::ref_ptr<osg::RefMatrix> > _ortho2D;
    osg::ref_ptr<osg::Uniform> _fade;

    /**
     * Constructs the decluttering draw callback.
     * @param context A shared context among all decluttering objects.
     */
    DeclutterDraw( DeclutterContext* context )
        : _context( context )
    {
        // create the fade uniform.
        _fade = AnnotationUtils::createFadeUniform();
        _fade->set( 1.0f );
    }

    /**
     * Draws a bin. Most of this code is copied from osgUtil::RenderBin::drawImplementation.
     * The modifications are (a) skipping code to render child bins, (b) setting a bin-global
     * projection matrix in orthographic space, and (c) calling our custom "renderLeaf()" method 
     * instead of RenderLeaf::render()
     */
    void drawImplementation( osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous )
    {
        osg::State& state = *renderInfo.getState();

        unsigned int numToPop = (previous ? osgUtil::StateGraph::numToPop(previous->_parent) : 0);
        if (numToPop>1) --numToPop;
        unsigned int insertStateSetPosition = state.getStateSetStackSize() - numToPop;

        if (bin->getStateSet())
        {
            state.insertStateSet(insertStateSetPosition, bin->getStateSet());
        }

        // apply a window-space projection matrix.
        const osg::Viewport* vp = renderInfo.getCurrentCamera()->getViewport();
        if ( vp )
        {
            //TODO see which is faster

            osg::ref_ptr<osg::RefMatrix>& m = _ortho2D.get();
            if ( !m.valid() )
                m = new osg::RefMatrix();

            m->makeOrtho2D( vp->x(), vp->x()+vp->width()-1, vp->y(), vp->y()+vp->height()-1 );
            state.applyProjectionMatrix( m.get() );

            //osg::ref_ptr<osg::RefMatrix> rm = new osg::RefMatrix( osg::Matrix::ortho2D(
            //    vp->x(), vp->x()+vp->width()-1,
            //    vp->y(), vp->y()+vp->height()-1 ) );
            //state.applyProjectionMatrix( rm.get() );
        }

        // render the list
        osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();
        for(osgUtil::RenderBin::RenderLeafList::reverse_iterator rlitr = leaves.rbegin();
            rlitr!= leaves.rend();
            ++rlitr)
        {
            osgUtil::RenderLeaf* rl = *rlitr;
            renderLeaf( rl, renderInfo, previous );
            previous = rl;
        }

        if ( bin->getStateSet() )
        {
            state.removeStateSet(insertStateSetPosition);
        }
    }

    /**
     * Renders a single leaf. We already applied the projection matrix, so here we only
     * need to apply a modelview matrix that specifies the ortho offset of the drawable.
     *
     * Most of this code is copied from RenderLeaf::draw() -- but I removed all the code
     * dealing with nested bins, since decluttering does not support them.
     */
    void renderLeaf( osgUtil::RenderLeaf* leaf, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous )
    {
        osg::State& state = *renderInfo.getState();

        // don't draw this leaf if the abort rendering flag has been set.
        if (state.getAbortRendering())
        {
            //cout << "early abort"<<endl;
            return;
        }

        state.applyModelViewMatrix( leaf->_modelview.get() );

        if (previous)
        {
            // apply state if required.
            osgUtil::StateGraph* prev_rg = previous->_parent;
            osgUtil::StateGraph* prev_rg_parent = prev_rg->_parent;
            osgUtil::StateGraph* rg = leaf->_parent;
            if (prev_rg_parent!=rg->_parent)
            {
                osgUtil::StateGraph::moveStateGraph(state,prev_rg_parent,rg->_parent);

                // send state changes and matrix changes to OpenGL.
                state.apply(rg->getStateSet());

            }
            else if (rg!=prev_rg)
            {
                // send state changes and matrix changes to OpenGL.
                state.apply(rg->getStateSet());
            }
        }
        else
        {
            // apply state if required.
            osgUtil::StateGraph::moveStateGraph(state,NULL,leaf->_parent->_parent);

            state.apply(leaf->_parent->getStateSet());
        }

        // if we are using osg::Program which requires OSG's generated uniforms to track
        // modelview and projection matrices then apply them now.
        if (state.getUseModelViewAndProjectionUniforms()) 
            state.applyModelViewAndProjectionUniformsIfRequired();
        
        // apply the fading uniform
        const osg::Program::PerContextProgram* pcp = state.getLastAppliedProgramObject();
        if ( pcp )
        {
            // todo: find a way to optimizer this..?
            _fade->set( leaf->_depth );
            pcp->apply( *_fade.get() );
        }
    
        // draw the drawable
        leaf->_drawable->draw(renderInfo);
        
        if (leaf->_dynamic)
        {
            state.decrementDynamicObjectCount();
        }
    }
};

//----------------------------------------------------------------------------

/**
 * The actual custom render bin
 * This wants to be in the global scope for the dynamic registration to work,
 * hence the annoyinging long class name
 */
class osgEarthAnnotationDeclutterRenderBin : public osgUtil::RenderBin
{
public:
    osgEarthAnnotationDeclutterRenderBin()
    {
        this->setName( OSGEARTH_DECLUTTER_BIN );
        _context = new DeclutterContext();
        clearSortingFunctor();
        setDrawCallback( new DeclutterDraw(_context.get()) );
    }

    void setSortingFunctor( DeclutterSortFunctor* f )
    {
        _f = f;
        setSortCallback( new DeclutterSort(_context.get(), f) );
    }

    void clearSortingFunctor()
    {
        setSortCallback( new DeclutterSort(_context.get()) );
    }

    osg::ref_ptr<DeclutterSortFunctor> _f;
    osg::ref_ptr<DeclutterContext>     _context;
};
//const std::string osgEarthAnnotationDeclutterRenderBin::BIN_NAME = OSGEARTH_DECLUTTER_BIN;

//----------------------------------------------------------------------------

//static

void
Decluttering::setEnabled( osg::StateSet* stateSet, bool enabled, int binNum )
{
    if ( stateSet )
    {
        if ( enabled )
        {
            osg::Program* p = dynamic_cast<osg::Program*>(stateSet->getAttribute(osg::StateAttribute::PROGRAM));
            if ( p == 0L || p->getName() != AnnotationUtils::PROGRAM_NAME() )
            {
                stateSet->setAttributeAndModes( AnnotationUtils::getAnnotationProgram(), 1 );
            }

            // just installs a default highlight uniform so the shader has default value.
            if ( !stateSet->getUniform( AnnotationUtils::UNIFORM_HIGHLIGHT() ) )
            {
                stateSet->addUniform( AnnotationUtils::createHighlightUniform() );
            }

            stateSet->setRenderBinDetails( binNum, OSGEARTH_DECLUTTER_BIN );
        }
        else
        {
            stateSet->removeAttribute( osg::StateAttribute::PROGRAM );
            stateSet->setRenderBinToInherit();
        }
    }
}

void
Decluttering::setSortFunctor( DeclutterSortFunctor* functor )
{
    // pull our prototype
    osgEarthAnnotationDeclutterRenderBin* bin = dynamic_cast<osgEarthAnnotationDeclutterRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_DECLUTTER_BIN ) );

    if ( bin )
    {
        bin->setSortingFunctor( functor );
    }
}

void
Decluttering::clearSortFunctor()
{
    // pull our prototype
    osgEarthAnnotationDeclutterRenderBin* bin = dynamic_cast<osgEarthAnnotationDeclutterRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_DECLUTTER_BIN ) );

    if ( bin )
    {
        bin->clearSortingFunctor();
    }
}

void
Decluttering::setOptions( const DeclutteringOptions& options )
{
    // pull our prototype
    osgEarthAnnotationDeclutterRenderBin* bin = dynamic_cast<osgEarthAnnotationDeclutterRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_DECLUTTER_BIN ) );

    if ( bin )
    {
        // activate priority-sorting through the options.
        if ( options.sortByPriority().isSetTo( true ) &&
             bin->_context->_options.sortByPriority() == false )
        {
            Decluttering::setSortFunctor(new DeclutterByPriority());
        }
        
        // communicate the new options on the shared context.
        bin->_context->_options = options;
    }
}

const DeclutteringOptions&
Decluttering::getOptions()
{
    static DeclutteringOptions s_defaultOptions;

    // pull our prototype
    osgEarthAnnotationDeclutterRenderBin* bin = dynamic_cast<osgEarthAnnotationDeclutterRenderBin*>(
        osgUtil::RenderBin::getRenderBinPrototype( OSGEARTH_DECLUTTER_BIN ) );

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

bool
DeclutterByPriority::operator()(const osgUtil::RenderLeaf* lhs, const osgUtil::RenderLeaf* rhs ) const
{
    float diff = 0.0f;
    const AnnotationData* lhsData = dynamic_cast<const AnnotationData*>(lhs->getDrawable()->getUserData());
    if ( lhsData )
    {
        const AnnotationData* rhsData = dynamic_cast<const AnnotationData*>(rhs->getDrawable()->getUserData());
        if ( rhsData )
        {
            diff = lhsData->getPriority() - rhsData->getPriority();
        }
    }

    if ( diff != 0.0f )
        return diff > 0.0f;

    // first fallback on depth:
    diff = lhs->_depth - rhs->_depth;
    if ( diff != 0.0f )
        return diff < 0.0f;

    // then fallback on traversal order.
    diff = float(lhs->_traversalNumber) - float(rhs->_traversalNumber);
    return diff < 0.0f;
}

//----------------------------------------------------------------------------

/** the actual registration. */
extern "C" void osgEarth_declutter(void) {}
static osgEarthRegisterRenderBinProxy<osgEarthAnnotationDeclutterRenderBin> s_regbin(OSGEARTH_DECLUTTER_BIN);
