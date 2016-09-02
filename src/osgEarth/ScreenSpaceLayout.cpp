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
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Containers>
#include <osgEarth/Utils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Extension>
#include <osgEarthAnnotation/BboxDrawable>
#include <osgUtil/RenderBin>
#include <osgUtil/StateGraph>
#include <osgText/Text>
#include <osg/UserDataContainer>
#include <osg/ValueObject>
#include <set>
#include <algorithm>

#define LC "[ScreenSpaceLayout] "

#define FADE_UNIFORM_NAME "oe_declutter_fade"

using namespace osgEarth;

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

    // Custom sorting functor that sorts drawables front-to-back, and when drawables share the
    // same parent Geode, sorts them in traversal order.
    struct SortFrontToBackPreservingGeodeTraversalOrder
    {
        bool operator()( const osgUtil::RenderLeaf* lhs, const osgUtil::RenderLeaf* rhs ) const
        {
            const osg::Node* lhsParentNode = lhs->getDrawable()->getParent(0);
            if ( lhsParentNode == rhs->getDrawable()->getParent(0) )
            {
                const osg::Geode* geode = static_cast<const osg::Geode*>(lhsParentNode);
                return geode->getDrawableIndex(lhs->getDrawable()) > geode->getDrawableIndex(rhs->getDrawable());
            }
            else
            {
                return ( lhs->_depth < rhs->_depth );
            }
        }
    };

    // Custom sorting functor that sorts drawables by Priority, and when drawables share the
    // same parent Geode, sorts them in traversal order.
    struct SortByPriorityPreservingGeodeTraversalOrder : public DeclutterSortFunctor
    {
        bool operator()( const osgUtil::RenderLeaf* lhs, const osgUtil::RenderLeaf* rhs ) const
        {
            const osg::Node* lhsParentNode = lhs->getDrawable()->getParent(0);
            if ( lhsParentNode == rhs->getDrawable()->getParent(0) )
            {
                const osg::Geode* geode = static_cast<const osg::Geode*>(lhsParentNode);
                return geode->getDrawableIndex(lhs->getDrawable()) > geode->getDrawableIndex(rhs->getDrawable());
            }

            else
            {            
                const ScreenSpaceLayoutData* lhsdata = dynamic_cast<const ScreenSpaceLayoutData*>(lhs->getDrawable()->getUserData());
                float lhsPriority = lhsdata ? lhsdata->_priority : 0.0f;
    
                const ScreenSpaceLayoutData* rhsdata = dynamic_cast<const ScreenSpaceLayoutData*>(rhs->getDrawable()->getUserData());
                float rhsPriority = rhsdata ? rhsdata->_priority : 0.0f;

                float diff = lhsPriority - rhsPriority;

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
        }
    };

    // Data structure shared across entire layout system.
    struct ScreenSpaceLayoutContext : public osg::Referenced
    {
        ScreenSpaceLayoutOptions _options;
    };

    // records information about each drawable.
    // TODO: a way to clear out this list when drawables go away
    struct DrawableInfo
    {
        DrawableInfo() : _lastAlpha(1.0), _lastScale(1.0), _lastXY(-1.0, -1.0) { }
        float _lastAlpha, _lastScale;
        osg::Vec2d _lastXY;
    };

    typedef std::map<const osg::Drawable*, DrawableInfo> DrawableMemory;
    
    typedef std::pair<const osg::Node*, osg::BoundingBox> RenderLeafBox;

    // Data structure stored one-per-View.
    struct PerCamInfo
    {
        PerCamInfo() : _lastTimeStamp(0), _firstFrame(true) { }

        // remembers the state of each drawable from the previous pass
        DrawableMemory _memory;
        
        // re-usable structures (to avoid unnecessary re-allocation)
        osgUtil::RenderBin::RenderLeafList _passed;
        osgUtil::RenderBin::RenderLeafList _failed;
        std::vector<RenderLeafBox>         _used;

        // time stamp of the previous pass, for calculating animation speed
        osg::Timer_t _lastTimeStamp;
        bool _firstFrame;
    };

    static bool s_declutteringEnabledGlobally = true;

    static const char* s_faderFS =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float " FADE_UNIFORM_NAME ";\n"
        "void oe_declutter_apply_fade(inout vec4 color) { \n"
        "    color.a *= " FADE_UNIFORM_NAME ";\n"
        "}\n";
}

//----------------------------------------------------------------------------

void
ScreenSpaceLayoutOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "min_animation_scale", _minAnimScale );
    conf.getIfSet( "min_animation_alpha", _minAnimAlpha );
    conf.getIfSet( "in_animation_time",   _inAnimTime );
    conf.getIfSet( "out_animation_time",  _outAnimTime );
    conf.getIfSet( "sort_by_priority",    _sortByPriority );
    conf.getIfSet( "snap_to_pixel",       _snapToPixel );
    conf.getIfSet( "max_objects",         _maxObjects );
}

Config
ScreenSpaceLayoutOptions::getConfig() const
{
    Config conf;
    conf.addIfSet( "min_animation_scale", _minAnimScale );
    conf.addIfSet( "min_animation_alpha", _minAnimAlpha );
    conf.addIfSet( "in_animation_time",   _inAnimTime );
    conf.addIfSet( "out_animation_time",  _outAnimTime );
    conf.addIfSet( "sort_by_priority",    _sortByPriority );
    conf.addIfSet( "snap_to_pixel",       _snapToPixel );
    conf.addIfSet( "max_objects",         _maxObjects );
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
    ScreenSpaceLayoutContext* _context;

    PerObjectFastMap<osg::Camera*, PerCamInfo> _perCam;

    /**
     * Constructs the new sorter.
     * @param f Custom declutter sorting predicate. Pass NULL to use the 
     *          default sorter (sort by distance-to-camera).
     */
    DeclutterSort( ScreenSpaceLayoutContext* context, DeclutterSortFunctor* f = 0L )
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
        if ( _customSortFunctor && s_declutteringEnabledGlobally )
        {
            // if there's a custom sorting function installed
            bin->copyLeavesFromStateGraphListToRenderLeafList();
            std::sort( leaves.begin(), leaves.end(), SortContainer( *_customSortFunctor ) );
        }
        else
        {
            // default behavior:
            bin->copyLeavesFromStateGraphListToRenderLeafList();
            std::sort( leaves.begin(), leaves.end(), SortFrontToBackPreservingGeodeTraversalOrder() );
        }

        // nothing to sort? bail out
        if ( leaves.size() == 0 )
            return;

        // access the view-specific persistent data:
        osg::Camera* cam   = bin->getStage()->getCamera();                
        PerCamInfo& local = _perCam.get( cam );

        osg::Timer_t now = osg::Timer::instance()->tick();
        if (local._firstFrame)
        {            
            local._firstFrame = false;
            local._lastTimeStamp = now;
        }

        // calculate the elapsed time since the previous pass; we'll use this for
        // the animations                
        float elapsedSeconds = osg::Timer::instance()->delta_s(local._lastTimeStamp, now);
        local._lastTimeStamp = now;

        // Reset the local re-usable containers
        local._passed.clear();          // drawables that pass occlusion test
        local._failed.clear();          // drawables that fail occlusion test
        local._used.clear();            // list of occupied bounding boxes in screen space

        // compute a window matrix so we can do window-space culling. If this is an RTT camera
        // with a reference camera attachment, we actually want to declutter in the window-space
        // of the reference camera. (e.g., for picking).
        const osg::Viewport* vp = cam->getViewport();

        osg::Matrix windowMatrix = vp->computeWindowMatrix();

        osg::Vec3f  refCamScale(1.0f, 1.0f, 1.0f);
        osg::Matrix refCamScaleMat;
        osg::Matrix refWindowMatrix = windowMatrix;

        if ( cam->isRenderToTextureCamera() )
        {
            osg::Camera* refCam = dynamic_cast<osg::Camera*>(cam->getUserData());
            if ( refCam )
            {
                const osg::Viewport* refVP = refCam->getViewport();
                refCamScale.set( vp->width() / refVP->width(), vp->height() / refVP->height(), 1.0 );
                refCamScaleMat.makeScale( refCamScale );
                refWindowMatrix = refVP->computeWindowMatrix();
            }
        }

        // Track the parent nodes of drawables that are obscured (and culled). Drawables
        // with the same parent node (typically a Geode) are considered to be grouped and
        // will be culled as a group.
        std::set<const osg::Node*> culledParents;

        const ScreenSpaceLayoutOptions& options = _context->_options;
        unsigned limit = *options.maxObjects();

        bool snapToPixel = options.snapToPixel() == true;

        osg::Matrix camVPW;
        camVPW.postMult(cam->getViewMatrix());
        camVPW.postMult(cam->getProjectionMatrix());
        camVPW.postMult(refWindowMatrix);
        //if (cam->getViewport())
        //    camVPW.postMult(cam->getViewport()->computeWindowMatrix());

        // Go through each leaf and test for visibility.
        // Enforce the "max objects" limit along the way.
        for(osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin(); 
            i != leaves.end() && local._passed.size() < limit; 
            ++i )
        {
            bool visible = true;

            osgUtil::RenderLeaf* leaf = *i;
            const osg::Drawable* drawable = leaf->getDrawable();
            const osg::Node*     drawableParent = drawable->getParent(0);

            const ScreenSpaceLayoutData* layoutData = dynamic_cast<const ScreenSpaceLayoutData*>(drawable->getUserData());

            // transform the bounding box of the drawable into window-space.
            osg::BoundingBox box = Utils::getBoundingBox(drawable);

            osg::Vec3f offset;
            osg::Quat rot;

            if (layoutData)
            {
                // local transformation data
                // and management of the label orientation (must be always readable)

                bool isText = dynamic_cast<const osgText::Text*>(drawable) != 0L;

                osg::Vec3d loc = layoutData->getAnchorPoint() * camVPW;
                osg::Vec3d proj = layoutData->getProjPoint() * camVPW;
                proj -= loc;
                
                float angle = atan2(proj.y(), proj.x());

                if ( isText && (angle < - osg::PI / 2. || angle > osg::PI / 2.) )
                {
                    // avoid the label characters to be inverted:
                    // use a symetric translation and adapt the rotation to be in the desired angles
                    offset.set( -layoutData->_pixelOffset.x() - box.xMax() - box.xMin(),
                                -layoutData->_pixelOffset.y() - box.yMax() - box.yMin(),
                                0.f );
                    angle -= osg::PI;
                }
                else
                {
                    offset.set( layoutData->_pixelOffset.x(), layoutData->_pixelOffset.y(), 0.f );
                }

                // handle the local rotation
                if ( angle != 0.f )
                {
                    rot.makeRotate ( angle, osg::Vec3d(0, 0, 1) );
                    osg::Vec3f ld = rot * ( osg::Vec3f(box.xMin(), box.yMin(), 0.) );
                    osg::Vec3f lu = rot * ( osg::Vec3f(box.xMin(), box.yMax(), 0.) );
                    osg::Vec3f ru = rot * ( osg::Vec3f(box.xMax(), box.yMax(), 0.) );
                    osg::Vec3f rd = rot * ( osg::Vec3f(box.xMax(), box.yMin(), 0.) );
                    if ( angle > - osg::PI / 2. && angle < osg::PI / 2.)
                        box.set( std::min(ld.x(), lu.x()), std::min(ld.y(), rd.y()), 0,
                            std::max(rd.x(), ru.x()), std::max(lu.y(), ru.y()), 0 );
                    else
                        box.set( std::min(ld.x(), lu.x()), std::min(lu.y(), ru.y()), 0,
                            std::max(ld.x(), lu.x()), std::max(ld.y(), rd.y()), 0 );
                }

                offset = refCamScaleMat * offset;

                // handle the local translation
                box.xMin() += offset.x();
                box.xMax() += offset.x();
                box.yMin() += offset.y();
                box.yMax() += offset.y();
            }

            static osg::Vec4d s_zero_w(0,0,0,1);
            osg::Matrix MVP = (*leaf->_modelview.get()) * (*leaf->_projection.get());
            osg::Vec4d clip = s_zero_w * MVP;
            osg::Vec3d clip_ndc( clip.x()/clip.w(), clip.y()/clip.w(), clip.z()/clip.w() );
            
            // if we are using a reference camera (like for picking), we do the decluttering in
            // its viewport so that they match. 
            osg::Vec3f winPos    = clip_ndc * windowMatrix;
            osg::Vec3f refWinPos = clip_ndc * refWindowMatrix;

            // The "declutter" box is the box we use to reserve screen space.
            // This must be unquantized regardless of whether snapToPixel is set.
            box.set(
                floor(refWinPos.x() + box.xMin()),
                floor(refWinPos.y() + box.yMin()),
                refWinPos.z(),
                ceil(refWinPos.x() + box.xMax()),
                ceil(refWinPos.y() + box.yMax()),
                refWinPos.z() );

            // if snapping is enabled, only snap when the camera stops moving.
            bool quantize = snapToPixel;
            if ( quantize )
            {
                DrawableInfo& info = local._memory[drawable];
                
                if ( info._lastXY.x() == winPos.x() && info._lastXY.y() == winPos.y() )
                {
                    // Quanitize the window draw coordinates to mitigate text rendering filtering anomalies.
                    // Drawing text glyphs on pixel boundaries mitigates aliasing.
                    // Adding 0.5 will cause the GPU to sample the glyph texels exactly on center.
                    winPos.x() = floor(winPos.x()) + 0.5;
                    winPos.y() = floor(winPos.y()) + 0.5;
                }
                else
                {
                    info._lastXY.set( winPos.x(), winPos.y() );
                }
            }            

            if ( s_declutteringEnabledGlobally )
            {
                // A max priority => never occlude.
                float priority = layoutData ? layoutData->_priority : 0.0f;

                if ( priority == FLT_MAX )
                {
                    visible = true;
                }
                
                // if this leaf is already in a culled group, skip it.
                else if ( culledParents.find(drawableParent) != culledParents.end() )
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
            // projection when it's drawn later. We'll also preserve the scale.
            osg::Matrix newModelView;
            if ( rot.zeroRotation() )
            {
                newModelView.makeTranslate( osg::Vec3f(winPos.x() + offset.x(), winPos.y() + offset.y(), 0) );
                newModelView.preMultScale( leaf->_modelview->getScale() * refCamScaleMat );
            }
            else
            {
                offset = rot * offset;
                newModelView.makeTranslate( osg::Vec3f(winPos.x() + offset.x(), winPos.y() + offset.y(), 0) );
                newModelView.preMultScale( leaf->_modelview->getScale() * refCamScaleMat );
                newModelView.preMultRotate( rot );
            }
            
            // Leaf modelview matrixes are shared (by objects in the traversal stack) so we 
            // cannot just replace it unfortunately. Have to make a new one. Perhaps a nice
            // allocation pool is in order here
            leaf->_modelview = new osg::RefMatrix( newModelView );
        }

        // copy the final draw list back into the bin, rejecting any leaves whose parents
        // are in the cull list.

        if ( s_declutteringEnabledGlobally )
        {
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
                bool isBbox = dynamic_cast<const osgEarth::Annotation::BboxDrawable*>(drawable) != 0L;
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

                if ( (!isText && !isBbox) || !fullyOut )
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
    }
};

namespace
{
    /**
     * Custom draw routine for our declutter render bin.
     */
    struct DeclutterDraw : public osgUtil::RenderBin::DrawCallback
    {
        ScreenSpaceLayoutContext*                  _context;
        PerThread< osg::ref_ptr<osg::RefMatrix> > _ortho2D;
        osg::ref_ptr<osg::Uniform>                _fade;

        /**
         * Constructs the decluttering draw callback.
         * @param context A shared context among all decluttering objects.
         */
        DeclutterDraw( ScreenSpaceLayoutContext* context )
            : _context( context )
        {
            // create the fade uniform.
            _fade = new osg::Uniform( osg::Uniform::FLOAT, FADE_UNIFORM_NAME );
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

                //m->makeOrtho2D( vp->x(), vp->x()+vp->width()-1, vp->y(), vp->y()+vp->height()-1 );
                m->makeOrtho( vp->x(), vp->x()+vp->width()-1, vp->y(), vp->y()+vp->height()-1, -1000, 1000);
                state.applyProjectionMatrix( m.get() );
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
                // todo: find a way to optimize this..?
                _fade->set( s_declutteringEnabledGlobally ? leaf->_depth : 1.0f );
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
}

//----------------------------------------------------------------------------

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
        setDrawCallback( new DeclutterDraw(_context.get()) );

        // needs its own state set for special magic.
        osg::StateSet* stateSet = new osg::StateSet();
        this->setStateSet( stateSet );

        //VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
        //vp->setFunction( "oe_declutter_apply_fade", s_faderFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.5f );
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
            Threading::ScopedMutexLock lock(_vpMutex);
            if (!_vpInstalled)
            {
                VirtualProgram* vp = VirtualProgram::getOrCreate(getStateSet());
                vp->setFunction( "oe_declutter_apply_fade", s_faderFS, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.5f );
                _vpInstalled = true;
                OE_INFO << LC << "Decluttering VP installed\n";
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
        setSortCallback( new DeclutterSort(_context.get(), f) );
    }

    void clearSortingFunctor()
    {
        setSortCallback( new DeclutterSort(_context.get()) );
    }

    osg::ref_ptr<DeclutterSortFunctor> _f;
    osg::ref_ptr<ScreenSpaceLayoutContext> _context;
    static Threading::Mutex _vpMutex;
    static bool _vpInstalled;
};

Threading::Mutex osgEarthScreenSpaceLayoutRenderBin::_vpMutex;
bool osgEarthScreenSpaceLayoutRenderBin::_vpInstalled = false;

//----------------------------------------------------------------------------

void
ScreenSpaceLayout::activate(osg::StateSet* stateSet, int binNum)
{
    if ( stateSet )
    {
        // the OVERRIDE prevents subsequent statesets from disabling the layout bin
        stateSet->setRenderBinDetails(
            binNum,
            OSGEARTH_SCREEN_SPACE_LAYOUT_BIN,
            osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);

        // Force a single shared layout bin per render stage
        stateSet->setNestRenderBins( false );
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
    s_declutteringEnabledGlobally = enabled;
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

/** the actual registration. */
extern "C" void osgEarth_declutter(void) {}
static osgEarthRegisterRenderBinProxy<osgEarthScreenSpaceLayoutRenderBin> s_regbin(OSGEARTH_SCREEN_SPACE_LAYOUT_BIN);


//----------------------------------------------------------------------------

// Extension for configuring the decluterring/SSL options from an Earth file.
namespace osgEarth
{
    class ScreenSpaceLayoutExtension : public Extension,
                                       public ScreenSpaceLayoutOptions
    {
    public:
        META_osgEarth_Extension(ScreenSpaceLayoutExtension);

        ScreenSpaceLayoutExtension(const ConfigOptions& co) : ScreenSpaceLayoutOptions(co)
        {
            // sets the global default options.
            ScreenSpaceLayout::setOptions(*this);
        }
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_screen_space_layout, ScreenSpaceLayoutExtension);
    REGISTER_OSGEARTH_EXTENSION(osgearth_decluttering,        ScreenSpaceLayoutExtension);
}
                                       