/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/Color>
#include <osgEarth/LineDrawable>
#include <osgEarth/GLUtils>
#include "rtree.h"

// https://github.com/JCash/voronoi
#define JC_VORONOI_IMPLEMENTATION
#define JCV_REAL_TYPE double
#define JCV_ATAN2 atan2
#define JCV_FLT_MAX 1.7976931348623157E+308
#include "jc_voronoi.h"

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
            if (lhs->getDrawable()->getNumParents() > 0 &&
                rhs->getDrawable()->getNumParents() > 0 &&
                rhs->getDrawable()->getParent(0) == lhs->getDrawable()->getParent(0))
            {
                const osg::Group* parent = static_cast<const osg::Group*>(lhs->getDrawable()->getParent(0));
                return parent->getChildIndex(lhs->getDrawable()) > parent->getChildIndex(rhs->getDrawable());
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
            if (lhs->getDrawable()->getNumParents() > 0 &&
                rhs->getDrawable()->getNumParents() > 0 &&
                rhs->getDrawable()->getParent(0) == lhs->getDrawable()->getParent(0))
            {
                const osg::Group* parent = static_cast<const osg::Group*>(lhs->getDrawable()->getParent(0));
                return parent->getChildIndex(lhs->getDrawable()) > parent->getChildIndex(rhs->getDrawable());
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
#if OSG_VERSION_GREATER_THAN(3,6,0)
                diff = float(lhs->_traversalOrderNumber) - float(rhs->_traversalOrderNumber);
#else
                diff = float(lhs->_traversalNumber) - float(rhs->_traversalNumber);
#endif
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
        DrawableInfo() : _lastAlpha(1.0f), _lastScale(1.0f), _frame(0u), _visible(true) { }
        float _lastAlpha, _lastScale;
        unsigned _frame;
        bool _visible;
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
        osg::Matrix _lastCamVPW;
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
    conf.get( "max_leader_length", _maxLeaderLen );
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
    conf.set("max_leader_length", _maxLeaderLen);
    return conf;
}

//----------------------------------------------------------------------------

namespace
{
    template<typename T>
    struct LCGIterator
    {
        T& _vec;
        unsigned _seed;
        unsigned _n;
        unsigned _index;
        unsigned _a, _c;
        LCGIterator(T& vec) : _vec(vec), _seed(0u), _index(0u) {
            _n = vec.size();
            _a = _n+1;
            _c = 15487457u; // a very large prime
        }
        bool hasMore() const {
            return _index < _n;
        }
        const typename T::value_type& next() {
            _seed = (_a*_seed + _c) % _n;
            _index++;
            return _vec[_seed];
        }
    };

    struct jcv_point_comparator {
        bool operator()(const jcv_point& a, const jcv_point& b) const {
            if (a.x < b.x) return true;
            return a.y < b.y;
        }
    };

    /**
    * Custom RenderLeaf sorting algorithm for deconflicting drawables
    * as callouts.
    */
    struct /*internal*/ CalloutImplementation : public osgUtil::RenderBin::SortCallback
    {
        struct BBox
        {
            BBox() { }
            BBox(const osg::BoundingBox& bbox);
            bool overlaps(const BBox& rhs) const;
            double overlap(const BBox& rhs) const;
            osg::Vec2d LL, UR;
        };

        struct Element
        {
            osg::Drawable* _drawable;
            osgEarth::Text* _text;
            osgUtil::RenderLeaf* _leaf;
            osg::ref_ptr<osg::RefMatrix> _matrix;
            unsigned _frame;
            osg::Vec3d _anchor;
            BBox _vpbbox;
            BBox _leaderbbox;
            osg::Vec3d _offsetVector;
            double _offsetLength;
            osg::Vec3d _bestVector;
            unsigned _leaderLineIndex;
            bool _conflicted;
            bool _moveRequested;
            int _moveAttempts;
            float _overlap;

            Element();
            bool operator < (const Element&) const; // comparator
            void move(float dir);
            void realign();
        };

        typedef std::map<osg::Drawable*, Element> Elements;
        typedef RTree<Element*, float, 2> SpatialIndex;

        struct CameraLocal 
        {
            CameraLocal();
            const osg::Camera* _camera;      // camera associated with the data structure
            unsigned _frame;
            osg::Matrix _scalebias;          // scale bias matrix for the viewport
            //osgUtil::RenderBin::RenderLeafList _leaves;
            Elements _elements;
            //mutable SpatialIndex _elementIndex;
            //mutable SpatialIndex _leaderIndex;     
            osg::Matrix _vpm;
            bool _vpmChanged;
            osg::ref_ptr<LineDrawable> _leaders;
            bool _leadersDirty;
            Color _leaderColor;
            std::vector<jcv_point> _points;
            std::map<jcv_point,Element*,jcv_point_comparator> _lookup;
        };

        ScreenSpaceLayoutContext* _context;
        DeclutterSortFunctor* _customSortFunctor;
        PerObjectFastMap<const osg::Camera*, CameraLocal> _cameraLocal;
        double _maxOverlap;
        int _maxMoveAttempts;
        bool _drawConflictedRecords;
        bool _resetWhenViewChanges;
        bool _declutterIncrementally;
        unsigned _movesThisFrame;
        bool _drawObscuredItems;

        //! Constructor
        CalloutImplementation(ScreenSpaceLayoutContext* context, DeclutterSortFunctor* f);

        //! Override from SortCallback
        void CalloutImplementation::sortImplementation(osgUtil::RenderBin*);

        void push(osgUtil::RenderLeaf* leaf, CameraLocal& local);

        void sort(CameraLocal& local);

        bool handleOverlap(Element*, const BBox&);
    };

    CalloutImplementation::BBox::BBox(const osg::BoundingBox& bbox) :
        LL(bbox.xMin(), bbox.yMin()),
        UR(bbox.xMax(), bbox.yMax())
    {
        //nop
    }

    bool
    CalloutImplementation::BBox::overlaps(const BBox& rhs) const
    {
        return overlap(rhs) > 0.0;
    }

    double
    CalloutImplementation::BBox::overlap(const BBox& rhs) const
    {
        double xmin = osg::maximum(LL.x(), rhs.LL.x());
        double xmax = osg::minimum(UR.x(), rhs.UR.x());
        if (xmin >= xmax) return 0.0;

        double ymin = osg::maximum(LL.y(), rhs.LL.y());
        double ymax = osg::minimum(UR.y(), rhs.UR.y());
        if (ymin >= ymax) return 0.0;

        double area = (UR.x() - LL.x())*(UR.y() - LL.y());
        double overlapArea = (xmax - xmin)*(ymax - ymin);

        return overlapArea / area;
    }


    CalloutImplementation::Element::Element() :
        _drawable(NULL),
        _text(NULL),
        _frame(0u),
        _leaderLineIndex(INT_MAX),
        _conflicted(false),
        _offsetVector(0,1,0),
        _bestVector(0,1,0),
        _moveAttempts(0),
        _moveRequested(false),
        _overlap(1.0f)
    {
        //todo
    }

    bool
    CalloutImplementation::Element::operator<(const Element& rhs) const
    {
        return ((intptr_t)_drawable < (intptr_t)rhs._drawable);
    }

    void
    CalloutImplementation::Element::move(float dir)
    {
        // rotate little more than 1/4 turn:
        const double rotation = osg::PI / 16; //1.7; //osg::PI / 32; //1.6;
        const osg::Quat q(dir*rotation, osg::Vec3d(0, 0, 1));
        _offsetVector = q * _offsetVector;
        realign();        
    }

    void
    CalloutImplementation::Element::realign()
    {
        if (!_text)
        {
            return;
        }

        if (_offsetVector.x() >= 0.5)
        {
            if (_offsetVector.y() >= 0.5)
                _text->setAlignment(osgEarth::Text::LEFT_BOTTOM);
            else if (_offsetVector.y() <= -0.5)
                _text->setAlignment(osgEarth::Text::LEFT_TOP);
            else
                _text->setAlignment(osgEarth::Text::LEFT_CENTER);
        }
        else if (_offsetVector.x() <= -0.5)
        {
            if (_offsetVector.y() >= 0.5)
                _text->setAlignment(osgEarth::Text::RIGHT_BOTTOM);
            else if (_offsetVector.y() <= -0.5)
                _text->setAlignment(osgEarth::Text::RIGHT_TOP);
            else
                _text->setAlignment(osgEarth::Text::RIGHT_CENTER);
        }
        else if (_offsetVector.y() >= 0.0)
        {
            _text->setAlignment(osgEarth::Text::CENTER_BOTTOM);
        }
        else if (_offsetVector.y() <= 0.0)
        {
            _text->setAlignment(osgEarth::Text::CENTER_TOP);
        }
        
        else if(_offsetVector.y() >= 0.0)
        {
            _text->setAlignment(osgEarth::Text::CENTER_BOTTOM);
        }
        else if (_offsetVector.y() <= 0.0)
        {
            _text->setAlignment(osgEarth::Text::CENTER_TOP);
        }
        else
        {
            _text->setAlignment(osgEarth::Text::CENTER_CENTER);
        }
    }

    CalloutImplementation::CameraLocal::CameraLocal() :
        _camera(0), 
        _frame(0), 
        _vpmChanged(true),
        _leadersDirty(false)
    {
        _leaders = new LineDrawable(GL_LINES);
        _leaders->setCullingActive(false);
        _leaders->setDataVariance(osg::Object::DYNAMIC);
        _leaders->setColor(osg::Vec4f(1, 1, 0, 1));
        _leaders->setLineWidth(1.5f);
        _leaders->setLineSmooth(true);
        GLUtils::setLighting(_leaders->getOrCreateStateSet(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    }

    CalloutImplementation::CalloutImplementation(ScreenSpaceLayoutContext* context, DeclutterSortFunctor* f) :
        _context(context),
        _customSortFunctor(f),
        _maxOverlap(0.0),
        _maxMoveAttempts(31),
        _drawObscuredItems(false),
        _declutterIncrementally(false),
        _resetWhenViewChanges(false)
    {
        //nop
    }

    void
    CalloutImplementation::push(osgUtil::RenderLeaf* leaf, CameraLocal& local)
    {
        static Element prototype;
        const osg::Vec3d zero(0,0,0);

        osg::Drawable* drawable = leaf->_drawable.get();
        std::pair<Elements::iterator,bool> result = local._elements.insert(std::make_pair(drawable,prototype));
        Element& element = result.first->second;

        bool isNew = 
            (result.second == true) ||
            (local._frame - element._frame > 1);

        element._frame = local._frame;
        element._leaf = leaf;

        if (element._drawable == NULL)
        {
            element._drawable = drawable;
            element._text = dynamic_cast<osgEarth::Text*>(drawable);
            element._text->setDataVariance(osg::Object::DYNAMIC);
        }

        element._anchor = 
            zero * 
            (*leaf->_modelview) * 
            (*leaf->_projection) * 
            local._scalebias;

        const osg::Viewport* vp = local._camera->getViewport();

        if (isNew)
        {
            element._offsetVector = element._anchor - osg::Vec3d(0.5*vp->width(), 0.5*vp->height(), 0.0);
            element._offsetVector.normalize();
            element._bestVector = element._offsetVector;
            element.realign();
            element._moveAttempts = 0;
            element._moveRequested = false;
            element._conflicted = false;
            element._overlap = 1.0f;
        }
        else if (_resetWhenViewChanges && local._vpmChanged)
        {
            element._offsetVector = element._anchor - osg::Vec3d(0.5*vp->width(), 0.5*vp->height(), 0.0);
            element._offsetVector.normalize();
            element._bestVector = element._offsetVector;
            element.realign();
            element._moveAttempts = 0;
            element._moveRequested = false;
            element._overlap = 1.0f;
        }

        if (element._leaderLineIndex == INT_MAX)
        {
            element._leaderLineIndex = local._leaders->getNumVerts();
            local._leaders->pushVertex(osg::Vec3f());
            local._leaders->pushVertex(osg::Vec3f());
            local._leadersDirty = true;
        }

        double leaderLen = osg::minimum((double)_context->_options.maxLeaderLength().get(), element._offsetLength);
        osg::Vec3d pos = element._anchor + element._offsetVector*leaderLen;
        const osg::BoundingBox& b = drawable->getBoundingBox();
        element._vpbbox.LL.set(pos.x() + b.xMin(), pos.y() + b.yMin());
        element._vpbbox.UR.set(pos.x() + b.xMax(), pos.y() + b.yMax());

        leaf->_modelview->makeTranslate(pos);

        // and update the leader line endpoints
        if (element._anchor != local._leaders->getVertex(element._leaderLineIndex))
        {
            local._leaders->setVertex(element._leaderLineIndex, element._anchor);
        }
        if (pos != local._leaders->getVertex(element._leaderLineIndex + 1))
        {
            local._leaders->setVertex(element._leaderLineIndex + 1, pos);
        }
        // and update the colors of the leader line
        if (element._text)
        {
            Color color =
                element._text->getDrawMode() & osgText::Text::BOUNDINGBOX ?
                element._text->getBoundingBoxColor() :
                local._leaderColor;

            local._leaders->setColor(element._leaderLineIndex, color);
            local._leaders->setColor(element._leaderLineIndex + 1, color);
        }
    }
    bool
    CalloutImplementation::handleOverlap(Element* lhs, const BBox& bbox)
    {
        float overlap = lhs->_vpbbox.overlap(bbox); // 0..1

        if (overlap > _maxOverlap) // allow for some overlap..
        {
            lhs->_conflicted = true;
            lhs->_overlap = overlap;
            lhs->move(1);
            
#if 0
            // do we still have some attempts left to deconflict this callout?
            if (lhs->_moveAttempts < _maxMoveAttempts)
            {
                // is the current overlap better than the best-so-far?
                if (overlap < lhs->_overlap)
                {
                    lhs->_overlap = overlap;
                    lhs->_bestVector = lhs->_offsetVector;
                }

                lhs->_moveRequested = true;
            }
            else
            {
                // ran out of move attempts.. accept the overlap.
                lhs->_offsetVector = lhs->_bestVector;
            }
#endif

            return true;
        }

        return false;
    }

    void
    CalloutImplementation::sort(CameraLocal& local)
    {
        if (local._elements.empty())
            return;

        if (_resetWhenViewChanges && local._camera)
        {
            osg::Matrix VPM = local._camera->getViewMatrix() * local._camera->getProjectionMatrix();
            local._vpmChanged = VPM != local._vpm;
            local._vpm = VPM;
        }

        const osg::Viewport* vp = local._camera->getViewport();

        local._points.clear();
        local._lookup.clear();

        for (Elements::reverse_iterator i = local._elements.rbegin();
            i != local._elements.rend();
            ++i)
        {
            Element& element = i->second;

            if (element._leaf != NULL)
            {
                if (element._frame == element._frame)
                {
                    // i.e., visible
                    element._leaf->_depth = 1.0f;

                    local._points.resize(local._points.size()+1);
                    jcv_point& back = local._points.back();
                    back.x = element._anchor.x();
                    back.y = element._anchor.y();
                    local._lookup[back] = &element;

#if 0
                    a_min[0] = rec._vpbbox.LL.x(), a_min[1] = rec._vpbbox.LL.y();
                    a_max[0] = rec._vpbbox.UR.x(), a_max[1] = rec._vpbbox.UR.y();

                    b_min[0] = rec._leaderbbox.LL.x(), b_min[1] = rec._leaderbbox.LL.y();
                    b_max[0] = rec._leaderbbox.UR.x(), b_max[1] = rec._leaderbbox.UR.y();

                    // does the label conflict with another label?
                    if (local._elementIndex.Search(a_min, a_max, &hits, 1) > 0)
                    {
                        // yes; do something about it
                        rec._conflicted = handleOverlap(&rec, (*hits.begin())->_vpbbox);
                    }

                    // does the label conflict with another leader?
                    else if (local._leaderIndex.Search(a_min, a_max, &hits, 1) > 0)
                    {
                        // yes; do something about it
                        rec._conflicted = handleOverlap(&rec, (*hits.begin())->_leaderbbox);
                    }

                    else
                    {
                        // If the track was conflicting but now is not, reset its state.
                        if (rec._conflicted)
                        {
                            rec._conflicted = false;
                            rec._moveAttempts = 0;
                            rec._moveRequested = false;
                            rec._overlap = 1.0f;
                        }
                    }

                    // record the areas of both the label and the leader.
                    if (_drawObscuredItems || !rec._conflicted)
                    {
                        local._elementIndex.Insert(a_min, a_max, &rec);   // label
                        local._leaderIndex.Insert(b_min, b_max, &rec);  // leader
                    }

                    rec._leaf->_depth = rec._conflicted ? 0.0f : 1.0f;
                    //if (rec._conflicted)
                    //{
                    //    rec._leaf->_depth = 0.0f;
                    //}
#endif
                }
                else
                {
                    //if(element._text->getText().createUTF8EncodedString()=="Honolulu")
                    //  OE_WARN << " ***  : E=" << element._frame << ", F=" << local._frame << std::endl;

                    element._leaf->_depth = 0.0f; // shouldn't happen
                }
            }
        }

        // Use a Voronoi diagram to find the best location for each callout.
        // Use the centroid of the point's site as the label location. 
        // The text alignment will alter this somewhat .. think about that later.
        jcv_diagram diagram;
        ::memset(&diagram, 0, sizeof(jcv_diagram));
        jcv_rect bounds;
        bounds.min.x = vp->x(), bounds.min.y = vp->y();
        bounds.max.x = vp->x() + vp->width(), bounds.max.y = vp->y() + vp->height();
        jcv_diagram_generate(local._points.size(), &local._points[0], &bounds, &diagram);

        const jcv_site* sites = jcv_diagram_get_sites(&diagram);
        for(int i=0; i<diagram.numsites; ++i)
        {
            const jcv_site* site = &sites[i];

            Element* element = local._lookup[site->p];
            if (element)
            {
                // calculate the centroid of the site
                jcv_point sum;
                sum.x = 0, sum.y = 0;
                int count = 0;

                const jcv_graphedge* edge = site->edges;
                while(edge)
                {
                    sum.x += edge->pos[0].x;
                    sum.y += edge->pos[0].y;
                    ++count;
                    edge = edge->next;
                }
                sum.x /= (jcv_real)count;
                sum.y /= (jcv_real)count;

                element->_offsetVector = osg::Vec3d(sum.x, sum.y, 0) - element->_anchor;
                element->_offsetLength = element->_offsetVector.length();
                element->_offsetVector.normalize();  
                element->realign();
            }
        }
        jcv_diagram_free(&diagram);

#if 0
        _movesThisFrame = 0u;

        if (_declutterIncrementally)
        {
            if (_walker == _callouts.end())
                _walker = _callouts.begin();

            const int triesPerFrame = 10;
            for (int tries = 0; tries < triesPerFrame && _walker != _callouts.end(); )
            {
                CalloutRecord& rec = _walker->second;

                if (frame - rec._frame <= 2)
                {
                    if (rec._conflicted && rec._moveRequested)
                    {
                        rec.move(1);
                        rec._moveAttempts++;
                        rec._moveRequested = false;
                        tries++;
                        _movesThisFrame++;
                    }
                    ++_walker;
                }

                else if (frame - rec._frame > 5 * 60 * 60) // expire after 5 minutes
                {
                    Callouts::iterator temp = _walker;
                    ++_walker;
                    _callouts.erase(temp);
                }

                else
                {
                    ++_walker;
                }
            }
        }

        else
#endif

#if 0
        {
            // declutter all callouts each frame
            for (Elements::reverse_iterator i = local._elements.rbegin();
                i != local._elements.rend();
                )
            {
                Element& element = i->second;

#if 0
                if (local._frame - element._frame <= 2)
                {
                    if (element._conflicted && element._moveRequested)
                    {
                        element.move(1);
                        element._moveAttempts++;
                        element._moveRequested = false;
                        _movesThisFrame++;
                    }
                    ++i;
                }

                else
#endif
                    
#if 0
                if (local._frame - element._frame > 5 * 60 * 60) // expire after 5 minutes
                {
                    ++i;
                    local._elements.erase(i.base());

                    //Callouts::reverse_iterator temp = i;
                    //++i;
                    //_callouts.erase(temp);
                }

                else
#endif
                {
                    ++i;
                }
            }
        }
#endif
    }

    // runs in CULL thread after culling completes
    void
    CalloutImplementation::sortImplementation(osgUtil::RenderBin* bin)
    {
        const ScreenSpaceLayoutOptions& options = _context->_options;

        bin->copyLeavesFromStateGraphListToRenderLeafList();

        osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();

        // first, sort the leaves:
        if (_customSortFunctor && s_declutteringEnabledGlobally)
        {
            // if there's a custom sorting function installed
            std::sort(leaves.begin(), leaves.end(), SortContainer(*_customSortFunctor));
        }
        else if (options.sortByDistance() == true)
        {
            // default behavior:
            std::sort(leaves.begin(), leaves.end(), SortFrontToBackPreservingGeodeTraversalOrder());
        }

        // nothing to sort? bail out
        if (leaves.empty())
            return;

        // access the per-camera persistent data:
        osg::Camera* cam = bin->getStage()->getCamera();

        // bail out if this camera is a master camera with no GC
        // (e.g., in a multi-screen layout)
        if (cam == NULL || (cam->getGraphicsContext() == NULL && !cam->isRenderToTextureCamera()))
            return;

        CameraLocal& local = _cameraLocal.get(cam);
        local._camera = cam;

        static osg::Vec4f invisible(1,0,0,0);
        local._leaders->setColor(invisible);

        osg::GraphicsContext* gc = cam->getGraphicsContext();
        local._frame = gc && gc->getState() && gc->getState()->getFrameStamp() ?
            gc->getState()->getFrameStamp()->getFrameNumber() : 0u;

        const osg::Viewport* vp = cam->getViewport();
        local._scalebias =
            osg::Matrix::translate(1, 1, 1) *
            osg::Matrix::scale(0.5*vp->width(), 0.5*vp->height(), 0.5);

        for(osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin();
            i != leaves.end();
            ++i)
        {
            push(*i, local);
        }

        sort(local);

#if 0
        for (osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin();
            i != leaves.end();
            ++i)
        {
            (*i)->_depth = 1.0f;
        }
#endif

        // clear out the RenderLeaf pointers (since they change each frame)
        for (Elements::iterator i = local._elements.begin();
            i != local._elements.end();
            ++i)
        {
            Element& element = i->second;
            element._leaf = NULL;
        }

#if 0
        for (Elements::iterator i = local._elements.begin();
            i != local._elements.end();
            ++i)
        {
            Element& element = i->second;

            if (element._frame < local._frame)
            {
                element._leaf->_depth = 0.0f;
            }

            if (element._conflicted)
            {
                if (!_drawObscuredItems)
                    element._leaf->_depth = 0.0f;
                else if (_movesThisFrame > 0u)
                    element._leaf->_depth = 0.0f;
            }
        }
#endif
    }

    //....................................................................

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
    struct /*internal*/ DeclutterImplementation : public osgUtil::RenderBin::SortCallback
    {
        DeclutterSortFunctor* _customSortFunctor;
        ScreenSpaceLayoutContext* _context;

        PerObjectFastMap<osg::Camera*, PerCamInfo> _perCam;

        /**
         * Constructs the new sorter.
         * @param f Custom declutter sorting predicate. Pass NULL to use the
         *          default sorter (sort by distance-to-camera).
         */
        DeclutterImplementation( ScreenSpaceLayoutContext* context, DeclutterSortFunctor* f = 0L )
            : _context(context), _customSortFunctor(f)
        {
            //nop
        }

        // override.
        // Sorts the bin. This runs in the CULL thread after the CULL traversal has completed.
        void sortImplementation(osgUtil::RenderBin* bin)
        {
            const ScreenSpaceLayoutOptions& options = _context->_options;

            osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();

            bin->copyLeavesFromStateGraphListToRenderLeafList();

            // first, sort the leaves:
            if ( _customSortFunctor && s_declutteringEnabledGlobally )
            {
                // if there's a custom sorting function installed
                std::sort( leaves.begin(), leaves.end(), SortContainer( *_customSortFunctor ) );
            }
            else if (options.sortByDistance() == true)
            {
                // default behavior:
                std::sort( leaves.begin(), leaves.end(), SortFrontToBackPreservingGeodeTraversalOrder() );
            }

            // nothing to sort? bail out
            if ( leaves.size() == 0 )
                return;

            // access the view-specific persistent data:
            osg::Camera* cam = bin->getStage()->getCamera();

            // bail out if this camera is a master camera with no GC
            // (e.g., in a multi-screen layout)
            if (cam == NULL || (cam->getGraphicsContext() == NULL && !cam->isRenderToTextureCamera()))
            {
                return;
            }

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

            // If the camera is actually an RTT slave camera, it's our picker, and we need to
            // adjust the scale to match it.
            if (cam->isRenderToTextureCamera() &&
                cam->getView() &&
                cam->getView()->getCamera() &&
                cam->getView()->getCamera() != cam)
                //cam->getView()->findSlaveIndexForCamera(cam) < cam->getView()->getNumSlaves())
            {
                osg::Camera* parentCam = cam->getView()->getCamera();
                const osg::Viewport* refVP = parentCam->getViewport();
                refCamScale.set( vp->width() / refVP->width(), vp->height() / refVP->height(), 1.0 );
                refCamScaleMat.makeScale( refCamScale );
                refWindowMatrix = refVP->computeWindowMatrix();
            }

            // Track the parent nodes of drawables that are obscured (and culled). Drawables
            // with the same parent node (typically a Geode) are considered to be grouped and
            // will be culled as a group.
            std::set<const osg::Node*> culledParents;

            unsigned limit = *options.maxObjects();

            bool snapToPixel = options.snapToPixel() == true;

            osg::Matrix camVPW;
            camVPW.postMult(cam->getViewMatrix());
            camVPW.postMult(cam->getProjectionMatrix());
            camVPW.postMult(refWindowMatrix);

            // has the camera moved?
            bool camChanged = camVPW != local._lastCamVPW;
            local._lastCamVPW = camVPW;

            // Go through each leaf and test for visibility.
            // Enforce the "max objects" limit along the way.
            for(osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin();
                i != leaves.end() && local._passed.size() < limit;
                ++i )
            {
                bool visible = true;

                osgUtil::RenderLeaf* leaf = *i;
                const osg::Drawable* drawable = leaf->getDrawable();
                const osg::Node*     drawableParent = drawable->getNumParents()? drawable->getParent(0) : 0L;

                const ScreenSpaceLayoutData* layoutData = dynamic_cast<const ScreenSpaceLayoutData*>(drawable->getUserData());

                // transform the bounding box of the drawable into window-space.
                osg::BoundingBox box = drawable->getBoundingBox();

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

                    if ( isText && (angle < -osg::PI_2 || angle > osg::PI_2) )
                    {
                        // avoid the label characters to be inverted:
                        // use a symetric translation and adapt the rotation to be in the desired angles
                        offset.set( -layoutData->_pixelOffset.x() - box.xMax() - box.xMin(),
                                    -layoutData->_pixelOffset.y() - box.yMax() - box.yMin(),
                                    0.f );
                        angle += angle < -osg::PI_2? osg::PI : -osg::PI; // JD #1029
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
                            box.set( osg::minimum(ld.x(), lu.x()), osg::minimum(ld.y(), rd.y()), 0,
                                osg::maximum(rd.x(), ru.x()), osg::maximum(lu.y(), ru.y()), 0 );
                        else
                            box.set( osg::minimum(ld.x(), lu.x()), osg::minimum(lu.y(), ru.y()), 0,
                                osg::maximum(ld.x(), lu.x()), osg::maximum(ld.y(), rd.y()), 0 );
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

                // Expand the box if this object is currently not visible, so that it takes a little
                // more room for it to before visible once again.
                DrawableInfo& info = local._memory[drawable];
                float buffer = info._visible ? 1.0f : 3.0f;

                // The "declutter" box is the box we use to reserve screen space.
                // This must be unquantized regardless of whether snapToPixel is set.
                box.set(
                    floor(refWinPos.x() + box.xMin())-buffer,
                    floor(refWinPos.y() + box.yMin())-buffer,
                    refWinPos.z(),
                    ceil(refWinPos.x() + box.xMax())+buffer,
                    ceil(refWinPos.y() + box.yMax())+buffer,
                    refWinPos.z() );

                // if snapping is enabled, only snap when the camera stops moving.
                bool quantize = snapToPixel;
                if ( quantize && !camChanged )
                {
                    // Quanitize the window draw coordinates to mitigate text rendering filtering anomalies.
                    // Drawing text glyphs on pixel boundaries mitigates aliasing.
                    // Adding 0.5 will cause the GPU to sample the glyph texels exactly on center.
                    winPos.x() = floor(winPos.x()) + 0.5;
                    winPos.y() = floor(winPos.y()) + 0.5;
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
                    else if ( drawableParent != 0L && culledParents.find(drawableParent) != culledParents.end() )
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
                    if (drawableParent)
                        local._used.push_back( std::make_pair(drawableParent, box) );

                    local._passed.push_back( leaf );
                }

                else
                {
                    // culled, so put the parent in the parents list so that any future leaves
                    // with the same parent will be trivially rejected
                    if (drawableParent)
                        culledParents.insert(drawableParent);

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
                    const osg::Node* drawableParent = drawable->getNumParents() > 0 ? drawable->getParent(0) : 0L;

                    if ( drawableParent == 0L || culledParents.find(drawableParent) == culledParents.end() )
                    {
                        DrawableInfo& info = local._memory[drawable];

                        bool fullyIn = true;

                        // scale in until at full scale:
                        if ( info._lastScale != 1.0f )
                        {
                            fullyIn = false;
                            info._lastScale += elapsedSeconds / osg::maximum(*options.inAnimationTime(), 0.001f);
                            if ( info._lastScale > 1.0f )
                                info._lastScale = 1.0f;
                        }

                        if ( info._lastScale != 1.0f )
                            leaf->_modelview->preMult( osg::Matrix::scale(info._lastScale,info._lastScale,1) );

                        // fade in until at full alpha:
                        if ( info._lastAlpha != 1.0f )
                        {
                            fullyIn = false;
                            info._lastAlpha += elapsedSeconds / osg::maximum(*options.inAnimationTime(), 0.001f);
                            if ( info._lastAlpha > 1.0f )
                                info._lastAlpha = 1.0f;
                        }

                        leaf->_depth = info._lastAlpha;
                        leaves.push_back( leaf );

                        info._frame++;
                        info._visible = true;
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
                    bool isBbox = drawable && drawable->className()=="BboxDrawable";
                    bool fullyOut = true;

                    if (info._frame > 0u)
                    {
                        if ( info._lastScale != *options.minAnimationScale() )
                        {
                            fullyOut = false;
                            info._lastScale -= elapsedSeconds / osg::maximum(*options.outAnimationTime(), 0.001f);
                            if ( info._lastScale < *options.minAnimationScale() )
                                info._lastScale = *options.minAnimationScale();
                        }

                        if ( info._lastAlpha != *options.minAnimationAlpha() )
                        {
                            fullyOut = false;
                            info._lastAlpha -= elapsedSeconds / osg::maximum(*options.outAnimationTime(), 0.001f);
                            if ( info._lastAlpha < *options.minAnimationAlpha() )
                                info._lastAlpha = *options.minAnimationAlpha();
                        }
                    }
                    else
                    {
                        // prevent first-frame "pop out"
                        info._lastScale = options.minAnimationScale().get();
                        info._lastAlpha = options.minAnimationAlpha().get();
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

                    info._frame++;
                    info._visible = false;
                }
            }
        }
    };
}

namespace
{
    /**
     * Custom draw routine for our declutter render bin.
     */
    struct DeclutterDraw : public osgUtil::RenderBin::DrawCallback
    {
        ScreenSpaceLayoutContext*                 _context;
        PerThread< osg::ref_ptr<osg::RefMatrix> > _ortho2D;
        osg::ref_ptr<osg::Uniform>                _fade;

        struct RunningState
        {
            RunningState() : lastFade(-1.0f), lastPCP(NULL) { }
            float lastFade;
            const osg::Program::PerContextProgram* lastPCP;
        };

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

            // initialize the fading uniform
            RunningState rs;

            // render the list
            osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();

            for(osgUtil::RenderBin::RenderLeafList::reverse_iterator rlitr = leaves.rbegin();
                rlitr!= leaves.rend();
                ++rlitr)
            {
                osgUtil::RenderLeaf* rl = *rlitr;
                if ( rl->_depth > 0.0f)
                {
                    renderLeaf( rl, renderInfo, previous, rs);
                    previous = rl;
                }
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
        void renderLeaf( osgUtil::RenderLeaf* leaf, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous, RunningState& rs)
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
                if (pcp != rs.lastPCP || leaf->_depth != rs.lastFade)
                {
                    rs.lastFade = s_declutteringEnabledGlobally ? leaf->_depth : 1.0f;
                    _fade->set( rs.lastFade );
                    pcp->apply( *_fade.get() );
                }
            }
            rs.lastPCP = pcp;

            // draw the drawable
            leaf->_drawable->draw(renderInfo);

            if (leaf->_dynamic)
            {
                state.decrementDynamicObjectCount();
            }
        }
    };


    /**
     * Custom draw routine for our declutter render bin.
     */
    struct CalloutDraw : public osgUtil::RenderBin::DrawCallback
    {
        ScreenSpaceLayoutContext*                 _context;
        PerThread< osg::ref_ptr<osg::RefMatrix> > _ortho2D;
        osg::ref_ptr<osg::Uniform>                _fade;
        CalloutImplementation*                    _sortCallback;

        struct RunningState
        {
            RunningState() : lastFade(-1.0f), lastPCP(NULL) { }
            float lastFade;
            const osg::Program::PerContextProgram* lastPCP;
        };

        /**
         * Constructs the decluttering draw callback.
         * @param context A shared context among all decluttering objects.
         */
        CalloutDraw(ScreenSpaceLayoutContext* context)
            : _context(context)
        {
            // create the fade uniform.
            _fade = new osg::Uniform(osg::Uniform::FLOAT, FADE_UNIFORM_NAME);
            _fade->set(1.0f);
        }

        /**
         * Draws a bin. Most of this code is copied from osgUtil::RenderBin::drawImplementation.
         * The modifications are (a) skipping code to render child bins, (b) setting a bin-global
         * projection matrix in orthographic space, and (c) calling our custom "renderLeaf()" method
         * instead of RenderLeaf::render()
         */
        void drawImplementation(osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous)
        {
            osg::State& state = *renderInfo.getState();

            unsigned int numToPop = (previous ? osgUtil::StateGraph::numToPop(previous->_parent) : 0);
            if (numToPop > 1) --numToPop;
            unsigned int insertStateSetPosition = state.getStateSetStackSize() - numToPop;

            if (bin->getStateSet())
            {
                state.insertStateSet(insertStateSetPosition, bin->getStateSet());
            }

            // apply a window-space projection matrix.
            const osg::Viewport* vp = renderInfo.getCurrentCamera()->getViewport();
            if (vp)
            {
                //TODO see which is faster

                osg::ref_ptr<osg::RefMatrix>& m = _ortho2D.get();
                if (!m.valid())
                    m = new osg::RefMatrix();

                //m->makeOrtho2D( vp->x(), vp->x()+vp->width()-1, vp->y(), vp->y()+vp->height()-1 );
                m->makeOrtho(vp->x(), vp->x() + vp->width() - 1, vp->y(), vp->y() + vp->height() - 1, -1000, 1000);
                state.applyProjectionMatrix(m.get());
            }

            // initialize the fading uniform
            RunningState rs;

            // render the list
            osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();

            for (osgUtil::RenderBin::RenderLeafList::reverse_iterator rlitr = leaves.rbegin();
                rlitr != leaves.rend();
                ++rlitr)
            {
                osgUtil::RenderLeaf* rl = *rlitr;
                //if (rl->_depth > 0.0f)
                {
                    renderLeaf(rl, renderInfo, previous, rs);
                    previous = rl;
                }
            }

            if (bin->getStateSet())
            {
                state.removeStateSet(insertStateSetPosition);
            }

            // the leader lines
            CalloutImplementation::CameraLocal& local = _sortCallback->_cameraLocal.get(renderInfo.getCurrentCamera());
                
            if (local._leadersDirty)
            {
                local._leaders->dirty();
                local._leadersDirty = false;
            }

            renderInfo.getState()->applyModelViewMatrix(osg::Matrix::identity());

            renderInfo.getState()->apply(local._leaders->getStateSet());
            renderInfo.getState()->apply(local._leaders->getGPUStateSet());
            glDepthFunc(GL_ALWAYS);
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);

            if (renderInfo.getState()->getUseModelViewAndProjectionUniforms())
                renderInfo.getState()->applyModelViewAndProjectionUniformsIfRequired();

            local._leaders->draw(renderInfo);
        }

        /**
         * Renders a single leaf. We already applied the projection matrix, so here we only
         * need to apply a modelview matrix that specifies the ortho offset of the drawable.
         *
         * Most of this code is copied from RenderLeaf::draw() -- but I removed all the code
         * dealing with nested bins, since decluttering does not support them.
         */
        void renderLeaf(osgUtil::RenderLeaf* leaf, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous, RunningState& rs)
        {
            osg::State& state = *renderInfo.getState();

            // don't draw this leaf if the abort rendering flag has been set.
            if (state.getAbortRendering())
            {
                //cout << "early abort"<<endl;
                return;
            }

            state.applyModelViewMatrix(leaf->_modelview.get());

            if (previous)
            {
                // apply state if required.
                osgUtil::StateGraph* prev_rg = previous->_parent;
                osgUtil::StateGraph* prev_rg_parent = prev_rg->_parent;
                osgUtil::StateGraph* rg = leaf->_parent;
                if (prev_rg_parent != rg->_parent)
                {
                    osgUtil::StateGraph::moveStateGraph(state, prev_rg_parent, rg->_parent);

                    // send state changes and matrix changes to OpenGL.
                    state.apply(rg->getStateSet());

                }
                else if (rg != prev_rg)
                {
                    // send state changes and matrix changes to OpenGL.
                    state.apply(rg->getStateSet());
                }
            }
            else
            {
                // apply state if required.
                osgUtil::StateGraph::moveStateGraph(state, NULL, leaf->_parent->_parent);

                state.apply(leaf->_parent->getStateSet());
            }

            // if we are using osg::Program which requires OSG's generated uniforms to track
            // modelview and projection matrices then apply them now.
            if (state.getUseModelViewAndProjectionUniforms())
                state.applyModelViewAndProjectionUniformsIfRequired();

            // apply the fading uniform
            const osg::Program::PerContextProgram* pcp = state.getLastAppliedProgramObject();
            if (pcp)
            {
                if (pcp != rs.lastPCP || leaf->_depth != rs.lastFade)
                {
                    rs.lastFade = s_declutteringEnabledGlobally ? leaf->_depth : 1.0f;
                    _fade->set(rs.lastFade);
                    pcp->apply(*_fade.get());
                }
            }
            rs.lastPCP = pcp;

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
                Threading::ScopedMutexLock lock(_vpMutex);
                if (!_vpInstalled)
                {
                    VirtualProgram* vp = VirtualProgram::getOrCreate(getStateSet());
                    vp->setName("ScreenSpaceLayout");
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
        static Threading::Mutex _vpMutex;
        static bool _vpInstalled;
    };

    Threading::Mutex osgEarthScreenSpaceLayoutRenderBin::_vpMutex;
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
    REGISTER_OSGEARTH_EXTENSION(osgearth_decluttering,        ScreenSpaceLayoutExtension);
}

