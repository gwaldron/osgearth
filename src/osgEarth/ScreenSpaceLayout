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
#ifndef OSGEARTH_SCREEN_SPACE_LAYOUT_H
#define OSGEARTH_SCREEN_SPACE_LAYOUT_H 1

#include <osgEarth/Common>
#include <osgEarth/Config>
#include <osg/Drawable>
#include <osgUtil/RenderLeaf>
#include <limits.h>

#define OSGEARTH_SCREEN_SPACE_LAYOUT_BIN "osgearth_ScreenSpaceLayoutBin"

namespace osgEarth
{
    /**
     * Interface that exposes layout information.
     */
    class ScreenSpaceLayoutData : public osg::Referenced
    {
    public:
        /** Install and return a LayoutData on a drawable. */
        static ScreenSpaceLayoutData* getOrCreate(osg::Drawable* drawable) {
            if (!drawable) return 0L;
            ScreenSpaceLayoutData* ld = dynamic_cast<ScreenSpaceLayoutData*>(drawable->getUserData());
            if (!ld) {
                ld = new ScreenSpaceLayoutData();
                drawable->setUserData(ld);
            }
            return ld;
        }

        /** Constructor */
        ScreenSpaceLayoutData() :
            _priority(0.0f),
            _pixelOffset(0, 0) { }

        /** Decluttering priority - FLT_MAX means don't declutter */
        void setPriority(float value) { _priority = value; }
        float getPriority() const     { return _priority; }

        /** Offset from geoposition in screen pixels */
        void setPixelOffset(const osg::Vec2s& value) { _pixelOffset = value; }
        const osg::Vec2s& getPixelOffset() const     { return _pixelOffset; }

        /** World point for label rotation reference */
        void setAnchorPoint(const osg::Vec3d& value) { _anchorPoint = value; }
        const osg::Vec3d& getAnchorPoint() const { return _anchorPoint; }

        /** Reference point for label rotation reference */
        void setProjPoint(const osg::Vec3d& value) { _refPoint = value; }
        const osg::Vec3d& getProjPoint() const { return _refPoint; }

    public:
        float      _priority;
        osg::Vec2s _pixelOffset;
        osg::Vec3d _anchorPoint, _refPoint;

    public:
        virtual ~ScreenSpaceLayoutData() { }
    };

    /**
     * Custom functor that compares two RenderLeaf's and returns TRUE if the left-hand one
     * is higher priority, otherwise FALSE. You can call setDeclutterPriorityFunctor()
     * to set a custom priority-sorting functor.
     */
    struct DeclutterSortFunctor : public osg::Referenced
    {
        virtual bool operator() ( const osgUtil::RenderLeaf* lhs, const osgUtil::RenderLeaf* rhs ) const =0;
        virtual ~DeclutterSortFunctor() { }
    };

    /**
     * Options to control the annotation decluttering engine.
     */
    class OSGEARTH_EXPORT ScreenSpaceLayoutOptions : public ConfigOptions
    {
    public:
        ScreenSpaceLayoutOptions( const ConfigOptions& co =ConfigOptions() )
            : ConfigOptions         ( co ),
              _minAnimAlpha         ( 0.35f ),
              _minAnimScale         ( 0.45f ),
              _inAnimTime           ( 0.40f ),
              _outAnimTime          ( 0.00f ),
              _sortByPriority       ( false ),
              _sortByDistance       ( true ),
              _snapToPixel          ( false ),
              _maxObjects           ( INT_MAX ),
              _renderBinNumber      ( 13 )
        {
            fromConfig(_conf);
        }

        virtual ~ScreenSpaceLayoutOptions() { }

        /** Alpha value of a fully-occluded object */
        optional<float>& minAnimationAlpha() { return _minAnimAlpha; }
        const optional<float>& minAnimationAlpha() const { return _minAnimAlpha; }

        /** Scale factor of a fully-occluded object */
        optional<float>& minAnimationScale() { return _minAnimScale; }
        const optional<float>& minAnimationScale() const { return _minAnimScale; }

        /** Time (in seconds) for an object to transition from occluded to visible */
        optional<float>& inAnimationTime() { return _inAnimTime; }
        const optional<float>& inAnimationTime() const { return _inAnimTime; }

        /** Time (in seconds) for an object to transition from visible to occluded */
        optional<float>& outAnimationTime() { return _outAnimTime; }
        const optional<float>& outAnimationTime() const { return _outAnimTime; }

        /** If set, activate the ScreenSpaceLayoutData priority-based sorting */
        optional<bool>& sortByPriority() { return _sortByPriority; }
        const optional<bool>& sortByPriority() const { return _sortByPriority; }

        /** If set, activate the ScreenSpaceLayoutData distance-based sorting */
        optional<bool>& sortByDistance() { return _sortByDistance; }
        const optional<bool>& sortByDistance() const { return _sortByDistance; }

        /** Whether to always start rendering text on a pixel boundary, thereby 
          * minimizing filtering artifacts. */
        optional<bool>& snapToPixel() { return _snapToPixel; }
        const optional<bool>& snapToPixel() const { return _snapToPixel; }

        /** Maximum number of objects to draw after sorting */
        optional<unsigned>& maxObjects() { return _maxObjects; }
        const optional<unsigned>& maxObjects() const { return _maxObjects; }

        /** Render bin number to use for the screen layout */
        optional<int>& renderOrder() { return _renderBinNumber; }
        const optional<int>& renderOrder() const { return _renderBinNumber; }

    public:

        Config getConfig() const;

    protected:
        optional<float>    _minAnimAlpha;
        optional<float>    _minAnimScale;
        optional<float>    _inAnimTime;
        optional<float>    _outAnimTime;
        optional<bool>     _sortByPriority;
        optional<bool>     _sortByDistance;
        optional<bool>     _snapToPixel;
        optional<unsigned> _maxObjects;
        optional<int>      _renderBinNumber;

        void fromConfig( const Config& conf );
    };

    struct OSGEARTH_EXPORT ScreenSpaceLayout
    {
        /**
         * Assigns a stateset to the screen-space layout engine.
         * Drawables rendered while this stateset is active will be projected from
         * scene space to 2D screen space with optional decluttering.
         */
        static void activate(osg::StateSet* stateSet); //, int binNum =13);

        /**
         * Deactivates the use of the screen-space layout engine for a stateset.
         */
        static void deactivate(osg::StateSet* stateSet);

        /**
         * Enables or disables decluttering globally.
         */
        static void setDeclutteringEnabled(bool enabled);

        /**
         * Applies the provided options to the layout engine.
         */
        static void setOptions(const ScreenSpaceLayoutOptions& options);

        /**
         * Fetches the current layout options
         */
        static const ScreenSpaceLayoutOptions& getOptions();

    public: // advanced

        /**
         * Sets a functor to use to determine render leaf priority for declutter sorting.
         */
        static void setSortFunctor( DeclutterSortFunctor* f );

        /**
         * Clears a custom priority functor that was set using setDeclutterPriorityFunctor,
         * reverting to the default behavior (which is to sort by distance from the camera).
         */
        static void clearSortFunctor();
    };

} // namespace osgEarth

#endif //OSGEARTH_SCREEN_SPACE_LAYOUT_H
