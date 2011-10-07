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
#include <osgEarth/Utils>
#include <osgUtil/RenderBin>

#define LC "[Declutter] "

using namespace osgEarth;
using namespace osgEarth::Annotation;

/**
 * A custom RenderLeaf sorting algorithm for decluttering objects.
 * It first sorts front-to-back so that objects closer to the camera get
 * priority. Then it goes through the drawables and removes any that 
 * try to occupy already occupied space (in eye space)
 *
 * We can easily modify this to do other interesting things, like shift
 * objects around or scale them based on occlusion.
 *
 * To submit an object for decluttering, all you have to do is call
 * obj->getStateSet()->setRenderBinDetails( bin, OSGEARTH_DECLUTTER_BIN );
 */
struct /*internal*/ DeclutterSort : public osgUtil::RenderBin::SortCallback
{
    void sortImplementation(osgUtil::RenderBin* bin)
    {
        // first sort by depth
        bin->sortFrontToBack();

        osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();
        std::vector<osg::BoundingBox> _used;

        for( osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin(); i != leaves.end(); )
        {
            bool passed = true;

            osgUtil::RenderLeaf* leaf = *i;
            const osg::Drawable* drawable = leaf->getDrawable();
            osg::BoundingBox box = drawable->getBound();

            // bring the drawable's bounding box into eye-space:
            osg::Matrix mvp = (*leaf->_modelview.get()) * (*leaf->_projection.get());
            box.set(
                osg::Vec3(box.xMin(),box.yMin(),box.zMin()) * mvp,
                osg::Vec3(box.xMax(),box.yMax(),box.zMax()) * mvp );

            // normalize the x/y extents:
            if ( box.xMin() > box.xMax() ) std::swap( box.xMin(), box.xMax() );
            if ( box.yMin() > box.yMax() ) std::swap( box.yMin(), box.yMax() );

            // weed out any drawables that are obscured by closer drawables.
            for( std::vector<osg::BoundingBox>::const_iterator j = _used.begin(); j != _used.end(); ++j )
            {
                // only need a 2D test:
                bool isClear =
                    box.xMin() > j->xMax() ||
                    box.xMax() < j->xMin() ||
                    box.yMin() > j->yMax() ||
                    box.yMax() < j->yMin();

                if ( !isClear )
                {
                    passed = false;
                    break;
                }
            }

            if ( passed )
            {
                _used.push_back( box );
                ++i;
            }
            else
            {
                // removes it altogether...
                i = leaves.erase( i );

#if 0
                // scales it down...
                leaf->_modelview->set( osg::Matrix::scale(osg::Vec3(.1,.1,.1)) * (*leaf->_modelview.get()) );
                ++i;
#endif
            }
        }        
    }
};

/**
 * The actual custom render bin
 */
class osgEarthAnnotationDeclutterRenderBin : public osgUtil::RenderBin
{
public:
    static const std::string BIN_NAME;

    osgEarthAnnotationDeclutterRenderBin() {
        setSortCallback( new DeclutterSort() );
    }
};
const std::string osgEarthAnnotationDeclutterRenderBin::BIN_NAME = OSGEARTH_DECLUTTER_BIN;

/**
 * Proxy class that registers the custom render bin's prototype with the
 * rendering system
 */
template<class T>
struct osgEarthAnnotationRegisterRenderBinProxy
{
    osgEarthAnnotationRegisterRenderBinProxy(const std::string& name)
    {
        _prototype = new T();
        osgUtil::RenderBin::addRenderBinPrototype(name, _prototype.get());
    }

    ~osgEarthAnnotationRegisterRenderBinProxy()
    {
        osgUtil::RenderBin::removeRenderBinPrototype(_prototype.get());
        _prototype = 0L;
    }

    osg::ref_ptr<T> _prototype;
};

/** the actual registration. */
extern "C" void osgEarth_declutter(void) {}
static osgEarthAnnotationRegisterRenderBinProxy<osgEarthAnnotationDeclutterRenderBin> s_regbin(
    osgEarthAnnotationDeclutterRenderBin::BIN_NAME);
