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
#include <set>

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
        // todo: a variation on this might be to sort by "priority" or some other
        // attribute found in the user data..?
        bin->sortFrontToBack();

        osgUtil::RenderBin::RenderLeafList& leaves = bin->getRenderLeafList();
        
        // List of render leaves that pass the initial visibility test
        osgUtil::RenderBin::RenderLeafList passed;
        passed.reserve( leaves.size() );

        // list of occlusion boxes
        std::vector<osg::BoundingBox> used;

        // Track the parent nodes of drawables that are obscured (and culled). Drawables
        // with the same parent node are considered to be grouped and will be culled as 
        // a group.
        std::set<const osg::Node*> culledParents;

        for( osgUtil::RenderBin::RenderLeafList::iterator i = leaves.begin(); i != leaves.end(); ++i )
        {
            bool visible = true;

            osgUtil::RenderLeaf* leaf = *i;
            const osg::Drawable* drawable = leaf->getDrawable();

            // if this leaf is already in a culled group, skip it.
            if ( culledParents.find( drawable->getParent(0) ) != culledParents.end() )
            {
                continue;
            }

            // bring the drawable's bounding box into eye-space:
            osg::BoundingBox box = drawable->getBound();

            osg::Matrix mvp = (*leaf->_modelview.get()) * (*leaf->_projection.get());
            box.set(
                osg::Vec3(box.xMin(),box.yMin(),box.zMin()) * mvp,
                osg::Vec3(box.xMax(),box.yMax(),box.zMax()) * mvp );

            // normalize the x/y extents:
            if ( box.xMin() > box.xMax() ) std::swap( box.xMin(), box.xMax() );
            if ( box.yMin() > box.yMax() ) std::swap( box.yMin(), box.yMax() );

            // weed out any drawables that are obscured by closer drawables.
            // TODO: think about a more efficient algorithm - right now we are just using
            // brute force to compare all bbox's
            for( std::vector<osg::BoundingBox>::const_iterator j = used.begin(); j != used.end(); ++j )
            {
                // only need a 2D test since we're in clip space
                bool isClear =
                    box.xMin() > j->xMax() ||
                    box.xMax() < j->xMin() ||
                    box.yMin() > j->yMax() ||
                    box.yMax() < j->yMin();

                if ( !isClear )
                {
                    visible = false;
                    break;
                }
            }

            if ( visible )
            {
                used.push_back( box );
                passed.push_back( leaf );
            }
            else
            {
                culledParents.insert( drawable->getParent(0) );

#if 0
                // scales it down...
                leaf->_modelview->set( osg::Matrix::scale(osg::Vec3(.1,.1,.1)) * (*leaf->_modelview.get()) );
                ++i;
#endif
            }
        }

        leaves.clear();
        for( osgUtil::RenderBin::RenderLeafList::const_iterator i=passed.begin(); i != passed.end(); ++i )
        {
            osgUtil::RenderLeaf* leaf = *i;
            if ( culledParents.find( leaf->getDrawable()->getParent(0) ) == culledParents.end() )
            {
                leaves.push_back( leaf );
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

/** the actual registration. */
extern "C" void osgEarth_declutter(void) {}
static osgEarthAnnotationRegisterRenderBinProxy<osgEarthAnnotationDeclutterRenderBin> s_regbin(
    osgEarthAnnotationDeclutterRenderBin::BIN_NAME);
