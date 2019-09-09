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
#ifndef OSGEARTHUTIL_PAGEDNODE_H
#define OSGEARTHUTIL_PAGEDNODE_H

#include <osgEarth/Common>
#include <osgEarth/optional>
#include <osg/PagedLOD>

namespace osgEarth
{
    /**
     * PagedNode is a group with a self-contained paged child.
     * It wraps the details of a PagedNode node into a container
     * that is easier to manage.
     *
     * To use, override the class. Call setNode() with the default
     * node to display, and implement loadChild() to load the 
     * paged child node when it comes within range. Set the page-in
     * range for the paged child node with setRange() and setRangeMode().
     * Finally, call setupPaging() to complete setup.
     */
    class OSGEARTH_EXPORT PagedNode : public osg::Group
    {
    public:
        PagedNode();

        //! Sets the node to display by default when children
        //! are not loaded.
        void setNode(osg::Node* node);

        //! Override to load paged child node
        virtual osg::Node* loadChild() { return 0L; }

        //! Bounding sphere of paged data
        virtual osg::BoundingSphere getChildBound() const;

        //! Returns true by default; override for custom behavior
        virtual bool hasChild() const;

        //! Call this after setting up your data on the attach point
        void setupPaging();

        /**
         * Gets the range factor
         */
        float getRangeFactor() const { return _rangeFactor; }

        /**
         * Sets the range factor
         */
        void setRangeFactor(float rangeFactor) { _rangeFactor = rangeFactor; }

        //! Sets the page-in rage mode (distance v. pixel-size; default is distance)
        void setRangeMode(const osg::LOD::RangeMode);

        //! Sets the range (or pixel size) at which to page in the child data.
        void setRange(float range) { _range = range; }

        //! Gets whether this node is additive.
        bool getAdditive() const { return _additive; }

        //! Sets whether the child replaces the default node (non-additive)
        //! or renders alongside the default node (additive)
        void setAdditive(bool value) { _additive = value; }

    protected:
        osg::Group* _attachPoint;
        osg::PagedLOD* _plod;
        bool _additive;
        optional<float> _range;
        float _rangeFactor;
    };    
}

#endif // OSGEARTHUTIL_PAGEDNODE_H
