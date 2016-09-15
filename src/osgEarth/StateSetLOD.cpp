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
#include <osgEarth/StateSetLOD>
#include <osgEarth/CullingUtils>

using namespace osgEarth;

#undef  LC
#define LC "[StateSetLOD] "

void
StateSetLOD::addStateSet(osg::StateSet* stateSet, float min, float max)
{
    if ( stateSet )
    {
        _ranges.push_back(std::make_pair(min, max));
        _stateSets.push_back(stateSet);
    }
}

void
StateSetLOD::setRange(unsigned index, float min, float max)
{
    if ( index < (unsigned)_ranges.size() )
    {
        _ranges[index].first = min;
        _ranges[index].second = max;
    }
}

void
StateSetLOD::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = osgEarth::Culling::asCullVisitor(nv);
        
        float value;
        if ( !_getValue.valid() )
        {
            osg::BoundingSphere::vec_type center = getBound().center();
            value = nv.getDistanceToViewPoint( center, true );
        }
        else
        {
            value = (*_getValue.get())(nv);
        }

        unsigned count = 0;
        for(unsigned i=0; i<(unsigned)_ranges.size(); ++i)
        {
            if (_ranges[i].first <= value && value < _ranges[i].second)
            {
                cv->pushStateSet(_stateSets[i].get());
                ++count;
            }
        }

        osg::Group::traverse(nv);

        for(unsigned i=0; i<count; ++i)
        {
            cv->popStateSet();
        }
    }
    else
    {
        osg::Group::traverse(nv);
    }
}
