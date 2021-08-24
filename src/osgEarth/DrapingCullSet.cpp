/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/DrapingCullSet>
#include <osgUtil/CullVisitor>

#define LC "[DrapingCullSet] "

using namespace osgEarth;
using namespace osgEarth::Util;


DrapingManager::DrapingManager() :
    _sets(OE_MUTEX_NAME),
    _renderBinNum(1)
{
    //nop
}

DrapingCullSet&
DrapingManager::get(const osg::Camera* cam)
{
    // Known issue: it is possible for a draping cull set to be "orphaned" - this
    // would happen if the cull set were populated and then not used. This is a
    // very unlikely scenario (because the scene graph would have to change mid-cull)
    // but nevertheless possible.
    return _sets.get(cam);
}

//............................................................................


DrapingCullSet::DrapingCullSet()
{
    // nop
}

const osg::BoundingSphere&
DrapingCullSet::getBound() const
{
    if (_data.empty())
    {
        static osg::BoundingSphere s_empty;
        return s_empty;
    }
    else
    {
        return _data.rbegin()->second._bs;
    }
}

void
DrapingCullSet::push(DrapeableNode* node, const osg::NodePath& path, const osg::FrameStamp* fs)
{
    FrameData& data = _data[fs->getFrameNumber()];

    Entry entry;
    entry._node = node;
    entry._path.setNodePath(path);
    entry._matrix = new osg::RefMatrix(osg::computeLocalToWorld(path));

    data._bs.expandBy(osg::BoundingSphere(
        node->getBound().center() * (*entry._matrix.get()),
        node->getBound().radius()));

    data._entries.emplace_back(std::move(entry));
}

void
DrapingCullSet::accept(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        if (nv.getFrameStamp() == nullptr)
            return;

        if (_data.empty())
            return;

        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );

        // We will use the visitor's path to prevent doubely-applying the statesets
        // of common ancestors
        const osg::NodePath& nvPath = nv.getNodePath();

        FrameData& data = _data.rbegin()->second;

        for(auto& entry : data._entries)
        {
            osg::ref_ptr<DrapeableNode> drapeable;
            if (entry._node.lock(drapeable) == false)
                continue;

            if (drapeable->getDrapingEnabled() == false)
                continue;

            // If there's an active (non-identity matrix), apply it
            if ( entry._matrix.valid() )
            {
                osg::ref_ptr<osg::RefMatrix> m = osg::clone(entry._matrix.get());
                m->postMult( *cv->getModelViewMatrix() );
                cv->pushModelViewMatrix( m.get(), osg::Transform::RELATIVE_RF );
            }

            // After pushing the matrix, we can perform the culling bounds test.
            if ( !cv->isCulled(drapeable->getBound() ) )
            {
                // Apply the statesets in the entry's node path, but skip over the ones that are
                // shared with the current visitor's path since they are already in effect.
                // Count them so we can pop them later.
                int numStateSets = 0;
                osg::RefNodePath nodePath;
                if ( entry._path.getRefNodePath(nodePath) )
                {
                    for(unsigned i=0; i<nodePath.size(); ++i)
                    {
                        if (nodePath[i].valid())
                        {
                            if (i >= nvPath.size() || nvPath[i] != nodePath[i].get())
                            {
                                osg::StateSet* stateSet = nodePath[i]->getStateSet();
                                if ( stateSet )
                                {
                                    cv->pushStateSet( stateSet );
                                    ++numStateSets;
                                }
                            }
                        }
                    }
                }

                // Cull the DrapeableNode's children (but not the DrapeableNode itself!)
                for(unsigned i=0; i<entry._node->getNumChildren(); ++i)
                {
                    drapeable->getChild(i)->accept( nv );
                }
            
                // pop the same number we pushed
                for(int i=0; i<numStateSets; ++i)
                {
                    cv->popStateSet();
                }
            }

            // pop the model view:
            if ( entry._matrix.valid() )
            {
                cv->popModelViewMatrix();
            }
        }

        _data.erase(--_data.end());
    }
}
