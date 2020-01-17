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
#include <osgEarth/Clamping>
#include <osgEarth/CullingUtils>
#include <osgEarth/LineDrawable>

using namespace osgEarth;

int         Clamping::AnchorAttrLocation        = osg::Drawable::ATTRIBUTE_6;
int         Clamping::HeightsAttrLocation       = osg::Drawable::FOG_COORDS;

const char* Clamping::AnchorAttrName            = "oe_clamp_attrs";
const char* Clamping::HasAttrsUniformName       = "oe_clamp_hasAttrs";
const char* Clamping::AltitudeOffsetUniformName = "oe_clamp_altitudeOffset";

const char* Clamping::HeightsAttrName           = "oe_clamp_height";

const float Clamping::ClampToAnchor = 1.0f;
const float Clamping::ClampToGround = 0.0f;

namespace
{
    struct ApplyDefaultsVisitor : public osg::NodeVisitor
    {
        float _verticalOffset;

        ApplyDefaultsVisitor(float verticalOffset)
        {
            _verticalOffset = verticalOffset;
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0 );
        }

        void applyGeometry(osg::Geometry* geom)
        {
            osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
            osg::Vec4Array* anchors = new osg::Vec4Array();
            anchors->setBinding(osg::Array::BIND_PER_VERTEX);
            anchors->reserve( verts->size() );
            for(unsigned i=0; i<verts->size(); ++i)
            {
                anchors->push_back( osg::Vec4f(
                    (*verts)[i].x(), (*verts)[i].y(),
                    _verticalOffset,
                    Clamping::ClampToGround) );
            }
            geom->setVertexAttribArray(Clamping::AnchorAttrLocation, anchors);
        }

        void apply(osg::Drawable& drawable)
        {
            osg::Geometry* geom = drawable.asGeometry();
            if (geom)
            {
                applyGeometry(geom);
            }
        }
    };
}

void
Clamping::applyDefaultClampingAttrs(osg::Node* node, float verticalOffset)
{
    if ( node )
    {
        ApplyDefaultsVisitor visitor( verticalOffset );
        node->accept( visitor );
    }
}

void
Clamping::applyDefaultClampingAttrs(osg::Geometry* geom, float verticalOffset)
{
    if ( geom )
    {
        ApplyDefaultsVisitor visitor( verticalOffset );
        visitor.apply( *geom );
    }
}


void
Clamping::installHasAttrsUniform(osg::StateSet* stateset)
{
    if ( stateset )
    {
        stateset->setDefine("OE_CLAMP_HAS_ATTRIBUTES");
        //stateset->addUniform( new osg::Uniform(Clamping::HasAttrsUniformName, true) );
    }
}

void
Clamping::setHeights(osg::Geometry* geom, osg::FloatArray* hats)
{
    if ( geom )
    {
        hats->setBinding(osg::Array::BIND_PER_VERTEX);
        hats->setNormalize(false);

        LineDrawable* line = dynamic_cast<LineDrawable*>(geom);
        if (line)
        {
            line->importVertexAttribArray(HeightsAttrLocation, hats);
        }
        else
        {
            geom->setVertexAttribArray(HeightsAttrLocation, hats);
        }
    }
}



#undef LC
#define LC "[ClampingCullSet] "

ClampingCullSet::ClampingCullSet() :
_frameCulled( true )
{
    // nop
}

void
ClampingCullSet::push(ClampableNode* node, const osg::NodePath& path, const osg::FrameStamp* fs)
{
    // Reset the set if this is the first push after a cull.
    if ( _frameCulled )
    {
        _frameCulled = false;
        _entries.clear();
        _bs.init();
    }

    _entries.push_back( Entry() );
    Entry& entry = _entries.back();
    entry._node = node;
    entry._path.setNodePath( path );
    entry._matrix = new osg::RefMatrix( osg::computeLocalToWorld(path) );
    entry._frame = fs ? fs->getFrameNumber() : 0;
    _bs.expandBy( osg::BoundingSphere(
        node->getBound().center() * (*entry._matrix.get()),
        node->getBound().radius() ));

    OE_DEBUG << LC << "Pushed " << node << " on frame " << entry._frame << std::endl;
}

void
ClampingCullSet::accept(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        ProxyCullVisitor* cv = dynamic_cast<ProxyCullVisitor*>(&nv);

        // We will use the visitor's path to prevent doubely-applying the statesets
        // of common ancestors
        const osg::NodePath& nvPath = nv.getNodePath();

        int frame = nv.getFrameStamp() ? nv.getFrameStamp()->getFrameNumber() : 0u;

        unsigned passed = 0u;

        for( std::vector<Entry>::iterator entry = _entries.begin(); entry != _entries.end(); ++entry )
        {
            if ( frame - entry->_frame > 1 )
                continue;

            // If there's an active (non-identity matrix), apply it
            if ( entry->_matrix.valid() )
            {
                entry->_matrix->postMult( *cv->getModelViewMatrix() );
                cv->pushModelViewMatrix( entry->_matrix.get(), osg::Transform::RELATIVE_RF );
            }

            // After pushing the matrix, we can perform the culling bounds test.
            if (!cv->isCulledByProxyFrustum(*entry->_node.get()))
            {
                // Apply the statesets in the entry's node path, but skip over the ones that are
                // shared with the current visitor's path since they are already in effect.
                // Count them so we can pop them later.
                int numStateSets = 0;
                osg::RefNodePath nodePath;
                if ( entry->_path.getRefNodePath(nodePath) )
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
                                    cv->getCullVisitor()->pushStateSet( stateSet );
                                    ++numStateSets;
                                }
                            }
                        }
                    }
                }

                // Cull the DrapeableNode's children (but not the DrapeableNode itself!)
                for(unsigned i=0; i<entry->_node->getNumChildren(); ++i)
                {
                    entry->_node->getChild(i)->accept( nv );
                }
            
                // pop the same number we pushed
                for(int i=0; i<numStateSets; ++i)
                {
                    cv->getCullVisitor()->popStateSet();
                }

                ++passed;
            }

            // pop the model view:
            if ( entry->_matrix.valid() )
            {
                cv->popModelViewMatrix();
            }
            
            //Registry::instance()->startActivity("ClampingCullSet", Stringify() << std::hex << this << std::dec << " / " << passed << "/" << _entries.size());
        }

        // mark this set so it will reset for the next frame
        _frameCulled = true;
    }
}

ClampingCullSet&
ClampingManager::get(const osg::Camera* cam)
{
    // Known issue: it is possible for a draping cull set to be "orphaned" - this
    // would happen if the cull set were populated and then not used. This is a
    // very unlikely scenario (because the scene graph would have to change mid-cull)
    // but nevertheless possible.
    //Registry::instance()->startActivity("ClampingManager", Stringify() << _sets.size());
    return _sets.get(cam);
}