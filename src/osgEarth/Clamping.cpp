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
#include <osgEarth/Clamping>
#include <osg/Drawable>
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>

using namespace osgEarth;

const int   Clamping::AnchorAttrLocation        = osg::Drawable::ATTRIBUTE_6;
const char* Clamping::AnchorAttrName            = "oe_clamp_attrs";
const char* Clamping::HasAttrsUniformName       = "oe_clamp_hasAttrs";
const char* Clamping::AltitudeOffsetUniformName = "oe_clamp_altitudeOffset";

const int   Clamping::HeightsAttrLocation       = osg::Drawable::FOG_COORDS;
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

        void apply(osg::Geometry* geom)
        {
            if ( geom )
            {
                osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
                osg::Vec4Array* anchors = new osg::Vec4Array();
                anchors->reserve( verts->size() );
                for(unsigned i=0; i<verts->size(); ++i)
                {
                    anchors->push_back( osg::Vec4f(
                        (*verts)[i].x(), (*verts)[i].y(),
                        _verticalOffset,
                        Clamping::ClampToGround) );
                }

                geom->setVertexAttribArray    (Clamping::AnchorAttrLocation, anchors);
                geom->setVertexAttribBinding  (Clamping::AnchorAttrLocation, geom->BIND_PER_VERTEX);
                geom->setVertexAttribNormalize(Clamping::AnchorAttrLocation, false);
            }
        }

        void apply(osg::Geode& geode) 
        {
            for(unsigned i=0; i<geode.getNumDrawables(); ++i)
            {
                apply( geode.getDrawable(i)->asGeometry() );
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
        visitor.apply( geom );
    }
}


void
Clamping::installHasAttrsUniform(osg::StateSet* stateset)
{
    if ( stateset )
    {
        stateset->addUniform( new osg::Uniform(Clamping::HasAttrsUniformName, true) );
    }
}

void
Clamping::setHeights(osg::Geometry* geom, osg::FloatArray* hats)
{
    if ( geom )
    {
        geom->setVertexAttribArray( HeightsAttrLocation, hats );
        geom->setVertexAttribBinding( HeightsAttrLocation, geom->BIND_PER_VERTEX );
        geom->setVertexAttribNormalize( HeightsAttrLocation, false );
    }
}
