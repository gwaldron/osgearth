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
#include <osgEarthAnnotation/BboxDrawable>
#include <osgEarthAnnotation/AnnotationUtils>

#include <osg/LineWidth>

using namespace osgEarth;
using namespace osgEarth::Annotation;


//------------------------------------------------------------------------

BboxDrawable::BboxDrawable( const osg::BoundingBox& box, const BBoxSymbol &bboxSymbol ) :
osg::Geometry()
{
    setUseVertexBufferObjects(true);

    float margin = bboxSymbol.margin().isSet() ? bboxSymbol.margin().value() : 2.f;
    osg::Vec3Array* v = new osg::Vec3Array();
    if ( bboxSymbol.geom().isSet() && bboxSymbol.geom().value() == BBoxSymbol::GEOM_BOX_ORIENTED )
    {
        float h = box.yMax() - box.yMin() + 2.f * margin;
        v->push_back( osg::Vec3(box.xMax()+margin+h/2.f, box.yMax()+margin-h/2.f, 0) );
    }
    v->push_back( osg::Vec3(box.xMax()+margin, box.yMax()+margin, 0) );
    v->push_back( osg::Vec3(box.xMin()-margin, box.yMax()+margin, 0) );
    v->push_back( osg::Vec3(box.xMin()-margin, box.yMin()-margin, 0) );
    v->push_back( osg::Vec3(box.xMax()+margin, box.yMin()-margin, 0) );
    setVertexArray(v);
    if ( v->getVertexBufferObject() )
        v->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

    osg::Vec4Array* c = new osg::Vec4Array();
    if ( bboxSymbol.fill().isSet() )
    {
        c->push_back( bboxSymbol.fill()->color() );
        addPrimitiveSet( new osg::DrawArrays(GL_POLYGON, 0, v->getNumElements()) );
    }

    if ( bboxSymbol.border().isSet() )
    {
        c->push_back( bboxSymbol.border()->color() );
        if ( bboxSymbol.border()->width().isSet() )
            getOrCreateStateSet()->setAttribute( new osg::LineWidth( bboxSymbol.border()->width().value() ));
        addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, 0, v->getNumElements()) );
    }

    setColorArray( c );
    setColorBinding( osg::Geometry::BIND_PER_PRIMITIVE_SET );

    // add the static "isText=true" uniform; this is a hint for the annotation shaders
    // if they get installed.
    static osg::ref_ptr<osg::Uniform> s_isTextUniform = new osg::Uniform(osg::Uniform::BOOL, AnnotationUtils::UNIFORM_IS_TEXT());
    s_isTextUniform->set( false );
    getOrCreateStateSet()->addUniform( s_isTextUniform.get() );

    // Disable culling since this bounding box will eventually be drawn in screen space.
#if OSG_MIN_VERSION_REQUIRED(3,4,0)
    setCullingActive(false);
#endif
}
