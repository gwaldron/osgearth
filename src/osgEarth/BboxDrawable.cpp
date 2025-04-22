/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/BboxDrawable>
#include <osg/LineWidth>

using namespace osgEarth;


//------------------------------------------------------------------------

BboxDrawable::BboxDrawable(const osg::BoundingBox& box, const BBoxSymbol& bboxSymbol) :
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

    osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_PER_PRIMITIVE_SET);
    if ( bboxSymbol.fill().isSet() )
    {
        c->push_back( bboxSymbol.fill()->color() );
        osg::DrawElements* de = new osg::DrawElementsUByte(GL_TRIANGLE_STRIP);
        de->addElement(0);
        de->addElement(1);
        de->addElement(3);
        de->addElement(2);
        addPrimitiveSet(de);
        //addPrimitiveSet( new osg::DrawArrays(GL_POLYGON, 0, v->getNumElements()) );
    }

    if ( bboxSymbol.border().isSet() )
    {
        c->push_back( bboxSymbol.border()->color() );
        auto& widthExpr = bboxSymbol.border()->width();
        if (widthExpr.isSet()) {
            Distance width = widthExpr->literal();
            getOrCreateStateSet()->setAttribute(new osg::LineWidth(width.as(Units::PIXELS)), 1);
        }

        addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, 0, v->getNumElements()) );
    }

    setColorArray( c );

    // Disable culling since this bounding box will eventually be drawn in screen space.
    setCullingActive(false);
}
