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

#include <osgEarthAnnotation/DrawState>
#include <osgEarthSymbology/Color>
#include <osg/MatrixTransform>
#include <osgFX/Outline>
#include <osg/ShapeDrawable>
#include <osg/CullFace>
#include <osg/Depth>

using namespace osgEarth::Annotation;

void
ScaleDrawStateTechnique::preTraverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
        osg::RefMatrix* proj = cv->getProjectionMatrix();
        osg::RefMatrix* newProj = new osg::RefMatrix();
        double l, r, b, t, n, f;
        proj->getFrustum( l, r, b, t, n, f );
        double x = (r-l)*0.01;
        l += x, r -= x, b += x, t -= x;
        newProj->makeFrustum( l, r, b, t, n, f );
        cv->pushProjectionMatrix( newProj );
    }
}

void
ScaleDrawStateTechnique::postTraverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
        cv->popProjectionMatrix();
    }
}

//--------------------------------------------------------------------------

OutlineDrawStateTechnique::OutlineDrawStateTechnique()
{
    osgFX::Outline* outline = new osgFX::Outline();
    outline->setWidth( 3.0f );
    outline->setColor( Color::Cyan );
    _injection = outline;
}

//--------------------------------------------------------------------------

EncircleDrawStateTechnique::EncircleDrawStateTechnique()
{
    osg::Group* g = new osg::Group();

    _xform = new osg::MatrixTransform();
    g->addChild( _xform );

    osg::Geode* geode = new osg::Geode();
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( new osg::Sphere(osg::Vec3(0,0,0), 1.0) );
    sd->setColor( Color(Color::Red,0.4) );

    osg::StateSet* s = sd->getOrCreateStateSet();
    s->setMode(GL_LIGHTING,0);
    s->setMode(GL_BLEND,1);
    s->setAttributeAndModes( new osg::Depth(osg::Depth::LESS, 0, 1, false), 1 );
    s->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), 1 );
    geode->addDrawable( sd );
    _xform->addChild( geode );

    _injection = g;
}

void
EncircleDrawStateTechnique::preTraverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        if ( _injection->getNumChildren() > 1 )
        {
            const osg::BoundingSphere& bs = _injection->getChild(1)->getBound();
            _xform->setMatrix(
                osg::Matrix::scale( bs.radius(),bs.radius(),bs.radius() ) *
                osg::Matrix::translate( bs.center() ) );
        }
    }
}
