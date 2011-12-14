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

using namespace osgEarth::Annotation;

void
ScaleDrawStateTechnique::preCull( osg::NodeVisitor& nv ) const
{
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
    osg::RefMatrix* proj = cv->getProjectionMatrix();
    osg::RefMatrix* newProj = new osg::RefMatrix();
    //double v, a, n, f;
    //proj->getPerspective(v, a, n, f);
    //v *= 0.95;
    //newProj->makePerspective(v, a, n, f);
    double l, r, b, t, n, f;
    proj->getFrustum( l, r, b, t, n, f );
    double x = (r-l)*0.01;
    l += x, r -= x, b += x, t -= x;
    newProj->makeFrustum( l, r, b, t, n, f );
    cv->pushProjectionMatrix( newProj );
}

void
ScaleDrawStateTechnique::postCull( osg::NodeVisitor& nv ) const
{
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
    cv->popProjectionMatrix();
}

