
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "Compiler"

using namespace osgEarth;
using namespace osgEarth::Buildings;

#define LC "[Compiler] "

void
Compiler::addCappedBox(const osg::Vec3f& LL,
                       const osg::Vec3f& UR,
                       osg::Vec3Array*   v,
                       osg::Vec3Array*   t) const
{
    osg::Vec3f
        BLL(LL), BLR(UR.x(),LL.y(),LL.z()), BUL(LL.x(),UR.y(),LL.z()), BUR(UR.x(),UR.y(),LL.z()),
        TLL(BLL.x(),BLL.y(),UR.z()), TLR(BLR.x(),BLR.y(),UR.z()), TUL(BUL.x(),BUL.y(),UR.z()), TUR(BUR.x(),BUR.y(),UR.z());

    // cap:
    v->push_back(TLL); v->push_back(TLR); v->push_back(TUL);
    v->push_back(TUL); v->push_back(TLR); v->push_back(TUR);

    //sides:
    v->push_back(BLL); v->push_back(BLR); v->push_back(TLL);
    v->push_back(TLL); v->push_back(BLR); v->push_back(TLR);

    v->push_back(BLR); v->push_back(BUR); v->push_back(TLR);
    v->push_back(TLR); v->push_back(BUR); v->push_back(TUR);

    v->push_back(BUR); v->push_back(BUL); v->push_back(TUR);
    v->push_back(TUR); v->push_back(BUL); v->push_back(TUL);

    v->push_back(BUL); v->push_back(BLL); v->push_back(TUL);
    v->push_back(TUL); v->push_back(BLL); v->push_back(TLL);

    //TODO.. just plaster it on for now
    for(int i=0; i<30; ++i)
        t->push_back(osg::Vec3f(0,0,0));
}

void
Compiler::generateNormals(const osg::Vec3Array* verts,
                          osg::Vec3Array*       normals) const
{   
    for(int i=0; i<verts->size(); i+=3)
    {
        osg::Vec3f n = ((*verts)[i+2]-(*verts)[i+1]) ^ ((*verts)[i]-(*verts)[i+1]);
        n.normalize();
        normals->push_back(n);
        normals->push_back(n);
        normals->push_back(n);
    }
}
