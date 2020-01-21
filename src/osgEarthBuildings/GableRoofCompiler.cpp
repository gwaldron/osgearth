
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
#include "GableRoofCompiler"
#include <osgEarth/Session>

using namespace osgEarth;
using namespace osgEarth::Buildings;

#define LC "[GableRoofCompiler] "

GableRoofCompiler::GableRoofCompiler(Session* session) :
_session( session )
{
    // build the unit-space template.
    osg::Vec3f LL(0, 0, 0), LM(0.5, 0, 2), LR(1, 0, 0),
               UL(0, 1, 0), UM(0.5, 1, 2), UR(1, 1, 0);

    osg::Vec3f texLL(1, 0, 0), texLM(1, 1, 0), texLR(0, 0, 0),
               texUL(0, 0, 0), texUM(0, 1, 0), texUR(1, 0, 0);

    // set the Y values here if you want the roofs to overhang the gables.
    // this is cool, but creates a backfacing polygon which might now work
    // wekk with shadows.
    osg::Vec3f UO(0,  0, 0),
               LO(0, -0, 0);

    _verts     = new osg::Vec3Array();
    _texCoords = new osg::Vec3Array();

    // first side:
    _verts->push_back(LL+LO); _texCoords->push_back(texLL);
    _verts->push_back(LM+LO); _texCoords->push_back(texLM);
    _verts->push_back(UL+UO); _texCoords->push_back(texUL);
    _verts->push_back(UL+UO); _texCoords->push_back(texUL);
    _verts->push_back(LM+LO); _texCoords->push_back(texLM);
    _verts->push_back(UM+UO); _texCoords->push_back(texUM);
    
    // second side:
    _verts->push_back(LR+LO); _texCoords->push_back(texLR);
    _verts->push_back(UR+UO); _texCoords->push_back(texUR);
    _verts->push_back(LM+LO); _texCoords->push_back(texUM);
    _verts->push_back(LM+LO); _texCoords->push_back(texUM);
    _verts->push_back(UR+UO); _texCoords->push_back(texUR);
    _verts->push_back(UM+UO); _texCoords->push_back(texLM);

    // south gable:
    _verts->push_back(UL); _texCoords->push_back(texUL);
    _verts->push_back(UM); _texCoords->push_back(texUL);
    _verts->push_back(UR); _texCoords->push_back(texUL);

    // north gable:
    _verts->push_back(LL); _texCoords->push_back(texUL);
    _verts->push_back(LR); _texCoords->push_back(texUL);
    _verts->push_back(LM); _texCoords->push_back(texUL);

    // add a chimney :)
    addCappedBox(osg::Vec3f(0.2, 0.2, 0.0), osg::Vec3f(0.3, 0.3, 2.5), _verts.get(), _texCoords.get());
}

bool
GableRoofCompiler::compile(CompilerOutput&       output,
                           const Building*       building,
                           const Elevation*      elevation,
                           const osg::Matrix&    world2local,
                           const osgDB::Options* readOptions) const
{
    if ( !building ) return false;
    if ( !elevation ) return false;
    if ( !elevation->getRoof() ) return false;

    const Roof* roof = elevation->getRoof();

    // precalculate the frame transformation; combining these will
    // prevent any precision loss during the transform.
    osg::Matrix frame = building->getReferenceFrame() * world2local;
    
    // find a texture:
    SkinResource* skin = roof->getSkinResource();
    osg::ref_ptr<osg::StateSet> stateSet;
    if ( skin )
    {
        stateSet = output.getSkinStateSet(skin, readOptions);
    }

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    geom->setUseVertexBufferObjects( true );
    geom->setUseDisplayList( false );

    // copy the unit-space template:
    osg::Vec3Array* verts = new osg::Vec3Array(_verts->begin(), _verts->end());
    geom->setVertexArray( verts );
    
    osg::Vec3Array* texCoords = 0L;
    if ( stateSet.valid() )
    {
        // copy the texture coordinates (though this may not be strictly necessary)
        texCoords = new osg::Vec3Array(_texCoords->begin(), _texCoords->end());
        geom->setTexCoordArray( 0, texCoords );
        geom->setStateSet( stateSet.get() );
    }

    osg::Vec3Array* normals = new osg::Vec3Array();
    normals->reserve( verts->size() );
    geom->setNormalArray( normals );
    geom->setNormalBinding( geom->BIND_PER_VERTEX );

    // highest point (this data is guaranteed to exist)
    float roofZ = elevation->getUppermostZ();

    // the AABB gives us the information to scale+bias the unit template 
    // to the proper size and shape:
    const osg::BoundingBox& aabb = elevation->getAxisAlignedBoundingBox();
    osg::Vec3f scale(aabb.xMax()-aabb.xMin(), aabb.yMax()-aabb.yMin(), 1.0f);
    osg::Vec3f bias (aabb.xMin(), aabb.yMin(), roofZ);

    osg::Vec2f tscale(1.0f, 1.0f);
    if ( skin )
        tscale.set(scale.x() / skin->imageWidth().get(), scale.y() / skin->imageHeight().get());

    // scale and bias the geometry, rotate it back to its actual location,
    // and transform into the final coordinate frame.
    for(int i=0; i<verts->size(); ++i)
    {
        osg::Vec3f& v = (*verts)[i];
        v = osg::componentMultiply(v, scale) + bias;
        elevation->unrotate( v );
        v = v * frame;

        if ( texCoords )
        {
            osg::Vec3f& tx = (*texCoords)[i];
            tx.x() *= tscale.y(), tx.y() *= tscale.x();         
        }
    }

    // calculate normals (after transforming the vertices)
    generateNormals( verts, normals );

    // and finally the triangles.
    geom->addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLES, 0, verts->size()) );

    output.addDrawable( geom.get(), roof->getTag() );

    return true;
}
