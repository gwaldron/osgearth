
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
#include "InstancedBuildingCompiler"
#include <osgEarth/Session>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Buildings;

#define LC "[InstancedBuildingCompiler] "

InstancedBuildingCompiler::InstancedBuildingCompiler(Session* session) :
_session( session )
{
    //nop
}

bool
InstancedBuildingCompiler::compile(const Building*    building,
                                   CompilerOutput&    output,
                                   const osg::Matrix& world2local,
                                   ProgressCallback*  progress)
{
    if ( building == 0L ||
         building->getElevations().size() == 0 ||
         building->getInstancedModelResource() == 0L )
    {
        return false;
    }

    // precalculate the frame transformation; combining these will
    // prevent any precision loss during the transform.
    osg::Matrix frame = building->getReferenceFrame() * world2local;
    
    // Load models:
    ModelResource* model = building->getInstancedModelResource();

    // Use the first elevation to compute the AABB.
    Elevation* elevation = building->getElevations().front().get();

    const osg::BoundingBox& space = elevation->getAxisAlignedBoundingBox();

    // dimensions of the AABB:       
    float spaceWidth  = space.xMax() - space.xMin();
    float spaceLength = space.yMax() - space.yMin();

    const osg::BoundingBox& bbox = model->getBoundingBox(0L);
    float modelWidth  = bbox.xMax() - bbox.xMin();
    float modelLength = bbox.yMax() - bbox.yMin();
    float modelHeight = bbox.zMax() - bbox.zMin();
    osg::Vec3f modelCenter = bbox.center();

    osg::Vec3d scale(
        model->canScaleToFitXY() == true ? (spaceWidth / modelWidth) : 1.0f,
        model->canScaleToFitXY() == true ? (spaceLength / modelLength) : 1.0f,
        model->canScaleToFitZ() == true ?  (elevation->getHeight()/modelHeight) : 1.0f
    );

    osg::Matrix matrix;
    
    osg::Vec3d spaceCenter = space.center();
    elevation->unrotate(spaceCenter);
    matrix.preMultTranslate( osg::Vec3d(spaceCenter.x(), spaceCenter.y(), 0.0) );

    // rotate the model:
    matrix.preMult( elevation->getRotation() );

    // scale the model to match the dimensions of the AABB.    
    matrix.preMultScale( scale );

    // translates model so it centers/clamps to (0,0,0).
    matrix.preMultTranslate( osg::Vec3d(-modelCenter.x(), -modelCenter.y(), 0.0) );
    
    output.addInstance( model, matrix * frame );
    
    return true;
}
