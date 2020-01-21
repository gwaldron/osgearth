
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
#include "InstancedRoofCompiler"
#include <osgEarth/Session>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>

using namespace osgEarth;
using namespace osgEarth::Buildings;

#define LC "[InstancedRoofCompiler] "

bool
InstancedRoofCompiler::compile(CompilerOutput&       output,
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
    
    // Load models:
    ModelResource* model = roof->getModelResource();
    if ( model )
    {
        const osg::BoundingBox& space = elevation->getAxisAlignedBoundingBox();

        // dimensions of the AABB:       
        float spaceWidth  = space.xMax() - space.xMin();
        float spaceLength = space.yMax() - space.yMin();

        const osg::BoundingBox& bbox = model->getBoundingBox(0L);
        float modelWidth  = bbox.xMax() - bbox.xMin();
        float modelLength = bbox.yMax() - bbox.yMin();
        osg::Vec3f modelCenter = bbox.center();

        float xRatio = spaceWidth/modelWidth;
        float yRatio = spaceLength/modelLength;

        osg::Matrix matrix;

        osg::Vec3d spaceCenter = space.center();
        elevation->unrotate(spaceCenter);
        matrix.preMultTranslate( osg::Vec3d(spaceCenter.x(), spaceCenter.y(), 0.0) );

        // rotate the model:
        matrix.preMult( elevation->getRotation() );

        // scale the model to match the dimensions of the AABB.
        matrix.preMultScale( osg::Vec3d(xRatio, yRatio, 1.0) );

        // translates model so it's centers on 0,0,0.
        matrix.preMultTranslate( osg::Vec3d(-modelCenter.x(), -modelCenter.y(), elevation->getHeight() - bbox.zMin()) );
        
        // scale the height of the roof model so it's in line with the x and y scaling:
        float minRatio = std::min(1.0f, std::min(xRatio, yRatio) );
        matrix.preMultScale( osg::Vec3d(1.0, 1.0, minRatio) );

        output.addInstance( model, matrix * frame );
    }

    return true;
}
