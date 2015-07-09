/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarth/ElevationField>
#include <osg/Vec3>

using namespace osgEarth;

ElevationField::ElevationField() :
_width ( 0 ),
_height( 0 ),
_dirty ( false )
{
    //nop
}

void
ElevationField::setSize(unsigned width, unsigned height)
{
    _width  = width;
    _height = height;

    // The heights vector has a one-sample border all the way around.
    _heights.resize( (width+2)*(height+2) );
    _heights.assign( (width+2)*(height+2), 0.0f );

    // Initialize the "border" samples to "NO DATA".
    for(unsigned s=0; s<(_width+2); ++s)
    {
        _heights[s] = NO_DATA_VALUE;
        _heights[_width*(_height+1) + s] = NO_DATA_VALUE;
    }
    for(unsigned t=1; t<(_height+1); ++t)
    {
        _heights[_width*t] = NO_DATA_VALUE;
        _heights[(_width*t)+(_width-1)] = NO_DATA_VALUE;
    }

    _slopes.resize( width*height );
    _slopes.assign( width*height, NO_DATA_VALUE );

    _curvatures.resize( width*height );
    _curvatures.assign( width*height, NO_DATA_VALUE );

    _dirty = true;
}

float
ElevationField::getHeight(unsigned x, unsigned y) const
{
    return _heights[(_width+2)*y + (x+1)];
}

void
ElevationField::update(float spacing)
{
    _spacing = spacing;
    _dirty = false;
    computeSlopeAndCurvature();
}


void
ElevationField::computeSlopeAndCurvature()
{
    unsigned w = _width+2, h = _height+2;
    
    float L = _spacing;
    osg::Vec3 up(0, 0, 1);

    _slopes.clear();
    _curvatures.clear();

    for(unsigned t=1; t<_height-1; ++t)
    {
        for(unsigned s=1; s<_width-1; ++s)
        {
            float centerZ = _heights[t*w + s];

            osg::Vec3
                west (-L,  0, 0),
                east ( L,  0, 0),
                north( 0, -L, 0),
                south( 0,  L, 0);

            west.z() = _heights[t*w + s-1];
            east.z() = _heights[t*w + s+1];

            if ( west.z() == NO_DATA_VALUE )
                west.z() = east.z() + 2.0*(centerZ-east.z());

            if ( east.z() == NO_DATA_VALUE )
                east.z() = west.z() + 2.0*(centerZ-west.z());
            
            south.z() = _heights[(t+1)*w + s];
            north.z() = _heights[(t-1)*w + s];

            if ( south.z() == NO_DATA_VALUE )
                south.z() = north.z() + 2.0*(centerZ-north.z());

            if ( north.z() == NO_DATA_VALUE )
                north.z() = south.z() + 2.0*(centerZ-south.z());

            // Normal vector: cross of two crossing vectors.
            osg::Vec3 normal = (east-west) ^ (north-south);
            normal.normalize();
            _normals.push_back(normal);
            
            // Slope [0..1]; 0=flat, 1=vertical. Dot product of the UP vector and the 
            // normal vector at a point.
            float slope = up * normal;
            _slopes.push_back(slope);

            // Curvature: http://goo.gl/mOcl93
            float L2 = L*L;
            float D = (0.5*(west.z() + east.z()) - centerZ) / L2;
            float E = (0.5*(south.z() + north.z()) - centerZ) / L2;

            // Curvature = [0...)?
            float curvature = -2.0*(D+E); //*100.0;
            _curvatures.push_back(curvature);
        }
    }
}
