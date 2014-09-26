/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/TraversalData>

using namespace osgEarth;

MapNodeCullData::MapNodeCullData()
{
    _stateSet = new osg::StateSet();

    _windowMatrixUniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "oe_WindowMatrix");
    _windowMatrixUniform->set( osg::Matrix::identity() );
    _stateSet->addUniform( _windowMatrixUniform.get() );

    _cameraAltitude = 0.0;
}
