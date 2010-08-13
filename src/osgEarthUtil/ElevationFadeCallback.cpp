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

#include <osgEarthUtil/ElevationFadeCallback>
#include <OpenThreads/ScopedLock>

using namespace osg;
using namespace osgEarth;
using namespace osgEarthUtil;
using namespace OpenThreads;

ElevationFadeCallback::ElevationFadeCallback():
_firstFrame(true),
_previousTime(0.0),
_animationTime(2.0f),
_currentElevation(0)
{
    //nop
}

void
ElevationFadeCallback::setElevationRange(MapLayer* layer,
                                         float maxElevation, float minElevation,
                                         float maxBuffer,    float minBuffer)
{
    _ranges.erase( layer );
    ElevationRange range;
    range.min = minElevation;
    range.max = maxElevation;
    range.minBuffer = osg::clampAbove( minBuffer, 0.0f );
    range.maxBuffer = maxBuffer >= 0.0f ? maxBuffer : 0.1 * (maxElevation-minElevation);
    _ranges[layer] = range;
}

void
ElevationFadeCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if ( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        //Get the current elevation
        _currentElevation = nv->getViewPoint().z();
        if (!_csn.valid())
        {
            _csn = findTopMostNodeOfType<osg::CoordinateSystemNode>(node);
        }
        if (_csn.valid())
        {
            osg::EllipsoidModel* em = _csn->getEllipsoidModel();
            if (em)
            {
                double x = nv->getViewPoint().x();
                double y = nv->getViewPoint().y();
                double z = nv->getViewPoint().z();
                double latitude, longitude;
                em->convertXYZToLatLongHeight(x, y, z, latitude, longitude, _currentElevation);
            }
        }
    }

    else if ( nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        if ( !_firstFrame )
        {
            double deltaTime = nv->getFrameStamp()->getReferenceTime() - _previousTime;
            double delta = osg::minimum(deltaTime / _animationTime, 1.0);

            for( LayerElevationRanges::iterator i = _ranges.begin(); i != _ranges.end(); ++i )
            {
                osg::ref_ptr<MapLayer> layerSafe = i->first.get();
                if ( layerSafe.valid() )
                {
                    float opacity = 1.0f;
                    const ElevationRange& range = i->second;
                    if ( _currentElevation < range.min-range.minBuffer || _currentElevation > range.max+range.maxBuffer )
                    {
                        opacity = 0.0f;
                    }
                    else
                    {
                        if ( _currentElevation > range.max && range.maxBuffer > 0.0f )
                        {
                            opacity = 1.0f - (_currentElevation-range.max)/range.maxBuffer;
                        }
                        else if ( _currentElevation < range.min && range.minBuffer > 0.0f )
                        {
                            opacity = (_currentElevation-range.min-range.minBuffer)/range.minBuffer;
                        }
                        else
                        {
                            opacity = 1.0f;
                        }
                    }

                    if ( opacity != layerSafe->opacity().value() )
                        layerSafe->setOpacity( opacity );
                }
            }
        }
        _firstFrame = false;
        _previousTime = nv->getFrameStamp()->getReferenceTime();
    }

    traverse( node, nv );
}

