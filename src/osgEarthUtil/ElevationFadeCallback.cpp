/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
}

void ElevationFadeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	MapNode* mapNode = static_cast<MapNode*>(node);
	if (mapNode)
	{
		if (nv->getVisitorType() == NodeVisitor::UPDATE_VISITOR)
		{
			if (!_firstFrame)
			{
				double deltaTime = nv->getFrameStamp()->getReferenceTime() - _previousTime;

				_previousTime = nv->getFrameStamp()->getReferenceTime();

				double delta = osg::minimum(deltaTime / _animationTime, 1.0);

                unsigned int numImageSources = 0;
                {
                    ScopedReadLock lock( mapNode->getMap()->getMapDataMutex() );
                    numImageSources = mapNode->getMap()->getImageMapLayers().size();
                }

				//Determine which layer should be active
				unsigned int activeLayer = numImageSources-1;
				for (unsigned int i = 0; i < numImageSources; ++i)
				{
					if (_currentElevation > getElevation(i))
					{
						activeLayer = i;
						break;
					}
				}

				bool dirtyLayers = false;
				for (unsigned int i = 0; i < numImageSources; ++i)
				{
					//If the layer that we are looking at is greater than the active layer, we want to fade it out to 0.0
					//Otherwise, we want the layers to go to 1.0
					float goalOpacity = (i > activeLayer) ? 0.0f : 1.0f;
					float currentOpacity = mapNode->getMap()->getImageMapLayers()[i]->opacity().value();

					if (goalOpacity != currentOpacity)
					{
						float opacityDelta = delta;
						if (currentOpacity > goalOpacity) opacityDelta = -opacityDelta;
						float newOpacity = currentOpacity + opacityDelta;
						mapNode->getMap()->getImageMapLayers()[i]->opacity() = newOpacity; //setOpacity(newOpacity);
					}
				}
			}
			_firstFrame = false;
		}
		else if (nv->getVisitorType() == NodeVisitor::CULL_VISITOR)
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
	}
	//Continue traversal
	traverse(node, nv);
}

double ElevationFadeCallback::getElevation(unsigned int i) const
{
	return (i < _elevations.size()) ? _elevations[i] : 0.0;
}

void ElevationFadeCallback::setElevation(unsigned int i, double elevation)
{
	if (i >= _elevations.size()) _elevations.resize( i + 1);
	_elevations[i] = elevation;
}

float ElevationFadeCallback::getAnimationTime() const
{
	return _animationTime;
}

void ElevationFadeCallback::setAnimationTime(float animationTime)
{
	_animationTime = animationTime;
}