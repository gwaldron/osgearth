/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2010 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#ifndef OSGGA_MULTITOUCH_TRACKBALL_MANIPULATOR2
#define OSGGA_MULTITOUCH_TRACKBALL_MANIPULATOR2 1

#include <osgGA/TrackballManipulator>


namespace osgGA {


class OSGGA_EXPORT MultiTouchTrackballManipulator2 : public TrackballManipulator
{
        typedef TrackballManipulator inherited;

    public:

        MultiTouchTrackballManipulator2( int flags = DEFAULT_SETTINGS );
        MultiTouchTrackballManipulator2( const MultiTouchTrackballManipulator2& tm,
                              const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY );

        META_Object( osgGA, MultiTouchTrackballManipulator2 );

        bool handle( const GUIEventAdapter& ea, GUIActionAdapter& us );

    protected:

        void handleMultiTouchDrag(GUIEventAdapter::TouchData* now, GUIEventAdapter::TouchData* last, const double eventTimeDelta);

        osg::ref_ptr<GUIEventAdapter::TouchData> _lastTouchData;
};


}

#endif /* OSGGA_MULTITOUCH_TRACKBALL_MANIPULATOR2 */
