/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef OSGEARTH_GLUTILS_H
#define OSGEARTH_GLUTILS_H 1

#include <osgEarth/Common>
#include <osg/StateSet>
#include <osg/OperationThread>

namespace osgEarth
{
    struct OSGEARTH_EXPORT GLUtils
    {
        //! Sets any default uniforms required by the implementation
        static void setGlobalDefaults(osg::StateSet* stateSet);

        //! Configure lighting (GL_LIGHTING)
        static void setLighting(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov);
        
        //! Configure line width (GL_LINE_WIDTH)
        static void setLineWidth(osg::StateSet* stateSet, float value, osg::StateAttribute::OverrideValue ov);

        //! Configure line stippling (GL_LINE_STIPPLE)
        static void setLineStipple(osg::StateSet* stateSet, int factor, unsigned short pattern, osg::StateAttribute::OverrideValue ov);

        //! Configure line antialiasing (GL_LINE_SMOOTH)
        static void setLineSmooth(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov);

        //! Configure point rendering size (GL_POINT_SIZE)
        static void setPointSize(osg::StateSet* stateSet, float value, osg::StateAttribute::OverrideValue ov);

        //! Configure point rounding/antialiasing (GL_POINT_SMOOTH)
        static void setPointSmooth(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov);

        //! Removes the state associated with a GL capability, causing it to inherit from above.
        //! and if one of: GL_LIGHTING, GL_LINE_WIDTH, GL_LINE_STIPPLE, GL_LINE_SMOOTH, GL_POINT_SIZE
        static void remove(osg::StateSet* stateSet, GLenum cap);
    };

    struct OSGEARTH_EXPORT GL3RealizeOperation : public osg::Operation
    {
        void operator()(osg::Object*);
    };
}

#endif // OSGEARTH_GLUTILS_H
