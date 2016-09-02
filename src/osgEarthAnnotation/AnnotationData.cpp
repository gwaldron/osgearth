/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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

#include <osgEarthAnnotation/AnnotationData>

using namespace osgEarth::Annotation;

AnnotationData::AnnotationData() :
_viewpoint( 0L ),
_priority ( 0.0f )
{
    //nop
}

AnnotationData::AnnotationData(const Config& conf) :
_viewpoint( 0L ),
_priority ( 0.0f )
{
    mergeConfig(conf);
}

AnnotationData::~AnnotationData()
{
    if ( _viewpoint )
        delete _viewpoint;
}

void
AnnotationData::mergeConfig(const Config& conf)
{
    _name        = conf.value("name");
    _description = conf.value("description");
    _priority    = conf.value<float>("priority", _priority);

    if ( conf.hasValue("viewpoint") )
    {
        _viewpoint = new Viewpoint( conf.value("viewpoint") );
    }
}

Config
AnnotationData::getConfig() const
{
    Config conf("annotation_data");

    if ( !_name.empty() )
        conf.add("name", _name);
    if ( !_description.empty() )
        conf.add("description", _description);
    if ( _priority != 0.0f )
        conf.add("priority", _priority );
    if ( _viewpoint )
        conf.add( "viewpoint", _viewpoint->getConfig() );

    return conf;
}

