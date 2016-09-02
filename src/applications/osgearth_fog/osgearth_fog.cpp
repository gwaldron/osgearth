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

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgEarth/Registry>
#include <osgDB/ReadFile>

#include <osgEarthUtil/Fog>
#include <osg/Fog>

using namespace osgEarth;
using namespace osgEarth::Util;

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();    

    // Setup a Fog state attribute    
    osg::Fog* fog = new osg::Fog;            
    fog->setColor( viewer.getCamera()->getClearColor() );                
    fog->setDensity( 0.02 );    
    fog->setMode(osg::Fog::LINEAR);
    fog->setStart(5.0);
    fog->setEnd(50.0);
    root->getOrCreateStateSet()->setAttributeAndModes( fog, osg::StateAttribute::ON );                
    
    // Attach the FogCallback to keep the Fog uniforms up to date.
    fog->setUpdateCallback(new FogCallback()); 

    // Add the regular cow.
    root->addChild(osgDB::readNodeFile("cow.osg"));

    // Add a shader based cow to the right for comparison.
    osg::MatrixTransform* mt = new osg::MatrixTransform;    
    osg::Node* cowShader = osgDB::readNodeFile("cow.osg.10,0,0.trans");    
    osgEarth::Registry::instance()->shaderGenerator().run(cowShader);    
    
    // Attach the fog effect so fog will take effect.
    FogEffect* fogEffect = new FogEffect;
    fogEffect->attach( cowShader->getOrCreateStateSet() );
    mt->addChild( cowShader );
    root->addChild(mt);

    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    
    viewer.setSceneData( root );

    while (!viewer.done())
    {        
        // Change fog modes ever 200 frames.
        if (viewer.getFrameStamp()->getFrameNumber() % 200 == 0)
        {
            if (fog->getMode() == osg::Fog::LINEAR)
            {
                fog->setMode(osg::Fog::EXP);
                OE_NOTICE << "switching to osg::Fog::EXP" << std::endl;
            }
            else if (fog->getMode() == osg::Fog::EXP)
            {
                fog->setMode(osg::Fog::EXP2);
                OE_NOTICE << "switching to osg::Fog::EXP2" << std::endl;
            }
            else
            {
                fog->setMode(osg::Fog::LINEAR);
                OE_NOTICE << "switching to osg::Fog::LINEAR" << std::endl;
            }
        }
        viewer.frame();
    }
    return 0;
}
