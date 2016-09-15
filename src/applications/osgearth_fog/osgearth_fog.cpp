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
#include <osgEarth/Lighting>
#include <osgEarth/PhongLightingEffect>

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

    osg::Group* rootShader = new osg::Group();
    root->addChild(rootShader);
   
    // Add a shader based cow to the right for comparison.
    osg::MatrixTransform* mt = new osg::MatrixTransform;    
    osg::Node* cowShader = osgDB::readNodeFile("cow.osg.10,0,0.trans");    
    osgEarth::Registry::instance()->shaderGenerator().run(cowShader);    

    // Add phong lighting.
    osgEarth::PhongLightingEffect* phong = new osgEarth::PhongLightingEffect();
    phong->attach(rootShader->getOrCreateStateSet());

    osg::Group  *_lights = new osg::Group();
    osg::LightSource *globalLight = new osg::LightSource();

    osg::Light* light = globalLight->getLight();

    light->setPosition(osg::Vec4(
      0.0, 0.0, 1.0, 0.0));

    light->setAmbient(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    light->setDiffuse(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));

    _lights->addChild( globalLight );

    osgEarth::GenerateGL3LightingUniforms visit;
    _lights->accept(visit);

    rootShader->addChild(_lights);
    
    // Attach the fog effect so fog will take effect.
    FogEffect* fogEffect = new FogEffect;
    fogEffect->attach( cowShader->getOrCreateStateSet() );
    mt->addChild( cowShader );
    rootShader->addChild(mt);


    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    
    viewer.setSceneData( root );

    float lightFactor = 1.0f;      
    float lightAngle = osg::PI/2.0;      
    while (!viewer.done())
    { 
        // Change light parameters ever 60 frames
        //if (viewer.getFrameStamp()->getFrameNumber() % 60 == 0)
        {
          lightFactor -= 0.05f;
          if (lightFactor <= 0.0f)
          {
            lightFactor = 1.0f;
          }
          //light->setDiffuse(osg::Vec4(lightFactor, lightFactor, lightFactor, 1.0f));

          lightAngle -= osg::PI/180;
          if (lightAngle <= -osg::PI)
          {
            lightAngle = osg::PI;
          }          
          light->setPosition(osg::Vec4(
             0.0, cos(lightAngle), sin(lightAngle), 0.0));

        }        

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
