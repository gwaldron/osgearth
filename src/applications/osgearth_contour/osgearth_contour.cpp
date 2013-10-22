/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

/**
 * This sample shows how to use osgEarth's built-in elevation data attributes
 * to apply contour-coloring to the terrain.
 */
#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osg/TransferFunction>
#include <osg/Texture1D>

using namespace osgEarth;
using namespace osgEarth::Util;

//-------------------------------------------------------------------------

// In the vertex shader, we use a vertex attribute that's genreated by the
// terrain engine. The attribute contains a vec4 which holds the unit
// extrusion vector in indexes[0,1,2] and the raw height in index[3].
// We just read the height, remap it to [0..1] and send it to the 
// fragment shader.

const char* vertexShader =
    "attribute vec4  osgearth_elevData; \n"
    "uniform float contour_xferMin; \n"
    "uniform float contour_xferRange; \n"
    "uniform float contour_xferMax; \n"
    "varying float contour_lookup; \n"

    "void setupContour(inout vec4 VertexModel) \n"
    "{ \n"
    "    float height = osgearth_elevData[3]; \n"
    "    float height_normalized = (height-contour_xferMin)/contour_xferRange; \n"
    "    contour_lookup = clamp( height_normalized, 0.0, 1.0 ); \n"
    "} \n";


// The fragment shader simply takes the texture index that we generated
// in the vertex shader and does a texture lookup. In this case we're
// just wholesale replacing the color, so if the map had any existing
// imagery, this will overwrite it.

const char* fragmentShader =
    "uniform sampler1D contour_colorMap; \n"
    "varying float contour_lookup; \n"

    "void colorContour( inout vec4 color ) \n"
    "{ \n"
    "    color = texture1D( contour_colorMap, contour_lookup ); \n"
    "} \n";



// Build the stateset necessary for drawing contours.
osg::StateSet* createStateSet( osg::TransferFunction1D* xfer, int unit )
{
    osg::StateSet* stateSet = new osg::StateSet();

    // Create a 1D texture from the transfer function's image.
    osg::Texture* tex = new osg::Texture1D( xfer->getImage() );
    tex->setResizeNonPowerOfTwoHint( false );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    stateSet->setTextureAttributeAndModes( unit, tex, osg::StateAttribute::ON );

    // Tell the shader program where to find it.
    stateSet->getOrCreateUniform( "contour_colorMap", osg::Uniform::SAMPLER_1D )->set( unit );

    // Install the shaders. We also bind osgEarth's elevation data attribute, which the 
    // terrain engine automatically generates at the specified location.
    // (By the way: if you want to draw image layers on top of the contoured terrain,
    // set the "priority" parameter to setFunction() to a negative number so that it draws
    // before the terrain's layers.)
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    vp->setFunction( "setupContour", vertexShader,   ShaderComp::LOCATION_VERTEX_MODEL);
    vp->setFunction( "colorContour", fragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING, -1.0 );
    vp->addBindAttribLocation( "osgearth_elevData", osg::Drawable::ATTRIBUTE_6 );
    //stateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

    // Install some uniforms that tell the shader the height range of the color map.
    stateSet->getOrCreateUniform( "contour_xferMin",   osg::Uniform::FLOAT )->set( xfer->getMinimum() );
    stateSet->getOrCreateUniform( "contour_xferRange", osg::Uniform::FLOAT )->set( xfer->getMaximum() - xfer->getMinimum() );

    return stateSet;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        if ( !mapNode )
            return -1;

        if ( mapNode->getMap()->getNumElevationLayers() == 0 )
            OE_WARN << "No elevation layers! The contour will be very boring." << std::endl;

        // Set up a transfer function for the elevation contours.
        osg::ref_ptr<osg::TransferFunction1D> xfer = new osg::TransferFunction1D();
        float s = 3000.0f;
        xfer->setColor( -1.0000 * s, osg::Vec4f(0, 0, 0.5, 1), false);
        xfer->setColor( -0.2500 * s, osg::Vec4f(0, 0, 1, 1), false);
        xfer->setColor(  0.0000 * s, osg::Vec4f(0, .5, 1, 1), false);
        xfer->setColor(  0.0625 * s, osg::Vec4f(.94,.94,.25,1), false);
        xfer->setColor(  0.1250 * s, osg::Vec4f(.125,.62,0,1), false);
        xfer->setColor(  0.3750 * s, osg::Vec4f(.87,.87,0,1), false);
        xfer->setColor(  0.7500 * s, osg::Vec4f(.5,.5,.5,1), false);
        xfer->setColor(  1.0000 * s, osg::Vec4f(1,1,1,1), false);
        xfer->updateImage();

        // request an available texture unit:
        int unit;
        mapNode->getTerrainEngine()->getTextureCompositor()->reserveTextureImageUnit(unit);

        // install the contour shaders on the terrain engine because we don't want
        // them affecting any model layers.
        mapNode->getTerrainEngine()->setStateSet( createStateSet(xfer.get(), unit) );
        
        viewer.setSceneData( node );
        viewer.run();
    }
    else
    {
        OE_NOTICE 
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }

    return 0;
}
