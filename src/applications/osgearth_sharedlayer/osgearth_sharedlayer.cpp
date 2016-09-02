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

/**
 * This sample demonstrates the use of shared layers. A shared image layer
 * is one that becomes available to other image layers during the rendering
 * phase. Thus if you share a layer, you can access it in a customer shader
 * and use it to modulate another layer.
 *
 * In this example, we render a masking layer from a shapefile using the
 * rasterization (agglite) driver. We don't draw this layer directly; instead
 * we use it as a mask and use that mask to modulate the imagery layer from
 * a custom shader, drawing random colors where the mask exists.
 */
#include <osgViewer/Viewer>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/ImageLayer>
#include <osgEarth/ColorFilter>
#include <osgEarth/StringUtils>
#include <osgEarth/MapNode>
#include <osgEarthSymbology/StyleSheet>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/tms/TMSOptions>


using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;


// forward declarations.
int usage( const std::string& msg );
ImageLayer* createImageryLayer();
ImageLayer* createSharedLayer(const std::string& shapefile);



/**
 * A custom ColorFilter that will modulate the color of a normal
 * ImageLayer by using data from a second "shared" image layer.
 */
class MyColorFilter : public osgEarth::ColorFilter
{
public:
    // Create the color filter, supplying the shared layer that we plan
    // to access
    MyColorFilter(ImageLayer* maskLayer)
    {
        _maskLayer = maskLayer;
    }

    // This runs when the color filter is first installed.
    void install(osg::StateSet* stateSet) const
    {
        // When we added the shared layer, osgEarth reserves a special
        // image unit for it. Retrieve that now sine we need it in order
        // to use its sampler.
        int unit = _maskLayer->shareImageUnit().get();

        // Make a vertex shader that will access the texture coordinates
        // for our shared layer.
        std::string vs = Stringify()
            << "varying vec4 mask_layer_texc; \n"
            << "void my_filter_vertex(inout vec4 VertexMODEL) \n"
            << "{ \n"
            << "    mask_layer_texc = gl_MultiTexCoord" << unit << "; \n"
            <<"} \n";

        // Make the fragment shader that will modulate the visible layer.
        // This is is simple -- it just maxes out the red component wherever
        // the shared "mask" layer exceed a certain alpha value.
        std::string fs =
            "uniform sampler2D mask_layer_tex; \n"
            "varying vec4 mask_layer_texc; \n"
            "void my_color_filter(inout vec4 color) \n"
            "{ \n"
            "    vec4 mask_texel = texture2D(mask_layer_tex, mask_layer_texc.st); \n"
            "    if ( mask_texel.a >= 0.5 ) \n"
            "    { \n"
            "        color.r = 1.0; \n"
            "    } \n"
            "} \n";

        // getOrCreate ensures we don't overwrite an existing VP.
        VirtualProgram* vp = VirtualProgram::getOrCreate( stateSet );

        // install the vertex function
        vp->setFunction( "my_filter_vertex",   vs, ShaderComp::LOCATION_VERTEX_MODEL );

        // install the color filter entry point. This is just a regular shader function,
        // not a pipeline injection, so just use setShader.
        vp->setShader( "my_color_filter", new osg::Shader(osg::Shader::FRAGMENT, fs) );

        // finally, bind our sampler uniform to the allocated image unit.
        stateSet->getOrCreateUniform("mask_layer_tex", osg::Uniform::SAMPLER_2D)->set( unit );
    }

    // The ColorFilter pipeline needs to know which function to call.
    std::string getEntryPointFunctionName() const
    {
        return "my_color_filter";
    }

    ImageLayer* _maskLayer;
};



// main program.
int main(int argc, char** argv)
{
    // set up the viewer
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // make sure we have a shape file.
    std::string shapefile;
    if ( !arguments.read("--shapefile", shapefile) )
        return usage("Missing required --shapefile argument");

    // install a motion model
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // create a visible imagery layer:
    ImageLayer* imagery = createImageryLayer();

    // create a masking layer using the shapefile:
    ImageLayer* sharedLayer = createSharedLayer( shapefile );

    // create a new map and add our two layers.
    MapNode* mapnode = new MapNode();
    mapnode->getMap()->addImageLayer( imagery );
    mapnode->getMap()->addImageLayer( sharedLayer );

    // make a custom color-filter shader that will modulate the imagery
    // using the texture from the shared layer. (Using a ColorFilter 
    // will apply the effect to just one layer; if you want to apply it
    // to all layers, you can just create a VirtualProgram and apply that
    // the the mapnode->getTerrainEngine() state set.)
    ColorFilter* filter = new MyColorFilter( sharedLayer );
    imagery->addColorFilter( filter );

    // done!
    viewer.setSceneData( mapnode );
    MapNodeHelper().configureView(&viewer);
    return viewer.run();
}


// Creates a visible imagery layer that we will modulate.
ImageLayer* createImageryLayer()
{
    TMSOptions tms;
    tms.url() = "http://readymap.org/readymap/tiles/1.0.0/7/";
    return new ImageLayer( "imagery", tms );
}


// Creates the shared "masking" layer based on a shapefile.
ImageLayer* createSharedLayer(const std::string& url)
{
    // configure the OGR driver to read the shapefile
    OGRFeatureOptions ogr;
    ogr.url() = url;

    // configure a rasterization driver to render the shapes to an
    // image layer.
    AGGLiteOptions agg;
    agg.featureOptions() = ogr;
    agg.styles() = new StyleSheet();

    // Render shapes as opaque white.
    Style style;
    style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::White;
    agg.styles()->addStyle( style );

    // configure the image layer. We set it to "shared" to tell osgEarth to
    // generate a special image unit, making this layer's textures available
    // to all other layers. We also set "visible" to false sine we don't
    // want to actually render this layer in this example.
    ImageLayerOptions layer( "mask", agg );
    layer.shared()  = true;
    layer.visible() = false;
    layer.cachePolicy() = CachePolicy::NO_CACHE;

    return new ImageLayer(layer);
}


// help/error information.
int usage( const std::string& msg )
{    
    OE_NOTICE
        << msg << "\n\n"
        << "USAGE: osgearth_sharedlayer \n"
        "             --shapefile <shapefile> \n"
        << std::endl;

    return -1;
}
