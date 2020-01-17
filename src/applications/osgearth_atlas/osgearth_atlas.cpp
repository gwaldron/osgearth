/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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

#include <osgEarth/Notify>
#include <osgEarth/XmlUtils>
#include <osgEarth/ImageUtils>
#include <osgEarthUtil/AtlasBuilder>
#include <osgEarthSymbology/ResourceLibrary>
#include <osgEarthSymbology/Skins>
#include <osgEarth/Utils>
#include <osgEarth/Lighting>

#include <osg/ArgumentParser>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2DArray>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgText/Text>

#include <string>
#include <set>

#define LC "[atlas] "

using namespace osgEarth;

int
usage(const char* msg, const char* name =0L)
{
    if ( msg )
    {
        OE_NOTICE << msg << std::endl;
    }

    if ( name )
    {
        OE_NOTICE
            << "\n"
            << name << " will compile an osgEarth resource catalog into a texture atlas."
            << "\n"
            << "\nUsage: " << name
            << "\n"
            << "\n      --build catalog.xml               : Build an atlas from the catalog"
            << "\n        --size <x> <y>                  : Maximum size of atlas textures"
            << "\n        --out-image <path>              : Output path for the atlas image (defaults to an OSGB file in"
            << "\n                                          the working directory). The paths in the resulting catalog"
            << "\n                                          file will point to this location using a relative path if possible."
            << "\n        --aux <pattern> <r> <g> <b> <a> : Build an auxiliary atlas for files matching the pattern"
            << "\n                                          \"filename_pattern.ext\", e.g., \"texture.jpg\" will match"
            << "\n                                          \"texture_NML.jpg\" for pattern = \"NML\". The RGBA are the"
            << "\n                                          default values to use when no match is found."
            << "\n"
            << "\n      --show  catalog.xml               : Display an atlas built with this tool"
            << "\n        --layer <num>                   : Show layer <num> of the atlas (default = 0)"
            << "\n        --labels                        : Label each atlas entry"
            << "\n        --aux <pattern>                 : Show atlas matching this auxiliary file pattern"
            << std::endl;
    }

    return 0;
}

//----------------------------------------------------------------------

int
build(osg::ArgumentParser& arguments)
{
    // the input resource catalog XML file:
    std::string inCatalogPath;
    if ( !arguments.read("--build", inCatalogPath) )
        return usage("Missing required argument catalog file");

    // the output texture atlas image path:
    std::string inCatalogFile = osgDB::getSimpleFileName(inCatalogPath);

    // check that the input file exists:
    if ( !osgDB::fileExists(inCatalogPath) )
        return usage("Input file not found");

    // open the resource library.
    osg::ref_ptr<osgEarth::Symbology::ResourceLibrary> lib =
        new osgEarth::Symbology::ResourceLibrary("unnamed", inCatalogPath);

    if ( !lib->initialize(0L) )
        return usage("Error loading input catalog file");
    
    // build the atlas.
    osgEarth::Util::AtlasBuilder        builder;
    osgEarth::Util::AtlasBuilder::Atlas atlas;

    // max x/y dimensions:
    unsigned sizex, sizey;
    if ( arguments.read("--size", sizex, sizey) )
        builder.setSize( sizex, sizey );

    // output location for image:
    std::string outImageFile;
    if ( !arguments.read("--out-image", outImageFile) )
        outImageFile  = osgDB::getNameLessExtension(inCatalogFile) + "_atlas.osgb";
        
    // the output catalog file describing the texture atlas contents:
    std::string outCatalogFile = osgDB::getSimpleFileName(outImageFile) + ".xml";

    // Whether to build RGB images
    bool rgb = arguments.read("--rgb");
    builder.setRGB( rgb );
    

    // auxiliary atlas patterns:
    std::string pattern;
    float r, g, b, a;
    while(arguments.read("--aux", pattern, r, g, b, a))
        builder.addAuxFilePattern(pattern, osg::Vec4f(r,g,b,a));

    if ( !builder.build(lib.get(), outImageFile, atlas) )
        return usage("Failed to build atlas");

    // write the atlas images. don't flip DDS's.
    osg::ref_ptr<osgDB::Options> writeOptions = new osgDB::Options();
    writeOptions->setOptionString("ddsNoAutoFlipWrite");

    osgDB::writeImageFile(*atlas._images.begin()->get(), outImageFile, writeOptions.get());
    OE_INFO << LC << "Wrote output image to \"" << outImageFile << "\"" << std::endl;
    
    // write any aux images.
    const std::vector<std::string>& auxPatterns = builder.auxFilePatterns();
    for(unsigned i=0; i<auxPatterns.size(); ++i)
    {
        std::string auxAtlasFile =
            osgDB::getNameLessExtension(outImageFile) +
            "_" + auxPatterns[i] + "." +
            osgDB::getFileExtension(outImageFile);

        osgDB::writeImageFile(*atlas._images[i + 1].get(), auxAtlasFile, writeOptions.get());
        
        OE_INFO << LC << "Wrote auxiliary image to \"" << auxAtlasFile << "\"" << std::endl;
    }

    // write the new catalog:
    osgEarth::XmlDocument catXML( atlas._lib->getConfig() );

    std::ofstream catOut(outCatalogFile.c_str());
    if ( !catOut.is_open() )
        return usage("Failed to open output catalog file for writing");

    catXML.store(catOut);
    catOut.close();

    OE_INFO << LC << "Wrote output catalog to \"" << outCatalogFile<< "\"" << std::endl;
    return 0;
}

//----------------------------------------------------------------------

int
show(osg::ArgumentParser& arguments)
{
    // find the resource library file:
    std::string inCatalogFile;
    if ( !arguments.read("--show", inCatalogFile) )
        return usage("Missing required catalog file name");

    int layer = 0;
    arguments.read("--layer", layer);

    bool drawLabels;
    drawLabels = arguments.read("--labels");

    std::string auxPattern;
    arguments.read("--aux", auxPattern);

    // open the resource library:
    osg::ref_ptr<osgEarth::Symbology::ResourceLibrary> lib =
        new osgEarth::Symbology::ResourceLibrary("temp", osgEarth::URI(inCatalogFile) );
    if ( lib->initialize(0L) == false )
        return usage("Failed to load resource catalog");

    // the atlas name is the library name without the extension. Not strictly true
    // but true if you didn't rename it :)
    std::string atlasFile = osgDB::getNameLessExtension(inCatalogFile);

    // check for an auxiliary pattern:
    if ( !auxPattern.empty() )
    {
        atlasFile =
            osgDB::getNameLessExtension(atlasFile) +
            "_" + auxPattern + "." +
            osgDB::getFileExtension(atlasFile);
    }

    osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(atlasFile);
    if (!image.valid())
        return usage("Failed to load atlas image");

    if ( layer > image->r()-1 )
        return usage("Specified layer does not exist");

    // geometry for the image layer:
    std::vector<osg::ref_ptr<osg::Image> > images;
    osgEarth::ImageUtils::flattenImage(image.get(), images);
    osg::Geode* geode = osg::createGeodeForImage(images[layer].get());

    const osg::BoundingBox& bbox = geode->getDrawable(0)->getBoundingBox();
    float width = bbox.xMax() - bbox.xMin();
    float height = bbox.zMax() - bbox.zMin();

    // geometry for the skins in that layer:
    osg::Geode* geode2 = new osg::Geode();
    osg::Geometry* geom = new osg::Geometry();
    geode2->addDrawable(geom);
    osg::Vec3Array* v = new osg::Vec3Array();
    geom->setVertexArray( v );
    osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
    (*c)[0].set(1,1,0,1);
    geom->setColorArray(c);
    osgEarth::Symbology::SkinResourceVector skins;
    lib->getSkins(skins);
    OE_WARN << "num = " << skins.size() << "\n";

    for(unsigned k=0; k<skins.size(); ++k)
    {
        if (skins[k]->imageLayer() == layer && skins[k]->atlasHint() != false)
        {
            float x = bbox.xMin() + width*skins[k]->imageBiasS().value();
            float y = bbox.zMin() + height*skins[k]->imageBiasT().value();
            float s = width*skins[k]->imageScaleS().value();
            float t = height*skins[k]->imageScaleT().value();

            v->push_back(osg::Vec3(x,     -0.01f, y    ));
            v->push_back(osg::Vec3(x + s, -0.01f, y    ));
            v->push_back(osg::Vec3(x + s, -0.01f, y + t));
            v->push_back(osg::Vec3(x,     -0.01f, y + t));

            geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, v->size()-4, 4));

            if (drawLabels)
            {
                osgText::Text* label = new osgText::Text();
                label->setText(skins[k]->name());
                label->setPosition(osg::Vec3(x+0.5*s, -0.005f, y+0.5*t));
                label->setAlignment(label->CENTER_CENTER);
                label->setAutoRotateToScreen(true);
                label->setCharacterSizeMode(label->SCREEN_COORDS);
                label->setCharacterSize(20.0f);
                label->setFont("arialbd.ttf");
                geode2->addDrawable(label);
            }
        }
    }

    osg::Group* root = new osg::Group();
    root->addChild( geode );
    root->addChild( geode2 );

    Lighting::set(root->getOrCreateStateSet(), 0);
    root->getOrCreateStateSet()->setMode(GL_CULL_FACE, 0);

    osgViewer::Viewer viewer;
    viewer.setSceneData( root );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.run();
    return 0;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // print usage info.
    if ( arguments.read("--help") )
        return usage(0L, argv[0]);

    if (arguments.find("--build") >= 0)
        return build(arguments);

    if (arguments.find("--show") >= 0)
        return show(arguments);

    return usage(0L, argv[0]);
}
