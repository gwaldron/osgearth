/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include <osg/Notify>
#include <osg/io_utils>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarth/Utils>

#include <osg/Version>
#include <osgEarth/Version>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static Grid* s_layerBox = NULL;

osg::Node*
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = ControlCanvas::get( view );

    // the outer container:
    s_layerBox = new Grid();
    s_layerBox->setBackColor(0,0,0,0.5);
    s_layerBox->setMargin( 10 );
    s_layerBox->setPadding( 10 );
    s_layerBox->setChildSpacing( 10 );
    s_layerBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setAbsorbEvents( true );
    s_layerBox->setVertAlign( Control::ALIGN_BOTTOM );

    canvas->addControl( s_layerBox );    
    return canvas;
}

struct AdjustHandler: public ControlEventHandler
{
    AdjustHandler( ImageLayer* layer, unsigned int index) :
        _layer(layer),
        _index(index)
        { }

    void onValueChanged( Control* control, float value ) {
        float adj = value - 50.0f;
        adj /= 50.0;

		if (_index < 3) {
            osg::Vec3f hsl = _layer->getHSLAdjust();        

            //Get the current value
            hsl._v[ _index] = adj;
            OE_NOTICE << "Setting HSL adjustment to " << hsl << std::endl;
            _layer->setHSLAdjust( hsl );
		} else {
            osg::Vec3f bcg = _layer->getBCGAdjust();

            //Get the current value
            bcg._v[ _index - 3 ] = adj;
            OE_NOTICE << "Setting BCG adjustment to " << bcg << std::endl;
            _layer->setBCGAdjust( bcg );
		}
    }

    unsigned int _index;
    ImageLayer* _layer;
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
        
    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_NOTICE << "Unable to load earth model." << std::endl;
        return 1;
    }

    osgViewer::Viewer viewer(arguments);
    
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    //Create the control panel
    root->addChild( createControlPanel(&viewer) );

    viewer.setSceneData( root );
    
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {   
        if (mapNode->getMap()->getNumImageLayers() == 0)
        {
            OE_NOTICE << "Please provide a map with at least one image layer" << std::endl;
            return 1;
        }
        //
        ImageLayer* layer = mapNode->getMap()->getImageLayerAt( mapNode->getMap()->getNumImageLayers()-1);

        //H
        LabelControl* hLabel = new LabelControl( "H" );      
        hLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 0, hLabel );

        HSliderControl* hAdjust = new HSliderControl( 0.0f, 100.0f, 50.0f );
        hAdjust->setWidth( 125 );
        hAdjust->setHeight( 12 );
        hAdjust->setVertAlign( Control::ALIGN_CENTER );
        hAdjust->addEventHandler( new AdjustHandler( layer, 0 ));
        s_layerBox->setControl( 1, 0, hAdjust );

        //S
        LabelControl* sLabel = new LabelControl( "S" );      
        sLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, sLabel );
        
        HSliderControl* sAdjust = new HSliderControl( 0.0f, 100.0f, 50.0f );
        sAdjust->setWidth( 125 );
        sAdjust->setHeight( 12 );
        sAdjust->setVertAlign( Control::ALIGN_CENTER );
        sAdjust->addEventHandler( new AdjustHandler( layer, 1) );
        s_layerBox->setControl( 1, 1, sAdjust );

        //L
        LabelControl* lLabel = new LabelControl( "L" );      
        lLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 2, lLabel );
        
        HSliderControl* lAdjust = new HSliderControl( 0.0f, 100.0f, 50.0f );
        lAdjust->setWidth( 125 );
        lAdjust->setHeight( 12 );
        lAdjust->setVertAlign( Control::ALIGN_CENTER );
        lAdjust->addEventHandler( new AdjustHandler( layer, 2) );
        s_layerBox->setControl( 1, 2, lAdjust );

        //B
        LabelControl* bLabel = new LabelControl( "B" );
        bLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 3, bLabel );

        HSliderControl* bAdjust = new HSliderControl( 0.0f, 100.0f, 50.0f );
        bAdjust->setWidth( 125 );
        bAdjust->setHeight( 12 );
        bAdjust->setVertAlign( Control::ALIGN_CENTER );
        bAdjust->addEventHandler( new AdjustHandler( layer, 3 ));
        s_layerBox->setControl( 1, 3, bAdjust );

        //C
        LabelControl* cLabel = new LabelControl( "C" );
        cLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 4, cLabel );

        HSliderControl* cAdjust = new HSliderControl( 0.0f, 100.0f, 50.0f );
        cAdjust->setWidth( 125 );
        cAdjust->setHeight( 12 );
        cAdjust->setVertAlign( Control::ALIGN_CENTER );
        cAdjust->addEventHandler( new AdjustHandler( layer, 4) );
        s_layerBox->setControl( 1, 4, cAdjust );

        //G
        LabelControl* gLabel = new LabelControl( "G" );
        gLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 5, gLabel );

        HSliderControl* gAdjust = new HSliderControl( 0.0f, 100.0f, 50.0f );
        gAdjust->setWidth( 125 );
        gAdjust->setHeight( 12 );
        gAdjust->setVertAlign( Control::ALIGN_CENTER );
        gAdjust->addEventHandler( new AdjustHandler( layer, 5) );
        s_layerBox->setControl( 1, 5, gAdjust );

    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());    
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
