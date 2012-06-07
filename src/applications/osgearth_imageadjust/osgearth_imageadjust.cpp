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

#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/HSLColorFilter>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static HSliderControl* s_hAdjust;
static HSliderControl* s_sAdjust;
static HSliderControl* s_lAdjust;



struct SetHSL: public ControlEventHandler
{
    SetHSL(HSLColorFilter* filter, unsigned index) :
        _filter(filter), _index(index)
        { }

    void onValueChanged( Control* control, float value )
    {
        osg::Vec3f hsl = _filter->getHSLOffset();
        hsl[_index] = value;
        _filter->setHSLOffset( hsl );
    }

    HSLColorFilter* _filter;
    unsigned        _index;
};



struct ResetHSL : public ControlEventHandler
{
    ResetHSL(HSLColorFilter* filter, HSliderControl* h, HSliderControl* s, HSliderControl* l) 
        : _filter(filter), _h(h), _s(s), _l(l) { }

    void onClick( Control* control )
    {
        _h->setValue( 0.0, false );
        _s->setValue( 0.0, false );
        _l->setValue( 0.0, false );
        _filter->setHSLOffset( osg::Vec3(0,0,0) );
    }

    HSLColorFilter* _filter;
    HSliderControl* _h;
    HSliderControl* _s;
    HSliderControl* _l;
};


Container*
createControlPanel(osgViewer::View* view)
{
    ControlCanvas* canvas = ControlCanvas::get( view, true );
    VBox* vbox = canvas->addControl(new VBox());
    vbox->setChildSpacing(10);
    return vbox;
}


void
addHSLControls(HSLColorFilter* filter, Container* container, unsigned i)
{
    // the outer container:
    Grid* s_layerBox = container->addControl(new Grid());
    s_layerBox->setBackColor(0,0,0,0.5);
    s_layerBox->setMargin( 10 );
    s_layerBox->setPadding( 10 );
    s_layerBox->setChildSpacing( 10 );
    s_layerBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setAbsorbEvents( true );
    s_layerBox->setVertAlign( Control::ALIGN_TOP );

    // Title:
    s_layerBox->setControl( 0, 0, new LabelControl(Stringify()<<"Layer "<<i, Color::Yellow) );
    
    // Hue:
    LabelControl* hLabel = new LabelControl( "Hue" );      
    hLabel->setVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setControl( 0, 1, hLabel );

    HSliderControl* hAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f );
    hAdjust->setWidth( 125 );
    hAdjust->setHeight( 12 );
    hAdjust->setVertAlign( Control::ALIGN_CENTER );
    hAdjust->addEventHandler( new SetHSL(filter, 0) );
    s_layerBox->setControl( 1, 1, hAdjust );

    // Saturation:
    LabelControl* sLabel = new LabelControl( "Saturation" );      
    sLabel->setVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setControl( 0, 2, sLabel );

    HSliderControl* sAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f );
    sAdjust->setWidth( 125 );
    sAdjust->setHeight( 12 );
    sAdjust->setVertAlign( Control::ALIGN_CENTER );
    sAdjust->addEventHandler( new SetHSL(filter, 1) );
    s_layerBox->setControl( 1, 2, sAdjust );

    // Lightness
    LabelControl* lLabel = new LabelControl( "Lightness" );      
    lLabel->setVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setControl( 0, 3, lLabel );

    HSliderControl* lAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f );
    lAdjust->setWidth( 125 );
    lAdjust->setHeight( 12 );
    lAdjust->setVertAlign( Control::ALIGN_CENTER );
    lAdjust->addEventHandler( new SetHSL(filter, 2) );
    s_layerBox->setControl( 1, 3, lAdjust );

    // Reset button
    LabelControl* resetButton = new LabelControl( "Reset" );
    resetButton->setBackColor( Color::Gray );
    resetButton->setActiveColor( Color::Blue );
    resetButton->addEventHandler( new ResetHSL(filter, hAdjust, sAdjust, lAdjust) );
    s_layerBox->setControl( 1, 4, resetButton );
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return -1;

    viewer.setSceneData( node );


    //Create the control panel
    Container* box = createControlPanel(&viewer);
    
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( node );
    if ( node )
    {   
        if (mapNode->getMap()->getNumImageLayers() == 0)
        {
            OE_NOTICE << "Please provide a map with at least one image layer" << std::endl;
            return 1;
        }

        // attach color filter to each layer.
        unsigned numLayers = mapNode->getMap()->getNumImageLayers();
        for( unsigned i=0; i<numLayers; ++i )
        {
            ImageLayer* layer = mapNode->getMap()->getImageLayerAt( i );

            HSLColorFilter* filter = new HSLColorFilter();
            layer->addColorFilter( filter );
            addHSLControls( filter, box, i );
        }
    }
    

    return viewer.run();
}
