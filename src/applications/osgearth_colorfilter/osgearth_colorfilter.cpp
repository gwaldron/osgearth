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

#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/BrightnessContrastColorFilter>
#include <osgEarthUtil/CMYKColorFilter>
#include <osgEarthUtil/GammaColorFilter>
#include <osgEarthUtil/HSLColorFilter>
#include <osgEarthUtil/RGBColorFilter>
#include <osgEarthUtil/ChromaKeyColorFilter>
#include <osgEarthSymbology/Color>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;


Container*
createControlPanel(osgViewer::View* view)
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate(view);
    VBox* vbox = canvas->addControl(new VBox());
    vbox->setChildSpacing(10);
    return vbox;
}



namespace HSL
{
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
        ResetHSL(HSliderControl* h, HSliderControl* s, HSliderControl* l) 
            : _h(h), _s(s), _l(l) { }

        void onClick( Control* control )
        {
            _h->setValue( 0.0 );
            _s->setValue( 0.0 );
            _l->setValue( 0.0 );
        }

        HSliderControl* _h;
        HSliderControl* _s;
        HSliderControl* _l;
    };


    void
    addControls(HSLColorFilter* filter, Container* container, unsigned i)
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
        s_layerBox->setControl( 0, 0, new LabelControl(Stringify()<<"Layer "<<i, osg::Vec4(1,1,0,1)));
        
        // Hue:
        LabelControl* hLabel = new LabelControl( "Hue" );      
        hLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, hLabel );

        HSliderControl* hAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new SetHSL(filter,0) );
        hAdjust->setWidth( 125 );
        hAdjust->setHeight( 12 );
        hAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 1, hAdjust );
        s_layerBox->setControl( 2, 1, new LabelControl(hAdjust) );

        // Saturation:
        LabelControl* sLabel = new LabelControl( "Saturation" );      
        sLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 2, sLabel );

        HSliderControl* sAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new SetHSL(filter,1) );
        sAdjust->setWidth( 125 );
        sAdjust->setHeight( 12 );
        sAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 2, sAdjust );
        s_layerBox->setControl( 2, 2, new LabelControl(sAdjust) );

        // Lightness
        LabelControl* lLabel = new LabelControl( "Lightness" );      
        lLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 3, lLabel );

        HSliderControl* lAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new SetHSL(filter,2) );
        lAdjust->setWidth( 125 );
        lAdjust->setHeight( 12 );
        lAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 3, lAdjust );
        s_layerBox->setControl( 2, 3, new LabelControl(lAdjust) );

        // Reset button
        LabelControl* resetButton = new LabelControl( "Reset" );
        resetButton->setBackColor( osg::Vec4(0.5,0.5,0.5,1) );
        resetButton->setActiveColor( osg::Vec4(0.5,0.5,1,1) );
        resetButton->addEventHandler( new ResetHSL(hAdjust, sAdjust, lAdjust) );
        s_layerBox->setControl( 1, 4, resetButton );
    }
}


namespace RGB
{
    struct Set: public ControlEventHandler
    {
        Set(RGBColorFilter* filter, unsigned index) :
            _filter(filter), _index(index)
            { }

        void onValueChanged( Control* control, float value )
        {
            osg::Vec3f hsl = _filter->getRGBOffset();
            hsl[_index] = value;
            _filter->setRGBOffset( hsl );
        }

        RGBColorFilter* _filter;
        unsigned        _index;
    };



    struct Reset : public ControlEventHandler
    {
        Reset(HSliderControl* r, HSliderControl* g, HSliderControl* b) 
            : _r(r), _g(g), _b(b) { }

        void onClick( Control* control )
        {
            _r->setValue( 0.0 );
            _g->setValue( 0.0 );
            _b->setValue( 0.0 );
        }

        HSliderControl* _r;
        HSliderControl* _g;
        HSliderControl* _b;
    };


    void
    addControls(RGBColorFilter* filter, Container* container, unsigned i)
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
        
        // Red:
        LabelControl* rLabel = new LabelControl( "Red" );      
        rLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, rLabel );

        HSliderControl* rAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new RGB::Set(filter,0) );
        rAdjust->setWidth( 125 );
        rAdjust->setHeight( 12 );
        rAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 1, rAdjust );
        s_layerBox->setControl( 2, 1, new LabelControl(rAdjust) );

        // Green:
        LabelControl* gLabel = new LabelControl( "Green" );      
        gLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 2, gLabel );

        HSliderControl* gAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new RGB::Set(filter,1) );
        gAdjust->setWidth( 125 );
        gAdjust->setHeight( 12 );
        gAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 2, gAdjust );
        s_layerBox->setControl( 2, 2, new LabelControl(gAdjust) );

        // Blue
        LabelControl* bLabel = new LabelControl( "Blue" );      
        bLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 3, bLabel );

        HSliderControl* bAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new RGB::Set(filter,2) );
        bAdjust->setWidth( 125 );
        bAdjust->setHeight( 12 );
        bAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 3, bAdjust );
        s_layerBox->setControl( 2, 3, new LabelControl(bAdjust) );

        // Reset button
        LabelControl* resetButton = new LabelControl( "Reset" );
        resetButton->setBackColor( Color::Gray );
        resetButton->setActiveColor( Color::Blue );
        resetButton->addEventHandler( new Reset(rAdjust, gAdjust, bAdjust) );
        s_layerBox->setControl( 1, 4, resetButton );
    }
}


namespace CMYK
{
    struct Set: public ControlEventHandler
    {
        Set(CMYKColorFilter* filter, unsigned index) :
            _filter(filter), _index(index)
            { }

        void onValueChanged( Control* control, float value )
        {
            osg::Vec4 cmyk = _filter->getCMYKOffset();
            cmyk[_index] = value;
            _filter->setCMYKOffset( cmyk );
        }

        CMYKColorFilter* _filter;
        unsigned         _index;
    };



    struct Reset : public ControlEventHandler
    {
        Reset(HSliderControl* c, HSliderControl* m, HSliderControl* y, HSliderControl* k)
            : _c(c), _m(m), _y(y), _k(k) { }

        void onClick( Control* control )
        {
            _c->setValue( 0.0 );
            _m->setValue( 0.0 );
            _y->setValue( 0.0 );
            _k->setValue( 0.0 );
        }

        HSliderControl* _c;
        HSliderControl* _m;
        HSliderControl* _y;
        HSliderControl* _k;
    };


    void
    addControls(CMYKColorFilter* filter, Container* container, unsigned i)
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
        
        // Cyan:
        LabelControl* cLabel = new LabelControl( "Cyan" );      
        cLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, cLabel );

        HSliderControl* cAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new CMYK::Set(filter,0) );
        cAdjust->setWidth( 125 );
        cAdjust->setHeight( 12 );
        cAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 1, cAdjust );
        s_layerBox->setControl( 2, 1, new LabelControl(cAdjust) );

        // Magenta:
        LabelControl* mLabel = new LabelControl( "Magenta" );      
        mLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 2, mLabel );

        HSliderControl* mAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new CMYK::Set(filter,1) );
        mAdjust->setWidth( 125 );
        mAdjust->setHeight( 12 );
        mAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 2, mAdjust );
        s_layerBox->setControl( 2, 2, new LabelControl(mAdjust) );

        // Yellow
        LabelControl* yLabel = new LabelControl( "Yellow" );      
        yLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 3, yLabel );

        HSliderControl* yAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new CMYK::Set(filter,2) );
        yAdjust->setWidth( 125 );
        yAdjust->setHeight( 12 );
        yAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 3, yAdjust );
        s_layerBox->setControl( 2, 3, new LabelControl(yAdjust) );

        // Black
        LabelControl* kLabel = new LabelControl( "Black" );      
        kLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 4, kLabel );

        HSliderControl* kAdjust = new HSliderControl( -1.0f, 1.0f, 0.0f, new CMYK::Set(filter,3) );
        kAdjust->setWidth( 125 );
        kAdjust->setHeight( 12 );
        kAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 4, kAdjust );
        s_layerBox->setControl( 2, 4, new LabelControl(kAdjust) );

        // Reset button
        LabelControl* resetButton = new LabelControl( "Reset" );
        resetButton->setBackColor( Color::Gray );
        resetButton->setActiveColor( Color::Blue );
        resetButton->addEventHandler( new Reset(cAdjust, mAdjust, yAdjust, kAdjust) );
        s_layerBox->setControl( 1, 5, resetButton );
    }
}


namespace BC
{
    struct Set: public ControlEventHandler
    {
        Set(BrightnessContrastColorFilter* filter, unsigned index) :
            _filter(filter), _index(index)
            { }

        void onValueChanged( Control* control, float value )
        {
            osg::Vec2f bc = _filter->getBrightnessContrast();
            bc[_index] = value;
            _filter->setBrightnessContrast( bc );
        }

        BrightnessContrastColorFilter* _filter;
        unsigned        _index;
    };



    struct Reset : public ControlEventHandler
    {
        Reset(HSliderControl* b, HSliderControl* c)
            : _b(b), _c(c) { }

        void onClick( Control* control )
        {
            _b->setValue( 1.0f );
            _c->setValue( 1.0f );
        }

        HSliderControl* _b;
        HSliderControl* _c;
    };


    void
    addControls(BrightnessContrastColorFilter* filter, Container* container, unsigned i)
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
        
        // Brightness:
        LabelControl* bLabel = new LabelControl( "Brightness" );      
        bLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, bLabel );

        HSliderControl* bAdjust = new HSliderControl( 0.0f, 5.0f, 1.0f, new BC::Set(filter,0) );
        bAdjust->setWidth( 125 );
        bAdjust->setHeight( 12 );
        bAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 1, bAdjust );
        s_layerBox->setControl( 2, 1, new LabelControl(bAdjust) );

        // Contrast:
        LabelControl* cLabel = new LabelControl( "Contrast" );      
        cLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 2, cLabel );

        HSliderControl* cAdjust = new HSliderControl( 0.0f, 5.0f, 1.0f, new BC::Set(filter,1) );
        cAdjust->setWidth( 125 );
        cAdjust->setHeight( 12 );
        cAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 2, cAdjust );
        s_layerBox->setControl( 2, 2, new LabelControl(cAdjust) );

        // Reset button
        LabelControl* resetButton = new LabelControl( "Reset" );
        resetButton->setBackColor( Color::Gray );
        resetButton->setActiveColor( Color::Blue );
        resetButton->addEventHandler( new Reset(bAdjust, cAdjust) );
        s_layerBox->setControl( 1, 3, resetButton );
    }
}


namespace GAMMA
{
    struct Set: public ControlEventHandler
    {
        Set(GammaColorFilter* filter) : _filter(filter) { }

        void onValueChanged( Control* control, float value )
        {
            _filter->setGamma( value );
        }

        GammaColorFilter* _filter;
    };



    struct Reset : public ControlEventHandler
    {
        Reset(HSliderControl* g)
            : _g(g) { }

        void onClick( Control* control )
        {
            _g->setValue( 1.0f );
        }

        HSliderControl*   _g;
    };


    void
    addControls(GammaColorFilter* filter, Container* container, unsigned i)
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
        
        // Gamma:
        LabelControl* gLabel = new LabelControl( "Gamma" );      
        gLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, gLabel );

        HSliderControl* gAdjust = new HSliderControl( 0.1f, 3.0f, 1.0f, new GAMMA::Set(filter) );
        gAdjust->setWidth( 125 );
        gAdjust->setHeight( 12 );
        gAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 1, gAdjust );
        s_layerBox->setControl( 2, 1, new LabelControl(gAdjust) );

        // Reset button
        LabelControl* resetButton = new LabelControl( "Reset" );
        resetButton->setBackColor( Color::Gray );
        resetButton->setActiveColor( Color::Blue );
        resetButton->addEventHandler( new Reset(gAdjust) );
        s_layerBox->setControl( 1, 3, resetButton );
    }
}

namespace CHROMAKEY
{
    struct SetColor: public ControlEventHandler
    {
        SetColor(ChromaKeyColorFilter* filter, unsigned index) :
            _filter(filter), _index(index)
            { }

        void onValueChanged( Control* control, float value )
        {
            osg::Vec3f color = _filter->getColor();
            color[_index] = value;
            _filter->setColor( color );
        }

        ChromaKeyColorFilter* _filter;
        unsigned        _index;
    };

    struct SetDistance: public ControlEventHandler
    {
        SetDistance(ChromaKeyColorFilter* filter) :
            _filter(filter)
            { }

        void onValueChanged( Control* control, float value )
        {
            _filter->setDistance( value );
        }

        ChromaKeyColorFilter* _filter;
    };



    struct Reset : public ControlEventHandler
    {
        Reset(HSliderControl* r, HSliderControl* g, HSliderControl* b, HSliderControl* distance) 
            : _r(r), _g(g), _b(b), _distance( distance) { }

        void onClick( Control* control )
        {
            _r->setValue( 0.0 );
            _g->setValue( 0.0 );
            _b->setValue( 0.0 );
            _distance->setValue( 0.0 );
        }

        HSliderControl* _r;
        HSliderControl* _g;
        HSliderControl* _b;
        HSliderControl* _distance;
    };


    void
    addControls(ChromaKeyColorFilter* filter, Container* container, unsigned i)
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
        
        // Red:
        LabelControl* rLabel = new LabelControl( "Red" );      
        rLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 1, rLabel );

        HSliderControl* rAdjust = new HSliderControl( 0.0f, 1.0f, 0.0f, new CHROMAKEY::SetColor(filter,0) );
        rAdjust->setWidth( 125 );
        rAdjust->setHeight( 12 );
        rAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 1, rAdjust );
        s_layerBox->setControl( 2, 1, new LabelControl(rAdjust) );

        // Green:
        LabelControl* gLabel = new LabelControl( "Green" );      
        gLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 2, gLabel );

        HSliderControl* gAdjust = new HSliderControl( 0.0f, 1.0f, 0.0f, new CHROMAKEY::SetColor(filter,1) );
        gAdjust->setWidth( 125 );
        gAdjust->setHeight( 12 );
        gAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 2, gAdjust );
        s_layerBox->setControl( 2, 2, new LabelControl(gAdjust) );

        // Blue
        LabelControl* bLabel = new LabelControl( "Blue" );      
        bLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 3, bLabel );

        HSliderControl* bAdjust = new HSliderControl( 0.0f, 1.0f, 0.0f, new CHROMAKEY::SetColor(filter,2) );
        bAdjust->setWidth( 125 );
        bAdjust->setHeight( 12 );
        bAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 3, bAdjust );
        s_layerBox->setControl( 2, 3, new LabelControl(bAdjust) );

        // Distance
        LabelControl* distLabel = new LabelControl( "Distance" );      
        distLabel->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 0, 4, distLabel );

        HSliderControl* distAdjust = new HSliderControl( 0.0f, 2.0f, 0.0f, new CHROMAKEY::SetDistance(filter) );
        distAdjust->setWidth( 125 );
        distAdjust->setHeight( 12 );
        distAdjust->setVertAlign( Control::ALIGN_CENTER );
        s_layerBox->setControl( 1, 4, distAdjust );
        s_layerBox->setControl( 2, 4, new LabelControl(distAdjust) );

        // Reset button
        LabelControl* resetButton = new LabelControl( "Reset" );
        resetButton->setBackColor( Color::Gray );
        resetButton->setActiveColor( Color::Blue );
        resetButton->addEventHandler( new Reset(rAdjust, gAdjust, bAdjust, distAdjust) );
        s_layerBox->setControl( 1, 5, resetButton );
    }
}



bool usage( const std::string& msg )
{
    OE_WARN << std::endl
        << msg << "\n\n"
        << "osgearth_colorfilter <earth_file> \n"
        << "            [--hsl]        Use the HSL (hue/saturation/lightness) filter\n"
        << "            [--rgb]        Use the RGB (red/green/blue/alpha) filter\n"
        << "            [--cmyk]       Use the CMYK (cyan/magenta/yellow/black) filter\n"
        << "            [--bc]         Use the Brightness/Contract filter\n"
        << "            [--gamma]      Use the Gamma filter\n"
        << "            [--chromakey]  Use the chromakey filter\n"
        << std::endl;
    return -1;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // Which filter?
    bool useHSL   = arguments.read("--hsl");
    bool useRGB   = arguments.read("--rgb");
    bool useCMYK  = arguments.read("--cmyk");
    bool useBC    = arguments.read("--bc");
    bool useGamma = arguments.read("--gamma");
    bool useChromaKey = arguments.read("--chromakey");

    if ( !useHSL && !useRGB && !useCMYK && !useBC && !useGamma && !useChromaKey )
    {
        return usage( "Please select one of the filter options!" );
    }

    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return usage( "Unable to load map from earth file!" );
    viewer.setSceneData( node );

    //Create the control panel
    Container* box = createControlPanel(&viewer);
    
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( node );
    if ( node )
    {   
        if (mapNode->getMap()->getNumImageLayers() == 0)
        {
            return usage("Please provide a map with at least one image layer.");
        }

        // attach color filter to each layer.
        unsigned numLayers = mapNode->getMap()->getNumImageLayers();
        for( unsigned i=0; i<numLayers; ++i )
        {
            ImageLayer* layer = mapNode->getMap()->getImageLayerAt( i );

            if ( layer->getEnabled() && layer->getVisible() )
            {
                if ( useHSL )
                {
                    HSLColorFilter* filter = new HSLColorFilter();
                    layer->addColorFilter( filter );
                    HSL::addControls( filter, box, i );
                }
                else if ( useRGB )
                {
                    RGBColorFilter* filter = new RGBColorFilter();
                    layer->addColorFilter( filter );
                    RGB::addControls( filter, box, i );
                }
                else if ( useCMYK )
                {
                    CMYKColorFilter* filter = new CMYKColorFilter();
                    layer->addColorFilter( filter );
                    CMYK::addControls( filter, box, i );
                }
                else if ( useBC )
                {
                    BrightnessContrastColorFilter* filter = new BrightnessContrastColorFilter();
                    layer->addColorFilter( filter );
                    BC::addControls( filter, box, i );
                }
                else if ( useGamma )
                {
                    GammaColorFilter* filter = new GammaColorFilter();
                    layer->addColorFilter( filter );
                    GAMMA::addControls( filter, box, i );
                }
                else if ( useChromaKey )
                {
                    ChromaKeyColorFilter* filter = new ChromaKeyColorFilter();
                    layer->addColorFilter( filter );
                    CHROMAKEY::addControls( filter, box , i );
                }
            }
        }
    }
    

    return viewer.run();
}
