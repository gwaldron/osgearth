/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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


#include <sstream>
#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgDB/FileNameUtils>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osgEarthSymbology/GeometryExtrudeSymbolizer>
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/SymbolicNode>
#include <osgEarthSymbology/MarkerSymbol>
#include <osgEarthSymbology/MarkerSymbolizer>
#include <osgEarthSymbology/ExtrudedSymbol>
#include <osgEarthSymbology/ModelSymbolizer>
#include <osgEarthUtil/Popups>
//#include <osgEarthUtil/PopupManager>
//#include <osgEarthUtil/TextPopup>
//#include <osgEarthUtil/WidgetIcon>
//#include <osgEarthUtil/WidgetNode>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osgUtil/Tessellator>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>

using namespace osgEarth::Symbology;
using namespace osgEarthUtil;

static double getRandomValueInOne()
{
    return ((rand() * 1.0)/(RAND_MAX-1));
}


Geometry* createLineGeometry(const osg::Vec3d& start)
{
    osg::ref_ptr<osg::Vec3dArray> array = new osg::Vec3dArray;
    array->push_back(start + osg::Vec3d(-100,-100,0));
    array->push_back(start + osg::Vec3d(100,-100,0));
    array->push_back(start + osg::Vec3d(100,100,0));
    array->push_back(start + osg::Vec3d(-100,100,0));
    return Geometry::create(Geometry::TYPE_LINESTRING, array);
}

Geometry* createRingGeometry(const osg::Vec3d& start)
{
    osg::ref_ptr<osg::Vec3dArray> array = new osg::Vec3dArray;
    array->push_back(start + osg::Vec3d(-100,-100,0));
    array->push_back(start + osg::Vec3d(100,-100,0));
    array->push_back(start + osg::Vec3d(100,100,0));
    array->push_back(start + osg::Vec3d(-100,100,0));
    return Geometry::create(Geometry::TYPE_RING, array);
}


Geometry* createPolygonGeometry(const osg::Vec3d& start)
{
    osg::ref_ptr<osg::Vec3dArray> array = new osg::Vec3dArray;
    array->push_back(start + osg::Vec3d(-100,-100,0));
    array->push_back(start + osg::Vec3d(-10,-10,0));
    array->push_back(start + osg::Vec3d(100,-100,0));
    array->push_back(start + osg::Vec3d(100,100,0));
    array->push_back(start + osg::Vec3d(-100,100,0));
    return Geometry::create(Geometry::TYPE_POLYGON, array);
}


Geometry* createPointsGeometry(const osg::Vec3d& start)
{
    osg::ref_ptr<osg::Vec3dArray> array = new osg::Vec3dArray;
    array->push_back(start + osg::Vec3d(-100,-100,0));
    array->push_back(start + osg::Vec3d(100,-100,0));
    array->push_back(start + osg::Vec3d(100,100,0));
    array->push_back(start + osg::Vec3d(-100,100,0));
    return Geometry::create(Geometry::TYPE_POINTSET, array);
}

struct SampleGeometryInput : public GeometryContent
{
    SampleGeometryInput()
    {
        // points
        {
        _geometryList.push_back(createPointsGeometry(osg::Vec3d(-250,0,0)));
        }

        // ring
        {
        _geometryList.push_back(createRingGeometry(osg::Vec3d(0,0,0)));
        }

        // line
        {
        _geometryList.push_back(createLineGeometry(osg::Vec3d(250,0,0)));
        }

        // polygon
        {
        _geometryList.push_back(createPolygonGeometry(osg::Vec3d(500,0,0)));
        }
    }
};


struct PopUpSymbolizerContext : public SymbolizerContext
{
    typedef std::vector<osg::ref_ptr<TextPopup> > TextPopupList;

    PopUpSymbolizerContext(PopupManager* windowmanager)  : _wm(windowmanager) {}
    osg::ref_ptr<PopupManager> _wm;
    TextPopupList _widgetList;
};



struct PopUpSymbolizer : public Symbolizer< State<GeometryContent> >
{
    static int PopUpIndex;

    bool compile(State<GeometryContent>* state,
                 osg::Group* attachPoint)
    {
        if ( !state || !attachPoint || !state->getContent() || !state->getStyle() )
            return false;

        PopUpSymbolizerContext* ctx = dynamic_cast<PopUpSymbolizerContext*>( state->getContext() );
        if (!ctx)
            return false;

        osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;

        const GeometryList& geometryList = state->getContent()->getGeometryList();
        for (GeometryList::const_iterator it = geometryList.begin(); it != geometryList.end(); ++it)
        {
            Geometry* geometry = *it;
            if (!geometry)
                continue;

            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry;
            switch( geometry->getType())
            {
            case Geometry::TYPE_POINTSET:
            case Geometry::TYPE_LINESTRING:
            case Geometry::TYPE_RING:
            case Geometry::TYPE_POLYGON:
            {
                const PopupSymbol* symbol = state->getStyle()->getSymbol<PopupSymbol>();
                if (symbol)
                {
                    //osg::Image* image = 0;
                    //if (!symbol->theme()->empty())
                    //    image = osgDB::readImageFile(symbol->theme().value());
                    //osgText::Font* font = 0;
                    //if (!symbol->font()->empty())
                    //    font = osgText::readFontFile(symbol->font().value());
                    //float size = symbol->size().value();
                    //osg::Vec4 color = symbol->fill()->color();
                    
                    for ( osg::Vec3dArray::iterator it = geometry->begin(); it != geometry->end(); ++it)
                    {
                        osg::MatrixTransform* transform = new osg::MatrixTransform;
                        transform->setMatrix(osg::Matrix::translate(*it));
                        if (!PopUpIndex) {
                            std::stringstream ss;
                            ss << "Hit i Key to see more popup" << std::endl;
                            ss << "And hit a second time to hide them" << std::endl;
                            std::string text = ss.str();
                            std::string title = "Info";

                            TextPopup* popup = ctx->_wm->createTextPopup(title, text, symbol);
                            //popup->setFocusColor(osg::Vec4(getRandomValueInOne(), getRandomValueInOne() , getRandomValueInOne(), 1.0));
                            popup->attach(transform);
                            popup->setAppear();

                        } else {
                            std::stringstream ss;
                            ss << "osgEarth" << std::endl;
                            ss << "rocks at " << *it << " miles" << std::endl;
                            std::string text = ss.str();
                            std::string title = "osgEarthUtil";
                            TextPopup* popup = ctx->_wm->createTextPopup(title, text, symbol);
                            //popup->setFocusColor(osg::Vec4(getRandomValueInOne(), getRandomValueInOne() , getRandomValueInOne(), 1.0));
                            popup->attach(transform);
                            ctx->_widgetList.push_back(popup);
                        }
                        PopUpIndex++;
                        newSymbolized->addChild(transform);
                        transform->addChild(osgDB::readNodeFile("../data/tree.ive"));
                    }
                }
            }
            break;
            }
        }

        if (newSymbolized->getNumChildren()) 
        {
            attachPoint->removeChildren(0, attachPoint->getNumChildren());
            attachPoint->addChild(newSymbolized.get());
            return true;
        }

        return false;
    }
};
int PopUpSymbolizer::PopUpIndex = 0;

struct PolygonPointSizeSymbol : public PolygonSymbol
{
    PolygonPointSizeSymbol() : _size (1.0) {}
    float& size() { return _size; }
    const float& size() const { return _size; }

protected:
    float _size;
};

struct GeometryPointSymbolizer : public GeometrySymbolizer
{
    bool update(State<GeometryContent>* state,
                osg::Group* attachPoint,
                SymbolizerContext* context )
    {
        if ( !state || !attachPoint || !state->getContent() || !state->getStyle() )
            return false;

        osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        newSymbolized->addChild(geode.get());

        const GeometryList& geometryList = state->getContent()->getGeometryList();
        for (GeometryList::const_iterator it = geometryList.begin(); it != geometryList.end(); ++it)
        {
            Geometry* geometry = *it;
            if (!geometry)
                continue;

            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry;
            osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

            osg::Vec4 color = osg::Vec4(1.0, 0.0, 1.0, 1.);

            switch( geometry->getType())
            {
            case Geometry::TYPE_POINTSET:
            {
                primMode = osg::PrimitiveSet::POINTS;
                const PointSymbol* point = state->getStyle()->getSymbol<PointSymbol>();
                if (point)
                {
                    color = point->fill()->color();

                    float size = point->size().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(size) );
                }
            }
            break;

            case Geometry::TYPE_LINESTRING:
            {
                primMode = osg::PrimitiveSet::LINE_STRIP;
                const LineSymbol* line = state->getStyle()->getSymbol<LineSymbol>();
                if (line) 
                {
                    color = line->stroke()->color();
                    float size = line->stroke()->width().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::LineWidth(size));
                }
            }
            break;

            case Geometry::TYPE_RING:
            {
                primMode = osg::PrimitiveSet::LINE_LOOP;
                const LineSymbol* line = state->getStyle()->getSymbol<LineSymbol>();
                if (line)
                {
                    color = line->stroke()->color();
                    float size = line->stroke()->width().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::LineWidth(size));
                }
            }
            break;

            case Geometry::TYPE_POLYGON:
            {
                // use polygon as point for this specific symbolizer
                // it would be simpler to use the symbol style->getPoint but here
                // we want to dmonstrate how to customize Symbol and Symbolizer
                primMode = osg::PrimitiveSet::POINTS;
                const PolygonPointSizeSymbol* poly = state->getStyle()->getSymbol<PolygonPointSizeSymbol>();
                if (poly)
                {
                    color = poly->fill()->color();

                    float size = poly->size();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(size) );
                }
            }
            break;
            }

            osg::Material* material = new osg::Material;
            material->setDiffuse(osg::Material::FRONT_AND_BACK, color);

            osgGeom->setVertexArray( geometry->toVec3Array() );
            osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, geometry->size() ) );

            osgGeom->getOrCreateStateSet()->setAttributeAndModes(material);
            osgGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, false);
            geode->addDrawable(osgGeom);

        }

        if (geode->getNumDrawables()) 
        {
            attachPoint->removeChildren(0, attachPoint->getNumChildren());
            attachPoint->addChild(newSymbolized.get());
            return true;
        }

        return false;
    }
};



class StyleEditor : public osgGA::GUIEventHandler
{
public:
    osg::ref_ptr<PopUpSymbolizerContext> _popupContext;
    int _state;

    StyleEditor(const ::StyleList& styles) : _styles(styles)
    {
        _state = 0;
    }

    void setPopupContext(PopUpSymbolizerContext* context) { _popupContext = context; }
    
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYUP):
        {
            if (ea.getKey() == 'i') {
                if (_popupContext.valid()) {
                    for (int i = 0; i < _popupContext->_widgetList.size(); ++i) {
                        if (_state) {
                            _popupContext->_widgetList[i]->setDisappear();
                        } else { 
                            _popupContext->_widgetList[i]->setAppear();
                        }
                    }
                    if (_state)
                        _state = 0;
                    else 
                        _state = 1;
                }
                return true;
            } else if (ea.getKey() == 'q') {
                osgEarth::Symbology::Style* style = _styles[0].get();
                PolygonSymbol* p = style->getSymbol<PolygonSymbol>();
                if (p)
                {
                    osg::Vec4 color = p->fill()->color();
                    color[0] = fmod(color[0]+0.5, 1.0);
                    color[2] = fmod(1 + color[0]-0.3, 1.0);
                    p->fill()->color() = color;
                    style->dirty();
                }
                return true;
            } else if (ea.getKey() == 'a') {
                Style* style = _styles[1].get();
                PolygonPointSizeSymbol* p = style->getSymbol<PolygonPointSizeSymbol>();
                if (p)
                {
                    osg::Vec4 color = p->fill()->color();
                    color[0] = fmod(color[0]+0.5, 1.0);
                    color[2] = fmod(1 + color[0]-0.3, 1.0);
                    p->fill()->color() = color;
                    p->size() = 0.1 + color[2] * 10;
                    style->dirty();
                }
                return true;
            } else if (ea.getKey() == 'z') {
                Style* style = _styles[2].get();
                ExtrudedLineSymbol* l = style->getSymbol<ExtrudedLineSymbol>();
                if (l)
                {
                    osg::Vec4 color = l->stroke()->color();
                    color[0] = fmod(color[0]+0.5, 1.0);
                    color[2] = fmod(1 + color[0]-0.3, 1.0);
                    l->stroke()->color() = color;
                    l->extrude()->height() = l->extrude()->height() + 200;
                }
                ExtrudedPolygonSymbol* p = style->getSymbol<ExtrudedPolygonSymbol>();
                if (p)
                {
                    osg::Vec4 color = p->fill()->color();
                    color[0] = fmod(color[0]+0.5, 1.0);
                    color[2] = fmod(1 + color[0]-0.3, 1.0);
                    p->fill()->color() = color;
                    p->extrude()->height() = p->extrude()->height() + 50;
                }
                style->dirty();
                return true;

            } else if (ea.getKey() == 'x') {
                Style* style = _styles[3].get();
                MarkerLineSymbol* l = style->getSymbol<MarkerLineSymbol>();
                if (l)
                {
                    if (l->interval().value() < 10)
                        l->interval() = 15;
                    else
                        l->interval() = 5;
                }

                MarkerPolygonSymbol* p = style->getSymbol<MarkerPolygonSymbol>();
                if (p)
                {
                    if (p->interval().value() < 10) {
                        p->interval() = 15;
                        p->randomRatio() = 0.1;
                    } else {
                        p->interval() = 5;
                        p->randomRatio() = 0.9;
                    }
                }
                style->dirty();
                return true;
            }
        }
        break;
        }
        return false;
    }
    
    ::StyleList _styles;
};


typedef SymbolicNode< State<GeometryContent> > GeometrySymbolicNode;


osg::Group* createSymbologyScene(PopupManager* wm)
{
    osg::Group* grp = new osg::Group;

    osg::ref_ptr<SampleGeometryInput> dataset = new SampleGeometryInput;
    StyleList styles;

    {
        osg::ref_ptr<Style> style = new Style;
        style->setName("PolygonSymbol-color");
        osg::ref_ptr<PolygonSymbol> polySymbol = new PolygonSymbol;
        polySymbol->fill()->color() = osg::Vec4(0,1,1,1);
        style->addSymbol(polySymbol.get());
        styles.push_back(style.get());
    }

    {
        osg::ref_ptr<Style> style = new Style;
        style->setName("Custom-PolygonPointSizeSymbol-size&color");
        osg::ref_ptr<PolygonPointSizeSymbol> polySymbol = new PolygonPointSizeSymbol;
        polySymbol->fill()->color() = osg::Vec4(1,0,0,1);
        polySymbol->size() = 2.0;
        style->addSymbol(polySymbol.get());
        styles.push_back(style.get());
    }


    // style for extruded
    {
        osg::ref_ptr<Style> style = new Style;
        style->setName("Extrude-Polygon&Line-height&color");
        osg::ref_ptr<ExtrudedPolygonSymbol> polySymbol = new ExtrudedPolygonSymbol;
        polySymbol->fill()->color() = osg::Vec4(1,0,0,1);
        polySymbol->extrude()->height() = 100;
        polySymbol->extrude()->offset() = 10;
        style->addSymbol(polySymbol.get());

        osg::ref_ptr<ExtrudedLineSymbol> lineSymbol = new ExtrudedLineSymbol;
        lineSymbol->stroke()->color() = osg::Vec4(0,0,1,1);
        lineSymbol->extrude()->height() = 150;
        lineSymbol->extrude()->offset() = 10;
        style->addSymbol(lineSymbol.get());
        styles.push_back(style.get());
    }


    // style for marker
    {
        osg::ref_ptr<Style> style = new Style;
        style->setName("Marker");
        osg::ref_ptr<MarkerSymbol> pointSymbol = new MarkerSymbol;
        pointSymbol->marker() = "../data/tree.ive";
        style->addSymbol(pointSymbol.get());

        osg::ref_ptr<MarkerLineSymbol> lineSymbol = new MarkerLineSymbol;
        lineSymbol->marker() = "../data/tree.ive";
        lineSymbol->interval() = 5;
        style->addSymbol(lineSymbol.get());

        osg::ref_ptr<MarkerPolygonSymbol> polySymbol = new MarkerPolygonSymbol;
        polySymbol->marker() = "../data/tree.ive";
        polySymbol->interval() = 20;
        polySymbol->randomRatio() = 1.0;
        style->addSymbol(polySymbol.get());

        styles.push_back(style.get());
    }


    // style for popup
    {
        osg::ref_ptr<Style> style = new Style;
        style->setName("Popup");
        osg::ref_ptr<TextSymbol> symbol = new TextSymbol;
        //symbol->theme() = "../data/popup-theme.png";
        symbol->font() = "arial.ttf";
        symbol->size() = 14;
        symbol->fill()->color() = osg::Vec4(getRandomValueInOne(), getRandomValueInOne() , getRandomValueInOne(), 0.7);
        style->addSymbol(symbol.get());
        styles.push_back(style.get());
    }


    /// associate the style / symbolizer to the symbolic node
    {
        GeometrySymbolicNode* node = new GeometrySymbolicNode();
        node->setSymbolizer( new GeometrySymbolizer() );

        node->getState()->setStyle( styles[0].get() );
        node->getState()->setContent( dataset.get() );

        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->setMatrix(osg::Matrix::translate(0, -250 , 0));
        tr->addChild(node);
        grp->addChild(tr);
    }


    {
        GeometrySymbolicNode* node = new GeometrySymbolicNode();
        node->setSymbolizer( new GeometryPointSymbolizer() );
        node->getState()->setStyle(styles[1].get());
        node->getState()->setContent(dataset.get());
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->addChild(node);
        tr->setMatrix(osg::Matrix::translate(0, 0 , 0));
        grp->addChild(tr);
    }


    {
        GeometrySymbolicNode* node = new GeometrySymbolicNode();
        node->setSymbolizer( new GeometryExtrudeSymbolizer() );
        node->getState()->setStyle(styles[2].get());
        node->getState()->setContent(dataset.get());
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->addChild(node);
        tr->setMatrix(osg::Matrix::translate(0, 250 , 0));
        grp->addChild(tr);
    }


    {
        GeometrySymbolicNode* node = new GeometrySymbolicNode();
        node->setSymbolizer( new MarkerSymbolizer() );
        node->getState()->setStyle(styles[3].get());
        node->getState()->setContent(dataset.get());
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->addChild(node);
        tr->setMatrix(osg::Matrix::translate(0, 500 , 0));
        grp->addChild(tr);
    }



    //{
    //    osg::ref_ptr<ModelSymbolizer> symbolizer = new ModelSymbolizer();
    //    osg::ref_ptr<SymbolicNode> node = new SymbolicNode;
    //    node->setSymbolizer(symbolizer.get());
    //    Style* style = new Style;
    //    std::string real = osgDB::getRealPath("../data/tree.ive");
    //    MarkerSymbol* marker = new MarkerSymbol;
    //    marker->marker() = real;
    //    node->setDataSet(dataset.get());
    //    style->addSymbol(marker);
    //    node->setStyle(style);
    //    osg::MatrixTransform* tr = new osg::MatrixTransform;
    //    tr->addChild(node);
    //    tr->setMatrix(osg::Matrix::scale(10,10,10) * osg::Matrix::translate(0, 750 , 0));
    //    grp->addChild(tr);
    //}

    StyleEditor* styleEditor = new StyleEditor(styles);
    {
        PopUpSymbolizerContext* ctx = new PopUpSymbolizerContext(wm);
        styleEditor->setPopupContext(ctx);

        SymbolicNode< State<GeometryContent> >* node = new SymbolicNode< State<GeometryContent> >();
        node->setSymbolizer( new PopUpSymbolizer() );
        node->getState()->setStyle(styles[4]);
        node->getState()->setContent(dataset.get());
        node->getState()->setContext(ctx);
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->addChild(node);
        tr->setMatrix(osg::Matrix::translate(0, 1000 , 0));
        grp->addChild(tr);
    }
    
    grp->addEventCallback(styleEditor);
    return grp;
}




int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // add some stock OSG handlers:
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    
    osg::Group* root = new osg::Group;
    viewer.setSceneData(root);
    viewer.realize();

    PopupManager* wm = new PopupManager(&viewer);
    osg::Node* node = createSymbologyScene(wm);
    root->addChild(node);
    return viewer.run();
}
