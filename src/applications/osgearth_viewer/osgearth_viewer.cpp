/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/XmlUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/Graticule>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/Viewpoint>
#include <osgEarthSymbology/Color>
#include <osgEarthFeatures/FeatureNode>
#include <osgEarthFeatures/FeatureSource>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_viewer [--graticule] [--autoclip] file.earth" << std::endl;
    OE_NOTICE << "   --graticule     : displays a lat/long grid in geocentric mode" << std::endl;
    OE_NOTICE << "   --sky           : activates the atmospheric model" << std::endl;
    OE_NOTICE << "   --animateSky    : animates the sun across the sky" << std::endl;
    OE_NOTICE << "   --autoclip      : activates the auto clip-plane handler" << std::endl;
    OE_NOTICE << "   --jump          : automatically jumps to first viewpoint" << std::endl;
        
    return -1;
}

static EarthManipulator* s_manip         = 0L;
static Control*          s_controlPanel  =0L;
static SkyNode*          s_sky           =0L;

struct SkySliderHandler : public ControlEventHandler
{
    virtual void onValueChanged( class Control* control, float value )
    {
        s_sky->setDateTime( 2011, 3, 6, value );
    }
};

struct ClickViewpointHandler : public ControlEventHandler
{
    ClickViewpointHandler( const Viewpoint& vp ) : _vp(vp) { }
    Viewpoint _vp;

    virtual void onClick( class Control* control, int buttonMask )
    {
        s_manip->setViewpoint( _vp, 4.5 );
    }
};

osg::Node*
createControlPanel( osgViewer::View* view, const std::vector<Viewpoint>& vps )
{
    ControlCanvas* canvas = ControlCanvas::get( view );

    VBox* main = new VBox();
    main->setBackColor(0,0,0,0.5);
    main->setMargin( 10 );
    main->setPadding( 10 );
    main->setChildSpacing( 10 );
    main->setAbsorbEvents( true );
    main->setVertAlign( Control::ALIGN_BOTTOM );

    if ( vps.size() > 0 )
    {
        // the viewpoint container:
        Grid* g = new Grid();
        g->setChildSpacing( 0 );
        g->setChildVertAlign( Control::ALIGN_CENTER );

        unsigned i;
        for( i=0; i<vps.size(); ++i )
        {
            const Viewpoint& vp = vps[i];
            std::stringstream buf;
            buf << (i+1);
            Control* num = new LabelControl(buf.str(), 16.0f, osg::Vec4f(1,1,0,1));
            num->setPadding( 4 );
            g->setControl( 0, i, num );

            Control* vpc = new LabelControl(vp.getName().empty() ? "<no name>" : vp.getName(), 16.0f);
            vpc->setPadding( 4 );
            vpc->setHorizFill( true );
            vpc->setActiveColor( Color::Blue );
            vpc->addEventHandler( new ClickViewpointHandler(vp) );
            g->setControl( 1, i, vpc );
        }
        main->addControl( g );
    }

    // sky time slider:
    if ( s_sky )
    {
        HBox* skyBox = new HBox();
        skyBox->setChildVertAlign( Control::ALIGN_CENTER );
        skyBox->setChildSpacing( 10 );
        skyBox->setHorizFill( true );

        skyBox->addControl( new LabelControl("Time: ", 16) );

        HSliderControl* skySlider = new HSliderControl( 0.0f, 24.0f, 18.0f );
        skySlider->setBackColor( Color::Gray );
        skySlider->setHeight( 12 );
        skySlider->setHorizFill( true, 200 );
        skySlider->addEventHandler( new SkySliderHandler );
        skyBox->addControl( skySlider );

        main->addControl( skyBox );
    }
    
    canvas->addControl( main );

    s_controlPanel = main;
    return canvas;
}

struct AnimateSunCallback : public osg::NodeCallback
{
    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        SkyNode* skyNode = static_cast<SkyNode*>(node);
        double hours = fmod( osg::Timer::instance()->time_s()/4.0, 24.0 );
        skyNode->setDateTime( 2011, 6, 6, hours );
        OE_INFO << "TIME: " << hours << std::endl;
    }
};

struct ViewpointHandler : public osgGA::GUIEventHandler
{
    ViewpointHandler( const std::vector<Viewpoint>& viewpoints )
        : _viewpoints( viewpoints ) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN )
        {
            int index = (int)ea.getKey() - (int)'1';
            if ( index >= 0 && index < (int)_viewpoints.size() )
            {
                s_manip->setViewpoint( _viewpoints[index], 4.5 );
            }
            else if ( ea.getKey() == 'v' )
            {
                Viewpoint vp = s_manip->getViewpoint();
                XmlDocument xml( vp.getConfig() );
                xml.store( std::cout );
                std::cout << std::endl;
            }
            else if ( ea.getKey() == '?' )
            {
                s_controlPanel->setVisible( !s_controlPanel->visible() );
            }
        }
        return false;
    }

    std::vector<Viewpoint> _viewpoints;
};


bool hitTest(osgViewer::View * view, unsigned traversalMask, float x, float y, osg::Vec3d & posWorld, osg::Vec3d & posLocal, osg::NodePath & nodePath, osg::ref_ptr<osg::Drawable> & drawable, unsigned & primIndex)
{
	osg::Vec3d vecLocal;
	osg::Vec3d vecWorld;
	bool ret;

	osg::Camera * camera = view->getCamera();

	osg::ref_ptr< osgUtil::LineSegmentIntersector > picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);

	osgUtil::IntersectionVisitor iv(picker.get());

	// get the traversal mask from the given camera if no mask has been specified
	if(traversalMask == (unsigned)-1)
		traversalMask = camera->getCullMask();
	iv.setTraversalMask(traversalMask);
	camera->accept(iv);

	ret = picker->containsIntersections();
	if (ret)
	{
		const osgUtil::LineSegmentIntersector::Intersection& hitr = picker->getFirstIntersection();
		posWorld = hitr.getWorldIntersectPoint();
		posLocal = hitr.getLocalIntersectPoint();
		nodePath = hitr.nodePath;
		drawable = hitr.drawable;
		primIndex = hitr.primitiveIndex;
	}
	else
	{
		nodePath.clear();
		posWorld = osg::Vec3d();
		posLocal = osg::Vec3d();
		drawable = NULL;
		primIndex = (unsigned)-1;
	}
	return ret;
}

bool hitTestPolytope(osgViewer::View * view, unsigned traversalMask, float x, float y, osg::Vec3d & posWorld, osg::Vec3d & posLocal, osg::NodePath & nodePath, osg::ref_ptr<osg::Drawable> & drawable, unsigned & primIndex)
{
	osg::Vec3d vecLocal;
	osg::Vec3d vecWorld;
	bool ret;

	osg::Camera * camera = view->getCamera();

	osg::ref_ptr< osgUtil::PolytopeIntersector > picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x, y, x, y);

	osgUtil::IntersectionVisitor iv(picker.get());

	// get the traversal mask from the given camera if no mask has been specified
	if(traversalMask == (unsigned)-1)
		traversalMask = camera->getCullMask();
	iv.setTraversalMask(traversalMask);
	camera->accept(iv);

	ret = picker->containsIntersections();
	if (ret)
	{
		const osgUtil::PolytopeIntersector::Intersection& hitr = picker->getFirstIntersection();
		posWorld = hitr.localIntersectionPoint * (const osg::Matrix&)hitr.matrix;
		posLocal = hitr.localIntersectionPoint;
		nodePath = hitr.nodePath;
		drawable = hitr.drawable;
		primIndex = hitr.primitiveIndex;
	}
	else
	{
		nodePath.clear();
		posWorld = osg::Vec3d();
		posLocal = osg::Vec3d();
		drawable = NULL;
		primIndex = (unsigned)-1;
	}
	return ret;
}

std::basic_ostream<char>& operator<<(std::basic_ostream<char>& os, const osg::Node * node)
{
		const osg::StateSet * stateSet = (node!=NULL)?node->getStateSet():NULL;
		os << "{this=" << (void*)node 
			<< ";name=" << (node?node->getName():"<null>")
			<< ";classname=" << (node?node->className():"<null>")
			<< ";libname=" << (node?node->libraryName():"<null>")
			<< ";mask=" << std::hex << (node?node->getNodeMask():0) << std::dec
			<< "}";
//	}
	return os;
}

std::basic_ostream<char>& operator<<(std::basic_ostream<char>& os, const osg::NodePath & path)
{
	os << "{size=" << path.size() << ";elements=[";
	for(osg::NodePath::const_iterator it = path.begin(); it != path.end();)
	{
		const osg::Node * node = *it;
		os << node;
		it++;
		if(it != path.end())
			os << ",";
	}
	return os  << "]}";
}


struct FeatureInfoHandler : public osgGA::GUIEventHandler
{
	FeatureInfoHandler( osgEarth::Map * map )
		: _map( map ) { }

	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
	{
		osg::Vec3d world;
		osg::Vec3d local;
		osg::NodePath path;
		osg::ref_ptr<osg::Drawable> drawable;
		unsigned primIndex = 0;
		bool hit = false;
		if ( ea.getEventType() == ea.KEYDOWN)
		{
			if(ea.getKey() == 'i')
				hit = hitTest((osgViewer::View*)aa.asView(), (unsigned)-1, ea.getX(), ea.getY(), world, local, path, drawable, primIndex);
			else if(ea.getKey() == 'u')
				hit = hitTestPolytope((osgViewer::View*)aa.asView(), (unsigned)-1, ea.getX(), ea.getY(), world, local, path, drawable, primIndex);
		}
		if(hit)
		{
			std::cout << "hit on " << path << std::endl;
			FeatureNode * featureNode = NULL;
			for(osg::NodePath::reverse_iterator it = path.rbegin(); !featureNode && it != path.rend(); it++)
				featureNode = dynamic_cast<FeatureNode *>(*it);

			if(featureNode)
			{
				FeatureSource * featureSource = featureNode->getSource();
				osgEarth::Features::FeatureID fid;
				FeatureMultiNode * featureMultiNode = dynamic_cast<FeatureMultiNode *>(featureNode);
				if(featureMultiNode)
					fid = featureMultiNode->getFID(drawable, primIndex);
				else
					fid = featureNode->getFID();
				std::string name;
				if(featureSource)
				{
					Feature * feature = featureSource->getFeature(fid);
					if(feature)
						name = feature->getString("name");
				}
				std::cerr << "hit feature " << fid << ":" << name << std::endl;
			}
			else
			{
				std::cerr << "hit no feature" << std::endl;
			}
		}
		return false;
	}

	osgEarth::Map * _map;
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    bool useGraticule = arguments.read( "--graticule" );
    bool useAutoClip  = arguments.read( "--autoclip" );
    bool animateSky   = arguments.read( "--animateSky");
    bool useSky       = arguments.read( "--sky" ) || animateSky;
    bool jump         = arguments.read( "--jump" );

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );

    osgViewer::Viewer viewer(arguments);
    
    s_manip = new EarthManipulator();
    viewer.setCameraManipulator( s_manip );

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    // create a graticule and clip plane handler.
    Graticule* graticule = 0L;
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        const Config& externals = mapNode->externalConfig();

        if ( mapNode->getMap()->isGeocentric() )
        {
            // the Graticule is a lat/long grid that overlays the terrain. It only works
            // in a round-earth geocentric terrain.
            if ( useGraticule )
            {
                graticule = new Graticule( mapNode->getMap() );
                root->addChild( graticule );
            }

            // Sky model.
            Config skyConf = externals.child( "sky" );
            if ( !skyConf.empty() )
                useSky = true;

            if ( useSky )
            {
                double hours = skyConf.value( "hours", 12.0 );
                s_sky = new SkyNode( mapNode->getMap() );
                s_sky->setDateTime( 2011, 3, 6, hours );
                s_sky->attach( &viewer );
                root->addChild( s_sky );
                if ( animateSky )
                {
                    s_sky->setUpdateCallback( new AnimateSunCallback());
                }
            }

            if ( externals.hasChild("autoclip") )
                useAutoClip = externals.child("autoclip").boolValue( useAutoClip );

            // the AutoClipPlaneHandler will automatically adjust the near/far clipping
            // planes based on your view of the horizon. This prevents near clipping issues
            // when you are very close to the ground. If your app never brings a user very
            // close to the ground, you may not need this.
            if ( useSky || useAutoClip )
            {
                viewer.getCamera()->addEventCallback( new AutoClipPlaneCallback() );
            }
        }

        // read in viewpoints, if any
        std::vector<Viewpoint> viewpoints;
        const ConfigSet children = externals.children("viewpoint");
        if ( children.size() > 0 )
        {
            s_manip->getSettings()->setArcViewpointTransitions( true );

            for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
                viewpoints.push_back( Viewpoint(*i) );

            viewer.addEventHandler( new ViewpointHandler(viewpoints) );
            if ( jump )
                s_manip->setViewpoint(viewpoints[0]);
        }

        if ( viewpoints.size() > 0 || s_sky )
            root->addChild( createControlPanel(&viewer, viewpoints) );
    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	viewer.addEventHandler(new FeatureInfoHandler(mapNode->getMap()));

    return viewer.run();
}
