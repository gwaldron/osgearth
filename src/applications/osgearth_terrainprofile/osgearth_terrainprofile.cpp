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
#include <osgEarth/GeoMath>
#include <osgEarthFeatures/Feature>
#include <osgEarthAnnotation/FeatureNode>
#include <osg/io_utils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;
using namespace osgEarth::Annotation;

/***************************************************/
class TerrainProfile
{
public:
    TerrainProfile();

    double getDistance( int i ) const;
    double getTotalDistance() const;

    void getElevationRanges(double &min, double &max );
    
    double getElevation( int i ) const;
    unsigned int getNumElevations() const;

    void setSpacing( double spacing );
    double getSpacing() const;

    void addElevation( double elevation );

    void clear();

private:
    double _spacing;
    typedef std::vector< double > ElevationList;
    ElevationList _elevations;
};

/***************************************************/
TerrainProfile::TerrainProfile():
_spacing( 1.0 )
{
}

void
TerrainProfile::clear()
{
    _elevations.clear();
}

double
TerrainProfile::getSpacing() const
{
    return _spacing;
}

void
TerrainProfile::setSpacing( double spacing )
{
    _spacing = spacing;
}

void
TerrainProfile::addElevation( double elevation )
{
    _elevations.push_back( elevation );
}

double
TerrainProfile::getElevation( int i ) const
{
    if (i >= 0 && i < _elevations.size()) return _elevations[i];
    return DBL_MAX;
}

double
TerrainProfile::getDistance( int i ) const
{
    return (double)i * _spacing;
}

double
TerrainProfile::getTotalDistance() const
{
    return getDistance( getNumElevations()-1 );
}

unsigned int
TerrainProfile::getNumElevations() const
{
    return _elevations.size();
}

void
TerrainProfile::getElevationRanges(double &min, double &max )
{
    min = DBL_MAX;
    max = -DBL_MAX;

    for (unsigned int i = 0; i < _elevations.size(); i++)
    {
        if (_elevations[i] < min) min = _elevations[i];
        if (_elevations[i] > max) max = _elevations[i];
    }
}

/**************************************************/

void computeTerrainProfile( osgEarth::MapNode* mapNode, double startX, double startY, double endX, double endY, unsigned int numSamples, TerrainProfile& profile)
{
    double startXRad = osg::DegreesToRadians( startX );
    double startYRad = osg::DegreesToRadians( startY );
    double endXRad = osg::DegreesToRadians( endX );
    double endYRad = osg::DegreesToRadians( endY );

    double distance = osgEarth::GeoMath::distance(startYRad, startXRad, endYRad, endXRad );
    
    double spacing = distance / ((double)numSamples - 1.0);
    
    profile.setSpacing( spacing );
    profile.clear();
    
    for (unsigned int i = 0; i < numSamples; i++)
    {
        double t = (double)i / (double)numSamples;
        double lat, lon;
        GeoMath::interpolate( startYRad, startXRad, endYRad, endXRad, t, lat, lon );
        double height;
        mapNode->getTerrain()->getHeight( osg::Vec3d( osg::RadiansToDegrees( lon ), osg::RadiansToDegrees( lat), 0.0), height);
        profile.addElevation( height );
    }
}

osg::Node* createBackground(double width, double height, const osg::Vec4f& backgroundColor)
{
    //Add a background quad
    osg::Geometry* geometry = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( 4 );
    verts->push_back( osg::Vec3(0,0,0));
    verts->push_back( osg::Vec3(width,0,0));
    verts->push_back( osg::Vec3(width,height,0));
    verts->push_back( osg::Vec3(0,height,0));
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back( backgroundColor );
    geometry->setColorArray( colors );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

    geometry->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );

    return geode;

}

osg::Camera* createHud(double width, double height)
{
    osg::Camera* hud = new osg::Camera;
    hud->setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));    
    hud->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hud->setViewMatrix(osg::Matrix::identity());    
    hud->setClearMask(GL_DEPTH_BUFFER_BIT);
    hud->setRenderOrder(osg::Camera::POST_RENDER);    
    hud->setAllowEventFocus(false);
    hud->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    hud->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF);
    hud->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON);

    return hud;
}

osg::Node* createProfileNode(TerrainProfile& profile, double graphWidth, double graphHeight, const osg::Vec4& color)
{    
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( profile.getNumElevations() );
    geom->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back( color );
    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );


    double minElevation, maxElevation;
    profile.getElevationRanges( minElevation, maxElevation );
    double elevationRange = maxElevation - minElevation;
    
    double totalDistance = profile.getTotalDistance();
    

    for (unsigned int i = 0; i < profile.getNumElevations(); i++)
    {
        double distance = profile.getDistance( i );
        double elevation = profile.getElevation( i );

        double x = (distance / totalDistance) * graphWidth;
        double y = ( (elevation - minElevation) / elevationRange) * graphHeight;
        verts->push_back( osg::Vec3(x, y, 0 ) );
    }
   
    geom->addPrimitiveSet( new osg::DrawArrays( GL_LINE_STRIP, 0, verts->size()) );
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geom );
    return geode; 
}

class TerrainProfileEventHandler : public osgGA::GUIEventHandler
{
public:
    TerrainProfileEventHandler(osgEarth::MapNode* mapNode, osg::Group* root, osg::Group* profileNode, double graphWidth, double graphHeight):
      _mapNode( mapNode ),
      _root( root ),
      _profileNode( profileNode ),
      _startValid( false ),
      _graphWidth( graphWidth ),
      _graphHeight( graphHeight )
    {        
    }


    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == ea.PUSH && ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        {
            osg::Vec3d world;
            if ( _mapNode->getTerrain()->getWorldCoordsUnderMouse( aa.asView(), ea.getX(), ea.getY(), world ))
            {
                osg::Vec3d map;
                _mapNode->getMap()->worldPointToMapPoint( world, map );

                if (!_startValid)
                {
                    _startValid = true;
                    _start = map;
                    if (_featureNode.valid())
                    {
                        _root->removeChild( _featureNode.get() );
                        _featureNode = 0;
                    }
                }
                else
                {
                    _end = map;
                    compute();
                    _startValid = false;                    
                }
            }        
        }
        return false;
    }

    void compute()
    {
        computeTerrainProfile( _mapNode, _start.x(), _start.y(), _end.x(), _end.y(), 100, _profile);
        _profileNode->removeChildren(0, _profileNode->getNumChildren());
        _profileNode->addChild( createProfileNode( _profile, _graphWidth, _graphHeight, osg::Vec4f(1,0,0,1)) );

        if (_featureNode.valid())
        {
            _root->removeChild( _featureNode.get() );
            _featureNode = 0;
        }

        LineString* line = new LineString();
        line->push_back( _start );
        line->push_back( _end );
        Feature* feature = new Feature();
        feature->setGeometry( line );
        feature->geoInterp() = GEOINTERP_GREAT_CIRCLE;    

        //Define a style for the line
        Style style;
        LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
        ls->stroke()->color() = Color::Yellow;
        ls->stroke()->width() = 2.0f;
        ls->tessellation() = 20;

        style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

        feature->style() = style;

        _featureNode = new FeatureNode( _mapNode, feature );
         //Disable lighting
        _featureNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        _root->addChild( _featureNode.get() );

    }




    osgEarth::MapNode* _mapNode;
    osg::Group* _profileNode;
    osg::Group* _root;
    TerrainProfile _profile;
    osg::ref_ptr< FeatureNode > _featureNode;
    bool _startValid;
    osg::Vec3d _start;
    osg::Vec3d _end;  
    double _graphWidth;
    double _graphHeight;
};




int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_NOTICE << "Unable to load earth model" << std::endl;
        return 1;
    }

    osg::Group* root = new osg::Group();

    osgEarth::MapNode * mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if (!mapNode)
    {
        OE_NOTICE << "Could not find MapNode " << std::endl;
        return 1;
    }

    osgEarth::Util::EarthManipulator* manip = new EarthManipulator();    
    viewer.setCameraManipulator( manip );
    
    root->addChild( earthNode );    

    double backgroundWidth = 500;
    double backgroundHeight = 500;

    double graphWidth = 200;
    double graphHeight = 100;

    //Add the hud
    osg::Camera* hud = createHud( backgroundWidth, backgroundHeight );
    root->addChild( hud );

    hud->addChild( createBackground( graphWidth, graphHeight, osg::Vec4(0,0,0,0.5)) );

    osg::Group* profileNode = new osg::Group;
    hud->addChild( profileNode );
       
    
    
    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );

    viewer.addEventHandler( new TerrainProfileEventHandler( mapNode, root, profileNode, graphWidth, graphHeight ) );



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

    return viewer.run();
}
