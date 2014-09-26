/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include "BoundaryUtil"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <osg/Notify>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/Optimizer>


int usage( char** argv, const std::string& msg )
{
  OSG_NOTICE << msg << "\n\n";
  OSG_NOTICE 
    << "osgEarth Boundary Generator Tool\n\n"
    << "Generates boundary geometry that you can use with an osgEarth <mask> layer in order\n"
    << "to stitch an external model into the terrain.\n\n"
    << "USAGE: " << argv[0] << " [options] model_file\n"
    << "           --out <file_name>    : output file for boundary geometry (default is boundary.txt)\n"
    << "           --tolerance <meters> : tolerance for combining similar verts along a boundary (default = 0.005)\n"
    << "           --precision <n>      : output precision of boundary coords (default=12)\n"
    << "           --no-geocentric      : skip geocentric reprojection (for flat databases)\n"
    << "           --convex-hull        : calculate a convex hull instead of a full boundary\n"
    << "           --verbose            : print progress to console\n"
    << "           --view               : show result in 3D window\n"
    << std::endl;

  
  return -1;
}

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    std::string outFile;
    if (!arguments.read("--out", outFile))
      outFile = "boundary.txt";

    double tolerance;
    if (arguments.read("--tolerance", tolerance))
        BoundaryUtil::setTolerance(tolerance);

    int precision = 12;
    arguments.read("--precision", precision);

    bool geocentric = !arguments.read("--no-geocentric");
    bool verbose = arguments.read("--verbose");
    bool convexOnly = arguments.read("--convex-hull");
    bool view = arguments.read("--view");

    osg::Node* modelNode = osgDB::readNodeFiles( arguments );
    if (!modelNode)
        return usage( argv, "Unable to load model." );

    osg::ref_ptr<osg::Vec3dArray> hull = BoundaryUtil::getBoundary(modelNode, geocentric, convexOnly);

    if ( !outFile.empty() )
    {
      if (hull.valid())
      {
        if (verbose)
          std::cout << std::endl << "hull.size() == " << hull->size() << std::endl;

        std::ofstream outStream;
        outStream.open(outFile.c_str());
        if (outStream.fail())
        {
          std::cout << "Unable to open " << outFile << " for writing." << std::endl;
        }
        else
        {
          outStream << "POLYGON((";

          osg::ref_ptr<osg::EllipsoidModel> em = new osg::EllipsoidModel();  

          for (int i=0; i < (int)hull->size(); i++)
          {
            const osg::Vec3d& vert = (*hull.get())[i];

            double lat, lon, height;
            em->convertXYZToLatLongHeight( vert.x(), vert.y(), vert.z(), lat, lon, height );
            lat = osg::RadiansToDegrees(lat);
            lon = osg::RadiansToDegrees(lon);

            if (verbose)
              std::cout << "  hull[" << i << "] == " << lon << ", " << lat << ", " << height << std::endl;

            outStream
                //<< std::fixed //std::setiosflags(std::ios_base::fixed)
                << std::setprecision(precision)
                << (i > 0 ? ", " : "") << lon << " " << lat << " " << height;
          }

          outStream << "))";
          outStream.close();

          std::cout << "Boundary data written to " << outFile << std::endl;
          
          if (!convexOnly)
            std::cout << "Boundary: " << (BoundaryUtil::simpleBoundaryTest(*hull) ? "VALID" : "INVALID") << std::endl;
        }
      }
      else
      {
        std::cout << "Could not find boundary." << std::endl;
      }
    }

    if (view)
    {
      osgViewer::Viewer viewer(arguments);

      osg::BoundingSphered bs;
      for( osg::Vec3dArray::iterator i = hull->begin(); i != hull->end(); ++i )
          bs.expandBy( *i );

      osg::MatrixTransform* xform = new osg::MatrixTransform();
      xform->setMatrix( osg::Matrix::translate( bs.center() ) );

      osg::Vec3Array* drawHull = new osg::Vec3Array();
      for( osg::Vec3dArray::iterator i = hull->begin(); i != hull->end(); ++i )
          drawHull->push_back( (*i) - bs.center() );

      osg::Group* root = new osg::Group();
      root->addChild( modelNode );
      root->addChild( xform );
      modelNode->getOrCreateStateSet()->setAttributeAndModes( new osg::PolygonOffset(1,1), 1 );
      
      osg::Geometry* boundaryGeometry = new osg::Geometry();
      boundaryGeometry->setVertexArray( drawHull );

      osg::Vec4Array* colors = new osg::Vec4Array;
      colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
      boundaryGeometry->setColorArray(colors);
      boundaryGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
      osg::StateSet* ss = boundaryGeometry->getOrCreateStateSet();
      ss->setAttributeAndModes( new osg::LineWidth(1.0), 1 );
      ss->setAttributeAndModes( new osg::Point(3.5), 1 );
      ss->setMode( GL_LIGHTING, 0 );

      boundaryGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,drawHull->size()));
      boundaryGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,drawHull->size()));

      osg::Geode* boundaryGeode = new osg::Geode();
      boundaryGeode->addDrawable(boundaryGeometry);
      xform->addChild(boundaryGeode);

      viewer.setSceneData( root );

      // add some stock OSG handlers:
      viewer.addEventHandler(new osgViewer::StatsHandler());
      viewer.addEventHandler(new osgViewer::WindowSizeHandler());
      viewer.addEventHandler(new osgViewer::ThreadingHandler());
      viewer.addEventHandler(new osgViewer::LODScaleHandler());
      viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
      viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

      return viewer.run();
    }

    return 0;
}
