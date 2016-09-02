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

#include <osg/Notify>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

#include <osgEarthUtil/TFSPackager>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;


int
usage( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgearth_tfs [options] filename" << std::endl
        << std::endl
        << "    filename           ; Shapefile (or other feature source data file)" << std::endl
        << "    --first-level      ; The first level where features will be added to the quadtree" << std::endl
        << "    --max-level        ; The maximum level of the feature quadtree" << std::endl
        << "    --max-features     ; The maximum number of features per tile" << std::endl
        << "    --grid             ; Generate a single level grid with the specified resolution.  Default units are meters. (ex. 50, 100km, 200mi)" << std::endl
        << "    --out              ; The destination directory" << std::endl
        << "    --layer            ; The name of the layer to be written to the metadata document" << std::endl
        << "    --description      ; The abstract/description of the layer to be written to the metadata document" << std::endl
        << "    --expression       ; The expression to run on the feature source, specific to the feature source" << std::endl
        << "    --order-by         ; Sort the features, if not already included in the expression. Append DESC for descending order!" << std::endl
        << "    --crop             ; Crops features instead of doing a centroid check.  Features can be added to multiple tiles when cropping is enabled" << std::endl
        << "    --dest-srs         ; The destination SRS string in any format osgEarth can understand (wkt, proj4, epsg).  If none is specified the source data SRS will be used" << std::endl
        << "    --bounds minx miny maxx maxy ; The bounding box to use as Level 0.  Feature extent will be used by default" << std::endl
        << std::endl;

    return -1;
}



int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if (argc < 2)
    {
        return usage("");
    }
    
    //The first level
    unsigned int firstLevel = 0;
    while (arguments.read("--first-level", firstLevel));

    //The max level
    unsigned int maxLevel = 6;
    while (arguments.read("--max-level", maxLevel));

    unsigned int maxFeatures = 300;
    while (arguments.read("--max-features", maxFeatures));    

    //The destination directory
    std::string destination = "out";
    while (arguments.read("--out", destination));

    //The name of the layer
    std::string layer = "layer";
    while (arguments.read("--layer", layer));

    //The description of the layer
    std::string description = "";
    while (arguments.read("--description", description));

    std::string queryExpression = "";
    while (arguments.read("--expression", queryExpression));

    std::string queryOrderBy = "";
    while (arguments.read("--order-by", queryOrderBy));

    CropFilter::Method cropMethod = CropFilter::METHOD_CENTROID;
    if (arguments.read("--crop"))
    {
        cropMethod = CropFilter::METHOD_CROPPING;
    }

    std::string destSRS;
    while(arguments.read("--dest-srs", destSRS));

    std::string grid;
    float gridSizeMeters = -1.0f;
    while (arguments.read("--grid", grid));
    if (!grid.empty())
    {
        float gridSize;
        Units units;
        if ( Units::parse(grid, gridSize, units, Units::METERS) ) {
             gridSizeMeters = Distance(gridSize, units).as(Units::METERS);
        }
    }

    // Custom bounding box
    Bounds bounds;
    double xmin=DBL_MAX, ymin=DBL_MAX, xmax=DBL_MIN, ymax=DBL_MIN;
    while (arguments.read("--bounds", xmin, ymin, xmax, ymax ))
    {
        bounds.xMin() = xmin;
        bounds.yMin() = ymin;
        bounds.xMax() = xmax;
        bounds.yMax() = ymax;
    }
    
    std::string filename;

    //Get the first argument that is not an option
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filename  = arguments[ pos ];
            break;
        }
    }

    if (filename.empty())
    {
        return usage( "Please provide a filename" );
    }

    //Open the feature source
    OGRFeatureOptions featureOpt;
    featureOpt.url() = filename;

    osg::ref_ptr< FeatureSource > features = FeatureSourceFactory::create( featureOpt );
    if (!features.valid())
    {
        OE_NOTICE << "Failed to open " << filename << std::endl;
        return 1;
    }

    Status s = features->open();
    if (s.isError())
    {
        OE_NOTICE << s.message() << ": " << filename << std::endl;
        return 1;
    }

    std::string method = cropMethod == CropFilter::METHOD_CENTROID ? "Centroid" : "Cropping";

    Query query;
    if (!queryExpression.empty())
    {
        query.expression() = queryExpression;
    }

    if (!queryOrderBy.empty())
    {
        query.orderby() = queryOrderBy;
    }    

    osg::Timer_t startTime = osg::Timer::instance()->tick();

    // Use the feature extent by default.
    GeoExtent ext = features->getFeatureProfile()->getExtent();
    if (!destSRS.empty())
    {
        ext = ext.transform(osgEarth::SpatialReference::create( destSRS ));
    }

    if (bounds.isValid())
    {
        // If a custom bounds was specified use that instead.
        ext = GeoExtent(osgEarth::SpatialReference::create( destSRS ), bounds);
    }

    if (gridSizeMeters > 0.0)
    {
        // Compute the level in the destination profile that is closest to the desired resolution.
        osg::ref_ptr< const osgEarth::Profile > profile = osgEarth::Profile::create(ext.getSRS(), ext.xMin(), ext.yMin(), ext.xMax(), ext.yMax(), 1, 1);
        float res = gridSizeMeters;
        // Estimate meters to degrees if necessary
        if (profile->getSRS()->isGeographic())
        {
            res /= 111000.0;
        }
        unsigned int level = profile->getLevelOfDetailForHorizResolution(res, 1.0);
        firstLevel = level;
        maxLevel = level;
        OE_NOTICE << "Computed grid level " << level << " from grid resolution of " << gridSizeMeters << "m" << std::endl;
    }

    OE_NOTICE << "Processing " << filename << std::endl
        << "  FirstLevel=" << firstLevel << std::endl
        << "  MaxLevel=" << maxLevel << std::endl
        << "  MaxFeatures=" << maxFeatures << std::endl
        << "  Destination=" << destination << std::endl
        << "  Layer=" << layer << std::endl
        << "  Description=" << description << std::endl
        << "  Expression=" << queryExpression << std::endl
        << "  OrderBy=" << queryOrderBy << std::endl
        << "  Method= " << method << std::endl
        << "  DestSRS= " << destSRS << std::endl
        << std::endl;

    //buildTFS( features.get(), firstLevel, maxLevel, maxFeatures, destination, layer, description, query, cropMethod);
    TFSPackager packager;
    packager.setFirstLevel( firstLevel );
    packager.setMaxLevel( maxLevel );
    packager.setMaxFeatures( maxFeatures );
    packager.setQuery( query );
    packager.setMethod( cropMethod );    
    packager.setDestSRS( destSRS );
    packager.setLod0Extent(ext);

    packager.package( features, destination, layer, description );
    osg::Timer_t endTime = osg::Timer::instance()->tick();
    OE_NOTICE << "Completed in " << osg::Timer::instance()->delta_s( startTime, endTime ) << " s " << std::endl;

    return 0;
}
