/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>

#include <osgEarth/TileIndex>

#include <osgEarth/OgrUtils>
#include <osgEarth/OGRFeatureSource>

#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Contrib;
using namespace std;

TileIndex::TileIndex()
{
}

TileIndex::~TileIndex()
{

}

TileIndex*
TileIndex::load(const std::string& filename)
{        
    if (!osgDB::fileExists( filename ) )
    {
        return 0;
    }

    //Load up an index file
    osg::ref_ptr<OGRFeatureSource> features = new OGRFeatureSource();
    features->setURL(filename);
    features->setBuildSpatialIndex(true);
    features->setOpenWrite(true);

    if (features->open().isError())
    {
        OE_NOTICE << "Can't load " << filename << std::endl;
        return 0;
    }

    TileIndex* index = new TileIndex();
    index->_features = features.get();
    index->_filename = filename;
    return index;
}

TileIndex*
TileIndex::create( const std::string& filename, const osgEarth::SpatialReference* srs )
{
    // Make sure the registry is loaded since that is where the OGR/GDAL registration happens
    osgEarth::Registry::instance();

    OGRSFDriverH driver = OGRGetDriverByName( "ESRI Shapefile" );    

    //Create the datasource itself.
    OGRDataSourceH dataSource = OGR_Dr_CreateDataSource( driver, filename.c_str(), NULL );
    if (dataSource == NULL)
    {
        OE_WARN << "failed to create " << filename.c_str() << std::endl;
        return 0;
    }

    //Create the layer
    OGRLayerH layer = OGR_DS_CreateLayer( dataSource, "index", (OGRSpatialReferenceH)srs->getHandle(), wkbPolygon, NULL );

    //Create the attribute name to use for the filename
    OGRFieldDefnH  field = OGR_Fld_Create("location", OFTString);
    OGR_L_CreateField( layer, field, TRUE);

    OGR_DS_Destroy( dataSource );

    return load( filename );
}


void
TileIndex::getFiles(const osgEarth::GeoExtent& extent, std::vector< std::string >& files)
{            
    files.clear();
    osgEarth::Query query;    

    GeoExtent transformed = extent.transform( _features->getFeatureProfile()->getSRS() );
    query.bounds() = transformed.bounds();
    osg::ref_ptr< osgEarth::FeatureCursor> cursor = _features->createFeatureCursor(query);

    while (cursor->hasMore())
    {
        osg::ref_ptr< osgEarth::Feature> feature = cursor->nextFeature();
        if (feature.valid())
        {
            std::string location = getFullPath(_filename, feature->getString("location"));
            files.push_back( location );
        }
    }    
}

bool TileIndex::add( const std::string& filename, const GeoExtent& extent )
{       
    osg::ref_ptr< Polygon > polygon = new Polygon();
    polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMin(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMax(), extent.bounds().yMin(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMax(), extent.bounds().yMax(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMax(), 0) );
    polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMin(), 0) );
   
    osg::ref_ptr< Feature > feature = new Feature( polygon.get(), extent.getSRS()  );
    feature->set("location", filename );
    
    const SpatialReference* wgs84 = SpatialReference::create("epsg:4326");
    feature->transform( wgs84 );

    return _features->insertFeature( feature.get() );    
    return true;
}
