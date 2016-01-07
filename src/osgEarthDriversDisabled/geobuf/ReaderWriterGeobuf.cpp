#include <osg/Notify>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>
#include <osgEarth/JsonUtils>
#include <osgEarthFeatures/FeatureListSource>

#include "geobuf.pb.h"

using namespace osgEarth;
using namespace osgEarth::Features;

class GeoBufReaderWriter: public osgDB::ReaderWriter
{
    public:
        GeoBufReaderWriter()
        {
            supportsExtension("geobuf","GeoBuf");
        }

        virtual const char* className() { return "GeoJSON Feature Reader"; }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension,"geobuf");
        }

        virtual ReadResult readObject(const std::string& fileName,const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(fileName)) )
                return ReadResult::FILE_NOT_HANDLED;

            if (osgDB::fileExists(fileName) && (osgDB::fileType(fileName) == osgDB::REGULAR_FILE))
            {
                std::ifstream in( fileName.c_str() );
                return readObject(in, options);
            }
            return ReadResult::ERROR_IN_READING_FILE;
        }

        virtual ReadResult readObject(std::istream& fin,const Options* options) const
        {
            osg::ref_ptr< FeatureListSource > featureSource = new FeatureListSource();

            if (Feature::featuresFromGeoJSON( fin, featureSource->getFeatures()))
            {
                return featureSource.release();
            }
            return ReadResult::ERROR_IN_READING_FILE;
        }

        virtual WriteResult writeObject(const osg::Object& object, const std::string& fileName, const osgDB::Options* options ) const
        {
            if ( !acceptsExtension( osgDB::getFileExtension(fileName) ) )
                return WriteResult::FILE_NOT_HANDLED;

            std::ofstream out( fileName.c_str());
            if ( out.is_open() )
                return writeObject( object, out, options );

            return WriteResult::ERROR_IN_WRITING_FILE;            
        }

        virtual WriteResult writeObject(const osg::Object& object, std::ostream& out, const osgDB::Options* options ) const
        {

          
            FeatureSource* featureSource = const_cast<FeatureSource*>(dynamic_cast<const FeatureSource*>(&object));
            if (!featureSource)
            {
                return WriteResult::ERROR_IN_WRITING_FILE; // i.e., no MapNode found in the graph.
            }

            Data data;                

            osgEarth::Features::FeatureList features;
            osg::ref_ptr< FeatureCursor > cursor = featureSource->createFeatureCursor();
            while (cursor.valid() && cursor->hasMore())
            {
                Data_Feature* feature = data.mutable_feature_collection()->mutable_features()->Add();
                feature->mutable_geometry()->set_type(Data_Geometry_Type::Data_Geometry_Type_POLYGON);
                feature->mutable_geometry()->add_coords(62.0);
            }
            
            return WriteResult::FILE_SAVED;
        }
};

REGISTER_OSGPLUGIN(geobuf, GeoBufReaderWriter)
