[1mdiff --git a/CMakeLists.txt b/CMakeLists.txt[m
[1mindex dbb4adbe9..5efca139e 100755[m
[1m--- a/CMakeLists.txt[m
[1m+++ b/CMakeLists.txt[m
[36m@@ -23,7 +23,7 @@[m [mset(OSGEARTH_PATCH_VERSION 3)[m
 set(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})[m
 [m
 # Increment this each time the ABI changes[m
[31m-set(OSGEARTH_SOVERSION 178)[m
[32m+[m[32mset(OSGEARTH_SOVERSION 177)[m
 [m
 # Require C++14, don't fall back, and don't use compiler-specific extensions:[m
 set(CMAKE_CXX_STANDARD 17)[m
[1mdiff --git a/src/applications/osgearth_annotation/osgearth_annotation.cpp b/src/applications/osgearth_annotation/osgearth_annotation.cpp[m
[1mindex 9284c96f0..49c34bf79 100644[m
[1m--- a/src/applications/osgearth_annotation/osgearth_annotation.cpp[m
[1m+++ b/src/applications/osgearth_annotation/osgearth_annotation.cpp[m
[36m@@ -128,7 +128,7 @@[m [mmain(int argc, char** argv)[m
                     traverse(n, nv);[m
             }[m
         };[m
[31m-        Geometry* geom = new osgEarth::Polygon();[m
[32m+[m[32m        Geometry* geom = new Polygon();[m
         geom->push_back( osg::Vec3d(0,   40, 0) );[m
         geom->push_back( osg::Vec3d(-60, 40, 0) );[m
         geom->push_back( osg::Vec3d(-60, 60, 0) );[m
[36m@@ -158,7 +158,7 @@[m [mmain(int argc, char** argv)[m
 [m
     // another rhumb box that crosses the antimeridian[m
     {[m
[31m-        Geometry* geom = new osgEarth::Polygon();[m
[32m+[m[32m        Geometry* geom = new Polygon();[m
         geom->push_back( -160., -30. );[m
         geom->push_back(  150., -20. );[m
         geom->push_back(  160., -45. );[m
[36m@@ -315,7 +315,7 @@[m [mmain(int argc, char** argv)[m
     // FeatureNode, where you create a geographic geometry and use it as an[m
     // annotation.[m
     {[m
[31m-        Geometry* utah = new osgEarth::Polygon();[m
[32m+[m[32m        Geometry* utah = new Polygon();[m
         utah->push_back( -114.052, 37.0   );[m
         utah->push_back( -109.054, 37.0   );[m
         utah->push_back( -109.054, 41.0   );[m
[1mdiff --git a/src/applications/osgearth_imgui/osgearth_imgui.cpp b/src/applications/osgearth_imgui/osgearth_imgui.cpp[m
[1mindex 87f3a7820..2907295a6 100644[m
[1m--- a/src/applications/osgearth_imgui/osgearth_imgui.cpp[m
[1m+++ b/src/applications/osgearth_imgui/osgearth_imgui.cpp[m
[36m@@ -23,7 +23,6 @@[m
 #include <osgEarthImGui/AnnotationsGUI>[m
 #include <osgEarthImGui/PickerGUI>[m
 #include <osgEarthImGui/OpenEarthFileGUI>[m
[31m-#include <osgEarthImGui/ResourceLibraryGUI>[m
 [m
 #ifdef OSGEARTH_HAVE_GEOCODER[m
 #include <osgEarthImGui/SearchGUI>[m
[36m@@ -99,7 +98,6 @@[m [mmain(int argc, char** argv)[m
         ui->add("Tools", new LayersGUI());[m
         ui->add("Tools", new PickerGUI());[m
         ui->add("Tools", new RenderingGUI());[m
[31m-        ui->add("Tools", new ResourceLibraryGUI());[m
         ui->add("Tools", new SceneGraphGUI());[m
 #ifdef OSGEARTH_HAVE_GEOCODER[m
         ui->add("Tools", new SearchGUI());[m
[1mdiff --git a/src/applications/osgearth_map/osgearth_map.cpp b/src/applications/osgearth_map/osgearth_map.cpp[m
[1mindex d7dcf6928..b4ee2dd2a 100644[m
[1m--- a/src/applications/osgearth_map/osgearth_map.cpp[m
[1m+++ b/src/applications/osgearth_map/osgearth_map.cpp[m
[36m@@ -165,7 +165,7 @@[m [mmain(int argc, char** argv)[m
     map->addLayer(compElev);[m
 [m
     // Terrain Constraint Layer:[m
[31m-    osgEarth::Polygon* maskGeom = new osgEarth::Polygon();[m
[32m+[m[32m    Polygon* maskGeom = new Polygon();[m
     maskGeom->push_back(osg::Vec3d(-111.0466, 42.0015, 0));[m
     maskGeom->push_back(osg::Vec3d(-111.0467, 40.9979, 0));[m
     maskGeom->push_back(osg::Vec3d(-109.0501, 41.0007, 0));[m
[1mdiff --git a/src/applications/osgearth_server/osgearth_server.cpp b/src/applications/osgearth_server/osgearth_server.cpp[m
[1mindex c6ba55568..96b717895 100644[m
[1m--- a/src/applications/osgearth_server/osgearth_server.cpp[m
[1m+++ b/src/applications/osgearth_server/osgearth_server.cpp[m
[36m@@ -121,7 +121,7 @@[m [mmain(int argc, char** argv)[m
             << "/" << x[m
             << "/" << y << std::endl;[m
 [m
[31m-        osg::ref_ptr<ElevationTile> elevTex;[m
[32m+[m[32m        osg::ref_ptr<ElevationTexture> elevTex;[m
         if (mapNode->getMap()->getElevationPool()->getTile(osgEarth::TileKey(std::stoi(z), std::stoi(x), std::stoi(y), mapNode->getMap()->getProfile()), false, elevTex, nullptr, nullptr))[m
         {[m
             std::string content = osgEarth::GDAL::heightFieldToTiff(elevTex->getHeightField());[m
[1mdiff --git a/src/applications/osgearth_tests/FeatureTests.cpp b/src/applications/osgearth_tests/FeatureTests.cpp[m
[1mindex 9d64df365..21417ae86 100644[m
[1m--- a/src/applications/osgearth_tests/FeatureTests.cpp[m
[1m+++ b/src/applications/osgearth_tests/FeatureTests.cpp[m
[36m@@ -99,7 +99,7 @@[m [mTEST_CASE("Geometry::crop a polygon to another polygon")[m
 [m
     Ring clip(&boundary);[m
 [m
[31m-    osgEarth::Polygon poly(&input);[m
[32m+[m[32m    Polygon poly(&input);[m
     auto* result = poly.crop(&clip);[m
 [m
     REQUIRE(result);[m
[36m@@ -112,7 +112,7 @@[m [mTEST_CASE("Geometry::crop a polygon and break it into 2 polygons")[m
 {[m
     // crop a polygon resulting in two output polygons:[m
     Vec input_vec = { {0,0,0}, {10,0,0}, {10,10,0}, {0,10,0}, {0,8,0}, {6,8,0}, {6,2,0}, {0,2,0}, {0,0,0} };[m
[31m-    osgEarth::Polygon input(&input_vec);[m
[32m+[m[32m    Polygon input(&input_vec);[m
     Vec boundary = { {5,-100,0}, {5,100,0}, {-100, 100, 0}, {-100, -100, 0} };[m
     Ring clip(&boundary);[m
 [m
[1mdiff --git a/src/osgEarth/BuildGeometryFilter.cpp b/src/osgEarth/BuildGeometryFilter.cpp[m
[1mindex 3993fce18..41f932ac4 100644[m
[1m--- a/src/osgEarth/BuildGeometryFilter.cpp[m
[1m+++ b/src/osgEarth/BuildGeometryFilter.cpp[m
[36m@@ -1023,7 +1023,7 @@[m [mnamespace[m
             z += geometry->at(i).z();[m
         z /= geometry->size();[m
 [m
[31m-        osgEarth::Polygon boundary;[m
[32m+[m[32m        Polygon boundary;[m
         boundary.resize(4);[m
 [m
         for(int x=0; x<(int)numCols; ++x)[m
[1mdiff --git a/src/osgEarth/Containers b/src/osgEarth/Containers[m
[1mindex 44c7b0869..05d9acdb0 100644[m
[1m--- a/src/osgEarth/Containers[m
[1m+++ b/src/osgEarth/Containers[m
[36m@@ -225,7 +225,7 @@[m [mnamespace osgEarth { namespace Util[m
     public:[m
         using ValueType = typename std::optional<V>;[m
         mutable int hits = 0;[m
[31m-        mutable int get_count = 0;[m
[32m+[m[32m        mutable int gets = 0;[m
 [m
         LRUCache(unsigned cap) : capacity(std::max(1u, cap))[m
         {[m
[36m@@ -242,7 +242,7 @@[m [mnamespace osgEarth { namespace Util[m
             cache.clear();[m
             map.clear();[m
             hits = 0;[m
[31m-            get_count = 0;[m
[32m+[m[32m            gets = 0;[m
             capacity = std::max(1u, value);[m
         }[m
 [m
[36m@@ -255,7 +255,7 @@[m [mnamespace osgEarth { namespace Util[m
             if (capacity == 0)[m
                 return {};[m
             std::scoped_lock L(mutex);[m
[31m-            ++get_count;[m
[32m+[m[32m            ++gets;[m
             auto it = map.find(key);[m
             if (it == map.end())[m
                 return {};[m
[36m@@ -324,7 +324,7 @@[m [mnamespace osgEarth { namespace Util[m
             }[m
 [m
             std::scoped_lock L(mutex);[m
[31m-            ++get_count;[m
[32m+[m[32m            ++gets;[m
             auto it = map.find(key);[m
             if (it != map.end()) {[m
                 // Move to back and return value[m
[36m@@ -368,7 +368,7 @@[m [mnamespace osgEarth { namespace Util[m
             std::scoped_lock L(mutex);[m
             cache.clear();[m
             map.clear();[m
[31m-            get_count = 0, hits = 0;[m
[32m+[m[32m            gets = 0, hits = 0;[m
         }[m
     };[m
 [m
[1mdiff --git a/src/osgEarth/ContourMap.glsl b/src/osgEarth/ContourMap.glsl[m
[1mindex 61f156bdf..fa08fc9fe 100644[m
[1m--- a/src/osgEarth/ContourMap.glsl[m
[1m+++ b/src/osgEarth/ContourMap.glsl[m
[36m@@ -74,11 +74,10 @@[m [mvoid oe_contour_fragment( inout vec4 color )[m
         offset = oe_contour_offset;[m
 #endif[m
 [m
[31m-        float encoded = texture(sampler2D(oe_elev_tex), oe_elev_coord).r;[m
[32m+[m[32m        vec2 encoded = texture(sampler2D(oe_elev_tex), oe_elev_coord).rg;[m
         float minh = oe_elev_min_max[0];[m
         float maxh = oe_elev_min_max[1];[m
[31m-        float height = (minh == maxh) ? encoded : mix(minh, maxh, encoded); [m
[31m-            // *mix(minh, maxh, dot(encoded, vec2(65280.0, 255.0)) / 65535.0); // RG8[m
[32m+[m[32m        float height = (minh == maxh) ? encoded.r : mix(minh, maxh, dot(encoded, vec2(65280.0, 255.0)) / 65535.0);[m
 [m
         height -= offset;[m
 [m
[1mdiff --git a/src/osgEarth/Controls.cpp b/src/osgEarth/Controls.cpp[m
[1mindex b29e88aac..3d546a95d 100644[m
[1m--- a/src/osgEarth/Controls.cpp[m
[1m+++ b/src/osgEarth/Controls.cpp[m
[36m@@ -1569,7 +1569,7 @@[m [mRoundedFrame::draw( const ControlContext& cx )[m
             // the corners are.[m
             float buffer = Geometry::hasBufferOperation() ? 10.0f : 0.0f;[m
 [m
[31m-            osg::ref_ptr<Geometry> geom = new osgEarth::Polygon();[m
[32m+[m[32m            osg::ref_ptr<Geometry> geom = new Polygon();[m
             geom->push_back( osg::Vec3d( buffer, buffer, 0 ) );[m
             geom->push_back( osg::Vec3d( _renderSize.x()-1-buffer, buffer, 0 ) );[m
             geom->push_back( osg::Vec3d( _renderSize.x()-1-buffer, _renderSize.y()-1-buffer, 0 ) );[m
[1mdiff --git a/src/osgEarth/CropFilter.cpp b/src/osgEarth/CropFilter.cpp[m
[1mindex 3b6cfceeb..80bbadbe1 100644[m
[1m--- a/src/osgEarth/CropFilter.cpp[m
[1m+++ b/src/osgEarth/CropFilter.cpp[m
[36m@@ -70,7 +70,7 @@[m [mCropFilter::push( FeatureList& input, FilterContext& context )[m
     {[m
 #ifdef OSGEARTH_HAVE_GEOS[m
         // create the intersection polygon:[m
[31m-        osgEarth::Polygon poly;[m
[32m+[m[32m        Polygon poly;[m
         poly.reserve(4);[m
         poly.push_back(osg::Vec3d(extent.xMin(), extent.yMin(), 0));[m
         poly.push_back(osg::Vec3d(extent.xMax(), extent.yMin(), 0));[m
[1mdiff --git a/src/osgEarth/Elevation b/src/osgEarth/Elevation[m
[1mindex fddeede40..7dc83eb2b 100644[m
[1m--- a/src/osgEarth/Elevation[m
[1m+++ b/src/osgEarth/Elevation[m
[36m@@ -19,7 +19,7 @@[m [mnamespace osgEarth[m
     class Map;[m
     class ProgressCallback;[m
 [m
[31m-    extern OSGEARTH_EXPORT osg::Texture* createEmptyElevationTile();[m
[32m+[m[32m    extern OSGEARTH_EXPORT osg::Texture* createEmptyElevationTexture();[m
 [m
     extern OSGEARTH_EXPORT osg::Texture* createEmptyNormalMapTexture();[m
 [m
[36m@@ -53,21 +53,20 @@[m [mnamespace osgEarth[m
         //! be in the SRS used to create the texture.[m
         ElevationSample getElevation(double x, double y) const;[m
 [m
[32m+[m[32m        //! Gets the elevation using normalized [0..1] coordinates.[m
[32m+[m[32m        ElevationSample getElevationUV(double u, double v) const;[m
[32m+[m
         //! Extent of the texture[m
         const GeoExtent& getExtent() const { return _extent; }[m
 [m
         //! Min and Max heights in the texture[m
         const std::pair<float, float> getMaxima() const { return _maxima; }[m
 [m
[31m-        //! Whether the data in this tile represents all native resolution data[m
[31m-        //! (versus a combination of native and fallback data)[m
[31m-        bool allHeightsAtNativeResolution() const { return _allHeightsAtNativeResolution; }[m
[31m-[m
         //! TileKey used to create this elevation grid[m
         const TileKey& getTileKey() const { return _tilekey; }[m
 [m
         //! Elevation data texture[m
[31m-        osg::Texture2D* getElevationTile() const {[m
[32m+[m[32m        osg::Texture2D* getElevationTexture() const {[m
             return _elevationTex.get();[m
         }[m
 [m
[36m@@ -80,8 +79,12 @@[m [mnamespace osgEarth[m
         //! contain a normal map.[m
         osg::Vec3 getNormal(double x, double y) const;[m
 [m
[32m+[m[32m        //! The packed normal at map coordinates, or (0,0,0,0) if the tile does not[m
[32m+[m[32m        //! contain a normal map.[m
[32m+[m[32m        void getPackedNormal(double x, double y, osg::Vec4& packed);[m
[32m+[m
         //! Generates a normal map for this object.[m
[31m-        void generateNormalMap(const Map* map, unsigned size, void* workingSet, ProgressCallback* progress);[m
[32m+[m[32m        void generateNormalMap(const Map* map, void* workingSet, ProgressCallback* progress);[m
 [m
         //! Direct access to the pixel reader[m
         const ImageUtils::PixelReader& reader() const { return _read; }[m
[36m@@ -99,48 +102,25 @@[m [mnamespace osgEarth[m
                 (int)(v * (double)(_read.t() - 1)));[m
         }[m
 [m
[32m+[m[32m        //! Ruggedness value at map coordinates (x,y)[m
[32m+[m[32m        //! (Warning: experimental function; may go away or change without notice.)[m
[32m+[m[32m        //! Note: currently disabled and will always return 0.[m
[32m+[m[32m        inline float getRuggedness(double x, double y) const;[m
[32m+[m
         //! The heightfield that was used to populate this object[m
         const osg::HeightField* getHeightField() const {[m
             return _heightField.get();[m
         }[m
 [m
[31m-        //! Supported elevation encodings[m
[31m-        enum class Encoding { R16, RG8, R32F };[m
[31m-[m
[31m-        //! Which encoding matches the given GL internal format[m
[31m-        static inline Encoding encodingFor(GLenum internalFormat) {[m
[31m-            return internalFormat == GL_R16 ? Encoding::R16 :[m
[31m-                internalFormat == GL_RG8 ? Encoding::RG8 :[m
[31m-                internalFormat == GL_R32F ? Encoding::R32F :[m
[31m-                Encoding::R16;[m
[31m-        }[m
[31m-[m
[31m-        //! Decodes the elevation from a sampled pixel value[m
[31m-        static inline float decodeElevation(const osg::Vec4& encoded, Encoding encoding, float lo, float hi) {[m
[31m-            return[m
[31m-                encoding == Encoding::R16 ? encoded.r() * 1.5259021e-5 * (hi - lo) + lo :[m
[31m-                encoding == Encoding::RG8 ? (encoded.r() * 65280.f + encoded.g() * 255.0) * 1.5259021e-5 * (hi - lo) + lo :[m
[31m-                encoded.r();[m
[31m-        }[m
[31m-[m
[31m-        //! Encoding used in this tile[m
[31m-        inline Encoding encoding() const {[m
[31m-            return _encoding;[m
[31m-        }[m
[31m-[m
         //! Gets the raw elevation floating point value at the normalized location[m
[31m-        inline float getRawElevationUV(double u, double v) const[m
[31m-        {[m
[31m-            return decodeElevation(_read(u, v), _encoding, _maxima.first, _maxima.second);[m
[31m-        }[m
[31m-[m
[31m-        //! Gets the elevation using normalized [0..1] coordinates.[m
[31m-        inline ElevationSample getElevationUV(double u, double v) const[m
[31m-        {[m
[31m-            u = osgEarth::clamp(u, 0.0, 1.0), v = osgEarth::clamp(v, 0.0, 1.0);[m
[31m-            auto h = decodeElevation(_read(u, v, 0, 0), _encoding, _maxima.first, _maxima.second);[m
[31m-            return ElevationSample(Distance(h, Units::METERS), _resolution);[m
[32m+[m[32m        inline float getRawElevationUV(double u, double v) const {[m
[32m+[m[32m            auto temp = _read(u, v, 0, 0);[m
[32m+[m[32m            if (_is16bitEncoded)[m
[32m+[m[32m                return _maxima.first + ((temp.r() * 65280.f + temp.g() * 255.0) * 1.5259021e-5) * (_maxima.second - _maxima.first);[m
[32m+[m[32m            else[m
[32m+[m[32m                return temp.r();[m
         }[m
[32m+[m[41m    [m
 [m
     private:[m
         TileKey _tilekey;[m
[36m@@ -153,10 +133,10 @@[m [mnamespace osgEarth[m
         osg::ref_ptr<osg::Texture2D> _normalTex;[m
         osg::ref_ptr<const osg::HeightField> _heightField;[m
         std::vector<float> _resolutions;[m
[32m+[m[32m        osg::ref_ptr<osg::Image> _ruggedness;[m
[32m+[m[32m        ImageUtils::PixelReader _readRuggedness;[m
         std::mutex _mutex;[m
[31m-        bool _allHeightsAtNativeResolution = true;[m
[31m-[m
[31m-        Encoding _encoding = Encoding::R16;[m
[32m+[m[32m        bool _is16bitEncoded = false;[m
     };[m
 [m
     /**[m
[36m@@ -168,16 +148,15 @@[m [mnamespace osgEarth[m
         osg::Texture2D* createNormalMap([m
             const TileKey& key,[m
             const class Map* map,[m
[31m-            unsigned tileSize,[m
[31m-            bool assumeNativeResolution,[m
             void* workingSet,[m
[32m+[m[32m            osg::Image* ruggedness,[m
             ProgressCallback* progress);[m
 [m
         //! Packs a 3-vec normal into RG (octohedral compression)[m
[31m-        inline static osg::Vec4 pack(const osg::Vec3& normal);[m
[32m+[m[32m        static void pack(const osg::Vec3& normal, osg::Vec4& packed);[m
 [m
         //! Unpacks the RG packed normal into a 3-vec.[m
[31m-        inline static osg::Vec3 unpack(const osg::Vec4& packed);[m
[32m+[m[32m        static void unpack(const osg::Vec4& packed, osg::Vec3& normal);[m
     };[m
 [m
     //! Revisioned key for elevation lookups (internal)[m
[36m@@ -208,39 +187,6 @@[m [mnamespace osgEarth[m
             }[m
         };[m
     }[m
[31m-[m
[31m-[m
[31m-    inline osg::Vec4 NormalMapGenerator::pack(const osg::Vec3& n)[m
[31m-    {[m
[31m-        osg::Vec4 p;[m
[31m-        // octohodreal normal packing[m
[31m-        float d = 1.0 / (fabs(n.x()) + fabs(n.y()) + fabs(n.z()));[m
[31m-        p.x() = n.x() * d;[m
[31m-        p.y() = n.y() * d;[m
[31m-[m
[31m-        if (n.z() < 0.0)[m
[31m-        {[m
[31m-            p.x() = (1.0 - fabs(p.y())) * (p.x() >= 0.0 ? 1.0 : -1.0);[m
[31m-            p.y() = (1.0 - fabs(p.x())) * (p.y() >= 0.0 ? 1.0 : -1.0);[m
[31m-        }[m
[31m-[m
[31m-        p.x() = 0.5f * (p.x() + 1.0f);[m
[31m-        p.y() = 0.5f * (p.y() + 1.0f);[m
[31m-        return p;[m
[31m-    }[m
[31m-[m
[31m-    inline osg::Vec3 NormalMapGenerator::unpack(const osg::Vec4& packed)[m
[31m-    {[m
[31m-        osg::Vec3 normal;[m
[31m-        normal.x() = packed.x() * 2.0 - 1.0;[m
[31m-        normal.y() = packed.y() * 2.0 - 1.0;[m
[31m-        normal.z() = 1.0 - fabs(normal.x()) - fabs(normal.y());[m
[31m-        float t = clamp(-normal.z(), 0.0f, 1.0f);[m
[31m-        normal.x() += (normal.x() > 0) ? -t : t;[m
[31m-        normal.y() += (normal.y() > 0) ? -t : t;[m
[31m-        normal.normalize();[m
[31m-        return normal;[m
[31m-    }[m
 }[m
 [m
 namespace std {[m
[1mdiff --git a/src/osgEarth/Elevation.cpp b/src/osgEarth/Elevation.cpp[m
[1mindex bbf029a34..8dfe1e986 100644[m
[1m--- a/src/osgEarth/Elevation.cpp[m
[1m+++ b/src/osgEarth/Elevation.cpp[m
[36m@@ -7,23 +7,22 @@[m
 #include <osgEarth/Registry>[m
 #include <osgEarth/Map>[m
 #include <osgEarth/Progress>[m
[31m-#include <osgEarth/Math>[m
[31m-#include <unordered_map>[m
[31m-#include <array>[m
[32m+[m[32m#include <osgEarth/Metrics>[m
 [m
 // for OSGEARTH_USE_16BIT_ELEVATION_TEXTURES[m
 #include <osgEarth/BuildConfig>[m
 [m
 using namespace osgEarth;[m
 [m
[31m-// NOTE: if you change this, don't forget to change the decoding in the SDK shaders as well![m
[32m+[m[32m// if defined, elevation textures are encoded as 2 8-bit channels[m
[32m+[m[32m// with the high order byte in RED and the low order byte in GREEN.[m
[32m+[m[32m// the encoded value maps to [0..1] where 0 corresponds to the minimum height[m
[32m+[m[32m// and 1 corresponds to the maximum height as encoded in the tile.[m
[32m+[m
 #ifdef OSGEARTH_USE_16BIT_ELEVATION_TEXTURES[m
[31m-    //#define ELEV_PIXEL_FORMAT GL_RG[m
[31m-    //#define ELEV_INTERNAL_FORMAT GL_RG8[m
[31m-    //#define ELEV_DATA_TYPE GL_UNSIGNED_BYTE[m
[31m-    #define ELEV_PIXEL_FORMAT GL_RED[m
[31m-    #define ELEV_INTERNAL_FORMAT GL_R16[m
[31m-    #define ELEV_DATA_TYPE GL_UNSIGNED_SHORT[m
[32m+[m[32m    #define ELEV_PIXEL_FORMAT GL_RG[m
[32m+[m[32m    #define ELEV_INTERNAL_FORMAT GL_RG8[m
[32m+[m[32m    #define ELEV_DATA_TYPE GL_UNSIGNED_BYTE[m
 #else[m
     #define ELEV_PIXEL_FORMAT GL_RED[m
     #define ELEV_INTERNAL_FORMAT GL_R32F[m
[36m@@ -31,7 +30,7 @@[m [musing namespace osgEarth;[m
 #endif[m
 [m
 osg::Texture*[m
[31m-osgEarth::createEmptyElevationTile()[m
[32m+[m[32mosgEarth::createEmptyElevationTexture()[m
 {[m
     osg::Image* image = new osg::Image();[m
     image->allocateImage(1, 1, 1, ELEV_PIXEL_FORMAT, ELEV_DATA_TYPE);[m
[36m@@ -54,9 +53,11 @@[m [mosgEarth::createEmptyNormalMapTexture()[m
     image->allocateImage(1, 1, 1, GL_RG, GL_UNSIGNED_BYTE);[m
     image->setInternalTextureFormat(GL_RG8);[m
     ImageUtils::PixelWriter write(image);[m
[31m-    write(osg::Vec4(0.5f, 0.5f, 0.0f, 0.0f), 0, 0); // BC5 octahedral encoded Z-up[m
[32m+[m[32m    osg::Vec4 packed;[m
[32m+[m[32m    NormalMapGenerator::pack(osg::Vec3(0,0,1), packed);[m
[32m+[m[32m    write(packed, 0, 0);[m
     osg::Texture2D* tex = new osg::Texture2D(image);[m
[31m-    tex->setInternalFormat(image->getInternalTextureFormat());[m
[32m+[m[32m    tex->setInternalFormat(GL_RG8);[m
     tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);[m
     tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);[m
     tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());[m
[36m@@ -64,7 +65,11 @@[m [mosgEarth::createEmptyNormalMapTexture()[m
     return tex;[m
 }[m
 [m
[31m-ElevationTile::ElevationTile(const TileKey& key, const GeoHeightField& in_hf, std::vector<float>&& resolutions) :[m
[32m+[m[32mElevationTile::ElevationTile([m
[32m+[m[32m    const TileKey& key,[m
[32m+[m[32m    const GeoHeightField& in_hf,[m
[32m+[m[32m    std::vector<float>&& resolutions) :[m
[32m+[m
     _tilekey(key),[m
     _extent(in_hf.getExtent()),[m
     _resolutions(std::move(resolutions))[m
[36m@@ -84,70 +89,25 @@[m [mElevationTile::ElevationTile(const TileKey& key, const GeoHeightField& in_hf, st[m
         heights->allocateImage(_heightField->getNumColumns(), _heightField->getNumRows(), 1, ELEV_PIXEL_FORMAT, ELEV_DATA_TYPE);[m
         heights->setInternalTextureFormat(ELEV_INTERNAL_FORMAT);[m
 [m
[31m-        _allHeightsAtNativeResolution = true;[m
[31m-        float nativeRes = _heightField->getYInterval(); // assume square pixels[m
[31m-[m
[31m-        if (heights->getPixelFormat() == GL_RED && heights->getDataType() == GL_UNSIGNED_SHORT)[m
[31m-        {[m
[31m-            // GL_RED/SHORT = 16-bit encoded relative height values (0..1 from min to max)[m
[31m-            auto* data = reinterpret_cast<GLshort*>(heights->data());[m
[31m-            unsigned rp = 0;[m
[31m-            for (auto h : _heightField->getHeightList())[m
[31m-            {[m
[31m-                float t = (h - _maxima.first) / (_maxima.second - _maxima.first);[m
[31m-                *data++ = (GLushort)(65535.0f * t);[m
[31m-[m
[31m-                // in the meantime determine whether this tile is native resolution.[m
[31m-                if (_allHeightsAtNativeResolution && !_resolutions.empty() && !osgEarth::equivalent(_resolutions[rp++], nativeRes))[m
[31m-                {[m
[31m-                    _allHeightsAtNativeResolution = false;[m
[31m-                }[m
[31m-            }[m
[31m-[m
[31m-            _encoding = Encoding::R16;[m
[31m-        }[m
[31m-        else if (heights->getPixelFormat() == GL_RG)[m
[32m+[m[32m        // GL_RG = 16-bit encoded relative height values (0..1 from min to max)[m
[32m+[m[32m        if (heights->getPixelFormat() == GL_RG)[m
         {[m
[31m-            // GL_RG = 16-bit encoded relative height values (0..1 from min to max)[m
             auto* data = heights->data();[m
[31m-            unsigned rp = 0;[m
             for (auto h : _heightField->getHeightList())[m
             {[m
                 float t = (h - _maxima.first) / (_maxima.second - _maxima.first);[m
                 int ti = (int)(65535.0f * t);[m
                 *data++ = (std::int8_t)((ti >> 8) & 0xFF); // high byte[m
                 *data++ = (std::int8_t)(ti & 0xFF); // low byte[m
[31m-[m
[31m-                // in the meantime determine whether this tile is native resolution.[m
[31m-                if (_allHeightsAtNativeResolution && !_resolutions.empty() && !osgEarth::equivalent(_resolutions[rp++], nativeRes))[m
[31m-                {[m
[31m-                    _allHeightsAtNativeResolution = false;[m
[31m-                }[m
             }[m
 [m
[31m-            _encoding = Encoding::RG8;[m
[32m+[m[32m            _is16bitEncoded = true;[m
         }[m
         else // GL_R32F[m
         {[m
             memcpy(heights->data(), _heightField->getHeightList().data(), sizeof(float) * _heightField->getNumRows() * _heightField->getNumColumns());[m
[31m-[m
[31m-            // native res detection[m
[31m-            for (unsigned rp = 0; rp < _resolutions.size(); ++rp)[m
[31m-            {[m
[31m-                if (!osgEarth::equivalent(_resolutions[rp], nativeRes))[m
[31m-                {[m
[31m-                    _allHeightsAtNativeResolution = false;[m
[31m-                    break;[m
[31m-                }[m
[31m-            }[m
[31m-[m
[31m-            _encoding = Encoding::R32F;[m
         }[m
 [m
[31m-        // Can't compress the elevation because it will no longer match up [m
[31m-        // at the tile seams due to the lossy compression of BC5 or BC7.[m
[31m-        //ImageUtils::compressImageInPlace(heights);[m
[31m-[m
         _elevationTex = new osg::Texture2D(heights);[m
         _elevationTex->setName(key.str() + ":elevation");[m
 [m
[36m@@ -167,7 +127,7 @@[m [mElevationTile::ElevationTile(const TileKey& key, const GeoHeightField& in_hf, st[m
         _read.setSampleAsTexture(false);[m
 [m
         _resolution = Distance([m
[31m-            getExtent().height() / ((double)(heights->t() - 1)),[m
[32m+[m[32m            getExtent().height() / ((double)(heights->t()-1)),[m
             getExtent().getSRS()->getUnits());[m
     }[m
 }[m
[36m@@ -181,6 +141,21 @@[m [mElevationTile::getElevation(double x, double y) const[m
     return getElevationUV(u, v);[m
 }[m
 [m
[32m+[m[32mElevationSample[m
[32m+[m[32mElevationTile::getElevationUV(double u, double v) const[m
[32m+[m[32m{[m
[32m+[m[32m    osg::Vec4f sample;[m
[32m+[m[32m    u = clamp(u, 0.0, 1.0), v = clamp(v, 0.0, 1.0);[m
[32m+[m[32m    _read(sample, u, v);[m
[32m+[m
[32m+[m[32m#ifdef OSGEARTH_USE_16BIT_ELEVATION_TEXTURES[m
[32m+[m[32m    float t = (sample.r() * 65280.0f + sample.g() * 255.0) / 65535.0f; // [0..1][m
[32m+[m[32m    sample.r() = _maxima.first + t * (_maxima.second - _maxima.first); // scale to min/max[m
[32m+[m[32m#endif[m
[32m+[m
[32m+[m[32m    return ElevationSample(Distance(sample.r(),Units::METERS), _resolution);[m
[32m+[m[32m}[m
[32m+[m
 osg::Vec3[m
 ElevationTile::getNormal(double x, double y) const[m
 {[m
[36m@@ -190,22 +165,68 @@[m [mElevationTile::getNormal(double x, double y) const[m
     {[m
         double u = (x - getExtent().xMin()) / getExtent().width();[m
         double v = (y - getExtent().yMin()) / getExtent().height();[m
[31m-        osg::Vec4 value = _readNormal(u, v);[m
[31m-        normal.set(value.r(), value.g(), value.b());[m
[32m+[m[32m        osg::Vec4 value;[m
[32m+[m[32m        _readNormal(value, u, v);[m
[32m+[m[32m        NormalMapGenerator::unpack(value, normal);[m
     }[m
     return normal;[m
 }[m
 [m
 void[m
[31m-ElevationTile::generateNormalMap(const Map* map, unsigned tileSize, void* workingSet, ProgressCallback* progress)[m
[32m+[m[32mElevationTile::getPackedNormal(double x, double y, osg::Vec4& packed)[m
[32m+[m[32m{[m[41m    [m
[32m+[m[32m    if (_readNormal.valid())[m
[32m+[m[32m    {[m
[32m+[m[32m        double u = (x - getExtent().xMin()) / getExtent().width();[m
[32m+[m[32m        double v = (y - getExtent().yMin()) / getExtent().height();[m
[32m+[m[32m        _readNormal(packed, u, v);[m
[32m+[m[32m    }[m
[32m+[m[32m    else[m
[32m+[m[32m    {[m
[32m+[m[32m        packed.set(0.0f, 0.0f, 0.0f, 0.0f);[m
[32m+[m[32m    }[m
[32m+[m[32m}[m
[32m+[m
[32m+[m[32mfloat[m
[32m+[m[32mElevationTile::getRuggedness(double x, double y) const[m
[32m+[m[32m{[m
[32m+[m[32m    float result = 0.0f;[m
[32m+[m[32m    if (_readRuggedness.valid())[m
[32m+[m[32m    {[m
[32m+[m[32m        double u = (x - getExtent().xMin()) / getExtent().width();[m
[32m+[m[32m        double v = (y - getExtent().yMin()) / getExtent().height();[m
[32m+[m[32m        osg::Vec4 value;[m
[32m+[m[32m        _readRuggedness(value, u, v);[m
[32m+[m[32m        result = value.r();[m
[32m+[m[32m    }[m
[32m+[m[32m    return result;[m
[32m+[m[32m}[m
[32m+[m
[32m+[m[32mvoid[m
[32m+[m[32mElevationTile::generateNormalMap(const Map* map, void* workingSet, ProgressCallback* progress)[m
 {[m
     std::lock_guard<std::mutex> lock(_mutex);[m
 [m
     if (!_normalTex.valid())[m
[31m-    {[m
[32m+[m[32m    {[m[41m        [m
[32m+[m[32m#ifdef USE_RUGGEDNESS[m
[32m+[m[32m        if (!_ruggedness.valid())[m
[32m+[m[32m        {[m
[32m+[m[32m            _ruggedness = new osg::Image();[m
[32m+[m[32m            _ruggedness->allocateImage(_read.s(), _read.t(), 1, GL_RED, GL_UNSIGNED_BYTE);[m
[32m+[m[32m            _readRuggedness.setImage(_ruggedness.get());[m
[32m+[m[32m            _readRuggedness.setBilinear(true);[m
[32m+[m[32m        }[m
[32m+[m[32m#endif[m
[32m+[m
         NormalMapGenerator gen;[m
 [m
[31m-        _normalTex = gen.createNormalMap(getTileKey(), map, tileSize, allHeightsAtNativeResolution(), workingSet, progress);[m
[32m+[m[32m        _normalTex = gen.createNormalMap([m
[32m+[m[32m            getTileKey(),[m
[32m+[m[32m            map,[m
[32m+[m[32m            workingSet,[m
[32m+[m[32m            _ruggedness.get(),[m
[32m+[m[32m            progress);[m
 [m
         if (_normalTex.valid())[m
         {[m
[36m@@ -226,393 +247,170 @@[m [mElevationTile::generateNormalMap(const Map* map, unsigned tileSize, void* workin[m
 #define LC "[NormalMapGenerator] "[m
 [m
 osg::Texture2D*[m
[31m-NormalMapGenerator::createNormalMap(const TileKey& key, const Map* map, unsigned tileSize,[m
[31m-    bool nativeResolutionFastPath, void* ws, ProgressCallback* progress)[m
[32m+[m[32mNormalMapGenerator::createNormalMap([m
[32m+[m[32m    const TileKey& key,[m
[32m+[m[32m    const Map* map,[m
[32m+[m[32m    void* ws,[m
[32m+[m[32m    osg::Image* ruggedness,[m
[32m+[m[32m    ProgressCallback* progress)[m
 {[m
     if (!map)[m
         return NULL;[m
 [m
[31m-    const bool compress = true;[m
[31m-[m
[31m-    // the sampling goes a LOT faster if the normal map size is the same as the elevation tile size,[m
[31m-    // since there are fewer unique samples to take.[m
[31m-    // It is actually (must) faster to sample 1/4 the points and then resize the image to 256x256 later[m
[31m-    // during the compression stage.[m
[31m-[m
[31m-    if (tileSize == 0)[m
[31m-        tileSize = compress ? 256 : ELEVATION_TILE_SIZE;[m
[32m+[m[32m    OE_PROFILING_ZONE;[m
 [m
     ElevationPool::WorkingSet* workingSet = static_cast<ElevationPool::WorkingSet*>(ws);[m
 [m
     osg::ref_ptr<osg::Image> image = new osg::Image();[m
[31m-    image->allocateImage(tileSize, tileSize, 1, GL_RG, GL_UNSIGNED_BYTE);[m
[32m+[m[32m    image->allocateImage(ELEVATION_TILE_SIZE, ELEVATION_TILE_SIZE, 1, GL_RG, GL_UNSIGNED_BYTE);[m
     image->setInternalTextureFormat(GL_RG8);[m
[31m-    ImageUtils::PixelWriter write(image.get());[m
 [m
     ElevationPool* pool = map->getElevationPool();[m
 [m
[31m-    const GeoExtent& ex = key.getExtent();[m
[31m-[m
[31m-    // fetch the base tile in order to get resolutions data.[m
[31m-    osg::ref_ptr<ElevationTile> heights;[m
[31m-    pool->getTile(key, false, heights, workingSet, progress); // no fallback![m
[32m+[m[32m    ImageUtils::PixelWriter write(image.get());[m
 [m
[31m-    if (!heights.valid())[m
[31m-        return nullptr;[m
[32m+[m[32m    ImageUtils::PixelWriter writeRuggedness(ruggedness);[m
 [m
[31m-    auto* srs = key.getProfile()->getSRS();[m
[31m-    Distance res(heights->getHeightField()->getXInterval(), srs->getUnits());[m
[31m-    double dy = srs->transformDistance(res, Units::METERS);[m
[32m+[m[32m    osg::Vec3 normal;[m
[32m+[m[32m    osg::Vec2 packedNormal;[m
[32m+[m[32m    osg::Vec4 pixel;[m
 [m
[31m-    // if we are assuming that all data in the tile is native resolution, we can use the tile[m
[31m-    // directly to sample points and create normals (except along the edges). This is way[m
[31m-    // faster than sending all the points to the elevation pool for sampling.[m
[31m-    if (nativeResolutionFastPath)[m
[31m-    {[m
[31m-        double x0 = ex.xMin();[m
[31m-        double y0 = ex.yMin();[m
[31m-        double x1 = ex.xMax();[m
[31m-        double y1 = ex.yMax();[m
[31m-        osg::Vec3 normal;[m
[31m-        std::array<osg::Vec3, 4> a;[m
[31m-        double z_west = 0.0, z_east = 0.0, z_north = 0.0, z_south = 0.0;[m
[31m-        double du = 1.0 / (double)(tileSize - 1);[m
[31m-        double dv = 1.0 / (double)(tileSize - 1);[m
[31m-[m
[31m-        // Process interior pixels only (excluding edges that need external sampling)[m
[31m-        for (int t = 1; t < tileSize - 1; ++t)[m
[31m-        {[m
[31m-            double v = (double)t / (double)(tileSize - 1);[m
[31m-            double y_or_lat = ex.yMin() + v * ex.height();[m
[32m+[m[32m    GeoPoint[m
[32m+[m[32m        north(key.getProfile()->getSRS()),[m
[32m+[m[32m        south(key.getProfile()->getSRS()),[m
[32m+[m[32m        east(key.getProfile()->getSRS()),[m
[32m+[m[32m        west(key.getProfile()->getSRS());[m
 [m
[31m-            for (int s = 1; s < tileSize - 1; ++s)[m
[31m-            {[m
[31m-                double u = (double)s / (double)(tileSize - 1);[m
[31m-                double x_or_lon = ex.xMin() + u * ex.width();[m
[32m+[m[32m    osg::Vec3 a[4];[m
 [m
[31m-                // For interior pixels, all neighbors are within the tile[m
[31m-                z_west = heights->getRawElevationUV(u - du, v);[m
[31m-                z_east = heights->getRawElevationUV(u + du, v);[m
[31m-                z_north = heights->getRawElevationUV(u, v + dv);[m
[31m-                z_south = heights->getRawElevationUV(u, v - dv);[m
[32m+[m[32m    const GeoExtent& ex = key.getExtent();[m
 [m
[31m-                double dx = srs->transformDistance(res, Units::METERS, y_or_lat);[m
[32m+[m[32m    // fetch the base tile in order to get resolutions data.[m
[32m+[m[32m    osg::ref_ptr<ElevationTile> heights;[m
[32m+[m[32m    pool->getTile(key, true, heights, workingSet, progress);[m
 [m
[31m-                // only attempt to create a normal vector if all the data is valid:[m
[31m-                // four valid corner points.[m
[31m-                if (z_west != NO_DATA_VALUE && z_east != NO_DATA_VALUE && z_north != NO_DATA_VALUE && z_south != NO_DATA_VALUE)[m
[31m-                {[m
[31m-                    a[0].set(-dx, 0, z_west);[m
[31m-                    a[1].set(dx, 0, z_east);[m
[31m-                    a[2].set(0, -dy, z_south);[m
[31m-                    a[3].set(0, dy, z_north);[m
[31m-                    normal = (a[1] - a[0]) ^ (a[3] - a[2]);[m
[31m-                    normal.normalize();[m
[31m-                }[m
[31m-                else[m
[31m-                {[m
[31m-                    normal.set(0, 0, 1);[m
[31m-                }[m
[32m+[m[32m    if (!heights.valid())[m
[32m+[m[32m        return NULL;[m
 [m
[31m-                write(pack(normal), s, t);[m
[31m-            }[m
[31m-        }[m
[32m+[m[32m    // build the sample set.[m
[32m+[m[32m    std::vector<osg::Vec4d> points(write.s() * write.t() * 4);[m
[32m+[m[32m    int p = 0;[m
[32m+[m[32m    for (int t = 0; t < write.t(); ++t)[m
[32m+[m[32m    {[m
[32m+[m[32m        double v = (double)t / (double)(write.t() - 1);[m
[32m+[m[32m        double y = ex.yMin() + v * ex.height();[m
[32m+[m[32m        east.y() = y;[m
[32m+[m[32m        west.y() = y;[m
 [m
[31m-        // now do the edges by using the elevation pool.[m
[31m-        struct Workspace[m
[32m+[m[32m        for (int s = 0; s < write.s(); ++s)[m
         {[m
[31m-            std::vector<osg::Vec4d> vectorToSample; // actual points we'll send to the elevation pool[m
[31m-        };[m
[31m-        static thread_local Workspace w;[m
[32m+[m[32m            double u = (double)s / (double)(write.s() - 1);[m
[32m+[m[32m            double x = ex.xMin() + u * ex.width();[m
[32m+[m[32m            north.x() = x;[m
[32m+[m[32m            south.x() = x;[m
 [m
[31m-        w.vectorToSample.clear();[m
[31m-        w.vectorToSample.reserve((tileSize+2) * 4);[m
[32m+[m[32m            double r = heights->getResolution(s, t);[m
 [m
[31m-        double r = res.getValue();[m
[32m+[m[32m            east.x() = x + r;[m
[32m+[m[32m            west.x() = x - r;[m
[32m+[m[32m            north.y() = y + r;[m
[32m+[m[32m            south.y() = y - r;[m
 [m
[31m-        // bottom row[m
[31m-        for (int s = 0; s < tileSize; ++s)[m
[31m-        {[m
[31m-            double u = (double)s / (double)(tileSize - 1);[m
[31m-            double x = ex.xMin() + u * ex.width();[m
[31m-            w.vectorToSample.emplace_back(x, ex.yMin() - r, 0, r);[m
[31m-        }[m
[31m-        // top row[m
[31m-        for (int s = 0; s < tileSize; ++s)[m
[31m-        {[m
[31m-            double u = (double)s / (double)(tileSize - 1);[m
[31m-            double x = ex.xMin() + u * ex.width();[m
[31m-            w.vectorToSample.emplace_back(x, ex.yMax() + r, 0, r);[m
[32m+[m[32m            points[p++].set(west.x(), west.y(), 0.0, r);[m
[32m+[m[32m            points[p++].set(east.x(), east.y(), 0.0, r);[m
[32m+[m[32m            points[p++].set(south.x(), south.y(), 0.0, r);[m
[32m+[m[32m            points[p++].set(north.x(), north.y(), 0.0, r);[m
         }[m
[31m-        // left column[m
[31m-        for (int t = 0; t < tileSize; ++t)[m
[31m-        {[m
[31m-            double v = (double)t / (double)(tileSize - 1);[m
[31m-            double y = ex.yMin() + v * ex.height();[m
[31m-            w.vectorToSample.emplace_back(ex.xMin() - r, y, 0, r);[m
[31m-        }[m
[31m-        // right column[m
[31m-        for (int t = 0; t < tileSize; ++t)[m
[31m-        {[m
[31m-            double v = (double)t / (double)(tileSize - 1);[m
[31m-            double y = ex.yMin() + v * ex.height();[m
[31m-            w.vectorToSample.emplace_back(ex.xMax() + r, y, 0, r);[m
[31m-        }[m
[31m-[m
[31m-        int sampleOK = map->getElevationPool()->sampleMapCoords([m
[31m-            w.vectorToSample.begin(), w.vectorToSample.end(),[m
[31m-            workingSet,[m
[31m-            progress);[m
[32m+[m[32m    }[m
 [m
[31m-        if (progress && progress->isCanceled())[m
[31m-        {[m
[31m-            // canceled. Bail.[m
[31m-            return nullptr;[m
[31m-        }[m
[32m+[m[32m    int sampleOK = map->getElevationPool()->sampleMapCoords([m
[32m+[m[32m        points.begin(), points.end(),[m
[32m+[m[32m        workingSet,[m
[32m+[m[32m        progress);[m
 [m
[31m-        if (sampleOK < 0)[m
[31m-        {[m
[31m-            OE_WARN << LC << "Internal error - contact support" << std::endl;[m
[31m-            return nullptr;[m
[31m-        }[m
[32m+[m[32m    if (progress && progress->isCanceled())[m
[32m+[m[32m    {[m
[32m+[m[32m        // canceled. Bail.[m
[32m+[m[32m        return NULL;[m
[32m+[m[32m    }[m
 [m
[31m-        // Now compute normals for edge pixels using the sampled external data[m
[31m-        // Data organization in w.vectorToSample:[m
[31m-        // [0 to tileSize-1]: bottom row (y = yMin - dy) - tileSize points[m
[31m-        // [tileSize to 2*tileSize-1]: top row (y = yMax + dy) - tileSize points[m
[31m-        // [2*tileSize to 3*tileSize-1]: left column (x = xMin - dx) - tileSize points[m
[31m-        // [3*tileSize to 4*tileSize-1]: right column (x = xMax + dx) - tileSize points[m
[32m+[m[32m    if (sampleOK < 0)[m
[32m+[m[32m    {[m
[32m+[m[32m        OE_WARN << LC << "Internal error - contact support" << std::endl;[m
[32m+[m[32m        return NULL;[m
[32m+[m[32m    }[m
 [m
[31m-        int bottomRowStart = 0;[m
[31m-        int topRowStart = tileSize;[m
[31m-        int leftColStart = 2 * tileSize;[m
[31m-        int rightColStart = 3 * tileSize;[m
[32m+[m[32m    auto* srs = key.getProfile()->getSRS();[m
[32m+[m[32m    Distance res(0.0, srs->getUnits());[m
[32m+[m[32m    double dx, dy;[m
[32m+[m[32m    osg::Vec4 riPixel;[m
 [m
[31m-        double dy = srs->transformDistance(res, Units::METERS, 0.0);[m
[32m+[m[32m    for (int t = 0; t < write.t(); ++t)[m
[32m+[m[32m    {[m
[32m+[m[32m        double v = (double)t / (double)(write.t() - 1);[m
[32m+[m[32m        double y_or_lat = ex.yMin() + v * ex.height();[m
 [m
[31m-        // Process all edge pixels[m
[31m-        for (int t = 0; t < tileSize; ++t)[m
[32m+[m[32m        for (int s = 0; s < write.s(); ++s)[m
         {[m
[31m-            for (int s = 0; s < tileSize; ++s)[m
[31m-            {[m
[31m-                // Skip interior pixels - they were already processed[m
[31m-                if (t > 0 && t < tileSize - 1 && s > 0 && s < tileSize - 1)[m
[31m-                    continue;[m
[31m-[m
[31m-                double u = (double)s / (double)(tileSize - 1);[m
[31m-                double v = (double)t / (double)(tileSize - 1);[m
[31m-                double y_or_lat = ex.yMin() + v * ex.height();[m
[31m-[m
[31m-                double z_west, z_east, z_north, z_south;[m
[31m-[m
[31m-                // Get west elevation (one pixel to the left)[m
[31m-                if (s == 0) {[m
[31m-                    // Left edge: use sampled left column data (external point)[m
[31m-                    int leftIdx = leftColStart + t;[m
[31m-                    z_west = w.vectorToSample[leftIdx].z();[m
[31m-                } else {[m
[31m-                    // Interior: use tile data (internal point)[m
[31m-                    z_west = heights->getRawElevationUV(u - du, v);[m
[31m-                }[m
[32m+[m[32m            int p = (4 * write.s() * t + 4 * s);[m
 [m
[31m-                // Get east elevation (one pixel to the right)[m
[31m-                if (s == tileSize - 1) {[m
[31m-                    // Right edge: use sampled right column data (external point)[m
[31m-                    int rightIdx = rightColStart + t;[m
[31m-                    z_east = w.vectorToSample[rightIdx].z();[m
[31m-                } else {[m
[31m-                    // Interior: use tile data (internal point)[m
[31m-                    z_east = heights->getRawElevationUV(u + du, v);[m
[31m-                }[m
[32m+[m[32m            res.set(points[p].w(), res.getUnits());[m
[32m+[m[32m            dx = srs->transformDistance(res, Units::METERS, y_or_lat);[m
[32m+[m[32m            dy = srs->transformDistance(res, Units::METERS, 0.0);[m
 [m
[31m-                // Get south elevation (one pixel down)[m
[31m-                if (t == 0) {[m
[31m-                    // Bottom edge: use sampled bottom row data (external point)[m
[31m-                    int bottomIdx = bottomRowStart + s;[m
[31m-                    z_south = w.vectorToSample[bottomIdx].z();[m
[31m-                } else {[m
[31m-                    // Interior: use tile data (internal point)[m
[31m-                    z_south = heights->getRawElevationUV(u, v - dv);[m
[31m-                }[m
[32m+[m[32m            riPixel.r() = 0.0f;[m
 [m
[31m-                // Get north elevation (one pixel up)[m
[31m-                if (t == tileSize - 1) {[m
[31m-                    // Top edge: use sampled top row data (external point)[m
[31m-                    int topIdx = topRowStart + s;[m
[31m-                    z_north = w.vectorToSample[topIdx].z();[m
[31m-                } else {[m
[31m-                    // Interior: use tile data (internal point)[m
[31m-                    z_north = heights->getRawElevationUV(u, v + dv);[m
[31m-                }[m
[31m-[m
[31m-                // Use the same fixed dx that was used for sampling external points[m
[31m-                // (In a perfect implementation, this should vary with latitude, but for[m
[31m-                // consistency with the sampling, we use the fixed value from line 331)[m
[32m+[m[32m            // only attempt to create a normal vector if all the data is valid:[m
[32m+[m[32m            // a valid resolution value and four valid corner points.[m
[32m+[m[32m            if (res.getValue() != FLT_MAX &&[m
[32m+[m[32m                points[p + 0].z() != NO_DATA_VALUE &&[m
[32m+[m[32m                points[p + 1].z() != NO_DATA_VALUE &&[m
[32m+[m[32m                points[p + 2].z() != NO_DATA_VALUE &&[m
[32m+[m[32m                points[p + 3].z() != NO_DATA_VALUE)[m
[32m+[m[32m            {[m
[32m+[m[32m                a[0].set(-dx, 0, points[p + 0].z());[m
[32m+[m[32m                a[1].set(dx, 0, points[p + 1].z());[m
[32m+[m[32m                a[2].set(0, -dy, points[p + 2].z());[m
[32m+[m[32m                a[3].set(0, dy, points[p + 3].z());[m
 [m
[31m-                double dx = srs->transformDistance(res, Units::METERS, y_or_lat);[m
[32m+[m[32m                normal = (a[1] - a[0]) ^ (a[3] - a[2]);[m
[32m+[m[32m                normal.normalize();[m
 [m
[31m-                // Compute normal if all data is valid[m
[31m-                if (z_west != NO_DATA_VALUE && z_east != NO_DATA_VALUE && z_north != NO_DATA_VALUE && z_south != NO_DATA_VALUE)[m
[32m+[m[32m                if (ruggedness)[m
                 {[m
[31m-                    a[0].set(-dx, 0, z_west);[m
[31m-                    a[1].set(+dx, 0, z_east);[m
[31m-                    a[2].set(0, -dy, z_south);[m
[31m-                    a[3].set(0, +dy, z_north);[m
[31m-                    normal = (a[1] - a[0]) ^ (a[3] - a[2]);[m
[31m-                    normal.normalize();[m
[32m+[m[32m                    // rudimentary normalized ruggedness index[m
[32m+[m[32m                    riPixel.r() = 0.25 * ([m
[32m+[m[32m                        fabs(points[p + 0].z() - points[p + 3].z()) +[m
[32m+[m[32m                        fabs(points[p + 1].z() - points[p + 0].z()) +[m
[32m+[m[32m                        fabs(points[p + 2].z() - points[p + 1].z()) +[m
[32m+[m[32m                        fabs(points[p + 3].z() - points[p + 2].z()));[m
[32m+[m[32m                    riPixel.r() = clamp(riPixel.r() / (float)dy, 0.0f, 1.0f);[m
[32m+[m[32m                    riPixel.r() = harden(harden(riPixel.r()));[m
                 }[m
[31m-                else[m
[31m-                {[m
[31m-                    normal.set(0, 0, 1);[m
[31m-                }[m
[31m-[m
[31m-                write(pack(normal), s, t);[m
[31m-            }[m
[31m-        }[m
[31m-    }[m
[31m-[m
[31m-    else[m
[31m-    {[m
[31m-        struct Point {[m
[31m-            double x, y;[m
[31m-        };[m
[31m-[m
[31m-        struct PointHash {[m
[31m-            inline std::size_t operator()(const Point& k) const {[m
[31m-                auto h1 = std::hash<double>()(k.x);[m
[31m-                auto h2 = std::hash<double>()(k.y);[m
[31m-                return h1 ^ (h2 << 1);[m
[31m-            }[m
[31m-        };[m
[31m-        struct PointEquals {[m
[31m-            inline bool operator()(const Point& lhs, const Point& rhs) const {[m
[31m-                return lhs.x == rhs.x && lhs.y == rhs.y;[m
             }[m
[31m-        };[m
[31m-[m
[31m-        struct Workspace {[m
[31m-            std::unordered_map<Point, int, PointHash, PointEquals> uniquePoints; // map a point to its index in vectorToSample[m
[31m-            std::vector<osg::Vec4d> vectorToSample; // actual points we'll send to the elevation pool[m
[31m-            std::vector<int> rasterIndex; // maps the raster offset to an entry in vectorToSample.[m
[31m-        };[m
[31m-        static thread_local Workspace w;[m
[31m-[m
[31m-        w.uniquePoints.clear();[m
[31m-        w.vectorToSample.clear();[m
[31m-        w.vectorToSample.reserve(write.s() * write.t() * 4);[m
[31m-        w.rasterIndex.clear();[m
[31m-        w.rasterIndex.reserve(write.s() * write.t() * 4);[m
[31m-[m
[31m-        for (int t = 0; t < write.t(); ++t)[m
[31m-        {[m
[31m-            double v = (double)t / (double)(write.t() - 1);[m
[31m-            double y = ex.yMin() + v * ex.height();[m
[31m-[m
[31m-            for (int s = 0; s < write.s(); ++s)[m
[32m+[m[32m            else[m
             {[m
[31m-                double u = (double)s / (double)(write.s() - 1);[m
[31m-                double x = ex.xMin() + u * ex.width();[m
[31m-                double r = heights->getResolution(s, t);[m
[31m-[m
[31m-                {[m
[31m-                    auto [iter, isNew] = w.uniquePoints.emplace(Point{ x - r, y }, (int)w.vectorToSample.size());[m
[31m-                    if (isNew) w.vectorToSample.emplace_back(x - r, y, 0, r);[m
[31m-                    w.rasterIndex.emplace_back(iter->second);[m
[31m-                }[m
[31m-                {[m
[31m-                    auto [iter, isNew] = w.uniquePoints.emplace(Point{ x + r, y }, (int)w.vectorToSample.size());[m
[31m-                    if (isNew) w.vectorToSample.emplace_back(x + r, y, 0, r);[m
[31m-                    w.rasterIndex.emplace_back(iter->second);[m
[31m-                }[m
[31m-                {[m
[31m-                    auto [iter, isNew] = w.uniquePoints.emplace(Point{ x, y - r }, (int)w.vectorToSample.size());[m
[31m-                    if (isNew) w.vectorToSample.emplace_back(x, y - r, 0, r);[m
[31m-                    w.rasterIndex.emplace_back(iter->second);[m
[31m-                }[m
[31m-                {[m
[31m-                    auto [iter, isNew] = w.uniquePoints.emplace(Point{ x, y + r }, (int)w.vectorToSample.size());[m
[31m-                    if (isNew) w.vectorToSample.emplace_back(x, y + r, 0, r);[m
[31m-                    w.rasterIndex.emplace_back(iter->second);[m
[31m-                }[m
[32m+[m[32m                normal.set(0, 0, 1);[m
             }[m
[31m-        }[m
[31m-[m
[31m-        int sampleOK = map->getElevationPool()->sampleMapCoords([m
[31m-            w.vectorToSample.begin(), w.vectorToSample.end(),[m
[31m-            workingSet,[m
[31m-            progress);[m
[31m-[m
[31m-        if (progress && progress->isCanceled())[m
[31m-        {[m
[31m-            // canceled. Bail.[m
[31m-            return NULL;[m
[31m-        }[m
[31m-[m
[31m-        if (sampleOK < 0)[m
[31m-        {[m
[31m-            OE_WARN << LC << "Internal error - contact support" << std::endl;[m
[31m-            return NULL;[m
[31m-        }[m
 [m
[31m-        osg::Vec3 normal;[m
[31m-        std::array<osg::Vec3, 4> a;[m
[31m-        auto* srs = key.getProfile()->getSRS();[m
[31m-        Distance res(0.0, srs->getUnits());[m
[31m-        double dx, dy;[m
[31m-        double z_west, z_east, z_south, z_north;[m
[32m+[m[32m            NormalMapGenerator::pack(normal, pixel);[m
 [m
[31m-        unsigned p = 0;[m
[32m+[m[32m            // TODO: won't actually be written until we make the format GL_RGB[m
[32m+[m[32m            // but we need to rewrite the curvature generator first[m
[32m+[m[32m            //pixel.b() = 0.0f; // 0.5f*(1.0f+normalMap->getCurvature(s, t));[m
 [m
[31m-        for (int t = 0; t < write.t(); ++t)[m
[31m-        {[m
[31m-            double v = (double)t / (double)(write.t() - 1);[m
[31m-            double y_or_lat = ex.yMin() + v * ex.height();[m
[32m+[m[32m            write(pixel, s, t);[m
 [m
[31m-            for (int s = 0; s < write.s(); ++s)[m
[32m+[m[32m            if (ruggedness)[m
             {[m
[31m-                auto sampleRes = w.vectorToSample[w.rasterIndex[p]].w();[m
[31m-                z_west = w.vectorToSample[w.rasterIndex[p++]].z();[m
[31m-                z_east = w.vectorToSample[w.rasterIndex[p++]].z();[m
[31m-                z_south = w.vectorToSample[w.rasterIndex[p++]].z();[m
[31m-                z_north = w.vectorToSample[w.rasterIndex[p++]].z();[m
[31m-[m
[31m-                res.set(sampleRes, res.getUnits());[m
[31m-                dx = srs->transformDistance(res, Units::METERS, y_or_lat);[m
[31m-                dy = srs->transformDistance(res, Units::METERS, 0.0);[m
[31m-[m
[31m-                // only attempt to create a normal vector if all the data is valid:[m
[31m-                // a valid resolution value and four valid corner points.[m
[31m-                if (res.getValue() != FLT_MAX &&[m
[31m-                    z_west != NO_DATA_VALUE && z_east != NO_DATA_VALUE && z_south != NO_DATA_VALUE && z_north != NO_DATA_VALUE)[m
[31m-                {[m
[31m-                    a[0].set(-dx, 0, z_west);[m
[31m-                    a[1].set(+dx, 0, z_east);[m
[31m-                    a[2].set(0, -dy, z_south);[m
[31m-                    a[3].set(0, +dy, z_north);[m
[31m-[m
[31m-                    normal = (a[1] - a[0]) ^ (a[3] - a[2]);[m
[31m-                    normal.normalize();[m
[31m-                }[m
[31m-                else[m
[31m-                {[m
[31m-                    normal.set(0, 0, 1);[m
[31m-                }[m
[31m-[m
[31m-                write(pack(normal), s, t);[m
[32m+[m[32m                writeRuggedness(riPixel, s, t);[m
             }[m
         }[m
     }[m
 [m
[31m-    if (compress)[m
[31m-    {[m
[31m-        // compress the image (using BC5/RGTC2) and generate mipmaps.[m
[31m-        ImageUtils::compressImageInPlace(image.get());[m
[31m-    }[m
[31m-[m
     osg::Texture2D* normalTex = new osg::Texture2D(image.get());[m
 [m
[31m-    normalTex->setDataVariance(osg::Object::STATIC);[m
[31m-    normalTex->setInternalFormat(image->getInternalTextureFormat());[m
[32m+[m[32m    normalTex->setInternalFormat(GL_RG8);[m
     normalTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);[m
     normalTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);[m
     normalTex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);[m
[36m@@ -620,7 +418,37 @@[m [mNormalMapGenerator::createNormalMap(const TileKey& key, const Map* map, unsigned[m
     normalTex->setResizeNonPowerOfTwoHint(false);[m
     normalTex->setMaxAnisotropy(1.0f);[m
     normalTex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());[m
[32m+[m[32m    ImageUtils::mipmapImageInPlace(image.get());[m
 [m
     return normalTex;[m
 }[m
 [m
[32m+[m[32mvoid[m
[32m+[m[32mNormalMapGenerator::pack(const osg::Vec3& n, osg::Vec4& p)[m
[32m+[m[32m{[m
[32m+[m[32m    // octohodreal normal packing[m
[32m+[m[32m    float d = 1.0/(fabs(n.x())+fabs(n.y())+fabs(n.z()));[m
[32m+[m[32m    p.x() = n.x() * d;[m
[32m+[m[32m    p.y() = n.y() * d;[m
[32m+[m
[32m+[m[32m    if (n.z() < 0.0)[m
[32m+[m[32m    {[m
[32m+[m[32m        p.x() = (1.0 - fabs(p.y())) * (p.x() >= 0.0? 1.0 : -1.0);[m
[32m+[m[32m        p.y() = (1.0 - fabs(p.x())) * (p.y() >= 0.0? 1.0 : -1.0);[m
[32m+[m[32m    }[m
[32m+[m
[32m+[m[32m    p.x() = 0.5f*(p.x()+1.0f);[m
[32m+[m[32m    p.y() = 0.5f*(p.y()+1.0f);[m
[32m+[m[32m}[m
[32m+[m
[32m+[m[32mvoid[m
[32m+[m[32mNormalMapGenerator::unpack(const osg::Vec4& packed, osg::Vec3& normal)[m
[32m+[m[32m{[m
[32m+[m[32m    normal.x() = packed.x()*2.0-1.0;[m
[32m+[m[32m    normal.y() = packed.y()*2.0-1.0;[m
[32m+[m[32m    normal.z() = 1.0-fabs(normal.x())-fabs(normal.y());[m
[32m+[m[32m    float t = clamp(-normal.z(), 0.0f, 1.0f);[m
[32m+[m[32m    normal.x() += (normal.x() > 0)? -t : t;[m
[32m+[m[32m    normal.y() += (normal.y() > 0)? -t : t;[m
[32m+[m[32m    normal.normalize();[m
[32m+[m[32m}[m
[1mdiff --git a/src/osgEarth/ElevationLayer.cpp b/src/osgEarth/ElevationLayer.cpp[m
[1mindex 886d8f570..5555d9a5f 100644[m
[1m--- a/src/osgEarth/ElevationLayer.cpp[m
[1m+++ b/src/osgEarth/ElevationLayer.cpp[m
[36m@@ -715,6 +715,13 @@[m [mElevationLayerVector::populateHeightField([m
                 useLayer = false;[m
             }[m
 [m
[32m+[m[32m            // GW - this was wrong because it would exclude layers with a maxDataLevel set[m
[32m+[m[32m            // below the requested LOD ... when in fact we need around for fallback.[m
[32m+[m[32m            //if ( !layer->isKeyInLegalRange(key) )[m
[32m+[m[32m            //{[m
[32m+[m[32m            //    useLayer = false;[m
[32m+[m[32m            //}[m
[32m+[m
             // Find the "best available" mapped key from the tile source:[m
             else[m
             {[m
[1mdiff --git a/src/osgEarth/ElevationPool b/src/osgEarth/ElevationPool[m
[1mindex e1f9d494d..c51aa0ede 100644[m
[1m--- a/src/osgEarth/ElevationPool[m
[1m+++ b/src/osgEarth/ElevationPool[m
[36m@@ -27,7 +27,6 @@[m [mnamespace osgEarth[m
         using WeakLUT = std::unordered_map<Internal::RevElevationKey, WeakPointer>;[m
 [m
     public:[m
[31m-[m
         //! User data that a client can use to speed up queries in[m
         //! a local geographic area or sample a custom set of layers.[m
         class OSGEARTH_EXPORT WorkingSet[m
[36m@@ -56,21 +55,6 @@[m [mnamespace osgEarth[m
             friend class ElevationPool;[m
         };[m
 [m
[31m-    private:[m
[31m-        struct MapData {[m
[31m-            osg::observer_ptr<const Map> map;[m
[31m-            ElevationLayerVector layers;[m
[31m-            size_t hash;[m
[31m-            int mapRevision;[m
[31m-            osg::ref_ptr<const Profile> mapProfile;[m
[31m-            osg::ref_ptr<const Profile> mapProfileNoVDatum;[m
[31m-            RasterInterpolation interpolation;[m
[31m-            std::map<const ElevationLayer*, void*> index;[m
[31m-        };[m
[31m-[m
[31m-        MapData snapshotMapData(WorkingSet* ws);[m
[31m-[m
[31m-    public:[m
         /**[m
          * Object for doing lots of queries in a localized area at a specific LOD.[m
          * Call prepareEnvelope to initialize one.[m
[36m@@ -104,7 +88,6 @@[m [mnamespace osgEarth[m
             }[m
 [m
         private:[m
[31m-            MapData _mapDataSnapshot;[m
             Internal::RevElevationKey _key;[m
             QuickCache _cache;[m
             double _pw, _ph, _pxmin, _pymin;[m
[36m@@ -215,6 +198,11 @@[m [mnamespace osgEarth[m
 [m
     private:[m
 [m
[32m+[m[32m        bool needsRefresh();[m
[32m+[m
[32m+[m[32m        // weak pointer to the map from whence this pool came[m
[32m+[m[32m        osg::observer_ptr<const Map> _map;[m
[32m+[m
         // stores weak pointers to elevation textures wherever they may exist[m
         // elsewhere in the system, including the local L2 LRU.[m
         WeakLUT _globalLUT;[m
[36m@@ -225,17 +213,28 @@[m [mnamespace osgEarth[m
         // alive in the global LUT (see above).[m
         mutable LRUCache<Internal::RevElevationKey, Pointer> _L2;[m
 [m
[32m+[m[32m        std::map<const ElevationLayer*, void*> _layerIndex;[m
[32m+[m
         // elevation tile size[m
         unsigned _tileSize;[m
 [m
[31m-        // current map data reflection[m
[31m-        MapData _mapData;[m
[31m-        Threading::ReadWriteMutex _mapDataMutex;[m
[32m+[m[32m        size_t _elevationHash;[m
[32m+[m[32m        int  _mapRevision;[m
[32m+[m
[32m+[m[32m        Threading::ReadWriteMutex _mutex;[m
[32m+[m
[32m+[m[32m        ElevationLayerVector _elevationLayers;[m[41m        [m
[32m+[m
[32m+[m[32m        size_t getElevationHash(WorkingSet*) const;[m
[32m+[m
[32m+[m[32m        void sync(const Map*, WorkingSet*);[m
[32m+[m
[32m+[m[32m        void refresh(const Map*);[m
 [m
[31m-        // internal - get a sample at a point[m
         ElevationSample getSample([m
             const GeoPoint& p,[m
             unsigned maxLOD,[m
[32m+[m[32m            const Map* map,[m
             WorkingSet* ws,[m
             ProgressCallback* progress);        [m
 [m
[36m@@ -243,9 +242,10 @@[m [mnamespace osgEarth[m
         int getLOD(double x, double y, WorkingSet*);[m
 [m
         osg::ref_ptr<ElevationTile> getOrCreateRaster([m
[31m-            MapData& snapshot,[m
             const Internal::RevElevationKey& key,[m
[32m+[m[32m            const Map* map,[m
             bool acceptLowerRes,[m
[32m+[m[32m            WorkingSet* ws,[m
             ProgressCallback* progress);[m
 [m
         bool findExistingRaster([m
[36m@@ -288,6 +288,7 @@[m [mnamespace osgEarth[m
             const Distance& resolution);[m
 [m
     protected:[m
[32m+[m
         osg::observer_ptr<const Map> _map;[m
         ElevationPool::WorkingSet _ws;[m
         jobs::jobpool* _arena;[m
[1mdiff --git a/src/osgEarth/ElevationPool.cpp b/src/osgEarth/ElevationPool.cpp[m
[1mindex 01235d931..017f235d2 100644[m
[1m--- a/src/osgEarth/ElevationPool.cpp[m
[1m+++ b/src/osgEarth/ElevationPool.cpp[m
[36m@@ -1,6 +1,6 @@[m
 [m
 /* osgEarth[m
[31m- * Copyright 2008-2025 Pelican Mapping[m
[32m+[m[32m * Copyright 2008-2016 Pelican Mapping[m
  * MIT License[m
  */[m
 #include <osgEarth/ElevationPool>[m
[36m@@ -20,106 +20,178 @@[m [musing namespace osgEarth;[m
 [m
 ElevationPool::ElevationPool() :[m
     _tileSize(257),[m
[31m-    _L2(64u)[m
[32m+[m[32m    _L2(64u),[m
[32m+[m[32m    _mapRevision(-1),[m
[32m+[m[32m    _elevationHash(0)[m
 {[m
 }[m
 [m
 const SpatialReference*[m
 ElevationPool::getMapSRS() const[m
 {[m
[31m-    osg::ref_ptr<const Map> safe_map;[m
[31m-    if (_mapData.map.lock(safe_map))[m
[31m-        return safe_map->getSRS();[m
[32m+[m[32m    osg::ref_ptr<const Map> map;[m
[32m+[m[32m    if (_map.lock(map))[m
[32m+[m[32m        return map->getSRS();[m
     else[m
         return nullptr;[m
 }[m
 [m
 namespace[m
 {[m
[32m+[m[32m#if 1[m
     using MaxLevelIndex = RTree<unsigned, double, 2>;[m
[32m+[m[32m#else[m
[32m+[m[32m    struct MaxLevelIndex[m
[32m+[m[32m    {[m
[32m+[m[32m        std::vector<osg::BoundingBox> _boxes;[m
[32m+[m
[32m+[m[32m        void Insert(const double a_min[2], const double a_max[2], unsigned maxLevel)[m
[32m+[m[32m        {[m
[32m+[m[32m            osg::BoundingBox box(a_min[0], a_min[1], 0.0, a_max[0], a_max[1], 0.0);[m
[32m+[m[32m            _boxes.push_back(box);[m
[32m+[m[32m        }[m
[32m+[m
[32m+[m[32m        template<class FUNC>[m
[32m+[m[32m        bool Search(const double a_min[2], const double a_max[2], FUNC&& callback)[m
[32m+[m[32m        {[m
[32m+[m[32m            osg::BoundingBox query(a_min[0], a_min[1], 0.0, a_max[0], a_max[1], 0.0);[m
[32m+[m[32m            for (unsigned i = 0; i < _boxes.size(); ++i)[m
[32m+[m[32m            {[m
[32m+[m[32m                if (query.intersects(_boxes[i]))[m
[32m+[m[32m                {[m
[32m+[m[32m                    if (callback(i) == false)[m
[32m+[m[32m                        return false;[m
[32m+[m[32m                }[m
[32m+[m[32m            }[m
[32m+[m[32m            return true;[m
[32m+[m[32m        }[m
[32m+[m[32m    };[m
[32m+[m[32m#endif[m
 }[m
 [m
 ElevationPool::~ElevationPool()[m
 {[m
[31m-    ScopedWriteLock lock(_mapDataMutex);[m
[32m+[m[32m    setMap(nullptr);[m
 [m
[31m-    for (auto& itr : _mapData.index)[m
[32m+[m[32m    for (auto& itr : _layerIndex)[m
[32m+[m[32m    {[m
         if (itr.second)[m
[32m+[m[32m        {[m
             delete static_cast<MaxLevelIndex*>(itr.second);[m
[32m+[m[32m        }[m
[32m+[m[32m    }[m
[32m+[m[32m    _layerIndex.clear();[m
 }[m
 [m
 void[m
 ElevationPool::setMap(const Map* map)[m
 {[m
[31m-    _L2.clear();[m
[32m+[m[32m    _map = map;[m
[32m+[m
[32m+[m[32m    if (map)[m
     {[m
[31m-        ScopedWriteLock lock(_globalLUTMutex);[m
[31m-        _globalLUT.clear();[m
[32m+[m[32m        refresh(map);[m
     }[m
[32m+[m[32m}[m
 [m
[31m-    MapData newData;[m
[32m+[m[32msize_t[m
[32m+[m[32mElevationPool::getElevationHash(WorkingSet* ws) const[m
[32m+[m[32m{[m
[32m+[m[32m    // yes, must do this every time because individual[m
[32m+[m[32m    // layers can "bump" their revisions (dynamic layers)[m[41m    [m
[32m+[m[32m    size_t hash = hash_value_unsigned(_mapRevision);[m
 [m
[31m-    if (map)[m
[32m+[m[32m    // using the working set or the baseline?[m
[32m+[m[32m    auto& layers =[m
[32m+[m[32m        (ws && ws->_elevationLayers.size() > 0) ? ws->_elevationLayers :[m
[32m+[m[32m        this->_elevationLayers;[m
[32m+[m
[32m+[m[32m    for (auto& layer : layers)[m
[32m+[m[32m        if (layer->isOpen())[m
[32m+[m[32m            hash = hash_value_unsigned(hash, layer->getRevision());[m
[32m+[m[32m        else[m
[32m+[m[32m            hash = hash_value_unsigned(hash, 0u);[m
[32m+[m
[32m+[m[32m    return hash;[m
[32m+[m[32m}[m
[32m+[m
[32m+[m[32mvoid[m
[32m+[m[32mElevationPool::sync(const Map* map, WorkingSet* ws)[m
[32m+[m[32m{[m
[32m+[m[32m    if (needsRefresh())[m
     {[m
[31m-        newData.map = map;[m
[31m-        newData.mapRevision = map->getOpenLayers<ElevationLayer>(newData.layers);[m
[31m-        newData.mapProfile = map->getProfile();[m
[31m-        newData.mapProfileNoVDatum = map->getProfileNoVDatum();[m
[31m-        newData.interpolation = map->getElevationInterpolation();[m
[31m-        newData.hash = 0;[m
[31m-        for (auto& layer : newData.layers)[m
[31m-            newData.hash = hash_value_unsigned(newData.hash, layer->getRevision());[m
[31m-[m
[31m-        double a_min[2], a_max[2];[m
[31m-[m
[31m-        for (auto i : newData.layers)[m
[32m+[m[32m        OE_PROFILING_ZONE;[m
[32m+[m
[32m+[m[32m        refresh(map);[m
[32m+[m
[32m+[m[32m        if (ws)[m
[32m+[m[32m            ws->_lru.clear();[m
[32m+[m[32m    }[m
[32m+[m[32m}[m
[32m+[m
[32m+[m[32mvoid[m
[32m+[m[32mElevationPool::refresh(const Map* map)[m
[32m+[m[32m{[m
[32m+[m[32m    ScopedWriteLock lk(_mutex);[m
[32m+[m
[32m+[m[32m    _elevationLayers.clear();[m
[32m+[m
[32m+[m[32m    for (auto& itr : _layerIndex)[m
[32m+[m[32m    {[m
[32m+[m[32m        if (itr.second)[m
         {[m
[31m-            const ElevationLayer* layer = i.get();[m
[31m-            DataExtentList dataExtents;[m
[31m-            layer->getDataExtents(dataExtents);[m
[32m+[m[32m            delete static_cast<MaxLevelIndex*>(itr.second);[m
[32m+[m[32m        }[m
[32m+[m[32m    }[m
[32m+[m[32m    _layerIndex.clear();[m
 [m
[31m-            MaxLevelIndex* layerIndex = new MaxLevelIndex();[m
[32m+[m[32m    _mapRevision = _map->getOpenLayers(_elevationLayers);[m
[32m+[m[32m    _elevationHash = getElevationHash(nullptr);[m
 [m
[31m-            for (auto de = dataExtents.begin(); de != dataExtents.end(); ++de)[m
[31m-            {[m
[31m-                GeoExtent extentInMapSRS = map->getProfile()->clampAndTransformExtent(*de);[m
[32m+[m[32m    double a_min[2], a_max[2];[m
 [m
[31m-                // Convert the max level so it's relative to the map profile:[m
[31m-                unsigned maxLevel = std::min(de->maxLevel().get(), layer->getMaxDataLevel());[m
[31m-                maxLevel = map->getProfile()->getEquivalentLOD(layer->getProfile(), maxLevel);[m
[32m+[m[32m    for (auto i : _elevationLayers)[m
[32m+[m[32m    {[m
[32m+[m[32m        const ElevationLayer* layer = i.get();[m
[32m+[m[32m        DataExtentList dataExtents;[m
[32m+[m[32m        layer->getDataExtents(dataExtents);[m
 [m
[31m-                if (extentInMapSRS.crossesAntimeridian())[m
[31m-                {[m
[31m-                    GeoExtent a, b;[m
[31m-                    extentInMapSRS.splitAcrossAntimeridian(a, b);[m
[32m+[m[32m        MaxLevelIndex* layerIndex = new MaxLevelIndex();[m
 [m
[31m-                    for (auto& ex : { a, b })[m
[31m-                    {[m
[31m-                        a_min[0] = ex.xMin(), a_min[1] = ex.yMin();[m
[31m-                        a_max[0] = ex.xMax(), a_max[1] = ex.yMax();[m
[31m-                        layerIndex->Insert(a_min, a_max, maxLevel);[m
[31m-                    }[m
[31m-                }[m
[31m-                else[m
[32m+[m[32m        for (auto de = dataExtents.begin(); de != dataExtents.end(); ++de)[m
[32m+[m[32m        {[m
[32m+[m[32m            GeoExtent extentInMapSRS = map->getProfile()->clampAndTransformExtent(*de);[m
[32m+[m
[32m+[m[32m            // Convert the max level so it's relative to the map profile:[m
[32m+[m[32m            unsigned maxLevel = std::min(de->maxLevel().get(), layer->getMaxDataLevel());[m
[32m+[m[32m            maxLevel = map->getProfile()->getEquivalentLOD(layer->getProfile(), maxLevel);[m
[32m+[m
[32m+[m[32m            if (extentInMapSRS.crossesAntimeridian())[m
[32m+[m[32m            {[m
[32m+[m[32m                GeoExtent a, b;[m
[32m+[m[32m                extentInMapSRS.splitAcrossAntimeridian(a, b);[m
[32m+[m
[32m+[m[32m                for (auto& ex : { a, b })[m
                 {[m
[31m-                    a_min[0] = extentInMapSRS.xMin(), a_min[1] = extentInMapSRS.yMin();[m
[31m-                    a_max[0] = extentInMapSRS.xMax(), a_max[1] = extentInMapSRS.yMax();[m
[32m+[m[32m                    a_min[0] = ex.xMin(), a_min[1] = ex.yMin();[m
[32m+[m[32m                    a_max[0] = ex.xMax(), a_max[1] = ex.yMax();[m
                     layerIndex->Insert(a_min, a_max, maxLevel);[m
                 }[m
             }[m
[31m-            newData.index[layer] = layerIndex;[m
[32m+[m[32m            else[m
[32m+[m[32m            {[m
[32m+[m[32m                a_min[0] = extentInMapSRS.xMin(), a_min[1] = extentInMapSRS.yMin();[m
[32m+[m[32m                a_max[0] = extentInMapSRS.xMax(), a_max[1] = extentInMapSRS.yMax();[m
[32m+[m[32m                layerIndex->Insert(a_min, a_max, maxLevel);[m
[32m+[m[32m            }[m
         }[m
[32m+[m[32m        _layerIndex[layer] = layerIndex;[m
     }[m
 [m
[31m-    {[m
[31m-        ScopedWriteLock lk(_mapDataMutex);[m
[31m-[m
[31m-        for (auto& itr : _mapData.index)[m
[31m-            if (itr.second)[m
[31m-                delete static_cast<MaxLevelIndex*>(itr.second);[m
[32m+[m[32m    _L2.clear();[m
 [m
[31m-        std::swap(_mapData, newData);[m
[31m-    }[m
[32m+[m[32m    ScopedWriteLock lock(_globalLUTMutex);[m
[32m+[m[32m    _globalLUT.clear();[m
 }[m
 [m
 int[m
[36m@@ -130,12 +202,12 @@[m [mElevationPool::getLOD(double x, double y, WorkingSet* ws)[m
 [m
     auto& layers =[m
         (ws && ws->_elevationLayers.size() > 0) ? ws->_elevationLayers :[m
[31m-        _mapData.layers;[m
[32m+[m[32m        this->_elevationLayers;[m
 [m
     for (auto& layerItr : layers)[m
     {[m
[31m-        auto itr = _mapData.index.find(layerItr.get());[m
[31m-        if (itr != _mapData.index.end())[m
[32m+[m[32m        auto itr = _layerIndex.find(layerItr.get());[m
[32m+[m[32m        if (itr != _layerIndex.end())[m
         {[m
             MaxLevelIndex* index = static_cast<MaxLevelIndex*>(itr->second);[m
             index->Search(point, point, [&](const unsigned& level)[m
[36m@@ -149,6 +221,22 @@[m [mElevationPool::getLOD(double x, double y, WorkingSet* ws)[m
     return maxiestMaxLevel;[m
 }[m
 [m
[32m+[m[32mbool[m
[32m+[m[32mElevationPool::needsRefresh()[m
[32m+[m[32m{[m
[32m+[m[32m    ScopedReadLock lk(_mutex);[m
[32m+[m
[32m+[m[32m    // Check to see if the overall data model has changed in the map[m
[32m+[m[32m    int mapRevision = _map.valid() ? static_cast<int>(_map->getDataModelRevision()) : 0;[m
[32m+[m[32m    if (mapRevision != _mapRevision)[m
[32m+[m[32m    {[m
[32m+[m[32m        return true;[m
[32m+[m[32m    }[m
[32m+[m
[32m+[m[32m    // Check to see if any of the elevation layers in our list have changed.[m
[32m+[m[32m    return getElevationHash(nullptr) != _elevationHash;[m
[32m+[m[32m}[m
[32m+[m
 ElevationPool::WorkingSet::WorkingSet(unsigned size) :[m
     _lru(size)[m
 {[m
[36m@@ -204,7 +292,12 @@[m [mElevationPool::findExistingRaster([m
 }[m
 [m
 osg::ref_ptr<ElevationTile>[m
[31m-ElevationPool::getOrCreateRaster(MapData& snapshot, const Internal::RevElevationKey& key, bool acceptLowerRes, ProgressCallback* progress)[m
[32m+[m[32mElevationPool::getOrCreateRaster([m
[32m+[m[32m    const Internal::RevElevationKey& key,[m
[32m+[m[32m    const Map* map,[m
[32m+[m[32m    bool acceptLowerRes,[m
[32m+[m[32m    WorkingSet* ws,[m
[32m+[m[32m    ProgressCallback* progress)[m
 {[m
     OE_PROFILING_ZONE;[m
 [m
[36m@@ -229,14 +322,20 @@[m [mElevationPool::getOrCreateRaster(MapData& snapshot, const Internal::RevElevation[m
         TileKey keyToUse;[m
         bool populated = false;[m
 [m
[31m-        for (keyToUse = key._tilekey; keyToUse.valid(); keyToUse.makeParent())[m
[32m+[m[32m        const ElevationLayerVector& layersToSample =[m
[32m+[m[32m            ws && !ws->_elevationLayers.empty() ? ws->_elevationLayers :[m
[32m+[m[32m            _elevationLayers;[m
[32m+[m
[32m+[m[32m        for (keyToUse = key._tilekey;[m
[32m+[m[32m            keyToUse.valid();[m
[32m+[m[32m            keyToUse.makeParent())[m
         {[m
[31m-            populated = snapshot.layers.populateHeightField([m
[32m+[m[32m            populated = layersToSample.populateHeightField([m
                 hf.get(),[m
                 &resolutions,[m
                 keyToUse,[m
[31m-                snapshot.mapProfileNoVDatum.get(),[m
[31m-                snapshot.interpolation,[m
[32m+[m[32m                map->getProfileNoVDatum(),[m
[32m+[m[32m                map->getElevationInterpolation(),[m
                 progress);[m
 [m
             // Resolve any invalid heights in the output heightfield.[m
[36m@@ -280,6 +379,10 @@[m [mElevationPool::getOrCreateRaster(MapData& snapshot, const Internal::RevElevation[m
         }[m
     }[m
 [m
[32m+[m[32m    // update WorkingSet:[m
[32m+[m[32m    if (ws)[m
[32m+[m[32m        ws->_lru.insert(key, result);[m
[32m+[m
     // update the L2 cache:[m
     _L2.insert(key, result);[m
 [m
[36m@@ -295,23 +398,24 @@[m [mElevationPool::getOrCreateRaster(MapData& snapshot, const Internal::RevElevation[m
 [m
 [m
 bool[m
[31m-ElevationPool::prepareEnvelope(ElevationPool::Envelope& env, const GeoPoint& refPoint,[m
[31m-    const Distance& resolution, WorkingSet* ws)[m
[32m+[m[32mElevationPool::prepareEnvelope([m
[32m+[m[32m    ElevationPool::Envelope& env,[m
[32m+[m[32m    const GeoPoint& refPoint,[m
[32m+[m[32m    const Distance& resolution,[m
[32m+[m[32m    WorkingSet* ws)[m
 {[m
[31m-    env._ws = ws;[m
[31m-[m
[31m-    env._mapDataSnapshot = snapshotMapData(ws);[m
[31m-[m
     env._pool = this;[m
     env._map = nullptr;[m
     env._profile = nullptr;[m
 [m
[31m-    if (_mapData.map.lock(env._map) == false || env._map->getProfile() == nullptr)[m
[32m+[m[32m    if (_map.lock(env._map) == false || env._map->getProfile() == nullptr)[m
         return false;[m
 [m
     env._profile = env._map->getProfile();[m
 [m
[31m-    env._key._revision = env._mapDataSnapshot.hash;[m
[32m+[m[32m    sync(env._map.get(), ws);[m
[32m+[m
[32m+[m[32m    env._key._revision = getElevationHash(ws);[m
 [m
     env._raster = nullptr;[m
     env._cache.clear();[m
[36m@@ -335,7 +439,7 @@[m [mElevationPool::prepareEnvelope(ElevationPool::Envelope& env, const GeoPoint& ref[m
     env._lod = std::min(getLOD(refPointMap.x(), refPointMap.y(), ws), (int)maxLOD);[m
 [m
     // This can happen if the elevation data publishes no data extents[m
[31m-    if (env._lod < 0 && !_mapData.layers.empty())[m
[32m+[m[32m    if (env._lod < 0 && !_elevationLayers.empty())[m
     {[m
         env._lod = maxLOD;[m
     }[m
[36m@@ -351,8 +455,11 @@[m [mElevationPool::prepareEnvelope(ElevationPool::Envelope& env, const GeoPoint& ref[m
 }[m
 [m
 int[m
[31m-ElevationPool::Envelope::sampleMapCoords(std::vector<osg::Vec3d>::iterator begin, std::vector<osg::Vec3d>::iterator end,[m
[31m-    ProgressCallback* progress, float failValue)[m
[32m+[m[32mElevationPool::Envelope::sampleMapCoords([m
[32m+[m[32m    std::vector<osg::Vec3d>::iterator begin,[m
[32m+[m[32m    std::vector<osg::Vec3d>::iterator end,[m
[32m+[m[32m    ProgressCallback* progress,[m
[32m+[m[32m    float failValue)[m
 {[m
     OE_PROFILING_ZONE;[m
 [m
[36m@@ -361,11 +468,13 @@[m [mElevationPool::Envelope::sampleMapCoords(std::vector<osg::Vec3d>::iterator begin[m
 [m
     if (_lod < 0)[m
     {[m
[31m-        for (auto i = begin; i != end; ++i)[m
[32m+[m[32m        for(auto i = begin; i != end; ++i)[m
             i->z() = failValue;[m
         return 0;[m
     }[m
 [m
[32m+[m[32m    ScopedReadLock lk(_pool->_mutex);[m
[32m+[m
     double u, v;[m
     double rx, ry;[m
     int tx, ty;[m
[36m@@ -404,9 +513,10 @@[m [mElevationPool::Envelope::sampleMapCoords(std::vector<osg::Vec3d>::iterator begin[m
                 if (iter == _cache.end())[m
                 {[m
                     _raster = _pool->getOrCreateRaster([m
[31m-                        _mapDataSnapshot,[m
                         _key,   // key to query[m
[32m+[m[32m                        _map.get(), // map to query[m
                         true,  // fall back on lower resolution data if necessary[m
[32m+[m[32m                        _ws,    // user's workingset[m
                         progress);[m
 [m
                     // bail on cancelation before using the quickcache[m
[36m@@ -416,9 +526,6 @@[m [mElevationPool::Envelope::sampleMapCoords(std::vector<osg::Vec3d>::iterator begin[m
                     }[m
 [m
                     _cache[_key] = _raster.get();[m
[31m-[m
[31m-                    if (_ws)[m
[31m-                        _ws->_lru.insert(_key, _raster);[m
                 }[m
                 else[m
                 {[m
[36m@@ -469,20 +576,22 @@[m [mElevationPool::sampleMapCoords([m
         return -1;[m
 [m
     osg::ref_ptr<const Map> map;[m
[31m-    if (_mapData.map.lock(map) == false || map->getProfile() == NULL)[m
[32m+[m[32m    if (_map.lock(map) == false || map->getProfile() == NULL)[m
         return -1;[m
 [m
[31m-    auto snapshot = snapshotMapData(ws);[m
[32m+[m[32m    sync(map.get(), ws);[m
 [m
[31m-    if (snapshot.layers.empty())[m
[32m+[m[32m    if (_elevationLayers.empty())[m
     {[m
         for (auto i = begin; i != end; ++i)[m
             i->z() = failValue;[m
         return 0;[m
     }[m
 [m
[32m+[m[32m    ScopedReadLock lk(_mutex);[m
[32m+[m
     Internal::RevElevationKey key;[m
[31m-    key._revision = snapshot.hash;[m
[32m+[m[32m    key._revision = getElevationHash(ws);[m
 [m
     osg::ref_ptr<ElevationTile> raster;[m
     osg::Vec4 elev;[m
[36m@@ -520,7 +629,7 @@[m [mElevationPool::sampleMapCoords([m
 [m
         pointRes.set(p.w(), units);[m
 [m
[31m-            double resolutionInMapUnits = srs->transformDistance(pointRes, units, p.y());[m
[32m+[m[32m        double resolutionInMapUnits = srs->transformDistance(pointRes, units, p.y());[m
 [m
         lod = profile->getLevelOfDetailForHorizResolution([m
             resolutionInMapUnits,[m
[36m@@ -549,9 +658,12 @@[m [mElevationPool::sampleMapCoords([m
 [m
                 if (iter == quickCache.end())[m
                 {[m
[31m-                    const bool fallback_if_necessary = true;[m
[31m-[m
[31m-                    raster = getOrCreateRaster(snapshot, key, fallback_if_necessary, progress);[m
[32m+[m[32m                    raster = getOrCreateRaster([m
[32m+[m[32m                        key,   // key to query[m
[32m+[m[32m                        map.get(), // map to query[m
[32m+[m[32m                        true,  // fall back on lower resolution data if necessary[m
[32m+[m[32m                        ws,    // user's workingset[m
[32m+[m[32m                        progress);[m
 [m
                     // bail on cancelation before using the quickcache[m
                     if (progress && progress->isCanceled())[m
[36m@@ -611,21 +723,22 @@[m [mElevationPool::sampleMapCoords([m
         return -1;[m
 [m
     osg::ref_ptr<const Map> map;[m
[31m-    if (_mapData.map.lock(map) == false || map->getProfile() == NULL)[m
[32m+[m[32m    if (_map.lock(map) == false || map->getProfile() == NULL)[m
         return -1;[m
 [m
[31m-    auto snapshot = snapshotMapData(ws);[m
[32m+[m[32m    sync(map.get(), ws);[m
 [m
[31m-    if (snapshot.layers.empty())[m
[32m+[m[32m    if (_elevationLayers.empty())[m
     {[m
         for (auto i = begin; i != end; ++i)[m
             i->z() = failValue;[m
         return 0;[m
     }[m
 [m
[32m+[m[32m    ScopedReadLock lk(_mutex);[m
[32m+[m
     Internal::RevElevationKey key;[m
[31m-    key._revision = snapshot.hash;[m
[31m-        [m
[32m+[m[32m    key._revision = getElevationHash(ws);[m
 [m
     osg::ref_ptr<ElevationTile> raster;[m
     double u, v;[m
[36m@@ -692,7 +805,12 @@[m [mElevationPool::sampleMapCoords([m
 [m
                 if (iter == quickCache.end())[m
                 {[m
[31m-                    raster = getOrCreateRaster(snapshot, key, true /* fallback */, progress);[m
[32m+[m[32m                    raster = getOrCreateRaster([m
[32m+[m[32m                        key,   // key to query[m
[32m+[m[32m                        map.get(), // map to query[m
[32m+[m[32m                        true,  // fall back on lower resolution data if necessary[m
[32m+[m[32m                        ws,    // user's workingset[m
[32m+[m[32m                        progress);[m
 [m
                     // bail on cancelation before using the quickcache[m
                     if (progress && progress->isCanceled())[m
[36m@@ -738,13 +856,21 @@[m [mElevationPool::sampleMapCoords([m
 }[m
 [m
 ElevationSample[m
[31m-ElevationPool::getSample(const GeoPoint& p, unsigned maxLOD, WorkingSet* ws, ProgressCallback* progress)[m
[32m+[m[32mElevationPool::getSample([m
[32m+[m[32m    const GeoPoint& p,[m
[32m+[m[32m    unsigned maxLOD,[m
[32m+[m[32m    const Map* map,[m
[32m+[m[32m    WorkingSet* ws,[m
[32m+[m[32m    ProgressCallback* progress)[m
 {[m
[31m-    auto snapshot = snapshotMapData(ws);[m
[32m+[m[32m    // ensure the Pool is in sync with the map[m
[32m+[m[32m    sync(map, ws);[m
 [m
[31m-    if (snapshot.layers.empty())[m
[32m+[m[32m    if (_elevationLayers.empty())[m
         return {};[m
 [m
[32m+[m[32m    ScopedReadLock lk(_mutex);[m
[32m+[m
     Internal::RevElevationKey key;[m
 [m
     // Need to limit maxLOD <= INT_MAX else std::min for lod will return -1 due to cast[m
[36m@@ -755,20 +881,18 @@[m [mElevationPool::getSample(const GeoPoint& p, unsigned maxLOD, WorkingSet* ws, Pro[m
 [m
     if (lod >= 0)[m
     {[m
[31m-        key._tilekey = snapshot.mapProfile->createTileKey(p.x(), p.y(), lod);[m
[31m-        key._revision = snapshot.hash;[m
[32m+[m[32m        key._tilekey = map->getProfile()->createTileKey(p.x(), p.y(), lod);[m
[32m+[m[32m        key._revision = getElevationHash(ws);[m
 [m
         osg::ref_ptr<ElevationTile> raster = getOrCreateRaster([m
[31m-            snapshot,[m
             key,   // key to query[m
[32m+[m[32m            map,   // map to query[m
             true,  // fall back on lower resolution data if necessary[m
[32m+[m[32m            ws,    // user's workingset[m
             progress);[m
 [m
         if (raster.valid())[m
         {[m
[31m-            if (ws)[m
[31m-                ws->_lru.insert(key, raster);[m
[31m-[m
             return raster->getElevation(p.x(), p.y());[m
         }[m
     }[m
[36m@@ -776,41 +900,49 @@[m [mElevationPool::getSample(const GeoPoint& p, unsigned maxLOD, WorkingSet* ws, Pro[m
 }[m
 [m
 ElevationSample[m
[31m-ElevationPool::getSample(const GeoPoint& p, WorkingSet* ws, ProgressCallback* progress)[m
[32m+[m[32mElevationPool::getSample([m
[32m+[m[32m    const GeoPoint& p,[m
[32m+[m[32m    WorkingSet* ws,[m
[32m+[m[32m    ProgressCallback* progress)[m
 {[m
     if (!p.isValid())[m
         return {};[m
 [m
[31m-    osg::ref_ptr<const Map> map;[m
[31m-    if (_mapData.map.lock(map) == false || map->getProfile() == nullptr)[m
[32m+[m[32m    osg::ref_ptr<const Map> map = _map.get();[m
[32m+[m[32m    if (!map.valid() || !map->getProfile())[m
[32m+[m[32m        return {};[m
[32m+[m
[32m+[m[32m    if (_elevationLayers.empty())[m
         return {};[m
 [m
[31m-    if (!p.getSRS()->isHorizEquivalentTo(map->getSRS()))[m
[32m+[m[32m    if (!p.getSRS()->isHorizEquivalentTo(map->getProfile()->getSRS()))[m
     {[m
         GeoPoint xp(p);[m
[31m-        xp.transformInPlace(map->getSRS());[m
[31m-        return getSample(xp, ~0, ws, progress);[m
[32m+[m[32m        xp.transformInPlace(map->getProfile()->getSRS());[m
[32m+[m[32m        return getSample(xp, ~0, map.get(), ws, progress);[m
     }[m
     else[m
     {[m
[31m-        return getSample(p, ~0, ws, progress);[m
[32m+[m[32m        return getSample(p, ~0, map.get(), ws, progress);[m
     }[m
 }[m
 [m
 ElevationSample[m
[31m-ElevationPool::getSample(const GeoPoint& p, const Distance& resolution, WorkingSet* ws, ProgressCallback* progress)[m
[32m+[m[32mElevationPool::getSample([m
[32m+[m[32m    const GeoPoint& p,[m
[32m+[m[32m    const Distance& resolution,[m
[32m+[m[32m    WorkingSet* ws,[m
[32m+[m[32m    ProgressCallback* progress)[m
 {[m
     if (!p.isValid())[m
         return {};[m
 [m
[32m+[m[32m    if (_elevationLayers.empty())[m
[32m+[m[32m        return {};[m
[32m+[m
     osg::ref_ptr<const Map> map;[m
[31m-    {[m
[31m-        ScopedReadLock lock(_mapDataMutex);[m
[31m-        if (_mapData.layers.empty())[m
[31m-            return {};[m
[31m-        if (_mapData.map.lock(map) == false || map->getProfile() == nullptr)[m
[31m-            return {};[m
[31m-    }[m
[32m+[m[32m    if (_map.lock(map) == false || map->getProfile() == NULL)[m
[32m+[m[32m        return {};[m
 [m
     // mostly right. :)[m
     double resolutionInMapUnits = SpatialReference::transformUnits([m
[36m@@ -822,59 +954,55 @@[m [mElevationPool::getSample(const GeoPoint& p, const Distance& resolution, WorkingS[m
         resolutionInMapUnits,[m
         ELEVATION_TILE_SIZE);[m
 [m
[31m-    if (!p.getSRS()->isHorizEquivalentTo(map->getSRS()))[m
[32m+[m[32m    if (!p.getSRS()->isHorizEquivalentTo(map->getProfile()->getSRS()))[m
     {[m
         GeoPoint xp(p);[m
[31m-        xp.transformInPlace(map->getSRS());[m
[31m-        return getSample(xp, maxLOD, ws, progress);[m
[32m+[m[32m        xp.transformInPlace(map->getProfile()->getSRS());[m
[32m+[m[32m        return getSample(xp, maxLOD, map.get(), ws, progress);[m
     }[m
     else[m
     {[m
[31m-        return getSample(p, maxLOD, ws, progress);[m
[32m+[m[32m        return getSample(p, maxLOD, map.get(), ws, progress);[m
     }[m
 }[m
 [m
 bool[m
[31m-ElevationPool::getTile(const TileKey& tilekey, bool acceptLowerRes, osg::ref_ptr<ElevationTexture>& out_tex,[m
[31m-    WorkingSet* ws, ProgressCallback* progress)[m
[32m+[m[32mElevationPool::getTile([m
[32m+[m[32m    const TileKey& tilekey,[m
[32m+[m[32m    bool acceptLowerRes,[m
[32m+[m[32m    osg::ref_ptr<ElevationTile>& out_tex,[m
[32m+[m[32m    WorkingSet* ws,[m
[32m+[m[32m    ProgressCallback* progress)[m
 {[m
[31m-    MapData snapshot = snapshotMapData(ws);[m
[32m+[m[32m    osg::ref_ptr<const Map> map;[m
[32m+[m[32m    if (!_map.lock(map))[m
[32m+[m[32m        return false;[m
[32m+[m
[32m+[m[32m    // ensure we are in sync with the map[m
[32m+[m[32m    sync(map.get(), ws);[m
[32m+[m
[32m+[m[32m    ScopedReadLock lk(_mutex);[m
 [m
     Internal::RevElevationKey key;[m
     key._tilekey = tilekey;[m
[31m-    key._revision = snapshot.hash;[m
[31m-[m
[31m-    out_tex = getOrCreateRaster(snapshot, key, acceptLowerRes, progress);[m
[32m+[m[32m    key._revision = getElevationHash(ws);[m
 [m
[31m-    if (ws)[m
[31m-        ws->_lru.insert(key, out_tex);[m
[32m+[m[32m    out_tex = getOrCreateRaster([m
[32m+[m[32m        key,[m
[32m+[m[32m        _map.get(),[m
[32m+[m[32m        acceptLowerRes,[m
[32m+[m[32m        ws,[m
[32m+[m[32m        progress);[m
 [m
     return out_tex.valid();[m
 }[m
 [m
[31m-ElevationPool::MapData[m
[31m-ElevationPool::snapshotMapData(WorkingSet* ws)[m
[31m-{[m
[31m-    ScopedWriteLock exclusive(_mapDataMutex);[m
[31m-[m
[31m-    // check for revision change.[m
[31m-    unsigned hash = 0;[m
[31m-    for (auto& layer : _mapData.layers)[m
[31m-        hash = hash_value_unsigned(hash, layer->getRevision());[m
[31m-[m
[31m-    // update if necessary.[m
[31m-    _mapData.hash = hash;[m
[31m-[m
[31m-    MapData out = _mapData;[m
[31m-    if (ws && !ws->_elevationLayers.empty())[m
[31m-        out.layers = ws->_elevationLayers;[m
[31m-[m
[31m-    return out;[m
[31m-}[m
[31m-[m
 //...................................................................[m
 [m
[31m-AsyncElevationSampler::AsyncElevationSampler(const Map* map, unsigned numThreads) :[m
[32m+[m[32mAsyncElevationSampler::AsyncElevationSampler([m
[32m+[m[32m    const Map* map,[m
[32m+[m[32m    unsigned numThreads) :[m
[32m+[m
     _map(map),[m
     _arena(nullptr)[m
 {[m
[1mdiff --git a/src/osgEarth/ExtrudeGeometryFilter.cpp b/src/osgEarth/ExtrudeGeometryFilter.cpp[m
[1mindex 578c4e976..f752e6b3a 100644[m
[1m--- a/src/osgEarth/ExtrudeGeometryFilter.cpp[m
[1m+++ b/src/osgEarth/ExtrudeGeometryFilter.cpp[m
[36m@@ -1295,7 +1295,7 @@[m [mExtrudeGeometryFilter::process( FeatureList& features, FilterContext& context )[m
                 }[m
                     [m
                 // prep the shapes by making sure all polys are open:[m
[31m-                static_cast<osgEarth::Polygon*>(part)->open();[m
[32m+[m[32m                static_cast<Polygon*>(part)->open();[m
             }[m
 [m
             // make a base cap if we're doing stencil volumes.[m
[1mdiff --git a/src/osgEarth/FeatureRasterizer.cpp b/src/osgEarth/FeatureRasterizer.cpp[m
[1mindex f4fde44b4..35d359916 100644[m
[1m--- a/src/osgEarth/FeatureRasterizer.cpp[m
[1m+++ b/src/osgEarth/FeatureRasterizer.cpp[m
[36m@@ -1156,7 +1156,7 @@[m [mFeatureRasterizer::render_agglite([m
         }[m
     }[m
 [m
[31m-    osgEarth::Polygon cropPoly(4);[m
[32m+[m[32m    Polygon cropPoly(4);[m
     cropPoly.push_back(osg::Vec3d(cropXMin, cropYMin, 0));[m
     cropPoly.push_back(osg::Vec3d(cropXMax, cropYMin, 0));[m
     cropPoly.push_back(osg::Vec3d(cropXMax, cropYMax, 0));[m
[1mdiff --git a/src/osgEarth/FlatteningLayer.cpp b/src/osgEarth/FlatteningLayer.cpp[m
[1mindex bad4b006b..9eafb2dee 100644[m
[1m--- a/src/osgEarth/FlatteningLayer.cpp[m
[1m+++ b/src/osgEarth/FlatteningLayer.cpp[m
[36m@@ -71,7 +71,7 @@[m [mnamespace[m
     // This will not always work with polygons that contain holes,[m
     // so we need to come up with a different algorithm if this becomes a problem.[m
     // Maybe try a random point generator and profile it.[m
[31m-    osg::Vec3d inline getInternalPoint(const osgEarth::Polygon* p)[m
[32m+[m[32m    osg::Vec3d inline getInternalPoint(const Polygon* p)[m
     {[m
         // Simple test: if the centroid is in the polygon, use it.[m
         osg::Vec3d centroid = p->getBounds().center();[m
[36m@@ -139,7 +139,7 @@[m [mnamespace[m
         return p->getBounds().center();[m
     }[m
 [m
[31m-    double getDistanceSquaredToClosestEdge(const osg::Vec3d& P, const osgEarth::Polygon* poly)[m
[32m+[m[32m    double getDistanceSquaredToClosestEdge(const osg::Vec3d& P, const Polygon* poly)[m
     {[m
         double Dmin = DBL_MAX;[m
         ConstSegmentIterator seg_iter(poly, true);[m
[36m@@ -217,7 +217,7 @@[m [mnamespace[m
                 double minD2 = DBL_MAX;//bufferWidth * bufferWidth; // minimum distance(squared) to closest polygon edge[m
                 double bufferWidth = 0.0;[m
 [m
[31m-                const osgEarth::Polygon* bestPoly = 0L;[m
[32m+[m[32m                const Polygon* bestPoly = 0L;[m
 [m
                 for (unsigned int geomIndex = 0; geomIndex < geom->getNumComponents(); geomIndex++)[m
                 {[m
[36m@@ -230,7 +230,7 @@[m [mnamespace[m
                         auto part = giter.next();[m
                         if (part->getType() == Geometry::TYPE_POLYGON)[m
                         {[m
[31m-                            auto polygon = static_cast<const osgEarth::Polygon*>(part);[m
[32m+[m[32m                            auto polygon = static_cast<const Polygon*>(part);[m
 [m
                             // Does the point P fall within the polygon?[m
                             if (polygon->contains2D(P.x(), P.y()))[m
[36m@@ -880,7 +880,7 @@[m [mFlatteningLayer::addedToMap(const Map* map)[m
     // Initialize the elevation pool with our map:[m
     //OE_DEBUG << LC << "Attaching elevation pool to map" << std::endl;[m
     _pool = map->getElevationPool();[m
[31m-    //_pool->setMap(map);[m
[32m+[m[32m    _pool->setMap(map);[m
 [m
     // Make a feature session[m
     _session = new Session(map);[m
[1mdiff --git a/src/osgEarth/GDAL.cpp b/src/osgEarth/GDAL.cpp[m
[1mindex 3cad0b5e0..410b3f907 100644[m
[1m--- a/src/osgEarth/GDAL.cpp[m
[1m+++ b/src/osgEarth/GDAL.cpp[m
[36m@@ -1405,71 +1405,49 @@[m [mGDAL::Driver::createHeightField(const TileKey& key, unsigned tileSize, ProgressC[m
         // Raw pointer to the height data output block:[m
         float* hf_raw = (float*)hf->getFloatArray()->getDataPointer();[m
 [m
[31m-        int hasNoDataValue = 0;[m
[31m-        float noDataValue = band->GetNoDataValue(&hasNoDataValue);[m
[31m-        if (!hasNoDataValue) noDataValue = NO_DATA_VALUE;[m
[31m-[m
         // If the interpolation is not nearest neighbor, we will use the[m
         // high-res sampling path. This is not terribly fast but it's accurate.[m
         if (gdalOptions().interpolation() != INTERP_NEAREST)[m
         {[m
[31m-            bool done = false;[m
[31m-[m
 #if GDAL_VERSION_NUM >= 3100000 // 3.10+[m
[32m+[m[32m            double ri, ci, realPart;[m
 [m
[31m-            if (!hasNoDataValue)[m
[31m-            {[m
[31m-                double ri, ci, realPart;[m
[32m+[m[32m            GDALRIOResampleAlg alg =[m
[32m+[m[32m                gdalOptions().interpolation() == INTERP_AVERAGE ? GRIORA_Average : // note: broken[m
[32m+[m[32m                gdalOptions().interpolation() == INTERP_BILINEAR ? GRIORA_Bilinear :[m
[32m+[m[32m                gdalOptions().interpolation() == INTERP_CUBIC ? GRIORA_Cubic :[m
[32m+[m[32m                gdalOptions().interpolation() == INTERP_CUBICSPLINE ? GRIORA_CubicSpline :[m
[32m+[m[32m                GRIORA_NearestNeighbour;[m
 [m
[31m-                GDALRIOResampleAlg alg =[m
[31m-                    gdalOptions().interpolation() == INTERP_AVERAGE ? GRIORA_Average : // note: broken[m
[31m-                    gdalOptions().interpolation() == INTERP_BILINEAR ? GRIORA_Bilinear :[m
[31m-                    gdalOptions().interpolation() == INTERP_CUBIC ? GRIORA_Cubic :[m
[31m-                    gdalOptions().interpolation() == INTERP_CUBICSPLINE ? GRIORA_CubicSpline :[m
[31m-                    GRIORA_NearestNeighbour;[m
[31m-[m
[31m-                for (unsigned r = 0; r < tileSize; ++r)[m
[32m+[m[32m            for (unsigned r = 0; r < tileSize; ++r)[m
[32m+[m[32m            {[m
[32m+[m[32m                double y = tile_ymin + (dy * (double)r);[m
[32m+[m[32m                for (unsigned c = 0; c < tileSize; ++c)[m
                 {[m
[31m-                    double y = tile_ymin + (dy * (double)r);[m
[31m-                    for (unsigned c = 0; c < tileSize; ++c)[m
[31m-                    {[m
[31m-                        double x = tile_xmin + (dx * (double)c);[m
[31m-                        GEO_TO_PIXEL(x, y, ci, ri);[m
[32m+[m[32m                    double x = tile_xmin + (dx * (double)c);[m
[32m+[m[32m                    GEO_TO_PIXEL(x, y, ci, ri);[m
 [m
[31m-                        // this function applies the 1/2 pixel offset for us for DEMs[m
[31m-                        double realPart = 0.0;[m
[31m-                        auto err = band->InterpolateAtPoint(ci, ri, alg, &realPart, nullptr);[m
[31m-                        if (err == CE_None)[m
[31m-                        {[m
[31m-                            hf->setHeight(c, r, (float)realPart * _linearUnits);[m
[31m-                        }[m
[32m+[m[32m                    // this function applies the 1/2 pixel offset for us for DEMs[m
[32m+[m[32m                    double realPart = 0.0;[m
[32m+[m[32m                    auto err = band->InterpolateAtPoint(ci, ri, alg, &realPart, nullptr);[m
[32m+[m[32m                    if (err == CE_None)[m
[32m+[m[32m                    {[m
[32m+[m[32m                        hf->setHeight(c, r, (float)realPart * _linearUnits);[m
                     }[m
                 }[m
[31m-                done = true;[m
             }[m
[31m-#endif[m
[31m-[m
[31m-            if (!done)[m
[32m+[m[32m#else[m
[32m+[m[32m            for (unsigned r = 0; r < tileSize; ++r)[m
             {[m
[31m-                // fallback in case either InterpolateAtPoint is not available, or there's a nodata value[m
[31m-                // (which is not supported by InterpolateAtPoint)[m
[31m-                for (unsigned r = 0; r < tileSize; ++r)[m
[32m+[m[32m                double y = tile_ymin + (dy * (double)r);[m
[32m+[m[32m                for (unsigned c = 0; c < tileSize; ++c)[m
                 {[m
[31m-                    double y = tile_ymin + (dy * (double)r);[m
[31m-                    for (unsigned c = 0; c < tileSize; ++c)[m
[31m-                    {[m
[31m-                        double x = tile_xmin + (dx * (double)c);[m
[31m-                        auto h = getInterpolatedDEMValue(band, x, y, true);[m
[31m-                        if (isValidValue(h, noDataValue))[m
[31m-                            h *= _linearUnits;[m
[31m-                        else[m
[31m-                            h = NO_DATA_VALUE;[m
[31m-[m
[31m-                        hf->setHeight(c, r, h);[m
[31m-                    }[m
[32m+[m[32m                    double x = tile_xmin + (dx * (double)c);[m
[32m+[m[32m                    float h = getInterpolatedDEMValue(band, x, y, true) * _linearUnits;[m
[32m+[m[32m                    hf->setHeight(c, r, h);[m
                 }[m
[31m-                done = true;[m
             }[m
[32m+[m[32m#endif[m
         }[m
 [m
         else // NEAREST NEIGHBOR fast path[m
[36m@@ -1490,10 +1468,6 @@[m [mGDAL::Driver::createHeightField(const TileKey& key, unsigned tileSize, ProgressC[m
                 (void*)hf_raw, tileSize, tileSize,[m
                 GDT_Float32, 0, 0);[m
 [m
[31m-            int gotNoDataValue = 0;[m
[31m-            float noDataValue = band->GetNoDataValue(&gotNoDataValue);[m
[31m-            if (!gotNoDataValue) noDataValue = NO_DATA_VALUE;[m
[31m-[m
             if (read_error != CE_None)[m
             {[m
                 //OE_WARN << LC << "RasterIO failed.\n";[m
[36m@@ -1509,10 +1483,7 @@[m [mGDAL::Driver::createHeightField(const TileKey& key, unsigned tileSize, ProgressC[m
                     if (t < halfHeight)[m
                         std::swap(hf_raw[t * tileSize + s], hf_raw[(tileSize - t - 1) * tileSize + s]);[m
 [m
[31m-                    if (isValidValue(hf_raw[t * tileSize + s], noDataValue))[m
[31m-                        hf_raw[t * tileSize + s] *= _linearUnits; // apply linear units[m
[31m-                    else[m
[31m-                        hf_raw[t * tileSize + s] = NO_DATA_VALUE;[m
[32m+[m[32m                    hf_raw[t * tileSize + s] *= _linearUnits; // apply linear units[m
                 }[m
             }[m
         }[m
[1mdiff --git a/src/osgEarth/GDALDEM.cpp b/src/osgEarth/GDALDEM.cpp[m
[1mindex 3cd1070f4..6fb944fa7 100644[m
[1m--- a/src/osgEarth/GDALDEM.cpp[m
[1m+++ b/src/osgEarth/GDALDEM.cpp[m
[36m@@ -370,7 +370,7 @@[m [mGDALDEMLayer::createImageImplementation(const TileKey& key, ProgressCallback* pr[m
     if (!_map.lock(map))[m
         return {};[m
 [m
[31m-    osg::ref_ptr<ElevationTile> tile;[m
[32m+[m[32m    osg::ref_ptr<ElevationTexture> tile;[m
     if (!map->getElevationPool()->getTile(key, false, tile, nullptr, progress))[m
         return {};[m
 [m
[1mdiff --git a/src/osgEarth/GEOS.cpp b/src/osgEarth/GEOS.cpp[m
[1mindex 4eb015707..b7821bba0 100644[m
[1m--- a/src/osgEarth/GEOS.cpp[m
[1m+++ b/src/osgEarth/GEOS.cpp[m
[36m@@ -130,7 +130,7 @@[m [mnamespace[m
 [m
                 if (shell)[m
                 {[m
[31m-                    const osgEarth::Polygon* poly = static_cast<const osgEarth::Polygon*>(input);[m
[32m+[m[32m                    const Polygon* poly = static_cast<const Polygon*>(input);[m
                     std::vector<GEOSGeom> holes;[m
 [m
                     if (poly->getHoles().size() > 0)[m
[36m@@ -165,7 +165,7 @@[m [mnamespace[m
     Geometry*[m
         exportPolygon_c(GEOSContextHandle_t handle, const GEOSGeometry* input)[m
     {[m
[31m-        osgEarth::Polygon* output = 0L;[m
[32m+[m[32m        Polygon* output = 0L;[m
 [m
         const GEOSGeometry* outerRing = GEOSGetExteriorRing_r(handle, input);[m
 [m
[36m@@ -175,7 +175,7 @@[m [mnamespace[m
 [m
             unsigned int outerSize;[m
             GEOSCoordSeq_getSize_r(handle, s, &outerSize);[m
[31m-            output = new osgEarth::Polygon(outerSize);[m
[32m+[m[32m            output = new Polygon(outerSize);[m
 [m
             for (unsigned int j = 0; j < outerSize; j++)[m
             {[m
[1mdiff --git a/src/osgEarth/GLUtils.cpp b/src/osgEarth/GLUtils.cpp[m
[1mindex b1fcd234c..f6f88e3a5 100644[m
[1m--- a/src/osgEarth/GLUtils.cpp[m
[1m+++ b/src/osgEarth/GLUtils.cpp[m
[36m@@ -1239,7 +1239,6 @@[m [mGLTexture::Profile::Profile([m
     GLint     wrapT,[m
     GLint     wrapR,[m
     GLfloat   maxAnisotropy) :[m
[31m-[m
     osg::Texture::TextureProfile([m
         target,[m
         numMipmapLevels,[m
[36m@@ -1253,130 +1252,7 @@[m [mGLTexture::Profile::Profile([m
     _wrapR(wrapR),[m
     _maxAnisotropy(maxAnisotropy)[m
 {[m
[31m-    // compute size ourselves since OSG's is incomplete..[m
[31m-    unsigned int numBitsPerTexel = 32;[m
[31m-[m
[31m-    switch (_internalFormat)[m
[31m-    {[m
[31m-    case 1:[m
[31m-    case GL_ALPHA:[m
[31m-    case GL_LUMINANCE:[m
[31m-    case GL_INTENSITY:[m
[31m-    case GL_RED:[m
[31m-    case GL_R8:[m
[31m-        numBitsPerTexel = 8;[m
[31m-        break;[m
[31m-[m
[31m-    case 2:[m
[31m-    case GL_LUMINANCE_ALPHA:[m
[31m-    case GL_RG:[m
[31m-    case GL_RG8:[m
[31m-    case GL_R16:[m
[31m-    case GL_R16F:[m
[31m-        numBitsPerTexel = 16;[m
[31m-        break;[m
[31m-[m
[31m-    case 3:[m
[31m-    case GL_RGB:[m
[31m-    case GL_BGR:[m
[31m-    case GL_RGB8:[m
[31m-        numBitsPerTexel = 24;[m
[31m-        break;[m
[31m-[m
[31m-    case 4:[m
[31m-    case GL_RGBA:[m
[31m-    case GL_BGRA:[m
[31m-    case GL_RGBA8:[m
[31m-    case GL_R32F:[m
[31m-        numBitsPerTexel = 32;[m
[31m-        break;[m
[31m-[m
[31m-    case GL_COMPRESSED_ALPHA_ARB:                      numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_INTENSITY_ARB:                  numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_LUMINANCE_ALPHA_ARB:            numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_RGB_S3TC_DXT1_EXT:              numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:             numBitsPerTexel = 4;  break;[m
[31m-[m
[31m-    case GL_COMPRESSED_RGB_ARB:                        numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:             numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:             numBitsPerTexel = 8;  break;[m
[31m-[m
[31m-    case GL_COMPRESSED_SIGNED_RED_RGTC1_EXT:           numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_RED_RGTC1_EXT:                  numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT:     numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_RED_GREEN_RGTC2_EXT:            numBitsPerTexel = 8;  break;[m
[31m-[m
[31m-    case GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG:           numBitsPerTexel = 2;  break;[m
[31m-    case GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG:          numBitsPerTexel = 2;  break;[m
[31m-    case GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG:           numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG:          numBitsPerTexel = 4;  break;[m
[31m-[m
[31m-    case GL_ETC1_RGB8_OES:                             numBitsPerTexel = 4;  break;[m
[31m-[m
[31m-    case GL_COMPRESSED_RGB8_ETC2:                      numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_SRGB8_ETC2:                     numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_RGB8_PUNCHTHROUGH_ALPHA1_ETC2:  numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_SRGB8_PUNCHTHROUGH_ALPHA1_ETC2: numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_RGBA8_ETC2_EAC:                 numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:          numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_R11_EAC:                        numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_SIGNED_R11_EAC:                 numBitsPerTexel = 4;  break;[m
[31m-    case GL_COMPRESSED_RG11_EAC:                       numBitsPerTexel = 8;  break;[m
[31m-    case GL_COMPRESSED_SIGNED_RG11_EAC:                numBitsPerTexel = 8;  break;[m
[31m-[m
[31m-        // ASTC[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_4x4_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_5x4_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_5x5_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_6x5_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_6x6_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_8x5_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_8x6_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_8x8_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_10x5_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_10x6_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_10x8_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_10x10_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_12x10_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:[m
[31m-    case GL_COMPRESSED_RGBA_ASTC_12x12_KHR:[m
[31m-    case GL_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:[m
[31m-    {[m
[31m-        _size = 0;[m
[31m-        for (int i = 0; i < std::max(_numMipmapLevels, 1); ++i)[m
[31m-        {[m
[31m-            GLint blockSize;[m
[31m-            GLint size;[m
[31m-            osg::Texture::getCompressedSize(_internalFormat, _width / (1 << i), _height / (1 << i), _depth, blockSize, size);[m
[31m-            _size += size;[m
[31m-        }[m
[31m-        return;[m
[31m-    }[m
[31m-    }[m
[31m-[m
[31m-    _size = (unsigned int)(ceil(double(_width * _height * _depth * numBitsPerTexel) / 8.0));[m
[31m-[m
[31m-    if (_numMipmapLevels > 1)[m
[31m-    {[m
[31m-        unsigned int mipmapSize = _size / 4;[m
[31m-        for (GLint i = 0; i < _numMipmapLevels && mipmapSize != 0; ++i)[m
[31m-        {[m
[31m-            _size += mipmapSize;[m
[31m-            mipmapSize /= 4;[m
[31m-        }[m
[31m-    }[m
[32m+[m[32m    //nop[m
 }[m
 [m
 bool[m
[1mdiff --git a/src/osgEarth/GeometryFactory.cpp b/src/osgEarth/GeometryFactory.cpp[m
[1mindex f7b5ce49e..ea2d83cb5 100644[m
[1m--- a/src/osgEarth/GeometryFactory.cpp[m
[1m+++ b/src/osgEarth/GeometryFactory.cpp[m
[36m@@ -21,7 +21,7 @@[m [mGeometryFactory::createCircle(const osg::Vec3d& center,[m
                               unsigned          numSegments,[m
                               Geometry*         geomToUse) const[m
 {[m
[31m-    Geometry* geom = geomToUse ? geomToUse : new osgEarth::Polygon();[m
[32m+[m[32m    Geometry* geom = geomToUse ? geomToUse : new Polygon();[m
 [m
     if ( numSegments == 0 )[m
     {[m
[36m@@ -142,7 +142,7 @@[m [mGeometryFactory::createEllipse(const osg::Vec3d& center,[m
                                unsigned          numSegments,[m
                                Geometry*         geomToUse) const[m
 {[m
[31m-    Geometry* geom = geomToUse ? geomToUse : new osgEarth::Polygon();[m
[32m+[m[32m    Geometry* geom = geomToUse ? geomToUse : new Polygon();[m
 [m
     if ( numSegments == 0 )[m
     {[m
[36m@@ -288,7 +288,7 @@[m [mGeometryFactory::createRectangle(const osg::Vec3d& center,[m
                                  const Distance&   width,[m
                                  const Distance&   height ) const[m
 {[m
[31m-    Geometry* geom = new osgEarth::Polygon();[m
[32m+[m[32m    Geometry* geom = new Polygon();[m
     [m
     if ( _srs.valid() && _srs->isGeographic() )[m
     {[m
[1mdiff --git a/src/osgEarth/HeightFieldUtils b/src/osgEarth/HeightFieldUtils[m
[1mindex e3bd8e6b5..7b7e60c7f 100644[m
[1m--- a/src/osgEarth/HeightFieldUtils[m
[1m+++ b/src/osgEarth/HeightFieldUtils[m
[36m@@ -183,6 +183,13 @@[m [mnamespace osgEarth { namespace Util[m
             const osg::HeightField* grid, [m
             const Ellipsoid& em,[m
             float verticalScale =1.0f );[m
[32m+[m
[32m+[m[32m        /**[m
[32m+[m[32m         * Utility function that will take sample points used for interpolation and copy valid values into any of the samples that are NO_DATA_VALUE.[m
[32m+[m[32m         * Returns false if all values are NO_DATA_VALUE.[m
[32m+[m[32m         * Returns true if all the values are valid or if we were able to replace NO_DATA_VALUE samples with valid values.[m
[32m+[m[32m         **/[m
[32m+[m[32m        static bool validateSamples(float &a, float &b, float &c, float &d);[m
     };[m
 } }[m
 [m
[1mdiff --git a/src/osgEarth/HeightFieldUtils.cpp b/src/osgEarth/HeightFieldUtils.cpp[m
[1mindex b99b29283..8dff13c15 100644[m
[1m--- a/src/osgEarth/HeightFieldUtils.cpp[m
[1m+++ b/src/osgEarth/HeightFieldUtils.cpp[m
[36m@@ -11,17 +11,34 @@[m [musing namespace osgEarth;[m
 using namespace osgEarth::Util;[m
 [m
 [m
[31m-#define VALIDATE_SAMPLES(a, b, c, d) \[m
[31m-    (((a) == NO_DATA_VALUE && (b) == NO_DATA_VALUE && (c) == NO_DATA_VALUE && (d) == NO_DATA_VALUE) ? false : \[m
[31m-     (((a) == NO_DATA_VALUE || (b) == NO_DATA_VALUE || (c) == NO_DATA_VALUE || (d) == NO_DATA_VALUE) ? \[m
[31m-      ([&]() { \[m
[31m-          float validValue = ((a) != NO_DATA_VALUE) ? (a) : ((b) != NO_DATA_VALUE) ? (b) : ((c) != NO_DATA_VALUE) ? (c) : (d); \[m
[31m-          if ((a) == NO_DATA_VALUE) (a) = validValue; \[m
[31m-          if ((b) == NO_DATA_VALUE) (b) = validValue; \[m
[31m-          if ((c) == NO_DATA_VALUE) (c) = validValue; \[m
[31m-          if ((d) == NO_DATA_VALUE) (d) = validValue; \[m
[31m-          return true; \[m
[31m-      })() : true))[m
[32m+[m[32mbool[m
[32m+[m[32mHeightFieldUtils::validateSamples(float &a, float &b, float &c, float &d)[m
[32m+[m[32m{[m
[32m+[m[32m    // If ALL the sample points are NO_DATA_VALUE then we can't do anything.[m
[32m+[m[32m    if (a == NO_DATA_VALUE && b == NO_DATA_VALUE && c == NO_DATA_VALUE && d == NO_DATA_VALUE)[m
[32m+[m[32m    {[m
[32m+[m[32m        return false;[m
[32m+[m[32m    }[m
[32m+[m
[32m+[m[32m    // If any of the samples are valid but some are NO_DATA_VALUE we can replace the nodata with valid values.[m
[32m+[m[32m    if (a == NO_DATA_VALUE ||[m
[32m+[m[32m        b == NO_DATA_VALUE ||[m[41m [m
[32m+[m[32m        c == NO_DATA_VALUE ||[m
[32m+[m[32m        d == NO_DATA_VALUE)[m
[32m+[m[32m    {[m
[32m+[m[32m        float validValue = a;[m
[32m+[m[32m        if (validValue == NO_DATA_VALUE) validValue = b;[m
[32m+[m[32m        if (validValue == NO_DATA_VALUE) validValue = c;[m
[32m+[m[32m        if (validValue == NO_DATA_VALUE) validValue = d;[m
[32m+[m
[32m+[m[32m        if (a == NO_DATA_VALUE) a = validValue;[m
[32m+[m[32m        if (b == NO_DATA_VALUE) b = validValue;[m
[32m+[m[32m        if (c == NO_DATA_VALUE) c = validValue;[m
[32m+[m[32m        if (d == NO_DATA_VALUE) d = validValue;[m
[32m+[m[32m    }[m
[32m+[m
[32m+[m[32m    return true;[m
[32m+[m[32m}[m
 [m
 float[m
 HeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, double c, double r, RasterInterpolation interpolation)[m
[36m@@ -46,7 +63,7 @@[m [mHeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, double c, double[m
         float lrHeight = hf->getHeight(colMax, rowMin);[m
 [m
         //Make sure not to use NoData in the interpolation[m
[31m-        if (!VALIDATE_SAMPLES(urHeight, llHeight, ulHeight, lrHeight))[m
[32m+[m[32m        if (!validateSamples(urHeight, llHeight, ulHeight, lrHeight))[m
         {[m
             return NO_DATA_VALUE;[m
         }[m
[36m@@ -98,7 +115,7 @@[m [mHeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, double c, double[m
         float lrHeight = hf->getHeight(colMax, rowMin);[m
 [m
         //Make sure not to use NoData in the interpolation[m
[31m-        if (!VALIDATE_SAMPLES(urHeight, llHeight, ulHeight, lrHeight))[m
[32m+[m[32m        if (!validateSamples(urHeight, llHeight, ulHeight, lrHeight))[m
         {[m
             return NO_DATA_VALUE;[m
         }[m
[36m@@ -163,7 +180,7 @@[m [mHeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, double c, double[m
         float lrHeight = hf->getHeight(colMax, rowMin);[m
 [m
         //Make sure not to use NoData in the interpolation[m
[31m-        if (!VALIDATE_SAMPLES(urHeight, llHeight, ulHeight, lrHeight))[m
[32m+[m[32m        if (!validateSamples(urHeight, llHeight, ulHeight, lrHeight))[m
         {[m
             return NO_DATA_VALUE;[m
         }[m
[1mdiff --git a/src/osgEarth/ImageOverlay.cpp b/src/osgEarth/ImageOverlay.cpp[m
[1mindex 13be4fcec..80e67afd4 100644[m
[1m--- a/src/osgEarth/ImageOverlay.cpp[m
[1m+++ b/src/osgEarth/ImageOverlay.cpp[m
[36m@@ -154,7 +154,7 @@[m [mImageOverlay::getConfig() const[m
 [m
     conf.set("alpha", _alpha);[m
 [m
[31m-    osg::ref_ptr<Geometry> g = new osgEarth::Polygon();[m
[32m+[m[32m    osg::ref_ptr<Geometry> g = new Polygon();[m
     g->push_back( osg::Vec3d(_lowerLeft.x(),  _lowerLeft.y(), 0) );[m
     g->push_back( osg::Vec3d(_lowerRight.x(), _lowerRight.y(), 0) );[m
     g->push_back( osg::Vec3d(_upperRight.x(), _upperRight.y(), 0) );[m
[1mdiff --git a/src/osgEarth/ImageToFeatureLayer.cpp b/src/osgEarth/ImageToFeatureLayer.cpp[m
[1mindex 124ad75e0..307bdbd2b 100644[m
[1m--- a/src/osgEarth/ImageToFeatureLayer.cpp[m
[1m+++ b/src/osgEarth/ImageToFeatureLayer.cpp[m
[36m@@ -142,7 +142,7 @@[m [mImageToFeatureSource::createFeatureCursorImplementation(const Query& query, Prog[m
                     {[m
                         // Increment the maxX to finish the row.[m
                         maxX = x + pixWidth;[m
[31m-                        osgEarth::Polygon* poly = new osgEarth::Polygon();[m
[32m+[m[32m                        Polygon* poly = new Polygon();[m
                         poly->push_back(minX, y);[m
                         poly->push_back(maxX, y);[m
                         poly->push_back(maxX, y + pixHeight);[m
[36m@@ -157,7 +157,7 @@[m [mImageToFeatureSource::createFeatureCursorImplementation(const Query& query, Prog[m
                     // The value is different, so complete the polygon and start a new one.[m
                     else if (color.r() != value)[m
                     {[m
[31m-                        osgEarth::Polygon* poly = new osgEarth::Polygon();[m
[32m+[m[32m                        Polygon* poly = new Polygon();[m
                         poly->push_back(minX, y);[m
                         poly->push_back(maxX, y);[m
                         poly->push_back(maxX, y + pixHeight);[m
[1mdiff --git a/src/osgEarth/ImageUtils b/src/osgEarth/ImageUtils[m
[1mindex 35b3bc9d3..75fe5aaad 100644[m
[1m--- a/src/osgEarth/ImageUtils[m
[1m+++ b/src/osgEarth/ImageUtils[m
[36m@@ -250,12 +250,6 @@[m [mnamespace osgEarth { namespace Util[m
          */[m
         static bool isFloatingPointInternalFormat( GLint internalFormat );[m
 [m
[31m-        /**[m
[31m-         * Returns the string representation of a GL texture internal format enum.[m
[31m-         * For example, given GL_RGBA8 the method will return the string "GL_RGBA8".[m
[31m-         */[m
[31m-        static std::string getInternalFormatString( GLint internalFormat );[m
[31m-[m
         /**[m
          * Compute a texture compression format suitable for the image.[m
          */[m
[36m@@ -479,18 +473,22 @@[m [mnamespace osgEarth { namespace Util[m
 [m
             //! Returns a color from an image at pixel (s,t,r,m)[m
             inline osg::Vec4f operator()(int s, int t, int r=0, int m=0) const {[m
[31m-                return _read(this, s, t, r, m);[m
[32m+[m[32m                osg::Vec4f temp;[m
[32m+[m[32m                _read(this, temp, s, t, r, m);[m
[32m+[m[32m                return temp;[m
             }[m
             inline void operator()(osg::Vec4f& output, int s, int t, int r=0, int m=0) const {[m
[31m-                output = _read(this, s, t, r, m);[m
[32m+[m[32m                _read(this, output, s, t, r, m);[m
             }[m
 [m
             //! Returns a color from an image at pixel (s,t,r,m)[m
             inline osg::Vec4f operator()(unsigned s, unsigned t, unsigned r=0, int m=0) const {[m
[31m-                return _read(this, s, t, r, m);[m
[32m+[m[32m                osg::Vec4f temp;[m
[32m+[m[32m                _read(this, temp, s, t, r, m);[m
[32m+[m[32m                return temp;[m
             }            [m
             inline void operator()(osg::Vec4f& output, unsigned s, unsigned t, int r=0, int m=0) const {[m
[31m-                output = _read(this, s, t, r, m);[m
[32m+[m[32m                _read(this, output, s, t, r, m);[m
             }[m
 [m
             //! composite version of pixel read operator[m
[36m@@ -500,21 +498,15 @@[m [mnamespace osgEarth { namespace Util[m
             }[m
             template<typename T>[m
             inline void operator()(osg::Vec4f& output, const T& composite) const {[m
[31m-                output = _read(this, composite.s(), composite.t(), composite.r(), 0);[m
[32m+[m[32m                _read(this, output, composite.s(), composite.t(), composite.r(), 0);[m
             }[m
 [m
             /** Reads a color from the image by unit coords [0..1] */[m
             osg::Vec4f operator()(float u, float v, int r=0, int m=0) const;[m
[31m-            [m
[31m-            inline void operator()(osg::Vec4f& output, float u, float v, int r=0, int m=0) const {[m
[31m-                output = operator()(u, v, r, m);[m
[31m-            }[m
[32m+[m[32m            void operator()(osg::Vec4f& output, float u, float v, int r=0, int m=0) const;[m
 [m
             osg::Vec4f operator()(double u, double v, int r=0, int m=0) const;[m
[31m-[m
[31m-            inline void operator()(osg::Vec4f& output, double u, double v, int t = 0, int m = 0) const {[m
[31m-                output = operator()(u, v, t, m);[m
[31m-            }[m
[32m+[m[32m            void operator()(osg::Vec4f& output, double u, double v, int t=0, int m=0) const;[m
 [m
             // internals:[m
             const unsigned char* data(int s=0, int t=0, int r=0, int m=0) const {[m
[36m@@ -523,7 +515,7 @@[m [mnamespace osgEarth { namespace Util[m
                     _image->getMipmapData(m-1) + (s>>m)*_colBytes + (t>>m)*(_rowBytes>>m) + r*(_imageBytes>>m);[m
             }[m
 [m
[31m-            typedef osg::Vec4f (*ReaderFunc)(const PixelReader* ia, int s, int t, int r, int m);[m
[32m+[m[32m            typedef void (*ReaderFunc)(const PixelReader* ia, osg::Vec4f& output, int s, int t, int r, int m);[m
 [m
             ReaderFunc _read;[m
             const osg::Image* _image;[m
[36m@@ -603,7 +595,7 @@[m [mnamespace osgEarth { namespace Util[m
 [m
             unsigned char* data(int s=0, int t=0, int r=0, int m=0) const;[m
 [m
[31m-            typedef void (*WriterFunc)(const PixelWriter* iw, const osg::Vec4 c, int s, int t, int r, int m);[m
[32m+[m[32m            typedef void (*WriterFunc)(const PixelWriter* iw, const osg::Vec4& c, int s, int t, int r, int m);[m
             WriterFunc _writer;[m
         };[m
 [m
[1mdiff --git a/src/osgEarth/ImageUtils.cpp b/src/osgEarth/ImageUtils.cpp[m
[1mindex 3fe810f8c..158401fd7 100644[m
[1m--- a/src/osgEarth/ImageUtils.cpp[m
[1m+++ b/src/osgEarth/ImageUtils.cpp[m
[36m@@ -704,12 +704,9 @@[m [mImageUtils::compressImage(const osg::Image* input, const std::string& method)[m
     {[m
         output = osg::clone(input, osg::CopyOp::DEEP_COPY_ALL);[m
 [m
[31m-[m
[32m+[m[32m        // RGB uses DXT1[m
         osg::Texture::InternalFormatMode mode;[m
[31m-[m
[31m-        if (input->getPixelFormat() == GL_RG)[m
[31m-            mode = osg::Texture::USE_RGTC2_COMPRESSION;[m
[31m-        else if (hasAlphaChannel(input))[m
[32m+[m[32m        if (hasAlphaChannel(input))[m
             mode = osg::Texture::USE_S3TC_DXT5_COMPRESSION;[m
         else[m
             mode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;[m
[36m@@ -744,14 +741,17 @@[m [mImageUtils::compressImageInPlace(osg::Image* input, const std::string& method)[m
     if (method.empty() || method == "none")[m
         return;[m
 [m
[32m+[m[32m    // RGB uses DXT1[m
     osg::Texture::InternalFormatMode mode;[m
 [m
[31m-    if (input->getPixelFormat() == GL_RG)[m
[31m-        mode = osg::Texture::USE_RGTC2_COMPRESSION;[m
[31m-    else if (hasAlphaChannel(input))[m
[32m+[m[32m    if (hasAlphaChannel(input))[m
[32m+[m[32m    {[m
         mode = osg::Texture::USE_S3TC_DXT5_COMPRESSION;[m
[32m+[m[32m    }[m
     else[m
[32m+[m[32m    {[m
         mode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;[m
[32m+[m[32m    }[m
 [m
     if (method == "gpu")[m
     {[m
[36m@@ -1605,89 +1605,6 @@[m [mImageUtils::isFloatingPointInternalFormat(GLint i)[m
         (i >= 0x8814 && i <= 0x881F);   // GL_RGBA32F_ARB, et al[m
 }[m
 [m
[31m-std::string[m
[31m-ImageUtils::getInternalFormatString(GLint internalFormat)[m
[31m-{[m
[31m-    switch (internalFormat)[m
[31m-    {[m
[31m-        // Basic pixel formats[m
[31m-        case GL_ALPHA:          return "GL_ALPHA";[m
[31m-        case GL_LUMINANCE:      return "GL_LUMINANCE";[m
[31m-        case GL_LUMINANCE_ALPHA: return "GL_LUMINANCE_ALPHA";[m
[31m-        case GL_INTENSITY:      return "GL_INTENSITY";[m
[31m-        case GL_RED:            return "GL_RED";[m
[31m-        case GL_RG:             return "GL_RG";[m
[31m-        case GL_RGB:            return "GL_RGB";[m
[31m-        case GL_RGBA:           return "GL_RGBA";[m
[31m-[m
[31m-        // Sized internal formats - Red/RG[m
[31m-        case GL_R8:             return "GL_R8";[m
[31m-        case GL_R16:            return "GL_R16";[m
[31m-        case GL_R16F:           return "GL_R16F";[m
[31m-        case GL_R32F:           return "GL_R32F";[m
[31m-        case GL_R8I:            return "GL_R8I";[m
[31m-        case GL_R8UI:           return "GL_R8UI";[m
[31m-        case GL_R16I:           return "GL_R16I";[m
[31m-        case GL_R16UI:          return "GL_R16UI";[m
[31m-        case GL_R32I:           return "GL_R32I";[m
[31m-        case GL_R32UI:          return "GL_R32UI";[m
[31m-        case GL_RG8:            return "GL_RG8";[m
[31m-        case GL_RG16:           return "GL_RG16";[m
[31m-        case GL_RG16F:          return "GL_RG16F";[m
[31m-        case GL_RG32F:          return "GL_RG32F";[m
[31m-        case GL_RG8I:           return "GL_RG8I";[m
[31m-        case GL_RG8UI:          return "GL_RG8UI";[m
[31m-        case GL_RG16I:          return "GL_RG16I";[m
[31m-        case GL_RG16UI:         return "GL_RG16UI";[m
[31m-        case GL_RG32I:          return "GL_RG32I";[m
[31m-        case GL_RG32UI:         return "GL_RG32UI";[m
[31m-[m
[31m-        // Sized internal formats - RGB[m
[31m-        case GL_RGB8:           return "GL_RGB8";[m
[31m-        case GL_RGB16:          return "GL_RGB16";[m
[31m-[m
[31m-        // Sized internal formats - RGBA[m
[31m-        case GL_RGBA8:          return "GL_RGBA8";[m
[31m-        case GL_RGBA16:         return "GL_RGBA16";[m
[31m-[m
[31m-        // Depth formats[m
[31m-        case GL_DEPTH_COMPONENT:   return "GL_DEPTH_COMPONENT";[m
[31m-        case GL_DEPTH_COMPONENT16: return "GL_DEPTH_COMPONENT16";[m
[31m-        case GL_DEPTH_COMPONENT24: return "GL_DEPTH_COMPONENT24";[m
[31m-        case GL_DEPTH_COMPONENT32: return "GL_DEPTH_COMPONENT32";[m
[31m-        case GL_DEPTH_COMPONENT32F: return "GL_DEPTH_COMPONENT32F";[m
[31m-[m
[31m-        // Compressed formats - generic[m
[31m-        case GL_COMPRESSED_ALPHA:           return "GL_COMPRESSED_ALPHA";[m
[31m-        case GL_COMPRESSED_LUMINANCE:       return "GL_COMPRESSED_LUMINANCE";[m
[31m-        case GL_COMPRESSED_LUMINANCE_ALPHA: return "GL_COMPRESSED_LUMINANCE_ALPHA";[m
[31m-        case GL_COMPRESSED_INTENSITY:       return "GL_COMPRESSED_INTENSITY";[m
[31m-        case GL_COMPRESSED_RGB:             return "GL_COMPRESSED_RGB";[m
[31m-        case GL_COMPRESSED_RGBA:            return "GL_COMPRESSED_RGBA";[m
[31m-[m
[31m-        // Compressed formats - RGTC[m
[31m-        case GL_COMPRESSED_RED_RGTC1_EXT:           return "GL_COMPRESSED_RED_RGTC1_EXT";[m
[31m-        case GL_COMPRESSED_SIGNED_RED_RGTC1_EXT:    return "GL_COMPRESSED_SIGNED_RED_RGTC1_EXT";[m
[31m-        case GL_COMPRESSED_RED_GREEN_RGTC2_EXT:     return "GL_COMPRESSED_RED_GREEN_RGTC2_EXT";[m
[31m-        case GL_COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT: return "GL_COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT";[m
[31m-[m
[31m-        // Compressed formats - S3TC/DXT[m
[31m-        case 0x83F0: /* GL_COMPRESSED_RGB_S3TC_DXT1_EXT */    return "GL_COMPRESSED_RGB_S3TC_DXT1_EXT";[m
[31m-        case 0x83F1: /* GL_COMPRESSED_RGBA_S3TC_DXT1_EXT */   return "GL_COMPRESSED_RGBA_S3TC_DXT1_EXT";[m
[31m-        case 0x83F2: /* GL_COMPRESSED_RGBA_S3TC_DXT3_EXT */   return "GL_COMPRESSED_RGBA_S3TC_DXT3_EXT";[m
[31m-        case 0x83F3: /* GL_COMPRESSED_RGBA_S3TC_DXT5_EXT */   return "GL_COMPRESSED_RGBA_S3TC_DXT5_EXT";[m
[31m-[m
[31m-        // Compressed formats - PVRTC[m
[31m-        case GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG:    return "GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG";[m
[31m-        case GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG:    return "GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG";[m
[31m-        case GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG:   return "GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG";[m
[31m-        case GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG:   return "GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG";[m
[31m-[m
[31m-        default:[m
[31m-            return "UNKNOWN";[m
[31m-    }[m
[31m-}[m
[31m-[m
 bool[m
 ImageUtils::sameFormat(const osg::Image* lhs, const osg::Image* rhs)[m
 {[m
[36m@@ -1768,18 +1685,18 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_DEPTH_COMPONENT, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float d = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);[m
[31m-            return osg::Vec4f(d, d, d, 1.0f);[m
[32m+[m[32m            out.set(d, d, d, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_DEPTH_COMPONENT, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m)[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)[m
         {[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
             (*ptr) = (T)(c.r() / GLTypeTraits<T>::scale(iw->_normalized));[m
[36m@@ -1789,18 +1706,18 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_LUMINANCE, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float red = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);[m
[31m-            return osg::Vec4f(red, red, red, 1.0f);[m
[32m+[m[32m            out.set(red, red, red, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_LUMINANCE, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m)[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)[m
         {[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
             (*ptr) = (T)(c.r() / GLTypeTraits<T>::scale(iw->_normalized));[m
[36m@@ -1810,18 +1727,18 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_RED, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float red = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);[m
[31m-            return osg::Vec4f(red, red, red, 1.0f);[m
[32m+[m[32m            out.set(red, red, red, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_RED, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m)[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)[m
         {[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
             (*ptr) = (T)(c.r() / GLTypeTraits<T>::scale(iw->_normalized));[m
[36m@@ -1831,18 +1748,18 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_ALPHA, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float a = float(*ptr) * GLTypeTraits<T>::scale(ia->_normalized);[m
[31m-            return osg::Vec4f(1.0f, 1.0f, 1.0f, a);[m
[32m+[m[32m            out.set(1.0f, 1.0f, 1.0f, a);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_ALPHA, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m)[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)[m
         {[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
             (*ptr) = (T)(c.a() / GLTypeTraits<T>::scale(iw->_normalized));[m
[36m@@ -1852,20 +1769,20 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_LUMINANCE_ALPHA, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             float scale = GLTypeTraits<T>::scale(ia->_normalized);[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float l = float(*ptr++) * scale;[m
             float a = float(*ptr) * scale;[m
[31m-            return osg::Vec4f(l, l, l, a);[m
[32m+[m[32m            out.set(l, l, l, a);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_LUMINANCE_ALPHA, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             double scale = GLTypeTraits<T>::scale(iw->_normalized);[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
[36m@@ -1877,20 +1794,20 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_RG, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             float scale = GLTypeTraits<T>::scale(ia->_normalized);[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float red = float(*ptr++) * scale;[m
             float g = float(*ptr++) * scale;[m
[31m-            return osg::Vec4f(red, g, 0.0f, 1.0f);[m
[32m+[m[32m            out.set(red, g, 0.0f, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_RG, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             double scale = GLTypeTraits<T>::scale(iw->_normalized);[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
[36m@@ -1902,21 +1819,21 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_RGB, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             float scale = GLTypeTraits<T>::scale(ia->_normalized);[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float red = float(*ptr++) * scale;[m
             float g = float(*ptr++) * scale;[m
             float b = float(*ptr) * scale;[m
[31m-            return osg::Vec4f(red, g, b, 1.0f);[m
[32m+[m[32m            out.set(red, g, b, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_RGB, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             double scale = GLTypeTraits<T>::scale(iw->_normalized);[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
[36m@@ -1929,7 +1846,7 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_RGBA, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             float scale = GLTypeTraits<T>::scale(ia->_normalized);[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
[36m@@ -1937,14 +1854,14 @@[m [mnamespace[m
             float g = float(*ptr++) * scale;[m
             float b = float(*ptr++) * scale;[m
             float a = float(*ptr) * scale;[m
[31m-            return osg::Vec4f(red, g, b, a);[m
[32m+[m[32m            out.set(red, g, b, a);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_RGBA, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m)[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m)[m
         {[m
             double scale = GLTypeTraits<T>::scale(iw->_normalized);[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
[36m@@ -1958,21 +1875,21 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_BGR, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             float scale = GLTypeTraits<T>::scale(ia->_normalized);[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
             float b = float(*ptr++) * scale;[m
             float g = float(*ptr++) * scale;[m
             float red = float(*ptr) * scale;[m
[31m-            return osg::Vec4f(red, g, b, 1.0f);[m
[32m+[m[32m            out.set(red, g, b, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_BGR, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             double scale = GLTypeTraits<T>::scale(iw->_normalized);[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
[36m@@ -1985,7 +1902,7 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<GL_BGRA, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             float scale = GLTypeTraits<T>::scale(ia->_normalized);[m
             const T* ptr = (const T*)ia->data(s, t, r, m);[m
[36m@@ -1993,14 +1910,14 @@[m [mnamespace[m
             float g = float(*ptr++) * scale;[m
             float red = float(*ptr++) * scale;[m
             float a = float(*ptr) * scale;[m
[31m-            return osg::Vec4f(red, g, b, a);[m
[32m+[m[32m            out.set(red, g, b, a);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<GL_BGRA, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             double scale = GLTypeTraits<T>::scale(iw->_normalized);[m
             T* ptr = (T*)iw->data(s, t, r, m);[m
[36m@@ -2014,16 +1931,16 @@[m [mnamespace[m
     template<typename T>[m
     struct ColorReader<0, T>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
[31m-            return osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f);[m
[32m+[m[32m            out.set(1.0f, 1.0f, 1.0f, 1.0f);[m
         }[m
     };[m
 [m
     template<typename T>[m
     struct ColorWriter<0, T>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             //nop[m
         }[m
[36m@@ -2032,11 +1949,11 @@[m [mnamespace[m
     template<>[m
     struct ColorReader<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             GLushort p = *(const GLushort*)ia->data(s, t, r, m);[m
             //internal format GL_RGB5_A1 is implied[m
[31m-            return osg::Vec4f([m
[32m+[m[32m            out.set([m
                 r5*(float)(p>>11),[m
                 r5*(float)((p&0x7c0)>>6),[m
                 r5*(float)((p&0x3e)>>1),[m
[36m@@ -2047,7 +1964,7 @@[m [mnamespace[m
     template<>[m
     struct ColorWriter<GL_UNSIGNED_SHORT_5_5_5_1, GLushort>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             GLushort[m
                 red = (unsigned short)(c.r()*255),[m
[36m@@ -2063,18 +1980,18 @@[m [mnamespace[m
     template<>[m
     struct ColorReader<GL_UNSIGNED_BYTE_3_3_2, GLubyte>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* ia, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* ia, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             GLubyte p = *(const GLubyte*)ia->data(s,t,r,m);[m
             // internal format GL_R3_G3_B2 is implied[m
[31m-            return osg::Vec4f( r3*(float)(p>>5), r3*(float)((p&0x28)>>2), r2*(float)(p&0x3), 1.0f );[m
[32m+[m[32m            out.set( r3*(float)(p>>5), r3*(float)((p&0x28)>>2), r2*(float)(p&0x3), 1.0f );[m
         }[m
     };[m
 [m
     template<>[m
     struct ColorWriter<GL_UNSIGNED_BYTE_3_3_2, GLubyte>[m
     {[m
[31m-        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f c, int s, int t, int r, int m )[m
[32m+[m[32m        static void write(const ImageUtils::PixelWriter* iw, const osg::Vec4f& c, int s, int t, int r, int m )[m
         {[m
             GLubyte* ptr = (GLubyte*)iw->data(s,t,r,m);[m
             OE_WARN << LC << "Target GL_UNSIGNED_BYTE_3_3_2 not yet implemented" << std::endl;[m
[36m@@ -2086,7 +2003,7 @@[m [mnamespace[m
     {[m
         // RGTC2 == BC5[m
         // https://learn.microsoft.com/en-us/windows/win32/direct3d10/d3d10-graphics-programming-guide-resources-block-compression#bc5[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* pr, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* pr, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             static const int BLOCK_SIZE_BYTES = 16;[m
 [m
[36m@@ -2096,8 +2013,6 @@[m [mnamespace[m
             int ls = s - 4 * bs, lt = t - 4 * bt;[m
             int index = ls + (4 * lt);[m
 [m
[31m-            osg::Vec4f out;[m
[31m-[m
             const std::uint64_t* block_ptr = (const std::uint64_t*)(pr->data() + blockStart);[m
 [m
             for (int n = 0; n <= 1; n++)[m
[36m@@ -2123,8 +2038,6 @@[m [mnamespace[m
             out[2] = 0.0;[m
             out[3] = 1.0;[m
 [m
[31m-            return out;[m
[31m-[m
 #if 0 // decodes an entire block - left here as a reference[m
                 float palette[8];[m
                 palette[0] = float((block >> 0) & 0xFF) / 255.0f;[m
[36m@@ -2163,7 +2076,7 @@[m [mnamespace[m
     template<>[m
     struct ColorReader<GL_COMPRESSED_RGB_S3TC_DXT1_EXT, GLubyte>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* pr, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* pr, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             static const int BLOCK_SIZE_BYTES = 8;[m
 [m
[36m@@ -2208,7 +2121,7 @@[m [mnamespace[m
 [m
             unsigned int index = (table >> (2*x)) & 0x00000003;[m
 [m
[31m-            return[m
[32m+[m[32m            out =[m
                 index == 0? c0 :[m
                 index == 1? c1 :[m
                 index == 2? c2 : c3;[m
[36m@@ -2218,7 +2131,7 @@[m [mnamespace[m
     template<>[m
     struct ColorReader<GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, GLubyte>[m
     {[m
[31m-        static osg::Vec4f read(const ImageUtils::PixelReader* pr, int s, int t, int r, int m)[m
[32m+[m[32m        static void read(const ImageUtils::PixelReader* pr, osg::Vec4f& out, int s, int t, int r, int m)[m
         {[m
             static const int BLOCK_SIZE_BYTES = 16;[m
 [m
[36m@@ -2287,7 +2200,7 @@[m [mnamespace[m
             unsigned color_lut = *(unsigned*)p;[m
             unsigned color_index = (color_lut >> (2 * blockIndex)) & 0x3; // 2-bit index[m
 [m
[31m-            return[m
[32m+[m[32m            out =[m
                 color_index == 0 ? c0 :[m
                 color_index == 1 ? c1 :[m
                 color_index == 2 ? c2 : c3;[m
[36m@@ -2297,8 +2210,7 @@[m [mnamespace[m
     //! Select an appropriate reader for the given data type.[m
     //! [m
     //! NOTE!![m
[31m-    //! (Subnote!! We're not doing this anymore! ... if the issue arises again deal with it)[m
[31m-    //! For UNSIGNED data types, we are using a SIGNED reader, except in the case[m
[32m+[m[32m    //! Fopr UNSIGNED data types, we are using a SIGNED reader, except in the case[m
     //! of GL_UNSIGNED_BYTE. This is because the OSG TIFF reader always declares[m
     //! the data to be unsigned even when it's not. That's a bug, but it does not[m
     //! hurt us to generate signed data for unsigned input, so this is an acceptable[m
[36m@@ -2317,13 +2229,13 @@[m [mnamespace[m
         case GL_UNSIGNED_BYTE:[m
             return &ColorReader<GLFormat, GLubyte>::read;[m
         case GL_SHORT:[m
[31m-            return &ColorReader<GLFormat, GLshort>::read;[m
         case GL_UNSIGNED_SHORT:[m
[31m-            return &ColorReader<GLFormat, GLushort>::read;[m
[32m+[m[32m            return &ColorReader<GLFormat, GLshort>::read;[m
[32m+[m[32m            //return &ColorReader<GLFormat, GLushort>::read;[m
         case GL_INT:[m
[31m-            return &ColorReader<GLFormat, GLint>::read;[m
         case GL_UNSIGNED_INT:[m
[31m-            return &ColorReader<GLFormat, GLuint>::read;[m
[32m+[m[32m            return &ColorReader<GLFormat, GLint>::read;[m
[32m+[m[32m            //return &ColorReader<GLFormat, GLuint>::read;[m
         case GL_FLOAT:[m
             return &ColorReader<GLFormat, GLfloat>::read;[m
         case GL_UNSIGNED_SHORT_5_5_5_1:[m
[36m@@ -2442,6 +2354,14 @@[m [mImageUtils::PixelReader::setTexture(const osg::Texture* tex)[m
     }[m
 }[m
 [m
[32m+[m[32mosg::Vec4[m
[32m+[m[32mImageUtils::PixelReader::operator()(float u, float v, int r, int m) const[m
[32m+[m[32m{[m
[32m+[m[32m    osg::Vec4f temp;[m
[32m+[m[32m    this->operator()(temp, u, v, r, m);[m
[32m+[m[32m    return temp;[m
[32m+[m[32m}[m
[32m+[m
 namespace {[m
     double fract(double x) {[m
         return x >= 0.0 ? (x - floor(x)) : (x - ceil(x));[m
[36m@@ -2511,19 +2431,17 @@[m [mnamespace {[m
     }[m
 }[m
 [m
[31m-osg::Vec4f[m
[31m-ImageUtils::PixelReader::operator()(float u, float v, int r, int m) const[m
[32m+[m[32mvoid[m
[32m+[m[32mImageUtils::PixelReader::operator()(osg::Vec4f& out, float u, float v, int r, int m) const[m
 {[m
     OE_SOFT_ASSERT(_image != nullptr);[m
 [m
[31m-    osg::Vec4f out;[m
[31m-[m
     if (!_bilinear)[m
     {[m
         // NN sample with clamp-to-edge from mesa in s_texfilter.c[m
         unsigned s, t;[m
         nnUVtoST(u, v, s, t, _image->s(), _image->t());[m
[31m-        out = _read(this, s, t, r, m);[m
[32m+[m[32m        _read(this, out, s, t, r, m);[m
     }[m
 [m
     else if (_sampleAsTexture)[m
[36m@@ -2579,10 +2497,11 @@[m [mImageUtils::PixelReader::operator()(float u, float v, int r, int m) const[m
             tplus1 = (t + 1 < _image->t()) ? t + 1 : t;[m
         }[m
 [m
[31m-        auto p1 = _read(this, s, t, r, m);[m
[31m-        auto p2 = _read(this, splus1, t, r, m);[m
[31m-        auto p3 = _read(this, s, tplus1, r, m);[m
[31m-        auto p4 = _read(this, splus1, tplus1, r, m);[m
[32m+[m[32m        osg::Vec4f p1, p2, p3, p4;[m
[32m+[m[32m        _read(this, p1, s, t, r, m);[m
[32m+[m[32m        _read(this, p2, splus1, t, r, m);[m
[32m+[m[32m        _read(this, p3, s, tplus1, r, m);[m
[32m+[m[32m        _read(this, p4, splus1, tplus1, r, m);[m
 [m
         p1 = p1 * (1.0 - fx) + p2 * fx;[m
         p2 = p3 * (1.0 - fx) + p4 * fx;[m
[36m@@ -2617,33 +2536,31 @@[m [mImageUtils::PixelReader::operator()(float u, float v, int r, int m) const[m
         float t1 = osg::minimum(t0 + 1.0f, sizeT);[m
         float tmix = t0 < t1 ? (t - t0) / (t1 - t0) : 0.0f;[m
 [m
[31m-        auto UL = _read(this, (int)s0, (int)t0, r, m); // upper left[m
[31m-        auto UR = _read(this, (int)s1, (int)t0, r, m); // upper right[m
[31m-        auto LL = _read(this, (int)s0, (int)t1, r, m); // lower left[m
[31m-        auto LR = _read(this, (int)s1, (int)t1, r, m); // lower right[m
[32m+[m[32m        osg::Vec4f UL, UR, LL, LR;[m
[32m+[m
[32m+[m[32m        _read(this, UL, (int)s0, (int)t0, r, m); // upper left[m
[32m+[m[32m        _read(this, UR, (int)s1, (int)t0, r, m); // upper right[m
[32m+[m[32m        _read(this, LL, (int)s0, (int)t1, r, m); // lower left[m
[32m+[m[32m        _read(this, LR, (int)s1, (int)t1, r, m); // lower right[m
 [m
[31m-        auto TOP = UL * (1.0f - smix) + UR * smix;[m
[31m-        auto BOT = LL * (1.0f - smix) + LR * smix;[m
[32m+[m[32m        osg::Vec4f TOP = UL * (1.0f - smix) + UR * smix;[m
[32m+[m[32m        osg::Vec4f BOT = LL * (1.0f - smix) + LR * smix;[m
 [m
         out = TOP * (1.0f - tmix) + BOT * tmix;[m
     }[m
[31m-[m
[31m-    return out;[m
 }[m
 [m
[31m-osg::Vec4f[m
[31m-ImageUtils::PixelReader::operator()(double u, double v, int r, int m) const[m
[32m+[m[32mvoid[m
[32m+[m[32mImageUtils::PixelReader::operator()(osg::Vec4f& out, double u, double v, int r, int m) const[m
 {[m
     OE_SOFT_ASSERT(_image != nullptr);[m
 [m
[31m-    osg::Vec4f out;[m
[31m-[m
     if (!_bilinear)[m
     {[m
         // NN sample with clamp-to-edge from mesa in s_texfilter.c[m
         unsigned s, t;[m
         nnUVtoST(u, v, s, t, _image->s(), _image->t());[m
[31m-        out = _read(this, s, t, r, m);[m
[32m+[m[32m        _read(this, out, s, t, r, m);[m
     }[m
 [m
     else if (_sampleAsTexture)[m
[36m@@ -2699,10 +2616,11 @@[m [mImageUtils::PixelReader::operator()(double u, double v, int r, int m) const[m
             tplus1 = (t + 1 < _image->t()) ? t + 1 : t;[m
         }[m
 [m
[31m-        auto p1 = _read(this, s, t, r, m);[m
[31m-        auto p2 = _read(this, splus1, t, r, m);[m
[31m-        auto p3 = _read(this, s, tplus1, r, m);[m
[31m-        auto p4 = _read(this, splus1, tplus1, r, m);[m
[32m+[m[32m        osg::Vec4f p1, p2, p3, p4;[m
[32m+[m[32m        _read(this, p1, s, t, r, m);[m
[32m+[m[32m        _read(this, p2, splus1, t, r, m);[m
[32m+[m[32m        _read(this, p3, s, tplus1, r, m);[m
[32m+[m[32m        _read(this, p4, splus1, tplus1, r, m);[m
 [m
         p1 = p1 * (1.0 - fx) + p2 * fx;[m
         p2 = p3 * (1.0 - fx) + p4 * fx;[m
[36m@@ -2737,18 +2655,26 @@[m [mImageUtils::PixelReader::operator()(double u, double v, int r, int m) const[m
         double t1 = osg::minimum(t0 + 1.0, sizeT);[m
         double tmix = t0 < t1 ? (t - t0) / (t1 - t0) : 0.0;[m
 [m
[31m-        auto UL = _read(this, (int)s0, (int)t0, r, m); // upper left[m
[31m-        auto UR = _read(this, (int)s1, (int)t0, r, m); // upper right[m
[31m-        auto LL = _read(this, (int)s0, (int)t1, r, m); // lower left[m
[31m-        auto LR = _read(this, (int)s1, (int)t1, r, m); // lower right[m
[32m+[m[32m        osg::Vec4f UL, UR, LL, LR;[m
 [m
[31m-        auto TOP = UL * (1.0f - smix) + UR * smix;[m
[31m-        auto BOT = LL * (1.0f - smix) + LR * smix;[m
[32m+[m[32m        _read(this, UL, (int)s0, (int)t0, r, m); // upper left[m
[32m+[m[32m        _read(this, UR, (int)s1, (int)t0, r, m); // upper right[m
[32m+[m[32m        _read(this, LL, (int)s0, (int)t1, r, m); // lower left[m
[32m+[m[32m        _read(this, LR, (int)s1, (int)t1, r, m); // lower right[m
[32m+[m
[32m+[m[32m        osg::Vec4f TOP = UL * (1.0f - smix) + UR * smix;[m
[32m+[m[32m        osg::Vec4f BOT = LL * (1.0f - smix) + LR * smix;[m
 [m
         out = TOP * (1.0f - tmix) + BOT * tmix;[m
     }[m
[32m+[m[32m}[m
 [m
[31m-    return out;[m
[32m+[m[32mosg::Vec4f[m
[32m+[m[32mImageUtils::PixelReader::operator()(double u, double v, int r, int m) const[m
[32m+[m[32m{[m
[32m+[m[32m    osg::Vec4f temp;[m
[32m+[m[32m    this->operator()(temp, u, v, r, m);[m
[32m+[m[32m    return temp;[m
 }[m
 [m
 bool[m
[1mdiff --git a/src/osgEarth/Map b/src/osgEarth/Map[m
[1mindex 776d40d6f..18ee0dd05 100644[m
[1m--- a/src/osgEarth/Map[m
[1m+++ b/src/osgEarth/Map[m
[36m@@ -242,6 +242,14 @@[m [mnamespace osgEarth[m
         osg::ref_ptr<CacheSettings> _cacheSettings;[m
         int _numTerrainPatchLayers;[m
 [m
[32m+[m[32m        //struct LayerCB : public LayerCallback {[m
[32m+[m[32m        //    LayerCB(Map*);[m
[32m+[m[32m        //    osg::observer_ptr<Map> _map;[m
[32m+[m[32m        //    void onOpen(Layer* layer);[m
[32m+[m[32m        //    void onClose(Layer* layer);[m
[32m+[m[32m        //};[m
[32m+[m[32m        //osg::ref_ptr<LayerCallback> _layerCB;[m
[32m+[m[32m        //friend struct LayerCB;[m
         void notifyOnLayerOpenOrClose(Layer*);[m
 [m
         void installLayerCallbacks(Layer*);[m
[1mdiff --git a/src/osgEarth/Map.cpp b/src/osgEarth/Map.cpp[m
[1mindex f76537142..3ffbf1d59 100644[m
[1m--- a/src/osgEarth/Map.cpp[m
[1m+++ b/src/osgEarth/Map.cpp[m
[36m@@ -31,25 +31,6 @@[m [mvoid Map::LayerCB::onClose(Layer* layer)[m
 }[m
 #endif[m
 [m
[31m-namespace[m
[31m-{[m
[31m-    struct UpdateElevationPoolCallback : public MapCallback[m
[31m-    {[m
[31m-        Map* _map;[m
[31m-        UpdateElevationPoolCallback(Map* map) : _map(map) {}[m
[31m-        void onMapModelChanged(const MapModelChange& change) override[m
[31m-        {[m
[31m-            if (change.getElevationLayer())[m
[31m-            {[m
[31m-                if (_map->getElevationPool())[m
[31m-                {[m
[31m-                    _map->getElevationPool()->setMap(_map);[m
[31m-                }[m
[31m-            }[m
[31m-        }[m
[31m-    };[m
[31m-}[m
[31m-[m
 //...................................................................[m
 [m
 Config[m
[36m@@ -204,9 +185,6 @@[m [mMap::init()[m
     _elevationPool = new ElevationPool();[m
     _elevationPool->setMap( this );[m
 [m
[31m-    // tell the elevation pool to refresh when layers open or close[m
[31m-    addMapCallback(new UpdateElevationPoolCallback(this));[m
[31m-[m
     _numTerrainPatchLayers = 0;[m
 }[m
 [m
[1mdiff --git a/src/osgEarth/MeasureTool.cpp b/src/osgEarth/MeasureTool.cpp[m
[1mindex 3705c8617..85f750378 100644[m
[1m--- a/src/osgEarth/MeasureTool.cpp[m
[1m+++ b/src/osgEarth/MeasureTool.cpp[m
[36m@@ -109,7 +109,7 @@[m [mMeasureToolHandler::rebuild()[m
 #ifdef SHOW_EXTENT[m
 [m
     // Define the extent feature:[m
[31m-    _extentFeature = new Feature( new osgEarth::Polygon(), getMapNode()->getMapSRS() );[m
[32m+[m[32m    _extentFeature = new Feature( new Polygon(), getMapNode()->getMapSRS() );[m
     _extentFeature->geoInterp() = GEOINTERP_RHUMB_LINE;[m
     _extentFeature->getOrCreateStyle()->add( alt );[m
     LineSymbol* extentLine = _extentFeature->style()->getOrCreate<LineSymbol>();[m
[1mdiff --git a/src/osgEarth/OgrUtils b/src/osgEarth/OgrUtils[m
[1mindex 9be8b1830..25577152d 100644[m
[1m--- a/src/osgEarth/OgrUtils[m
[1m+++ b/src/osgEarth/OgrUtils[m
[36m@@ -30,7 +30,7 @@[m [mnamespace osgEarth { namespace Util[m
     {[m
         static void populate( OGRGeometryH geomHandle, Geometry* target, int numPoints );[m
     [m
[31m-        static osgEarth::Polygon* createPolygon( OGRGeometryH geomHandle, bool rewindPolygons = true);[m
[32m+[m[32m        static Polygon* createPolygon( OGRGeometryH geomHandle, bool rewindPolygons = true);[m
 [m
         static MultiGeometry* createTIN(OGRGeometryH geomHandle);[m
        [m
[1mdiff --git a/src/osgEarth/OgrUtils.cpp b/src/osgEarth/OgrUtils.cpp[m
[1mindex e6e3e5868..a7ee7a446 100644[m
[1m--- a/src/osgEarth/OgrUtils.cpp[m
[1m+++ b/src/osgEarth/OgrUtils.cpp[m
[36m@@ -62,7 +62,7 @@[m [mOgrUtils::createTIN(OGRGeometryH geomHandle)[m
             unsigned int numSubParts = OGR_G_GetGeometryCount(partRef);[m
             OGRGeometryH subPartRef = OGR_G_GetGeometryRef(partRef, 0);[m
             unsigned int numSubPoints = OGR_G_GetPointCount(subPartRef);[m
[31m-            osgEarth::Polygon *output = new osgEarth::Polygon(numSubPoints);[m
[32m+[m[32m            Polygon *output = new Polygon(numSubPoints);[m
             populate(subPartRef, output, numSubPoints);[m
             output->open();[m
 [m
[36m@@ -76,16 +76,16 @@[m [mOgrUtils::createTIN(OGRGeometryH geomHandle)[m
     return multi;[m
 }[m
 [m
[31m-osgEarth::Polygon*[m
[32m+[m[32mPolygon*[m
 OgrUtils::createPolygon( OGRGeometryH geomHandle, bool rewindPolygons)[m
 {[m
[31m-    osgEarth::Polygon* output = 0L;[m
[32m+[m[32m    Polygon* output = 0L;[m
 [m
     int numParts = OGR_G_GetGeometryCount( geomHandle );[m
     if ( numParts == 0 )[m
     {[m
         int numPoints = OGR_G_GetPointCount( geomHandle );[m
[31m-        output = new osgEarth::Polygon( numPoints );[m
[32m+[m[32m        output = new Polygon( numPoints );[m
         populate( geomHandle, output, numPoints );[m
 [m
         if (rewindPolygons)[m
[36m@@ -103,7 +103,7 @@[m [mOgrUtils::createPolygon( OGRGeometryH geomHandle, bool rewindPolygons)[m
 [m
             if ( p == 0 )[m
             {[m
[31m-                output = new osgEarth::Polygon( numPoints );[m
[32m+[m[32m                output = new Polygon( numPoints );[m
                 populate( partRef, output, numPoints );[m
                 if (rewindPolygons)[m
                 {[m
[1mdiff --git a/src/osgEarth/PBRMaterial.cpp b/src/osgEarth/PBRMaterial.cpp[m
[1mindex d46a4d133..47e8822cc 100644[m
[1m--- a/src/osgEarth/PBRMaterial.cpp[m
[1m+++ b/src/osgEarth/PBRMaterial.cpp[m
[36m@@ -85,7 +85,7 @@[m [mnamespace[m
                     {[m
                         read(a, iter.s(), iter.t());[m
                         temp.set(a.x() * 2.0f - 1.0f, a.y() * 2.0f - 1.0f, a.z() * 2.0f - 1.0f);[m
[31m-                        packed = NormalMapGenerator::pack(temp);[m
[32m+[m[32m                        NormalMapGenerator::pack(temp, packed);[m
                         write(packed, iter.s(), iter.t());[m
                     });[m
             }[m
[1mdiff --git a/src/osgEarth/StringUtils b/src/osgEarth/StringUtils[m
[1mindex b99b2becc..5cc307f94 100644[m
[1m--- a/src/osgEarth/StringUtils[m
[1m+++ b/src/osgEarth/StringUtils[m
[36m@@ -10,7 +10,6 @@[m
 #include <osg/Vec4>[m
 #include <osg/Vec4ub>[m
 #include <string>[m
[31m-#include <cstdint>[m
 #include <algorithm>[m
 #include <vector>[m
 #include <sstream>[m
[1mdiff --git a/src/osgEarth/TerrainOptions b/src/osgEarth/TerrainOptions[m
[1mindex e966974f5..be44c8b3e 100644[m
[1m--- a/src/osgEarth/TerrainOptions[m
[1m+++ b/src/osgEarth/TerrainOptions[m
[36m@@ -25,8 +25,8 @@[m [mnamespace osgEarth[m
         OE_OPTION(unsigned, firstLOD, 0u);[m
         OE_OPTION(bool, enableLighting, true);[m
         OE_OPTION(bool, enableBlending, true);[m
[32m+[m[32m        OE_OPTION(bool, compressNormalMaps, false);[m
         OE_OPTION(unsigned, minNormalMapLOD, 0u);[m
[31m-        OE_OPTION(unsigned, normalMapTileSize, 256u);[m
         OE_OPTION(bool, gpuTessellation, false);[m
         OE_OPTION(float, tessellationLevel, 2.5f);[m
         OE_OPTION(float, tessellationRange, 75.0f);[m
[36m@@ -58,7 +58,6 @@[m [mnamespace osgEarth[m
         OE_OPTION(bool, createTilesAsync, true);[m
         OE_OPTION(bool, createTilesGrouped, true);[m
         OE_OPTION(bool, restrictPolarSubdivision, true);[m
[31m-        OE_OPTION(bool, gpuPaging, false);[m
 [m
         virtual Config getConfig() const;[m
     private:[m
[36m@@ -109,6 +108,7 @@[m [mnamespace osgEarth[m
         void setEnableBlending(const bool& value);[m
         const bool& getEnableBlending() const;[m
 [m
[32m+[m[32m        //! (Currently not used)[m
         //! Minimum level of detail at which to generate elevation-based normal maps,[m
         //! assuming normal maps have been activated. This mitigates the overhead of [m
         //! calculating normal maps for very high altitude scenes where they are no[m
[36m@@ -116,10 +116,6 @@[m [mnamespace osgEarth[m
         void setMinNormalMapLOD(const unsigned& value);[m
         const unsigned& getMinNormalMapLOD() const;[m
 [m
[31m-        //! Size of normal map textures, in pixels, in each dimension.[m
[31m-        void setNormalMapTileSize(const unsigned& vlaue);[m
[31m-        const unsigned& getNormalMapTileSize() const;[m
[31m-[m
         //! Whether the terrain engine will be using GPU tessellation shaders.[m
         //! Even if the terrain option is set to true, the getting will return[m
         //! false if tessellation shader support is not available.[m
[36m@@ -246,9 +242,35 @@[m [mnamespace osgEarth[m
         void setRestrictPolarSubdivision(const bool& value);[m
         const bool& getRestrictPolarSubdivision() const;[m
 [m
[31m-        //! Whether to use GPU-based paging of terrain textures (in NVGL mode only)[m
[31m-        void setGPUPaging(const bool& value);[m
[31m-        const bool& getGPUPaging() const;[m
[32m+[m[32m        //! @deprecated[m
[32m+[m[32m        //! Scale factor for background loading priority of terrain tiles.[m
[32m+[m[32m        //! Default = 1.0. Make it higher to prioritize terrain loading over[m
[32m+[m[32m        //! other modules.[m
[32m+[m[32m        OE_DEPRECATED("NO OP")[m
[32m+[m[32m        void setPriorityScale(const float& value);[m
[32m+[m[32m        const float& getPriorityScale() const;[m
[32m+[m
[32m+[m[32m        //! @deprecated[m
[32m+[m[32m        //! Whether to activate debugging mode[m
[32m+[m[32m        OE_DEPRECATED("NO OP")[m
[32m+[m[32m        void setDebug(const bool& value);[m
[32m+[m[32m        const bool& getDebug() const;[m
[32m+[m
[32m+[m[32m        //! @deprecated[m
[32m+[m[32m        //! Whether to compress the normal maps before sending to the GPU[m
[32m+[m[32m        OE_DEPRECATED("NO OP")[m
[32m+[m[32m        void setCompressNormalMaps(const bool& value);[m
[32m+[m[32m        const bool& getCompressNormalMaps() const;[m
[32m+[m
[32m+[m[32m        //! @deprecated[m
[32m+[m[32m        //! The minimum level of detail to which the terrain should subdivide (no matter what).[m
[32m+[m[32m        //! If you leave this unset, the terrain will subdivide until the map layers[m
[32m+[m[32m        //! stop providing data (default behavior). If you set a value, the terrain will subdivide[m
[32m+[m[32m        //! to the specified LOD no matter what (and may continue farther if higher-resolution[m
[32m+[m[32m        //! data is available).[m
[32m+[m[32m        OE_DEPRECATED("NO OP")[m
[32m+[m[32m        void setMinLOD(const unsigned& value);[m
[32m+[m[32m        const unsigned& getMinLOD() const;[m
 [m
         TerrainOptionsAPI();[m
         TerrainOptionsAPI(TerrainOptions*);[m
[1mdiff --git a/src/osgEarth/TerrainOptions.cpp b/src/osgEarth/TerrainOptions.cpp[m
[1mindex 7ccedc4d1..d169cdc68 100644[m
[1m--- a/src/osgEarth/TerrainOptions.cpp[m
[1m+++ b/src/osgEarth/TerrainOptions.cpp[m
[36m@@ -28,8 +28,8 @@[m [mTerrainOptions::getConfig() const[m
     conf.set( "lighting", _enableLighting );[m
     conf.set( "cluster_culling", _clusterCulling );[m
     conf.set( "blending", _enableBlending );[m
[32m+[m[32m    conf.set( "compress_normal_maps", _compressNormalMaps);[m
     conf.set( "min_normal_map_lod", _minNormalMapLOD );[m
[31m-    conf.set( "normal_map_tile_size", _normalMapTileSize);[m
     conf.set( "tessellation", _gpuTessellation );[m
     conf.set( "tessellation_level", tessellationLevel());[m
     conf.set( "tessellation_range", tessellationRange());[m
[36m@@ -64,7 +64,6 @@[m [mTerrainOptions::getConfig() const[m
     conf.set("create_tiles_async", createTilesAsync());[m
     conf.set("create_tiles_grouped", createTilesGrouped());[m
     conf.set("restrict_polar_subdivision", restrictPolarSubdivision());[m
[31m-    conf.set("gpu_paging", gpuPaging());[m
 [m
     conf.set("expiration_range", minExpiryRange()); // legacy[m
     conf.set("expiration_threshold", minResidentTiles()); // legacy[m
[36m@@ -84,12 +83,12 @@[m [mTerrainOptions::fromConfig(const Config& conf)[m
     conf.get( "lighting", _enableLighting );[m
     conf.get( "cluster_culling", _clusterCulling );[m
     conf.get( "blending", _enableBlending );[m
[32m+[m[32m    conf.get( "compress_normal_maps", _compressNormalMaps);[m
     conf.get( "min_normal_map_lod", _minNormalMapLOD );[m
[31m-    conf.get( "normal_map_tile_size", _normalMapTileSize);[m
     conf.get( "tessellation", _gpuTessellation );[m
     conf.get( "gpu_tessellation", _gpuTessellation); //bc[m
[31m-    conf.get( "tessellation_level", tessellationLevel());[m
[31m-    conf.get( "tessellation_range", tessellationRange());[m
[32m+[m[32m    conf.get("tessellation_level", tessellationLevel());[m
[32m+[m[32m    conf.get("tessellation_range", tessellationRange());[m
     conf.get( "debug", _debug );[m
     conf.get( "bin_number", _renderBinNumber );[m
     conf.get( "min_expiry_time", _minExpiryTime);[m
[36m@@ -122,7 +121,6 @@[m [mTerrainOptions::fromConfig(const Config& conf)[m
     conf.get("create_tiles_async", createTilesAsync());[m
     conf.get("create_tiles_grouped", createTilesGrouped());[m
     conf.get("restrict_polar_subdivision", restrictPolarSubdivision());[m
[31m-    conf.get("gpu_paging", gpuPaging());[m
 [m
     conf.get("expiration_range", minExpiryRange()); // legacy[m
     conf.get("expiration_threshold", minResidentTiles()); // legacy[m
[36m@@ -170,14 +168,16 @@[m [mTerrainOptionsAPI::TerrainOptionsAPI(const TerrainOptionsAPI& rhs) :[m
 OE_OPTION_IMPL(TerrainOptionsAPI, int, TileSize, tileSize);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, float, MinTileRangeFactor, minTileRangeFactor);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MaxLOD, maxLOD);[m
[32m+[m[32mOE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MinLOD, minLOD);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, FirstLOD, firstLOD);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, EnableLighting, enableLighting);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, ClusterCulling, clusterCulling);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, EnableBlending, enableBlending);[m
[32m+[m[32mOE_OPTION_IMPL(TerrainOptionsAPI, bool, CompressNormalMaps, compressNormalMaps);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MinNormalMapLOD, minNormalMapLOD);[m
[31m-OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, NormalMapTileSize, normalMapTileSize);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, float, TessellationLevel, tessellationLevel);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, float, TessellationRange, tessellationRange);[m
[32m+[m[32mOE_OPTION_IMPL(TerrainOptionsAPI, bool, Debug, debug);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, int, RenderBinNumber, renderBinNumber);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, CastShadows, castShadows);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, LODMethod, LODMethod, lodMethod)[m
[36m@@ -196,6 +196,7 @@[m [mOE_OPTION_IMPL(TerrainOptionsAPI, bool, NormalizeEdges, normalizeEdges);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, MorphTerrain, morphTerrain);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, MorphImagery, morphImagery);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MergesPerFrame, mergesPerFrame);[m
[32m+[m[32mOE_OPTION_IMPL(TerrainOptionsAPI, float, PriorityScale, priorityScale);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, std::string, TextureCompressionMethod, textureCompression);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, Concurrency, concurrency);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, float, ScreenSpaceError, screenSpaceError);[m
[36m@@ -204,7 +205,6 @@[m [mOE_OPTION_IMPL(TerrainOptionsAPI, bool, Visible, visible);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, CreateTilesAsync, createTilesAsync);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, CreateTilesGrouped, createTilesGrouped);[m
 OE_OPTION_IMPL(TerrainOptionsAPI, bool, RestrictPolarSubdivision, restrictPolarSubdivision);[m
[31m-OE_OPTION_IMPL(TerrainOptionsAPI, bool, GPUPaging, gpuPaging);[m
 [m
 bool[m
 TerrainOptionsAPI::getGPUTessellation() const[m
[1mdiff --git a/src/osgEarth/TerrainTileModelFactory.cpp b/src/osgEarth/TerrainTileModelFactory.cpp[m
[1mindex 0f0f04f6c..616e6e3ba 100644[m
[1m--- a/src/osgEarth/TerrainTileModelFactory.cpp[m
[1m+++ b/src/osgEarth/TerrainTileModelFactory.cpp[m
[36m@@ -17,10 +17,10 @@[m
 [m
 using namespace osgEarth;[m
 [m
[31m-#define LABEL_IMAGERY "Terrain images"[m
[31m-#define LABEL_NORMALMAP "Terrain normals"[m
[31m-#define LABEL_ELEVATION "Terrain elevation"[m
[31m-#define LABEL_COVERAGE "Terrain coverage"[m
[32m+[m[32m#define LABEL_IMAGERY "Terrain textures"[m
[32m+[m[32m#define LABEL_NORMALMAP "Terrain textures"[m
[32m+[m[32m#define LABEL_ELEVATION "Terrain textures"[m
[32m+[m[32m#define LABEL_COVERAGE "Terrain textures"[m
 [m
 //.........................................................................[m
 [m
[36m@@ -429,7 +429,7 @@[m [mTerrainTileModelFactory::addElevation([m
         return;[m
 [m
 [m
[31m-    osg::ref_ptr<ElevationTile> elevTex;[m
[32m+[m[32m    osg::ref_ptr<ElevationTexture> elevTex;[m
 [m
     const bool acceptLowerRes = false;[m
 [m
[36m@@ -438,10 +438,10 @@[m [mTerrainTileModelFactory::addElevation([m
         if (elevTex.valid())[m
         {[m
             model->elevation.revision = combinedRevision;[m
[31m-            model->elevation.texture = Texture::create(elevTex->getElevationTile());[m
[32m+[m[32m            model->elevation.texture = Texture::create(elevTex->getElevationTexture());[m
 [m
             auto [minh, maxh] = elevTex->getMaxima();[m
[31m-            if (elevTex->encoding() != ElevationTile::Encoding::R32F)[m
[32m+[m[32m            if (model->elevation.texture->getPixelFormat() == GL_RG)[m
             {[m
                 model->elevation.texture->minValue() = minh;[m
                 model->elevation.texture->maxValue() = maxh;[m
[36m@@ -452,10 +452,10 @@[m [mTerrainTileModelFactory::addElevation([m
             model->elevation.minHeight = minh;[m
             model->elevation.maxHeight = maxh;[m
 [m
[31m-            if (_options.useNormalMaps() == true && key.getLOD() >= _options.minNormalMapLOD().value())[m
[32m+[m[32m            if (_options.useNormalMaps() == true)[m
             {[m
                 // Make a normal map if it doesn't already exist[m
[31m-                elevTex->generateNormalMap(map, _options.normalMapTileSize().value(), &_workingSet, progress);[m
[32m+[m[32m                elevTex->generateNormalMap(map, &_workingSet, progress);[m
 [m
                 if (elevTex->getNormalMapTexture())[m
                 {[m
[1mdiff --git a/src/osgEarth/TextureArena b/src/osgEarth/TextureArena[m
[1mindex 1eff76e0d..0f6979ed5 100644[m
[1m--- a/src/osgEarth/TextureArena[m
[1m+++ b/src/osgEarth/TextureArena[m
[36m@@ -100,10 +100,6 @@[m [mnamespace osgEarth[m
         };[m
         mutable osg::buffered_object<GLObjects> _globjects;[m
 [m
[31m-        // for gpu paging support:[m
[31m-        OE_PROPERTY(int, ownerRevision, -1);[m
[31m-        OE_PROPERTY(bool, dormant, false);[m
[31m-[m
         ~Texture();[m
 [m
     protected:[m
[36m@@ -141,19 +137,6 @@[m [mnamespace osgEarth[m
         void setAutoRelease(bool value);[m
         bool getAutoRelease() const { return _autoRelease; }[m
 [m
[31m-        //! Whether to automatically free the GPU memory associates with dormant[m
[31m-        //! textures. A dormat texture exists in the arena, but has not been accessed[m
[31m-        //! by a recent call to add(). Auto Paging can greatly reduce GPU memory usage[m
[31m-        //! at the expense of potential frame drops. Default is false.[m
[31m-        //! When true, the developer must update the arena's revision (by calling[m
[31m-        //! setRevision) before calling add() to use textures.[m
[31m-        void setAutoPaging(bool value);[m
[31m-        bool getAutoPaging() const { return _autoPaging; }[m
[31m-[m
[31m-        //! When auto paging is enabled, sets the current revision that textures must[m
[31m-        //! match to avoid being paged out of the GPU.[m
[31m-        void setRevision(int value) { _ownerRevision = value; }[m
[31m-[m
         //! Sets the GLSL binding point for the arena. Default is 1.[m
         void setBindingPoint(unsigned value);[m
 [m
[36m@@ -227,8 +210,6 @@[m [mnamespace osgEarth[m
         bool _useUBO = false;[m
         mutable int _releasePtr = 0;[m
         unsigned _maxDim = 65536u;[m
[31m-        bool _autoPaging = false;[m
[31m-        int _ownerRevision = 0;[m
 [m
         mutable Mutex _m;[m
 [m
[1mdiff --git a/src/osgEarth/TextureArena.cpp b/src/osgEarth/TextureArena.cpp[m
[1mindex c283e7a8a..8c2f56236 100644[m
[1m--- a/src/osgEarth/TextureArena.cpp[m
[1m+++ b/src/osgEarth/TextureArena.cpp[m
[36m@@ -196,8 +196,6 @@[m [mTexture::compileGLObjects(osg::State& state) const[m
     if (!needsCompile(state))[m
         return false;[m
 [m
[31m-    OE_DEBUG << LC << "Compiling " << name() << std::endl;[m
[31m-[m
     OE_PROFILING_ZONE;[m
     OE_PROFILING_ZONE_TEXT(name().c_str());[m
     OE_SOFT_ASSERT_AND_RETURN(dataLoaded() == true, false);[m
[36m@@ -232,14 +230,11 @@[m [mTexture::compileGLObjects(osg::State& state) const[m
         }[m
 [m
         GLenum pixelFormat = image->getPixelFormat();[m
[31m-        GLenum dataType = image->getDataType();[m
 [m
         GLenum gpuInternalFormat =[m
             image->isCompressed() ? image->getInternalTextureFormat() :[m
             internalFormat().isSet() ? internalFormat().get() :[m
[31m-            pixelFormat == GL_RED && dataType == GL_FLOAT ? GL_R32F :[m
[31m-            pixelFormat == GL_RED && dataType == GL_UNSIGNED_SHORT ? GL_R16 :[m
[31m-            pixelFormat == GL_RED && dataType == GL_UNSIGNED_BYTE ? GL_R8 :[m
[32m+[m[32m            pixelFormat == GL_RED ? GL_R32F :[m
             pixelFormat == GL_RG ? GL_RG8 :[m
             pixelFormat == GL_RGB ? GL_RGB8 :[m
             GL_RGBA8;[m
[36m@@ -509,10 +504,8 @@[m [mTexture::releaseGLObjects(osg::State* state, bool force) const[m
         }[m
     }[m
 [m
[31m-#if 1 // GW TESTING[m
     if (osgTexture().valid())[m
         osgTexture()->releaseGLObjects(state);[m
[31m-#endif[m
 }[m
 [m
 [m
[36m@@ -543,12 +536,6 @@[m [mTextureArena::setAutoRelease(bool value)[m
     _autoRelease = value;[m
 }[m
 [m
[31m-void[m
[31m-TextureArena::setAutoPaging(bool value)[m
[31m-{[m
[31m-    _autoPaging = value;[m
[31m-}[m
[31m-[m
 void[m
 TextureArena::setBindingPoint(unsigned value)[m
 {[m
[36m@@ -621,20 +608,7 @@[m [mTextureArena::add(Texture::Ptr tex, const osgDB::Options* readOptions)[m
     // First check whether it's already there; if so, return the index.[m
     int existingIndex = find_no_lock(tex);[m
     if (existingIndex >= 0)[m
[31m-    {[m
[31m-        if (tex->dormant())[m
[31m-        {[m
[31m-            for (unsigned i = 0; i < _globjects.size(); ++i)[m
[31m-            {[m
[31m-                if (_globjects[i]._inUse)[m
[31m-                {[m
[31m-                    _globjects[i]._toCompile.push(existingIndex);[m
[31m-                }[m
[31m-            }[m
[31m-            tex->dormant() = false;[m
[31m-        }[m
         return existingIndex;[m
[31m-    }[m
 [m
     OE_SOFT_ASSERT_AND_RETURN(tex->_host == nullptr, -1,[m
         "Illegal attempt to add a Texture to more than one TextureArena");[m
[36m@@ -688,8 +662,7 @@[m [mTextureArena::add(Texture::Ptr tex, const osgDB::Options* readOptions)[m
                 // normalize the internal texture format[m
                 GLenum internalFormat =[m
                     tex->internalFormat().isSet() ? tex->internalFormat().get() :[m
[31m-                    image->getPixelFormat() == GL_RED && image->getDataType() == GL_FLOAT ? GL_R32F :[m
[31m-                    image->getPixelFormat() == GL_RED && image->getDataType() == GL_UNSIGNED_SHORT ? GL_R16 :[m
[32m+[m[32m                    image->getPixelFormat() == GL_RED ? GL_R32F :[m
                     image->getPixelFormat() == GL_RG ? GL_RG8 :[m
                     image->getPixelFormat() == GL_RGB ? GL_RGB8 :[m
                     image->getPixelFormat() == GL_RGBA ? GL_RGBA8 :[m
[36m@@ -739,6 +712,10 @@[m [mTextureArena::add(Texture::Ptr tex, const osgDB::Options* readOptions)[m
     {[m
         if (_globjects[i]._inUse)[m
         {[m
[32m+[m[32m            //if (index < _globjects[i]._handles.size())[m
[32m+[m[32m            //{[m
[32m+[m[32m            //    _globjects[i]._handles[index] = 0;[m
[32m+[m[32m            //}[m
             _globjects[i]._toCompile.push(index);[m
         }[m
     }[m
[36m@@ -797,31 +774,6 @@[m [mTextureArena::purgeTextureIfOrphaned_no_lock(unsigned index)[m
         tex->releaseGLObjects(nullptr);[m
         tex = nullptr;[m
     }[m
[31m-[m
[31m-    else if (_autoPaging && tex && tex->ownerRevision() < _ownerRevision)[m
[31m-    {[m
[31m-        //tex->releaseGLObjects(nullptr);[m
[31m-        OE_DEBUG << LC << "Purging texture '" << tex->name() << "' tex=" << tex->ownerRevision()[m
[31m-            << " arena=" << _ownerRevision << std::endl;[m
[31m-[m
[31m-        tex->ownerRevision() = _ownerRevision;[m
[31m-[m
[31m-        tex->releaseGLObjects(nullptr, true);[m
[31m-[m
[31m-        int index = find_no_lock(tex);[m
[31m-[m
[31m-        for (unsigned i = 0; i < _globjects.size(); ++i)[m
[31m-        {[m
[31m-            GLObjects& gc = _globjects[i];[m
[31m-            if (gc._inUse)[m
[31m-            {[m
[31m-                gc._handles[index] = 0;[m
[31m-                gc._handleBufferDirty = true;[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        tex->dormant() = true;[m
[31m-    }[m
 }[m
 [m
 void[m
[1mdiff --git a/src/osgEarth/TileIndex.cpp b/src/osgEarth/TileIndex.cpp[m
[1mindex ff8e7e6b3..ef58bc7fb 100644[m
[1m--- a/src/osgEarth/TileIndex.cpp[m
[1m+++ b/src/osgEarth/TileIndex.cpp[m
[36m@@ -104,7 +104,7 @@[m [mTileIndex::getFiles(const osgEarth::GeoExtent& extent, std::vector< std::string[m
 [m
 bool TileIndex::add( const std::string& filename, const GeoExtent& extent )[m
 {       [m
[31m-    osg::ref_ptr< osgEarth::Polygon > polygon = new osgEarth::Polygon();[m
[32m+[m[32m    osg::ref_ptr< Polygon > polygon = new Polygon();[m
     polygon->push_back( osg::Vec3d(extent.bounds().xMin(), extent.bounds().yMin(), 0) );[m
     polygon->push_back( osg::Vec3d(extent.bounds().xMax(), extent.bounds().yMin(), 0) );[m
     polygon->push_back( osg::Vec3d(extent.bounds().xMax(), extent.bounds().yMax(), 0) );[m
[1mdiff --git a/src/osgEarth/TiledModelLayer.cpp b/src/osgEarth/TiledModelLayer.cpp[m
[1mindex f0fcd1d02..efe98b6f4 100644[m
[1m--- a/src/osgEarth/TiledModelLayer.cpp[m
[1m+++ b/src/osgEarth/TiledModelLayer.cpp[m
[36m@@ -106,15 +106,6 @@[m [munsigned TiledModelLayer::getMaxLevel() const[m
 osg::ref_ptr<osg::Node>[m
 TiledModelLayer::createTile(const TileKey& key, ProgressCallback* progress) const[m
 {[m
[31m-    if (getStatus().isError())[m
[31m-        return {};[m
[31m-[m
[31m-    if (!getProfile())[m
[31m-    {[m
[31m-        setStatus(Status::ResourceUnavailable, "No profile");[m
[31m-        return {};[m
[31m-    }[m
[31m-[m
     NetworkMonitor::ScopedRequestLayer layerRequest(getName());[m
 [m
     osg::ref_ptr<osg::Node> result;[m
[1mdiff --git a/src/osgEarth/VerticalDatum b/src/osgEarth/VerticalDatum[m
[1mindex bc9f63b30..b171a6aec 100644[m
[1m--- a/src/osgEarth/VerticalDatum[m
[1m+++ b/src/osgEarth/VerticalDatum[m
[36m@@ -71,17 +71,14 @@[m [mnamespace osgEarth[m
          * Converts an MSL value (height relative to a mean sea level model) to the[m
          * corresponding HAE value (height above the model's reference ellipsoid)[m
          */[m
[31m-        inline double msl2hae(double lat_deg, double lon_deg, double msl) const {[m
[31m-            return _geoid.valid() ? msl + _geoid->getHeight(lat_deg, lon_deg, INTERP_BILINEAR) : msl;[m
[31m-        }[m
[32m+[m[32m        virtual double msl2hae( double lat_deg, double lon_deg, double msl ) const;[m
 [m
         /**[m
          * Converts an HAE value (height above the model's reference ellipsoid) to the[m
          * corresponding MSL value (height relative to a mean sea level model)[m
          */[m
[31m-        inline double hae2msl(double lat_deg, double lon_deg, double hae) const {[m
[31m-            return _geoid.valid() ? hae - _geoid->getHeight(lat_deg, lon_deg, INTERP_BILINEAR) : hae;[m
[31m-        }[m
[32m+[m[32m        virtual double hae2msl(double lat_deg, double lon_deg, double hae) const;[m
[32m+[m
 [m
     public: // properties[m
 [m
[1mdiff --git a/src/osgEarth/VerticalDatum.cpp b/src/osgEarth/VerticalDatum.cpp[m
[1mindex acf8dcf85..b6a800619 100644[m
[1m--- a/src/osgEarth/VerticalDatum.cpp[m
[1m+++ b/src/osgEarth/VerticalDatum.cpp[m
[36m@@ -196,30 +196,16 @@[m [mVerticalDatum::transform(const VerticalDatum* from,[m
         ystep = (ne.y()-sw.y()) / double(rows-1);[m
     }[m
 [m
[31m-    auto fromUnits = from ? from->getUnits() : Units::METERS;[m
[31m-    auto toUnits = to ? to->getUnits() : Units::METERS;[m
[31m-[m
     for( unsigned c=0; c<cols; ++c)[m
     {[m
         double lon = sw.x() + xstep*double(c);[m
         for( unsigned r=0; r<rows; ++r)[m
         {[m
             double lat = sw.y() + ystep*double(r);[m
[31m-            float h = hf->getHeight(c, r);[m
[32m+[m[32m            float& h = hf->getHeight(c, r);[m
             if (h != NO_DATA_VALUE)[m
             {[m
[31m-                //VerticalDatum::transform( from, to, lat, lon, h );[m
[31m-[m
[31m-                if (from)[m
[31m-                    h = from->msl2hae(lat, lon, h);[m
[31m-[m
[31m-                if (fromUnits != toUnits)[m
[31m-                    h = fromUnits.convertTo(toUnits, h);[m
[31m-[m
[31m-                if (to)[m
[31m-                    h = to->hae2msl(lat, lon, h);[m
[31m-[m
[31m-                hf->setHeight(c, r, h);[m
[32m+[m[32m                VerticalDatum::transform( from, to, lat, lon, h );[m
             }[m
         }[m
     }[m
[36m@@ -227,6 +213,18 @@[m [mVerticalDatum::transform(const VerticalDatum* from,[m
     return true;[m
 }[m
 [m
[32m+[m[32mdouble[m[41m [m
[32m+[m[32mVerticalDatum::msl2hae( double lat_deg, double lon_deg, double msl ) const[m
[32m+[m[32m{[m
[32m+[m[32m    return _geoid.valid() ? msl + _geoid->getHeight(lat_deg, lon_deg, INTERP_BILINEAR) : msl;[m
[32m+[m[32m}[m
[32m+[m
[32m+[m[32mdouble[m
[32m+[m[32mVerticalDatum::hae2msl( double lat_deg, double lon_deg, double hae ) const[m
[32m+[m[32m{[m
[32m+[m[32m    return _geoid.valid() ? hae - _geoid->getHeight(lat_deg, lon_deg, INTERP_BILINEAR) : hae;[m
[32m+[m[32m}[m
[32m+[m
 bool [m
 VerticalDatum::isEquivalentTo( const VerticalDatum* rhs ) const[m
 {[m
[1mdiff --git a/src/osgEarthDrivers/engine_mp/TileModelCompiler.cpp b/src/osgEarthDrivers/engine_mp/TileModelCompiler.cpp[m
[1mindex 12364a2c8..f1a8392e0 100644[m
[1m--- a/src/osgEarthDrivers/engine_mp/TileModelCompiler.cpp[m
[1m+++ b/src/osgEarthDrivers/engine_mp/TileModelCompiler.cpp[m
[36m@@ -762,7 +762,7 @@[m [mnamespace[m
             int num_i = max_i - min_i + 1;[m
             int num_j = max_j - min_j + 1;[m
 [m
[31m-            osg::ref_ptr<osgEarth::Polygon> maskSkirtPoly = new osgEarth::Polygon();[m
[32m+[m[32m            osg::ref_ptr<Polygon> maskSkirtPoly = new Polygon();[m[41m[m
             maskSkirtPoly->resize(num_i * 2 + num_j * 2 - 4);[m
 [m
             for (int i = 0; i < num_i; i++)[m
[36m@@ -856,7 +856,7 @@[m [mnamespace[m
                 // Add the outter stitching bounds to the collection of vertices to be used for triangulation[m
                 //	coordsArray->insert(coordsArray->end(), (*mr)._internal->begin(), (*mr)._internal->end());[m
                 //Create local polygon representing mask[m
[31m-                osg::ref_ptr<Polygon> maskPoly = new osgEarth::Polygon();[m
[32m+[m[32m                osg::ref_ptr<Polygon> maskPoly = new Polygon();[m[41m[m
                 for (osg::Vec3dArray::iterator it = (*mr)._boundary->begin(); it != (*mr)._boundary->end(); ++it)[m
                 {[m
                     osg::Vec3d local;[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/EngineContext.cpp b/src/osgEarthDrivers/engine_rex/EngineContext.cpp[m
[1mindex 13d144338..dcbfc38bc 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/EngineContext.cpp[m
[1m+++ b/src/osgEarthDrivers/engine_rex/EngineContext.cpp[m
[36m@@ -50,8 +50,6 @@[m [mEngineContext::EngineContext([m
         Registry::instance()->getMaxTextureSize());[m
 [m
     _textures->setMaxTextureSize(maxSize);[m
[31m-[m
[31m-    _textures->setAutoPaging(_options.getGPUPaging());[m
 }[m
 [m
 osg::ref_ptr<const Map>[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/LayerDrawable b/src/osgEarthDrivers/engine_rex/LayerDrawable[m
[1mindex b56bd99f0..b43e3e4e5 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/LayerDrawable[m
[1m+++ b/src/osgEarthDrivers/engine_rex/LayerDrawable[m
[36m@@ -172,7 +172,6 @@[m [mnamespace osgEarth { namespace REX[m
             RenderState();[m
 [m
             bool dirty;[m
[31m-            int revision = -1;[m
 [m
             DrawTileCommands tiles;[m
             std::vector<GL4Tile> tilebuf;[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/LayerDrawable.cpp b/src/osgEarthDrivers/engine_rex/LayerDrawable.cpp[m
[1mindex 72524adf2..0cea5265e 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/LayerDrawable.cpp[m
[1m+++ b/src/osgEarthDrivers/engine_rex/LayerDrawable.cpp[m
[36m@@ -169,8 +169,6 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
 [m
     if (_tiles != _rs.tiles)[m
     {[m
[31m-        ++_rs.revision;[m
[31m-[m
         // Next assemble the TileBuffer structures[m
         if (_rs.tilebuf.size() < _tiles.size())[m
         {[m
[36m@@ -203,7 +201,6 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
                 const Sampler& color = (*tile._colorSamplers)[SamplerBinding::COLOR];[m
                 if (color._texture != nullptr)[m
                 {[m
[31m-                    color._texture->ownerRevision() = _rs.revision;[m
                     buf.colorIndex = textures->add(color._texture);[m
                     COPY_MAT4F(color._matrix, buf.colorMat);[m
                 }[m
[36m@@ -211,7 +208,6 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
                 const Sampler& parent = (*tile._colorSamplers)[SamplerBinding::COLOR_PARENT];[m
                 if (parent._texture != nullptr)[m
                 {[m
[31m-                    parent._texture->ownerRevision() = _rs.revision;[m
                     buf.parentIndex = textures->add(parent._texture);[m
                     COPY_MAT4F(parent._matrix, buf.parentMat);[m
                 }[m
[36m@@ -227,7 +223,6 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
                     s._texture->compress() = false;[m
                     s._texture->mipmap() = false;[m
                     s._texture->keepImage() = true; // never discard.. we use it elsewhere[m
[31m-                    s._texture->ownerRevision() = _rs.revision;[m
                     buf.elevIndex = textures->add(s._texture);[m
                     COPY_MAT4F(s._matrix, buf.elevMat);[m
 [m
[36m@@ -251,10 +246,9 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
                 const Sampler& s = (*tile._sharedSamplers)[SamplerBinding::NORMAL];[m
                 if (s._texture)[m
                 {[m
[31m-                    s._texture->compress() = true;[m
[32m+[m[32m                    s._texture->compress() = false;[m
                     s._texture->mipmap() = true;[m
                     s._texture->maxAnisotropy() = 1.0f;[m
[31m-                    s._texture->ownerRevision() = _rs.revision;[m
                     buf.normalIndex = textures->add(s._texture);[m
                     COPY_MAT4F(s._matrix, buf.normalMat);[m
                 }[m
[36m@@ -272,7 +266,6 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
                         s._texture->compress() = false;[m
                         s._texture->mipmap() = false;[m
                         s._texture->maxAnisotropy() = 1.0f;[m
[31m-                        s._texture->ownerRevision() = _rs.revision;[m
                         buf.landcoverIndex = textures->add(s._texture);[m
                         COPY_MAT4F(s._matrix, buf.landcoverMat);[m
                     }[m
[36m@@ -293,7 +286,6 @@[m [mLayerDrawableNVGL::refreshRenderState()[m
                             s._texture->compress() = false;[m
                             s._texture->mipmap() = true;[m
                             //s._arena_texture->_maxAnisotropy = 4.0f;[m
[31m-                            s._texture->ownerRevision() = _rs.revision;[m
                             buf.sharedIndex[k] = textures->add(s._texture);[m
                             COPY_MAT4F(s._matrix, buf.sharedMat[k]);[m
                         }[m
[36m@@ -477,8 +469,6 @@[m [mLayerDrawableNVGL::drawImplementation(osg::RenderInfo& ri) const[m
 [m
             OE_SOFT_ASSERT(_rs.commands.size() == _rs.tiles.size());[m
         }[m
[31m-[m
[31m-        _context->textures()->setRevision(_rs.revision);[m
     }[m
 [m
     // Apply the the texture arena:[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/RexEngine.SDK.GL4.glsl b/src/osgEarthDrivers/engine_rex/RexEngine.SDK.GL4.glsl[m
[1mindex d1ca5fa55..af691975e 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/RexEngine.SDK.GL4.glsl[m
[1m+++ b/src/osgEarthDrivers/engine_rex/RexEngine.SDK.GL4.glsl[m
[36m@@ -40,11 +40,11 @@[m [mfloat oe_terrain_getElevation(in vec2 uv)[m
     if (index >= 0)[m
     {[m
         vec2 uv_scaledBiased = oe_terrain_getElevationCoord(uv);[m
[31m-        float encoded = texture(sampler2D(oe_terrain_tex[index]), uv_scaledBiased).r;[m
[32m+[m[32m        vec2 encoded = texture(sampler2D(oe_terrain_tex[index]), uv_scaledBiased).rg;[m
         float minh = oe_tile[oe_tileID].elevMin;[m
         float maxh = oe_tile[oe_tileID].elevMax;[m
[31m-        return minh == maxh ? encoded : mix(minh, maxh, encoded);[m
[31m-            //mix(minh, maxh, dot(encoded, vec2(65280.0, 255.0)) / 65535.0); // RG8[m
[32m+[m[32m        return minh == maxh ? encoded.r :[m
[32m+[m[32m            mix(minh, maxh, dot(encoded, vec2(65280.0, 255.0)) / 65535.0);[m
     }[m
     return 0.0;[m
 }[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/RexEngine.SDK.glsl b/src/osgEarthDrivers/engine_rex/RexEngine.SDK.glsl[m
[1mindex f070a39f8..d2d0289cf 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/RexEngine.SDK.glsl[m
[1m+++ b/src/osgEarthDrivers/engine_rex/RexEngine.SDK.glsl[m
[36m@@ -33,11 +33,11 @@[m [mfloat oe_terrain_getElevation(in vec2 uv)[m
         + oe_tile_elevTexelCoeff.x * oe_tile_elevationTexMatrix[3].st     // bias[m
         + oe_tile_elevTexelCoeff.y;[m
 [m
[31m-    float encoded = texture(oe_tile_elevationTex, uv_scaledBiased).r;[m
[32m+[m[32m    vec2 encoded = texture(oe_tile_elevationTex, uv_scaledBiased).rg;[m
[32m+[m
     float minh = oe_tile_elevMinMax[0];[m
     float maxh = oe_tile_elevMinMax[1];[m
[31m-    return minh == maxh ? encoded : mix(minh, maxh, encoded);[m
[31m-        //mix(minh, maxh, dot(encoded, vec2(65280.0, 255.0)) / 65535.0); // RG8[m
[32m+[m[32m    return minh == maxh ? encoded.r : mix(minh, maxh, dot(encoded, vec2(65280.0, 255.0)) / 65535.0);[m
 }[m
 [m
 // Read the elevation at the build-in tile coordinates (convenience)[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/RexTerrainEngineNode.cpp b/src/osgEarthDrivers/engine_rex/RexTerrainEngineNode.cpp[m
[1mindex 989734f1e..880dc667c 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/RexTerrainEngineNode.cpp[m
[1m+++ b/src/osgEarthDrivers/engine_rex/RexTerrainEngineNode.cpp[m
[36m@@ -616,7 +616,7 @@[m [mRexTerrainEngineNode::setupRenderBindings()[m
         elevation.usage() = SamplerBinding::ELEVATION;[m
         elevation.samplerName() = "oe_tile_elevationTex";[m
         elevation.matrixName() = "oe_tile_elevationTexMatrix";[m
[31m-        elevation.setDefaultTexture(osgEarth::createEmptyElevationTile());[m
[32m+[m[32m        elevation.setDefaultTexture(osgEarth::createEmptyElevationTexture());[m
         elevation.getDefaultTexture()->setName("terrain default elevation");[m
 [m
         if (!GLUtils::useNVGL())[m
[36m@@ -698,7 +698,6 @@[m [mRexTerrainEngineNode::dirtyTerrainOptions()[m
     if (arena)[m
     {[m
         arena->setMaxTextureSize(options.getMaxTextureSize());[m
[31m-        arena->setAutoPaging(options.getGPUPaging());[m
     }[m
 [m
     _tiles->setNotifyNeighbors(options.getNormalizeEdges() == true);[m
[36m@@ -1397,6 +1396,12 @@[m [mRexTerrainEngineNode::updateState()[m
                 _surfaceSS->setDefine("OE_TERRAIN_BLEND_IMAGERY");[m
             }[m
 [m
[32m+[m[32m            // Compressed normal maps[m
[32m+[m[32m            if (options.getCompressNormalMaps())[m
[32m+[m[32m            {[m
[32m+[m[32m                _surfaceSS->setDefine("OE_COMPRESSED_NORMAL_MAP");[m
[32m+[m[32m            }[m
[32m+[m
             // Morphing (imagery and terrain)[m
             if (_morphingSupported)[m
             {[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/TerrainCuller.cpp b/src/osgEarthDrivers/engine_rex/TerrainCuller.cpp[m
[1mindex fd386dda6..9a2de6bac 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/TerrainCuller.cpp[m
[1m+++ b/src/osgEarthDrivers/engine_rex/TerrainCuller.cpp[m
[36m@@ -145,7 +145,7 @@[m [mTerrainCuller::addDrawCommand(UID uid, const TileRenderModel* model, const Rende[m
             {[m
                 auto& elevSampler = model->_sharedSamplers[SamplerBinding::ELEVATION];[m
                 if (elevSampler._texture &&[m
[31m-                    ElevationTile::encodingFor(elevSampler._texture->internalFormat().value()) != ElevationTile::Encoding::R32F &&[m
[32m+[m[32m                    elevSampler._texture->getPixelFormat() == GL_RG &&[m
                     elevSampler._texture->minValue().isSet() &&[m
                     elevSampler._texture->maxValue().isSet())[m
                 {[m
[1mdiff --git a/src/osgEarthDrivers/engine_rex/TileDrawable.cpp b/src/osgEarthDrivers/engine_rex/TileDrawable.cpp[m
[1mindex 266162174..e419190de 100644[m
[1m--- a/src/osgEarthDrivers/engine_rex/TileDrawable.cpp[m
[1m+++ b/src/osgEarthDrivers/engine_rex/TileDrawable.cpp[m
[36m@@ -101,14 +101,15 @@[m [mTileDrawable::setElevationRaster(Texture::Ptr image, const osg::Matrixf& scaleBi[m
         const osg::Vec3Array& normals = *static_cast<osg::Vec3Array*>(_geom->getNormalArray());[m
         const osg::Vec3Array& units = *static_cast<osg::Vec3Array*>(_geom->getTexCoordArray());[m
 [m
[32m+[m[32m        //OE_INFO << LC << _key.str() << " - rebuilding height cache" << std::endl;[m
[32m+[m
         ImageUtils::PixelReader readElevation(_elevationRaster->osgTexture()->getImage(0));[m
         readElevation.setBilinear(true);[m
         osg::Vec4f sample;[m
 [m
[31m-        auto encoding = ElevationTile::encodingFor(_elevationRaster->internalFormat().value());[m
         bool decode16bitHeight = _elevationRaster->minValue().isSet();[m
[31m-        float minh = _elevationRaster->minValue().value();[m
[31m-        float maxh = _elevationRaster->maxValue().value();[m
[32m+[m[32m        float minh = decode16bitHeight ? _elevationRaster->minValue().value() : 0.0f;[m
[32m+[m[32m        float maxh = decode16bitHeight ? _elevationRaster->maxValue().value() : 0.0f;[m
 [m
         float[m
             scaleU = _elevationScaleBias(0,0),[m
[36m@@ -125,12 +126,18 @@[m [mTileDrawable::setElevationRaster(Texture::Ptr image, const osg::Matrixf& scaleBi[m
         {[m
             if ( ((int)units[i].z() & VERTEX_HAS_ELEVATION) == 0)[m
             {[m
[31m-                sample = readElevation([m
[32m+[m[32m                readElevation([m
[32m+[m[32m                    sample,[m
                     clamp(units[i].x()*scaleU + biasU, 0.0f, 1.0f),[m
                     clamp(units[i].y()*scaleV + biasV, 0.0f, 1.0f));[m
 [m
[31m-                auto h = ElevationTile::decodeElevation(sample, encoding, minh, maxh);[m
[31m-                _mesh[i] = verts[i] + normals[i] * h;[m
[32m+[m[32m                if (decode16bitHeight)[m
[32m+[m[32m                {[m
[32m+[m[32m                    float t = (sample.r() * 65280.0f + sample.g() * 255.0) / 65535.0f; // [0..1][m
[32m+[m[32m                    sample.r() = minh + t * (maxh - minh); // scale to min/max[m
[32m+[m[32m                }[m
[32m+[m
[32m+[m[32m                _mesh[i] = verts[i] + normals[i] * sample.r();[m
             }[m
             else[m
             {[m
[1mdiff --git a/src/osgEarthDrivers/fastdxt/FastDXTImageProcessor.cpp b/src/osgEarthDrivers/fastdxt/FastDXTImageProcessor.cpp[m
[1mindex 653d41b96..6a7bd3729 100644[m
[1m--- a/src/osgEarthDrivers/fastdxt/FastDXTImageProcessor.cpp[m
[1m+++ b/src/osgEarthDrivers/fastdxt/FastDXTImageProcessor.cpp[m
[36m@@ -16,118 +16,6 @@[m
 using namespace osgEarth;[m
 using namespace osgEarth::Util;[m
 [m
[31m-// Helper function to convert RGB/RGBA to RG8 for BC5 compression[m
[31m-osg::Image* convertToRG8(const osg::Image* image)[m
[31m-{[m
[31m-    if (!image) return nullptr;[m
[31m-    [m
[31m-    int numComponents = 0;[m
[31m-    if (image->getPixelFormat() == GL_RGB) numComponents = 3;[m
[31m-    else if (image->getPixelFormat() == GL_RGBA) numComponents = 4;[m
[31m-    else return nullptr;[m
[31m-    [m
[31m-    int width = image->s();[m
[31m-    int height = image->t();[m
[31m-    int depth = image->r();[m
[31m-    [m
[31m-    // Create new RG image[m
[31m-    unsigned char* rgData = new unsigned char[width * height * depth * 2];[m
[31m-    const unsigned char* srcData = image->data();[m
[31m-    [m
[31m-    // Extract RG channels[m
[31m-    for (int i = 0; i < width * height * depth; ++i)[m
[31m-    {[m
[31m-        rgData[i * 2 + 0] = srcData[i * numComponents + 0]; // Red[m
[31m-        rgData[i * 2 + 1] = srcData[i * numComponents + 1]; // Green[m
[31m-    }[m
[31m-    [m
[31m-    osg::Image* rgImage = new osg::Image();[m
[31m-    rgImage->setImage(width, height, depth, GL_RG8, GL_RG, GL_UNSIGNED_BYTE, [m
[31m-                      rgData, osg::Image::USE_NEW_DELETE);[m
[31m-    [m
[31m-    return rgImage;[m
[31m-}[m
[31m-[m
[31m-void padImageToMultipleOf4(osg::Image* input)[m
[31m-{[m
[31m-    if (input->s() % 4 == 0 && input->t() % 4 == 0)[m
[31m-        return; // Already a multiple of 4[m
[31m-[m
[31m-    unsigned int newS = (input->s() + 3) & ~3; // Round up to next multiple of 4[m
[31m-    unsigned int newT = (input->t() + 3) & ~3; // Round up to next multiple of 4[m
[31m-[m
[31m-    osg::ref_ptr<osg::Image> padded = new osg::Image();[m
[31m-    padded->allocateImage(newS, newT, input->r(), input->getPixelFormat(), input->getDataType());[m
[31m-[m
[31m-    ImageUtils::PixelReader read(input);[m
[31m-    ImageUtils::PixelWriter write(padded);[m
[31m-[m
[31m-    osg::Vec4 pixel;[m
[31m-[m
[31m-    for (unsigned t = 0; t < read.t(); ++t)[m
[31m-    {[m
[31m-        for (unsigned s = 0; s < read.s(); ++s)[m
[31m-        {[m
[31m-            read(pixel, s, t);[m
[31m-            write(pixel, s, t);[m
[31m-        }[m
[31m-[m
[31m-        // pad remaining columns in the output row with the same pixel value:[m
[31m-        for (unsigned ps = read.s(); ps < newS; ++ps)[m
[31m-        {[m
[31m-            write(pixel, ps, t);[m
[31m-        }[m
[31m-    }[m
[31m-[m
[31m-    // pad the remaining rows in the output image with the last row's pixel values:[m
[31m-    for (unsigned pt = read.t(); pt < newT; ++pt)[m
[31m-    {[m
[31m-        for (unsigned ps = 0; ps < newS; ++ps)[m
[31m-        {[m
[31m-            // read from the last valid row:[m
[31m-            read(pixel, ps < (unsigned)read.s() ? ps : (unsigned)read.s() - 1, (unsigned)read.t() - 1);[m
[31m-            write(pixel, ps, pt);[m
[31m-        }[m
[31m-    }[m
[31m-[m
[31m-    padded->setAllocationMode(osg::Image::NO_DELETE);[m
[31m-[m
[31m-    input->setImage(newS, newT, input->r(), input->getInternalTextureFormat(), [m
[31m-        input->getPixelFormat(), input->getDataType(), padded->data(), osg::Image::USE_NEW_DELETE);[m
[31m-}[m
[31m-[m
[31m-void scaleImage(osg::Image* image, int new_s, int new_t)[m
[31m-{[m
[31m-    if (image->s() == new_s && image->t() == new_t)[m
[31m-        return; // No scaling needed[m
[31m-[m
[31m-    // allocate new image:[m
[31m-    osg::ref_ptr<osg::Image> scaled = new osg::Image();[m
[31m-    scaled->allocateImage(new_s, new_t, image->r(), image->getPixelFormat(), image->getDataType());[m
[31m-[m
[31m-    ImageUtils::PixelReader read(image);[m
[31m-    ImageUtils::PixelWriter write(scaled);[m
[31m-[m
[31m-    osg::Vec4 pixel;[m
[31m-[m
[31m-    for (unsigned t = 0; t < (unsigned)new_t; ++t)[m
[31m-    {[m
[31m-        float v = (float)t / (float)(new_t - 1);[m
[31m-[m
[31m-        for (unsigned s = 0; s < (unsigned)new_s; ++s)[m
[31m-        {[m
[31m-            float u = (float)s / (float)(new_s - 1);[m
[31m-[m
[31m-            read(pixel, u, v);[m
[31m-            write(pixel, s, t);[m
[31m-        }[m
[31m-    }[m
[31m-[m
[31m-    scaled->setAllocationMode(osg::Image::NO_DELETE);[m
[31m-    image->setImage(new_s, new_t, image->r(), image->getInternalTextureFormat(),[m
[31m-        image->getPixelFormat(), image->getDataType(), scaled->data(), osg::Image::USE_NEW_DELETE);[m
[31m-}[m
[31m-[m
 class FastDXTProcessor : public osgDB::ImageProcessor[m
 {[m
 public:[m
[36m@@ -152,12 +40,21 @@[m [mpublic:[m
         {[m
             unsigned int s = osg::Image::computeNearestPowerOfTwo(input.s());[m
             unsigned int t = osg::Image::computeNearestPowerOfTwo(input.t());[m
[31m-            //input.scaleImage(s, t, input.r());[m
[32m+[m[32m            input.scaleImage(s, t, input.r());[m
[32m+[m[32m        }[m
[32m+[m
[32m+[m[32m        osg::Image* sourceImage = &input;[m
 [m
[31m-            scaleImage(&input, s, t);[m
[32m+[m[32m        //FastDXT only works on RGBA imagery so we must convert it[m
[32m+[m[32m        osg::ref_ptr< osg::Image > rgba;[m
[32m+[m[32m        if (input.getPixelFormat() != GL_RGBA)[m
[32m+[m[32m        {[m
[32m+[m[32m            rgba = ImageUtils::convertToRGBA8(&input);[m
[32m+[m[32m            sourceImage = rgba.get();[m
         }[m
 [m
[31m-        // Determine compression parameters first[m
[32m+[m[32m        OE_SOFT_ASSERT_AND_RETURN(sourceImage != nullptr, void());[m
[32m+[m
         int format;[m
         GLenum compressedPixelFormat;[m
         int minLevelSize;[m
[36m@@ -174,51 +71,12 @@[m [mpublic:[m
             compressedPixelFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;[m
             minLevelSize = 16;[m
             break;[m
[31m-        case osg::Texture::USE_RGTC2_COMPRESSION:[m
[31m-            format = FORMAT_BC5;[m
[31m-            compressedPixelFormat = GL_COMPRESSED_RED_GREEN_RGTC2_EXT;[m
[31m-            minLevelSize = 16;[m
[31m-            break;[m
         default:[m
             OSG_WARN << "Unhandled compressed format" << compressedFormat << std::endl;[m
             return;[m
             break;[m
         }[m
 [m
[31m-        osg::Image* sourceImage = &input;[m
[31m-[m
[31m-        //Handle format conversion based on compression target[m
[31m-        osg::ref_ptr< osg::Image > converted;[m
[31m-        if (format == FORMAT_BC5)[m
[31m-        {[m
[31m-            // BC5 needs RG format - for now accept GL_RG8 directly or convert from multi-channel formats[m
[31m-            if (input.getPixelFormat() != GL_RG)[m
[31m-            {[m
[31m-                // Convert to RG8 by extracting first two channels from RGB/RGBA[m
[31m-                if (input.getPixelFormat() == GL_RGB || input.getPixelFormat() == GL_RGBA)[m
[31m-                {[m
[31m-                    converted = convertToRG8(&input);[m
[31m-                    sourceImage = converted.get();[m
[31m-                }[m
[31m-                else[m
[31m-                {[m
[31m-                    OSG_WARN << "BC5 compression requires GL_RG, GL_RGB, or GL_RGBA input format" << std::endl;[m
[31m-                    return;[m
[31m-                }[m
[31m-            }[m
[31m-        }[m
[31m-        else[m
[31m-        {[m
[31m-            //DXT1/DXT5 only work on RGBA imagery so we must convert it[m
[31m-            if (input.getPixelFormat() != GL_RGBA)[m
[31m-            {[m
[31m-                converted = ImageUtils::convertToRGBA8(&input);[m
[31m-                sourceImage = converted.get();[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        OE_SOFT_ASSERT_AND_RETURN(sourceImage != nullptr, void());[m
[31m-[m
         if (generateMipMap)[m
         {[m
             // size in bytes of the top-level image set (sum of [0..r-1])[m
[1mdiff --git a/src/osgEarthDrivers/fastdxt/dxt.cpp b/src/osgEarthDrivers/fastdxt/dxt.cpp[m
[1mindex a61662468..e7fd2130f 100644[m
[1m--- a/src/osgEarthDrivers/fastdxt/dxt.cpp[m
[1m+++ b/src/osgEarthDrivers/fastdxt/dxt.cpp[m
[36m@@ -541,99 +541,6 @@[m [mvoid EmitAlphaIndicesFast( const byte *colorBlock, const byte minAlpha, const by[m
   EmitByte( (indices[13] >> 1) | (indices[14] << 2) | (indices[15] << 5) , outData);[m
 }[m
 [m
[31m-// Helper functions for BC5 compression[m
[31m-[m
[31m-// Extract 4x4 block of RG data (2 channels per pixel)[m
[31m-void ExtractBlockRG( const byte *inPtr, int width, byte *colorBlock )[m
[31m-{[m
[31m-  for ( int j = 0; j < 4; j++ ) {[m
[31m-    memcpy( &colorBlock[j*4*2], inPtr, 4*2 ); // 4 pixels * 2 channels[m
[31m-    inPtr += width * 2; // advance to next row[m
[31m-  }[m
[31m-}[m
[31m-[m
[31m-// Find min/max values for a single channel in RG data[m
[31m-void GetMinMaxSingleChannel( const byte *colorBlock, int channelOffset, int stride, byte &minVal, byte &maxVal )[m
[31m-{[m
[31m-  minVal = 255;[m
[31m-  maxVal = 0;[m
[31m-  [m
[31m-  for ( int i = 0; i < 16; i++ ) {[m
[31m-    byte val = colorBlock[i*stride + channelOffset];[m
[31m-    if ( val < minVal ) minVal = val;[m
[31m-    if ( val > maxVal ) maxVal = val;[m
[31m-  }[m
[31m-}[m
[31m-[m
[31m-// Emit indices for a single channel (similar to alpha compression)[m
[31m-void EmitSingleChannelIndicesFast( const byte *colorBlock, int channelOffset, int stride, [m
[31m-                                   const byte minVal, const byte maxVal, byte *&outData )[m
[31m-{[m
[31m-  byte indices[16];[m
[31m-  byte mid = ( maxVal - minVal ) / ( 2 * 7 );[m
[31m-  byte ab1 = minVal + mid;[m
[31m-  byte ab2 = ( 6 * maxVal + 1 * minVal ) / 7 + mid;[m
[31m-  byte ab3 = ( 5 * maxVal + 2 * minVal ) / 7 + mid;[m
[31m-  byte ab4 = ( 4 * maxVal + 3 * minVal ) / 7 + mid;[m
[31m-  byte ab5 = ( 3 * maxVal + 4 * minVal ) / 7 + mid;[m
[31m-  byte ab6 = ( 2 * maxVal + 5 * minVal ) / 7 + mid;[m
[31m-  byte ab7 = ( 1 * maxVal + 6 * minVal ) / 7 + mid;[m
[31m-[m
[31m-  for ( int i = 0; i < 16; i++ ) {[m
[31m-    byte val = colorBlock[i*stride + channelOffset];[m
[31m-[m
[31m-    int b1 = ( val <= ab1 );[m
[31m-    int b2 = ( val <= ab2 );[m
[31m-    int b3 = ( val <= ab3 );[m
[31m-    int b4 = ( val <= ab4 );[m
[31m-    int b5 = ( val <= ab5 );[m
[31m-    int b6 = ( val <= ab6 );[m
[31m-    int b7 = ( val <= ab7 );[m
[31m-[m
[31m-    int index = ( b1 + b2 + b3 + b4 + b5 + b6 + b7 + 1 ) & 7;[m
[31m-    indices[i] = index ^ ( 2 > index );[m
[31m-  }[m
[31m-[m
[31m-  EmitByte( (indices[ 0] >> 0) | (indices[ 1] << 3) | (indices[ 2] << 6) , outData);[m
[31m-  EmitByte( (indices[ 2] >> 2) | (indices[ 3] << 1) | (indices[ 4] << 4) | (indices[ 5] << 7) , outData);[m
[31m-  EmitByte( (indices[ 5] >> 1) | (indices[ 6] << 2) | (indices[ 7] << 5) , outData);[m
[31m-  EmitByte( (indices[ 8] >> 0) | (indices[ 9] << 3) | (indices[10] << 6) , outData);[m
[31m-  EmitByte( (indices[10] >> 2) | (indices[11] << 1) | (indices[12] << 4) | (indices[13] << 7) , outData);[m
[31m-  EmitByte( (indices[13] >> 1) | (indices[14] << 2) | (indices[15] << 5) , outData);[m
[31m-}[m
[31m-[m
[31m-// BC5 compression function - compresses two channels (RG) separately[m
[31m-void CompressImageBC5( const byte *inBuf, byte *outBuf, int width, int height, int &outputBytes )[m
[31m-{[m
[31m-  ALIGN16( byte *outData );[m
[31m-  ALIGN16( byte block[32] ); // 4x4 pixels * 2 channels = 32 bytes[m
[31m-  [m
[31m-  outData = outBuf;[m
[31m-  for ( int j = 0; j < height; j += 4, inBuf += width * 2*4 ) {[m
[31m-    for ( int i = 0; i < width; i += 4 ) {[m
[31m-      [m
[31m-      // Extract 4x4 block of RG data[m
[31m-      ExtractBlockRG( inBuf + i * 2, width, block );[m
[31m-      [m
[31m-      // Compress Red channel (first 8 bytes)[m
[31m-      byte minRed = 255, maxRed = 0;[m
[31m-      GetMinMaxSingleChannel( block, 0, 2, minRed, maxRed ); // channel 0, stride 2[m
[31m-      [m
[31m-      EmitByte( maxRed, outData );[m
[31m-      EmitByte( minRed, outData );[m
[31m-      EmitSingleChannelIndicesFast( block, 0, 2, minRed, maxRed, outData );[m
[31m-      [m
[31m-      // Compress Green channel (next 8 bytes)[m
[31m-      byte minGreen = 255, maxGreen = 0;[m
[31m-      GetMinMaxSingleChannel( block, 1, 2, minGreen, maxGreen ); // channel 1, stride 2[m
[31m-      [m
[31m-      EmitByte( maxGreen, outData );[m
[31m-      EmitByte( minGreen, outData );[m
[31m-      EmitSingleChannelIndicesFast( block, 1, 2, minGreen, maxGreen, outData );[m
[31m-    }[m
[31m-  }[m
[31m-  outputBytes = int( outData - outBuf );[m
[31m-}[m
 [m
 double ComputeError( const byte *original, const byte *dxt, int width, int height)[m
 {[m
[1mdiff --git a/src/osgEarthDrivers/fastdxt/dxt.h b/src/osgEarthDrivers/fastdxt/dxt.h[m
[1mindex 635a99d93..03a70aba7 100644[m
[1m--- a/src/osgEarthDrivers/fastdxt/dxt.h[m
[1m+++ b/src/osgEarthDrivers/fastdxt/dxt.h[m
[36m@@ -64,14 +64,5 @@[m [mvoid CompressImageDXT5( const byte *inBuf, byte *outBuf, int width, int height,[m
 // Compress to DXT5 format, first convert to YCoCg color space[m
 void CompressImageDXT5YCoCg( const byte *inBuf, byte *outBuf, int width, int height, int &outputBytes );[m
 [m
[31m-// Compress to BC5 format (2-channel RG compression)[m
[31m-void CompressImageBC5( const byte *inBuf, byte *outBuf, int width, int height, int &outputBytes );[m
[31m-[m
[31m-// BC5 helper functions[m
[31m-void ExtractBlockRG( const byte *inPtr, int width, byte *colorBlock );[m
[31m-void GetMinMaxSingleChannel( const byte *colorBlock, int channelOffset, int stride, byte &minVal, byte &maxVal );[m
[31m-void EmitSingleChannelIndicesFast( const byte *colorBlock, int channelOffset, int stride, [m
[31m-                                   const byte minVal, const byte maxVal, byte *&outData );[m
[31m-[m
 // Compute error between two images[m
 double ComputeError( const byte *original, const byte *dxt, int width, int height);[m
[1mdiff --git a/src/osgEarthDrivers/fastdxt/libdxt.cpp b/src/osgEarthDrivers/fastdxt/libdxt.cpp[m
[1mindex a880d6f8e..7187777f8 100644[m
[1m--- a/src/osgEarthDrivers/fastdxt/libdxt.cpp[m
[1m+++ b/src/osgEarthDrivers/fastdxt/libdxt.cpp[m
[36m@@ -65,15 +65,6 @@[m [mvoid *slave5ycocg(void *arg)[m
 	return NULL;[m
 }[m
 [m
[31m-void *slavebc5(void *arg)[m
[31m-{[m
[31m-	work_t *param = (work_t*) arg;[m
[31m-	int nbbytes = 0;[m
[31m-	CompressImageBC5( param->in, param->out, param->width, param->height, nbbytes);[m
[31m-	param->nbb = nbbytes;[m
[31m-	return NULL;[m
[31m-}[m
[31m-[m
 int CompressDXT(const byte *in, byte *out, int width, int height, int format)[m
 { [m
   int        nbbytes;[m
[36m@@ -96,9 +87,6 @@[m [mint CompressDXT(const byte *in, byte *out, int width, int height, int format)[m
       case FORMAT_DXT5YCOCG:[m
           slave5ycocg(&job);[m
           break;[m
[31m-      case FORMAT_BC5:[m
[31m-          slavebc5(&job);[m
[31m-          break;[m
   }[m
 [m
   // Join all the threads[m
[1mdiff --git a/src/osgEarthDrivers/fastdxt/libdxt.h b/src/osgEarthDrivers/fastdxt/libdxt.h[m
[1mindex f7dfa90d2..5728d8e2e 100644[m
[1m--- a/src/osgEarthDrivers/fastdxt/libdxt.h[m
[1m+++ b/src/osgEarthDrivers/fastdxt/libdxt.h[m
[36m@@ -28,7 +28,6 @@[m
 #define FORMAT_DXT1      1[m
 #define FORMAT_DXT5      2[m
 #define FORMAT_DXT5YCOCG 3[m
[31m-#define FORMAT_BC5       4[m
 [m
 [m
 int CompressDXT(const byte *in, byte *out, int width, int height, int format);[m
[1mdiff --git a/src/osgEarthDrivers/kml/KML_Polygon.cpp b/src/osgEarthDrivers/kml/KML_Polygon.cpp[m
[1mindex 48737f1be..9e00d7f32 100644[m
[1m--- a/src/osgEarthDrivers/kml/KML_Polygon.cpp[m
[1m+++ b/src/osgEarthDrivers/kml/KML_Polygon.cpp[m
[36m@@ -23,7 +23,7 @@[m [mKML_Polygon::parseStyle(xml_node<>* node, KMLContext& cx, Style& style)[m
 void[m
 KML_Polygon::parseCoords( xml_node<>* node, KMLContext& cx )[m
 {[m
[31m-    osgEarth::Polygon* poly = new osgEarth::Polygon();[m
[32m+[m[32m    Polygon* poly = new Polygon();[m
 [m
     xml_node<>* outer = node->first_node("outerboundaryis", 0, false);[m
     if ( outer )[m
[1mdiff --git a/src/osgEarthImGui/CMakeLists.txt b/src/osgEarthImGui/CMakeLists.txt[m
[1mindex 8d00d2c9c..38989831c 100644[m
[1m--- a/src/osgEarthImGui/CMakeLists.txt[m
[1m+++ b/src/osgEarthImGui/CMakeLists.txt[m
[36m@@ -27,7 +27,6 @@[m [mset(STOCK_PANELS[m
     NodeGraphGUI[m
     PickerGUI[m
     RenderingGUI[m
[31m-    ResourceLibraryGUI[m
     SceneGraphGUI[m
     SearchGUI[m
     ShaderGUI[m
[1mdiff --git a/src/osgEarthImGui/ImGuiPanel b/src/osgEarthImGui/ImGuiPanel[m
[1mindex 6cf12293b..d48c04625 100644[m
[1m--- a/src/osgEarthImGui/ImGuiPanel[m
[1m+++ b/src/osgEarthImGui/ImGuiPanel[m
[36m@@ -14,13 +14,11 @@[m
 #include <osgEarth/GeoData>[m
 #include <osgEarth/NodeUtils>[m
 #include <osgEarth/StringUtils>[m
[31m-#include <osgEarth/AnnotationUtils>[m
 [m
 #include <osgDB/ReadFile>[m
 #include <osg/observer_ptr>[m
 #include <osg/StateSet>[m
 #include <osg/RenderInfo>[m
[31m-#include <osg/ShapeDrawable>[m
 #include <osgViewer/View>[m
 #include <memory>[m
 [m
[36m@@ -419,18 +417,6 @@[m [mnamespace ImGuiEx[m
         }[m
     }[m
 [m
[31m-    static void Texture(GLuint id, unsigned width, unsigned height, GLenum internalFormat = 0)[m
[31m-    {[m
[31m-        if (internalFormat == GL_COMPRESSED_RED_GREEN_RGTC2_EXT) // add more if necessary[m
[31m-        {[m
[31m-            ImGui::Text("(Preview not available)");[m
[31m-        }[m
[31m-        else[m
[31m-        {[m
[31m-            ImGui::Image((void*)(std::intptr_t)id, ImVec2((float)width, (float)height), ImVec2(0, 1), ImVec2(1, 0), ImVec4(1, 1, 1, 1), ImVec4(1, 1, 0, 1));[m
[31m-        }[m
[31m-    }[m
[31m-[m
     static int InputTextCallback(ImGuiInputTextCallbackData* data)[m
     {[m
         std::string* str = (std::string*)data->UserData;[m
[36m@@ -509,72 +495,4 @@[m [mnamespace ImGuiEx[m
 [m
         return true;[m
     }[m
[31m-[m
[31m-    class ImGuiPreview[m
[31m-    {[m
[31m-    public:[m
[31m-        ImGuiPreview()[m
[31m-        {[m
[31m-            const int size = 256;[m
[31m-[m
[31m-            _tex = new osg::Texture2D();[m
[31m-            _tex->setTextureSize(size, size);[m
[31m-            _tex->setInternalFormat(GL_RGBA8);[m
[31m-[m
[31m-            _rtt = new osg::Camera();[m
[31m-            _rtt->setRenderOrder(osg::Camera::POST_RENDER);[m
[31m-            _rtt->setViewport(0, 0, 256, 256);[m
[31m-            _rtt->attach(osg::Camera::COLOR_BUFFER, _tex);[m
[31m-            _rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);[m
[31m-            _rtt->setProjectionMatrixAsPerspective(35.0f, 1.0f, 1.0f, 1000.0f);[m
[31m-            _rtt->setClearColor(osg::Vec4(0,0,0,1));[m
[31m-[m
[31m-            _sceneView = new osgUtil::SceneView();[m
[31m-            _sceneView->setCamera(_rtt);[m
[31m-            _sceneView->setViewport(0, 0, size, size);[m
[31m-        }[m
[31m-[m
[31m-        void setNode(osg::Node* node)[m
[31m-        {[m
[31m-            if (!node || dynamic_cast<osg::Camera*>(node)) return;[m
[31m-            osgEarth::Registry::shaderGenerator().run(node);[m
[31m-            auto group = new osg::Group();[m
[31m-            osgEarth::VirtualProgram::getOrCreate(group->getOrCreateStateSet())->setInheritShaders(false);[m
[31m-            group->addChild(node);[m
[31m-            _sceneView->setSceneData(group);[m
[31m-        }[m
[31m-[m
[31m-        void render(osg::RenderInfo& ri)[m
[31m-        {[m
[31m-            auto node = _sceneView->getSceneData();[m
[31m-            if (!node) return;[m
[31m-[m
[31m-            auto bs = node->getBound();[m
[31m-            auto r = bs.radius() * 3.0f;[m
[31m-            auto center = bs.center();[m
[31m-[m
[31m-            angle += osg::PI / 180.0;[m
[31m-            auto x = r * cos(angle);[m
[31m-            auto y = r * sin(angle);[m
[31m-[m
[31m-            auto eye = center + osg::Vec3d(x, y, 0);[m
[31m-            auto up = osg::Vec3d(0, 0, 1);[m
[31m-[m
[31m-            _rtt->setViewMatrixAsLookAt(eye, center, up);[m
[31m-[m
[31m-            _sceneView->setRenderInfo(ri);[m
[31m-            _sceneView->init();[m
[31m-            _sceneView->update();[m
[31m-            _sceneView->cull();[m
[31m-            _sceneView->draw();[m
[31m-[m
[31m-            ImGuiEx::OSGTexture(_tex.get(), ri);[m
[31m-        }[m
[31m-[m
[31m-    private:[m
[31m-        double angle = 0;[m
[31m-        osg::ref_ptr<osgUtil::SceneView> _sceneView;[m
[31m-        osg::ref_ptr<osg::Camera> _rtt;[m
[31m-        osg::ref_ptr<osg::Texture2D> _tex;[m
[31m-    };[m
 }[m
[1mdiff --git a/src/osgEarthImGui/LayersGUI b/src/osgEarthImGui/LayersGUI[m
[1mindex f3689d51a..7bdc06e5e 100644[m
[1m--- a/src/osgEarthImGui/LayersGUI[m
[1m+++ b/src/osgEarthImGui/LayersGUI[m
[36m@@ -115,7 +115,7 @@[m [mnamespace osgEarth[m
         {[m
             AddWMSDialog()[m
             {[m
[31m-                strncpy(url, "http://readymap.org/readymap/tiles", sizeof(url));[m
[32m+[m[32m                strcpy(url, "http://readymap.org/readymap/tiles");[m
                 memset(name, 0, sizeof(name));[m
             }[m
 [m
[36m@@ -219,7 +219,7 @@[m [mnamespace osgEarth[m
                     if (ImGui::IsItemClicked())[m
                     {[m
                         selectedWMSLayer = layer;[m
[31m-                        strncpy(name, selectedWMSLayer->getTitle().c_str(), sizeof(name));[m
[32m+[m[32m                        strcpy(name, selectedWMSLayer->getTitle().c_str());[m
                     }[m
                     ImGui::TableNextColumn();[m
                     ImGui::Text("%s", layer->getTitle().c_str());[m
[1mdiff --git a/src/osgEarthImGui/NetworkMonitorGUI b/src/osgEarthImGui/NetworkMonitorGUI[m
[1mindex 4249e1d57..839f7787c 100644[m
[1m--- a/src/osgEarthImGui/NetworkMonitorGUI[m
[1m+++ b/src/osgEarthImGui/NetworkMonitorGUI[m
[36m@@ -267,7 +267,7 @@[m [mnamespace osgEarth[m
                         ImGui::TableNextColumn();[m
                         ImGui::Text("%s", itr->second.layer.c_str()); ImGui::NextColumn();[m
                         char buf[64];[m
[31m-                        snprintf(buf, sizeof(buf), "%.1lf", itr->second.getDuration());[m
[32m+[m[32m                        sprintf(buf, "%.1lf", itr->second.getDuration());[m
                         ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(buf).x[m
                             - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);[m
 [m
[1mdiff --git a/src/osgEarthImGui/PickerGUI b/src/osgEarthImGui/PickerGUI[m
[1mindex 51501636f..72f0c71d5 100644[m
[1m--- a/src/osgEarthImGui/PickerGUI[m
[1m+++ b/src/osgEarthImGui/PickerGUI[m
[36m@@ -26,7 +26,7 @@[m [mnamespace osgEarth[m
         osg::ref_ptr<osg::Texture2D> _previewTexture;[m
         osg::ref_ptr<osg::StateSet> _previewStateSet;[m
 [m
[31m-        PickerGUI() : ImGuiPanel("Picker") {}[m
[32m+[m[32m        PickerGUI() : ImGuiPanel("Picker") { }[m
 [m
         const char* highlight_shader = R"([m
             #pragma vp_function check_for_highlight, vertex_clip[m
[36m@@ -93,23 +93,10 @@[m [mnamespace osgEarth[m
                                 {[m
                                     // Got a pick:[m
                                     FeatureIndex* index = Registry::objectIndex()->get<FeatureIndex>(id).get();[m
[31m-                                    if (index)[m
[31m-                                    {[m
[31m-                                        Feature* feature = index ? index->getFeature(id) : 0L;[m
[31m-                                        _pickedFeature = feature;[m
[31m-                                        _pickedAnnotation = Registry::objectIndex()->get<AnnotationNode>(id).get();[m
[31m-                                        _highlightUniform->set(id);[m
[31m-                                    }[m
[31m-                                    else[m
[31m-                                    {[m
[31m-                                        MetadataNode* mdn = Registry::objectIndex()->get<MetadataNode>(id).get();[m
[31m-                                        if (mdn)[m
[31m-                                        {[m
[31m-                                            unsigned int index = mdn->getIndexFromObjectID(id);[m
[31m-                                            _pickedFeature = mdn->getFeature(index);[m
[31m-                                            _highlightUniform->set(id);[m
[31m-                                        }[m
[31m-                                    }[m
[32m+[m[32m                                    Feature* feature = index ? index->getFeature(id) : 0L;[m
[32m+[m[32m                                    _pickedFeature = feature;[m
[32m+[m[32m                                    _pickedAnnotation = Registry::objectIndex()->get<AnnotationNode>(id).get();[m
[32m+[m[32m                                    _highlightUniform->set(id);[m
                                 }[m
                                 else[m
                                 {[m
[36m@@ -163,7 +150,7 @@[m [mnamespace osgEarth[m
                         ImGui::Separator();[m
                         ImGui::Text("Picked Feature:");[m
                         ImGuiLTable::Begin("picked feature", ImGuiTableFlags_Borders);[m
[31m-                        ImGuiLTable::Text("FID", "%" PRIu64, _pickedFeature->getFID());[m
[32m+[m[32m                        ImGuiLTable::Text("FID", "%" PRIu64 , _pickedFeature->getFID());[m
                         for (auto& attr : _pickedFeature->getAttrs())[m
                         {[m
                             ImGuiLTable::Text(attr.first.c_str(), "%s", attr.second.getString().c_str());[m
[1mdiff --git a/src/osgEarthImGui/RenderingGUI b/src/osgEarthImGui/RenderingGUI[m
[1mindex b64d8b0f2..7928695c0 100644[m
[1m--- a/src/osgEarthImGui/RenderingGUI[m
[1m+++ b/src/osgEarthImGui/RenderingGUI[m
[36m@@ -11,7 +11,6 @@[m
 #include <osgEarth/ShaderLoader>[m
 #include <chrono>[m
 #include <list>[m
[31m-#include <unordered_map>[m
 [m
 namespace {[m
     const char* render_view_normals = R"([m
[36m@@ -359,8 +358,6 @@[m [mnamespace osgEarth[m
 [m
     class NVGLInspectorGUI : public ImGuiPanel[m
     {[m
[31m-        GLint _expanded = -1;[m
[31m-[m
     public:[m
         NVGLInspectorGUI() : ImGuiPanel("NVGL Inspector")[m
         {[m
[36m@@ -415,7 +412,7 @@[m [mnamespace osgEarth[m
                                 total += obj->size();[m
 [m
                             char header[128];[m
[31m-                            snprintf(header, sizeof(header), "%s (%d @ %.1lf MB)###%s", cat.first.c_str(), (int)cat.second.size(), (double)total / 1048576., cat.first.c_str());[m
[32m+[m[32m                            sprintf(header, "%s (%d @ %.1lf MB)###%s", cat.first.c_str(), (int)cat.second.size(), (double)total / 1048576., cat.first.c_str());[m
 [m
                             if (sort_by_size)[m
                             {[m
[36m@@ -428,34 +425,18 @@[m [mnamespace osgEarth[m
                             {[m
                                 ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Borders;[m
                                 flags &= ~ImGuiTableFlags_BordersOuter;[m
[31m-                                if (ImGui::BeginTable("globj", 2, flags))[m
[32m+[m[32m                                if (ImGui::BeginTable("globj", 4, flags))[m
                                 {[m
                                     for (auto& obj : cat.second)[m
                                     {[m
[31m-                                        ImGui::PushID(obj->name());[m
[31m-[m
                                         ImGui::TableNextColumn();[m
                                         ImGui::Text("%6.1lf KB", (double)obj->size() / (double)1024.);[m
[31m-[m
                                         ImGui::TableNextColumn();[m
[31m-                                        if (auto tex = dynamic_cast<GLTexture*>(obj.get()))[m
[31m-                                        {[m
[31m-                                            bool isExpanded = false;[m
[31m-                                            if (ImGui::Selectable(obj->uid().c_str(), &isExpanded))[m
[31m-                                                _expanded = _expanded == obj->name() ? -1 : obj->name();[m
[31m-                                            if (_expanded == obj->name())[m
[31m-                                            {[m
[31m-                                                ImGui::Text("%d x %d", tex->profile()._width, tex->profile()._height);[m
[31m-                                                ImGui::Text("%s", ImageUtils::getInternalFormatString(tex->profile()._internalFormat).c_str());[m
[31m-                                                ImGuiEx::Texture(tex->name(), tex->profile()._width, tex->profile()._height, tex->profile()._internalFormat);[m
[31m-                                            }[m
[31m-                                        }[m
[31m-                                        else[m
[31m-                                        {[m
[31m-                                            ImGui::Text("%s", obj->uid().c_str());[m
[31m-                                        }[m
[31m-[m
[31m-                                        ImGui::PopID();[m
[32m+[m[32m                                        ImGui::Text("%d", obj->recycles());[m
[32m+[m[32m                                        ImGui::TableNextColumn();[m
[32m+[m[32m                                        ImGui::Text("%s", obj->uid().c_str());[m
[32m+[m[32m                                        ImGui::TableNextColumn();[m
[32m+[m[32m                                        ImGui::Text("(%d)", obj->name());[m
                                     }[m
                                     ImGui::EndTable();[m
                                 }[m
[1mdiff --git a/src/osgEarthImGui/ResourceLibraryGUI b/src/osgEarthImGui/ResourceLibraryGUI[m
[1mdeleted file mode 100644[m
[1mindex cdc8c28c4..000000000[m
[1m--- a/src/osgEarthImGui/ResourceLibraryGUI[m
[1m+++ /dev/null[m
[36m@@ -1,727 +0,0 @@[m
[31m-/* osgEarth[m
[31m- * Copyright 2025 Pelican Mapping[m
[31m- * MIT License[m
[31m- */[m
[31m-#pragma once[m
[31m-[m
[31m-#include <osgEarthImGui/ImGuiPanel>[m
[31m-#include <osgEarth/ResourceLibrary>[m
[31m-#include <osgEarth/ModelResource>[m
[31m-#include <osgEarth/IconResource>[m
[31m-#include <osgEarth/XmlUtils>[m
[31m-#include <osgEarth/StringUtils>[m
[31m-#include <osgDB/ReadFile>[m
[31m-#include <osgDB/Registry>[m
[31m-#include <osgDB/FileUtils>[m
[31m-#include <osgDB/FileNameUtils>[m
[31m-#include <fstream>[m
[31m-[m
[31m-#if defined(__has_include)[m
[31m-#if __has_include(<third_party/portable-file-dialogs/portable-file-dialogs.h>)[m
[31m-#include <third_party/portable-file-dialogs/portable-file-dialogs.h>[m
[31m-#define HAS_PFD[m
[31m-#endif[m
[31m-#endif[m
[31m-[m
[31m-namespace osgEarth[m
[31m-{[m
[31m-    using namespace osgEarth::Util;[m
[31m-[m
[31m-    struct ResourceLibraryGUI : public ImGuiPanel[m
[31m-    {[m
[31m-        ResourceLibraryGUI() :[m
[31m-            ImGuiPanel("Resource Library")[m
[31m-        {[m
[31m-            memset(_nameFilter, 0, sizeof(_nameFilter));[m
[31m-            memset(_tagFilter, 0, sizeof(_tagFilter));[m
[31m-        }[m
[31m-[m
[31m-        void draw(osg::RenderInfo& ri) override[m
[31m-        {[m
[31m-            if (!isVisible())[m
[31m-                return;[m
[31m-[m
[31m-            if (!ImGui::Begin(name(), visible()))[m
[31m-            {[m
[31m-                ImGui::End();[m
[31m-                return;[m
[31m-            }[m
[31m-[m
[31m-            // File selection UI[m
[31m-#ifdef HAS_PFD[m
[31m-            if (ImGui::Button("Open Library File..."))[m
[31m-            {[m
[31m-                auto f = pfd::open_file("Choose Resource Library", pfd::path::home(),[m
[31m-                    { "XML Files", "*.xml", "All Files", "*" });[m
[31m-[m
[31m-                if (!f.result().empty())[m
[31m-                {[m
[31m-                    _libraryPath = f.result()[0];[m
[31m-                    loadLibrary();[m
[31m-                }[m
[31m-            }[m
[31m-#endif[m
[31m-[m
[31m-            if (!_libraryPath.empty())[m
[31m-            {[m
[31m-                ImGui::SameLine();[m
[31m-                ImGui::Text("Library: %s", _library.valid() ? _library->getName().c_str() : _libraryPath.c_str());[m
[31m-            }[m
[31m-[m
[31m-            if (_library.valid())[m
[31m-            {[m
[31m-                // Display statistics[m
[31m-                SkinResourceVector skins;[m
[31m-                _library->getSkins(skins);[m
[31m-[m
[31m-                ModelResourceVector models;[m
[31m-                _library->getModels(models);[m
[31m-[m
[31m-                ImGui::Text("Resources: %d skins, %d models", (int)skins.size(), (int)models.size());[m
[31m-                ImGui::Separator();[m
[31m-[m
[31m-                // Filter controls[m
[31m-                ImGui::Text("Name:");[m
[31m-                ImGui::SameLine();[m
[31m-                ImGui::SetNextItemWidth(200);[m
[31m-                ImGui::InputText("##name", _nameFilter, 256);[m
[31m-[m
[31m-                ImGui::SameLine();[m
[31m-                ImGui::Text("Tags:");[m
[31m-                ImGui::SameLine();[m
[31m-                ImGui::SetNextItemWidth(300);[m
[31m-                ImGui::InputText("##tags", _tagFilter, 256);[m
[31m-[m
[31m-                ImGui::Separator();[m
[31m-[m
[31m-                // Tabs for different resource types[m
[31m-                if (ImGui::BeginTabBar("ResourceTabs"))[m
[31m-                {[m
[31m-                    // Skins tab with preview[m
[31m-                    if (ImGui::BeginTabItem("Skins"))[m
[31m-                    {[m
[31m-                        // Two-column layout: skins table on left, preview on right[m
[31m-                        if (ImGui::BeginTable("SkinsLayout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_Borders))[m
[31m-                        {[m
[31m-                            // Left column: skins table[m
[31m-                            ImGui::TableNextColumn();[m
[31m-                            ImGui::BeginChild("SkinsList");[m
[31m-                            drawSkinsTable(skins, ri);[m
[31m-                            ImGui::EndChild();[m
[31m-[m
[31m-                            // Right column: preview[m
[31m-                            ImGui::TableNextColumn();[m
[31m-                            drawPreview(ri);[m
[31m-[m
[31m-                            ImGui::EndTable();[m
[31m-                        }[m
[31m-                        ImGui::EndTabItem();[m
[31m-                    }[m
[31m-[m
[31m-                    // Instances tab with preview[m
[31m-                    if (ImGui::BeginTabItem("Instances"))[m
[31m-                    {[m
[31m-                        // Two-column layout: instances table on left, properties on right[m
[31m-                        if (ImGui::BeginTable("InstancesLayout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_Borders))[m
[31m-                        {[m
[31m-                            // Left column: instances table[m
[31m-                            ImGui::TableNextColumn();[m
[31m-                            ImGui::BeginChild("InstancesList");[m
[31m-                            drawInstancesTable(models);[m
[31m-                            ImGui::EndChild();[m
[31m-[m
[31m-                            // Right column: properties[m
[31m-                            ImGui::TableNextColumn();[m
[31m-                            drawInstanceProperties();[m
[31m-[m
[31m-                            ImGui::EndTable();[m
[31m-                        }[m
[31m-                        ImGui::EndTabItem();[m
[31m-                    }[m
[31m-[m
[31m-                    ImGui::EndTabBar();[m
[31m-                }[m
[31m-            }[m
[31m-            else if (!_libraryPath.empty())[m
[31m-            {[m
[31m-                ImGui::TextColored(ImVec4(1, 0.5, 0, 1), "Failed to load library");[m
[31m-            }[m
[31m-[m
[31m-            ImGui::End();[m
[31m-        }[m
[31m-[m
[31m-        void drawSkinsTable(const SkinResourceVector& skins, osg::RenderInfo& ri)[m
[31m-        {[m
[31m-            // Parse tags filter[m
[31m-            TagVector filterTags;[m
[31m-            if (strlen(_tagFilter) > 0)[m
[31m-            {[m
[31m-                std::string tagsStr(_tagFilter);[m
[31m-                filterTags = StringTokenizer()[m
[31m-                    .delim(" ")[m
[31m-                    .standardQuotes()[m
[31m-                    .keepEmpties(false)[m
[31m-                    .tokenize(tagsStr);[m
[31m-            }[m
[31m-[m
[31m-            static ImGuiTableFlags flags =[m
[31m-                ImGuiTableFlags_ScrollY |[m
[31m-                ImGuiTableFlags_RowBg |[m
[31m-                ImGuiTableFlags_BordersOuter |[m
[31m-                ImGuiTableFlags_BordersV |[m
[31m-                ImGuiTableFlags_Resizable |[m
[31m-                ImGuiTableFlags_SizingFixedFit;[m
[31m-[m
[31m-            // Use available space for the skins table[m
[31m-            ImVec2 outer_size = ImVec2(0.0f, 0.0f);[m
[31m-[m
[31m-            if (ImGui::BeginTable("SkinsTable", 8, flags, outer_size))[m
[31m-            {[m
[31m-                ImGui::TableSetupScrollFreeze(0, 1);[m
[31m-                ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthFixed, 200.0f);[m
[31m-                ImGui::TableSetupColumn("Image URI", ImGuiTableColumnFlags_WidthStretch);[m
[31m-                ImGui::TableSetupColumn("Width", ImGuiTableColumnFlags_WidthFixed, 60.0f);[m
[31m-                ImGui::TableSetupColumn("Height", ImGuiTableColumnFlags_WidthFixed, 60.0f);[m
[31m-                ImGui::TableSetupColumn("Min Height", ImGuiTableColumnFlags_WidthFixed, 80.0f);[m
[31m-                ImGui::TableSetupColumn("Max Height", ImGuiTableColumnFlags_WidthFixed, 80.0f);[m
[31m-                ImGui::TableSetupColumn("Tiled", ImGuiTableColumnFlags_WidthFixed, 50.0f);[m
[31m-                ImGui::TableSetupColumn("File Size", ImGuiTableColumnFlags_WidthFixed, 90.0f);[m
[31m-                ImGui::TableHeadersRow();[m
[31m-[m
[31m-                for (auto& skin : skins)[m
[31m-                {[m
[31m-                    // Apply name filter[m
[31m-                    if (strlen(_nameFilter) > 0)[m
[31m-                    {[m
[31m-                        std::string skinName = skin->name();[m
[31m-                        if (skinName.find(_nameFilter) == std::string::npos)[m
[31m-                            continue;[m
[31m-                    }[m
[31m-[m
[31m-                    // Apply tags filter[m
[31m-                    if (!filterTags.empty() && !skin->containsTags(filterTags))[m
[31m-                        continue;[m
[31m-                    ImGui::PushID(skin.get());[m
[31m-                    ImGui::TableNextRow();[m
[31m-[m
[31m-                    // Make the entire row selectable[m
[31m-                    ImGui::TableSetColumnIndex(0);[m
[31m-                    bool selected = (_selectedSkin.get() == skin.get());[m
[31m-                    ImGuiSelectableFlags selectable_flags = ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowItemOverlap;[m
[31m-                    if (ImGui::Selectable("##row", selected, selectable_flags, ImVec2(0, 0)))[m
[31m-                    {[m
[31m-                        setSelectedSkin(skin.get(), ri);[m
[31m-                    }[m
[31m-[m
[31m-                    // Draw cell contents[m
[31m-                    ImGui::SameLine();[m
[31m-                    ImGui::Text("%s", skin->name().c_str());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(1);[m
[31m-                    std::string imageURI;[m
[31m-                    if (skin->imageURI().isSet())[m
[31m-                        imageURI = skin->imageURI()->full();[m
[31m-[m
[31m-                    //else if (skin->material().isSet() && skin->material()->colorURI().isSet())[m
[31m-                    //    imageURI = skin->material()->colorURI()->full();[m
[31m-                    ImGui::Text("%s", imageURI.c_str());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(2);[m
[31m-                    ImGui::Text("%.1f", skin->imageWidth().get());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(3);[m
[31m-                    ImGui::Text("%.1f", skin->imageHeight().get());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(4);[m
[31m-                    ImGui::Text("%.1f", skin->minObjectHeight().get());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(5);[m
[31m-                    if (skin->maxObjectHeight().get() >= FLT_MAX)[m
[31m-                        ImGui::Text("inf");[m
[31m-                    else[m
[31m-                        ImGui::Text("%.1f", skin->maxObjectHeight().get());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(6);[m
[31m-                    ImGui::Text("%s", skin->isTiled().get() ? "Yes" : "No");[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(7);[m
[31m-                    if (!imageURI.empty())[m
[31m-                    {[m
[31m-                        std::string filePath = resolveFilePath(imageURI);[m
[31m-                        std::string sizeStr;[m
[31m-                        bool fileExists = getFileSize(filePath, sizeStr);[m
[31m-                        if (fileExists)[m
[31m-                            ImGui::Text("%s", sizeStr.c_str());[m
[31m-                        else[m
[31m-                            ImGui::TextColored(ImVec4(1, 0, 0, 1), "File not found");[m
[31m-                    }[m
[31m-[m
[31m-                    ImGui::PopID();[m
[31m-                }[m
[31m-[m
[31m-                ImGui::EndTable();[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        void drawInstancesTable(const ModelResourceVector& models)[m
[31m-        {[m
[31m-            // Parse tags filter[m
[31m-            TagVector filterTags;[m
[31m-            if (strlen(_tagFilter) > 0)[m
[31m-            {[m
[31m-                std::string tagsStr(_tagFilter);[m
[31m-                filterTags = StringTokenizer()[m
[31m-                    .delim(" ")[m
[31m-                    .standardQuotes()[m
[31m-                    .keepEmpties(false)[m
[31m-                    .tokenize(tagsStr);[m
[31m-            }[m
[31m-[m
[31m-            static ImGuiTableFlags flags =[m
[31m-                ImGuiTableFlags_ScrollY |[m
[31m-                ImGuiTableFlags_RowBg |[m
[31m-                ImGuiTableFlags_BordersOuter |[m
[31m-                ImGuiTableFlags_BordersV |[m
[31m-                ImGuiTableFlags_Resizable |[m
[31m-                ImGuiTableFlags_SizingFixedFit;[m
[31m-[m
[31m-            // Use available space for the instances table[m
[31m-            ImVec2 outer_size = ImVec2(0.0f, 0.0f);[m
[31m-[m
[31m-            if (ImGui::BeginTable("InstancesTable", 6, flags, outer_size))[m
[31m-            {[m
[31m-                ImGui::TableSetupScrollFreeze(0, 1);[m
[31m-                ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthFixed, 200.0f);[m
[31m-                ImGui::TableSetupColumn("URI", ImGuiTableColumnFlags_WidthStretch);[m
[31m-                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 80.0f);[m
[31m-                ImGui::TableSetupColumn("Scale XY", ImGuiTableColumnFlags_WidthFixed, 80.0f);[m
[31m-                ImGui::TableSetupColumn("Scale Z", ImGuiTableColumnFlags_WidthFixed, 80.0f);[m
[31m-                ImGui::TableSetupColumn("File Size", ImGuiTableColumnFlags_WidthFixed, 90.0f);[m
[31m-                ImGui::TableHeadersRow();[m
[31m-[m
[31m-                for (auto& model : models)[m
[31m-                {[m
[31m-                    // Apply name filter[m
[31m-                    if (strlen(_nameFilter) > 0)[m
[31m-                    {[m
[31m-                        std::string modelName = model->name();[m
[31m-                        if (modelName.find(_nameFilter) == std::string::npos)[m
[31m-                            continue;[m
[31m-                    }[m
[31m-[m
[31m-                    // Apply tags filter[m
[31m-                    if (!filterTags.empty() && !model->containsTags(filterTags))[m
[31m-                        continue;[m
[31m-                    ImGui::PushID(model.get());[m
[31m-                    ImGui::TableNextRow();[m
[31m-[m
[31m-                    // Make the entire row selectable[m
[31m-                    ImGui::TableSetColumnIndex(0);[m
[31m-                    bool selected = (_selectedModel.get() == model.get());[m
[31m-                    ImGuiSelectableFlags selectable_flags = ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowItemOverlap;[m
[31m-                    if (ImGui::Selectable("##row", selected, selectable_flags, ImVec2(0, 0)))[m
[31m-                    {[m
[31m-                        setSelectedModel(model.get());[m
[31m-                    }[m
[31m-[m
[31m-                    // Draw cell contents[m
[31m-                    ImGui::SameLine();[m
[31m-                    ImGui::Text("%s", model->name().c_str());[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(1);[m
[31m-                    if (model->uri().isSet())[m
[31m-                        ImGui::Text("%s", model->uri()->full().c_str());[m
[31m-                    else[m
[31m-                        ImGui::Text("(none)");[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(2);[m
[31m-                    ImGui::Text("%s", model->is2D() ? "2D" : "3D");[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(3);[m
[31m-                    if (model->canScaleToFitXY().isSet())[m
[31m-                        ImGui::Text("%s", model->canScaleToFitXY().get() ? "Yes" : "No");[m
[31m-                    else[m
[31m-                        ImGui::Text("-");[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(4);[m
[31m-                    if (model->canScaleToFitZ().isSet())[m
[31m-                        ImGui::Text("%s", model->canScaleToFitZ().get() ? "Yes" : "No");[m
[31m-                    else[m
[31m-                        ImGui::Text("-");[m
[31m-[m
[31m-                    ImGui::TableSetColumnIndex(5);[m
[31m-                    if (model->uri().isSet())[m
[31m-                    {[m
[31m-                        std::string modelURI = model->uri()->full();[m
[31m-                        std::string filePath = resolveFilePath(modelURI);[m
[31m-                        std::string sizeStr;[m
[31m-                        bool fileExists = getFileSize(filePath, sizeStr);[m
[31m-                        if (fileExists)[m
[31m-                            ImGui::Text("%s", sizeStr.c_str());[m
[31m-                        else[m
[31m-                            ImGui::TextColored(ImVec4(1, 0, 0, 1), "File not found");[m
[31m-                    }[m
[31m-[m
[31m-                    ImGui::PopID();[m
[31m-                }[m
[31m-[m
[31m-                ImGui::EndTable();[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        void loadLibrary()[m
[31m-        {[m
[31m-            _library = nullptr;[m
[31m-[m
[31m-            if (_libraryPath.empty())[m
[31m-                return;[m
[31m-[m
[31m-            try[m
[31m-            {[m
[31m-                URI uri(_libraryPath);[m
[31m-                osg::ref_ptr<XmlDocument> xml = XmlDocument::load(uri);[m
[31m-[m
[31m-                if (xml.valid())[m
[31m-                {[m
[31m-                    Config conf = xml->getConfig();[m
[31m-[m
[31m-                    // Handle both "resources" root and documents with "resources" child[m
[31m-                    if (conf.key() == "resources")[m
[31m-                    {[m
[31m-                        _library = new ResourceLibrary(conf);[m
[31m-                    }[m
[31m-                    else[m
[31m-                    {[m
[31m-                        const Config& child = conf.child("resources");[m
[31m-                        if (!child.empty())[m
[31m-                            _library = new ResourceLibrary(child);[m
[31m-                    }[m
[31m-[m
[31m-                    if (_library.valid())[m
[31m-                    {[m
[31m-                        _library->initialize(nullptr);[m
[31m-                        dirtySettings();[m
[31m-                    }[m
[31m-                }[m
[31m-            }[m
[31m-            catch (...)[m
[31m-            {[m
[31m-                _library = nullptr;[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        void setSelectedSkin(SkinResource* skin, osg::RenderInfo& ri)[m
[31m-        {[m
[31m-            if (_selectedSkin.get() == skin)[m
[31m-                return;[m
[31m-[m
[31m-            _selectedSkin = skin;[m
[31m-[m
[31m-            // Clear previous texture/image[m
[31m-            if (_currentTexture.valid())[m
[31m-            {[m
[31m-                _currentTexture->releaseGLObjects();[m
[31m-                _currentTexture = nullptr;[m
[31m-            }[m
[31m-            _currentImage = nullptr;[m
[31m-[m
[31m-            if (!skin)[m
[31m-                return;[m
[31m-[m
[31m-            // Try to load the image from the skin[m
[31m-            try[m
[31m-            {[m
[31m-                // Get the base path from the library file[m
[31m-                std::string basePath = osgDB::getFilePath(_libraryPath);[m
[31m-[m
[31m-                // Try imageURI first[m
[31m-                if (skin->imageURI().isSet())[m
[31m-                {[m
[31m-                    std::string imageFile = skin->imageURI()->full();[m
[31m-[m
[31m-                    // If it's a relative path, combine it with the library's base path[m
[31m-                    if (!osgDB::isAbsolutePath(imageFile))[m
[31m-                    {[m
[31m-                        imageFile = osgDB::concatPaths(basePath, imageFile);[m
[31m-                    }[m
[31m-[m
[31m-                    _currentImage = osgDB::readImageFile(imageFile);[m
[31m-                }[m
[31m-[m
[31m-                // Create texture for preview[m
[31m-                if (_currentImage.valid())[m
[31m-                {[m
[31m-                    _currentTexture = new osg::Texture2D(_currentImage.get());[m
[31m-                    _currentTexture->setResizeNonPowerOfTwoHint(false);[m
[31m-                }[m
[31m-            }[m
[31m-            catch (...)[m
[31m-            {[m
[31m-                _currentImage = nullptr;[m
[31m-                _currentTexture = nullptr;[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        void drawPreview(osg::RenderInfo& ri)[m
[31m-        {[m
[31m-            ImGui::BeginChild("Preview");[m
[31m-[m
[31m-            if (!_selectedSkin.valid())[m
[31m-            {[m
[31m-                ImGui::Text("Select a skin to preview");[m
[31m-            }[m
[31m-            else[m
[31m-            {[m
[31m-                SkinResource* skin = _selectedSkin.get();[m
[31m-[m
[31m-                ImGui::TextColored(ImVec4(1, 1, 0, 1), "Skin Properties");[m
[31m-                ImGui::Separator();[m
[31m-[m
[31m-                ImGui::Text("Name: %s", skin->name().c_str());[m
[31m-[m
[31m-                std::string imageURI;[m
[31m-                if (skin->imageURI().isSet())[m
[31m-                    imageURI = skin->imageURI()->full();[m
[31m-                ImGui::Text("Image URI: %s", imageURI.c_str());[m
[31m-[m
[31m-                // Display file size[m
[31m-                if (!imageURI.empty())[m
[31m-                {[m
[31m-                    std::string filePath = resolveFilePath(imageURI);[m
[31m-                    std::string sizeStr;[m
[31m-                    bool fileExists = getFileSize(filePath, sizeStr);[m
[31m-                    if (fileExists)[m
[31m-                        ImGui::Text("File Size: %s", sizeStr.c_str());[m
[31m-                    else[m
[31m-                        ImGui::TextColored(ImVec4(1, 0, 0, 1), "File Size: File not found");[m
[31m-                }[m
[31m-[m
[31m-                ImGui::Text("Dimensions (meters): %.1f x %.1f", skin->imageWidth().get(), skin->imageHeight().get());[m
[31m-                ImGui::Text("Min Object Height: %.1f m", skin->minObjectHeight().get());[m
[31m-[m
[31m-                if (skin->maxObjectHeight().get() >= FLT_MAX)[m
[31m-                    ImGui::Text("Max Object Height: inf");[m
[31m-                else[m
[31m-                    ImGui::Text("Max Object Height: %.1f m", skin->maxObjectHeight().get());[m
[31m-[m
[31m-                ImGui::Text("Tiled: %s", skin->isTiled().get() ? "Yes" : "No");[m
[31m-[m
[31m-                // Display tags if available[m
[31m-                if (!skin->tagString().empty())[m
[31m-                {[m
[31m-                    ImGui::Text("Tags: %s", skin->tagString().c_str());[m
[31m-                }[m
[31m-[m
[31m-                ImGui::Separator();[m
[31m-[m
[31m-                // Display texture preview and image properties[m
[31m-                if (_currentTexture.valid() && _currentImage.valid())[m
[31m-                {[m
[31m-                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Texture Preview");[m
[31m-                    ImGui::Separator();[m
[31m-[m
[31m-                    ImGuiEx::OSGTexture(_currentTexture.get(), ri, 250, 0);[m
[31m-[m
[31m-                    ImGui::Separator();[m
[31m-                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Image Properties");[m
[31m-                    ImGui::Separator();[m
[31m-[m
[31m-                    ImGui::Text("File: %s", _currentImage->getFileName().c_str());[m
[31m-                    ImGui::Text("Pixel Dimensions: %d x %d", _currentImage->s(), _currentImage->t());[m
[31m-                    ImGui::Text("Compressed: %s", _currentImage->isCompressed() ? "Yes" : "No");[m
[31m-                    ImGui::Text("Data Type: %s", getGLString(_currentImage->getDataType()).c_str());[m
[31m-                    ImGui::Text("Texture Format: %s", getGLString(_currentImage->getInternalTextureFormat()).c_str());[m
[31m-                    ImGui::Text("Mipmap Levels: %d", _currentImage->getNumMipmapLevels());[m
[31m-                    ImGui::Text("Pixel Format: %s", getGLString(_currentImage->getPixelFormat()).c_str());[m
[31m-                }[m
[31m-                else if (!_currentTexture.valid())[m
[31m-                {[m
[31m-                    ImGui::Separator();[m
[31m-                    ImGui::TextColored(ImVec4(1, 0.5, 0, 1), "Failed to load texture");[m
[31m-                    ImGui::Text("Check that the image file exists and path is correct");[m
[31m-                }[m
[31m-            }[m
[31m-[m
[31m-            ImGui::EndChild();[m
[31m-        }[m
[31m-[m
[31m-        void setSelectedModel(ModelResource* model)[m
[31m-        {[m
[31m-            if (_selectedModel.get() == model)[m
[31m-                return;[m
[31m-[m
[31m-            _selectedModel = model;[m
[31m-        }[m
[31m-[m
[31m-        void drawInstanceProperties()[m
[31m-        {[m
[31m-            ImGui::BeginChild("InstanceProperties");[m
[31m-[m
[31m-            if (!_selectedModel.valid())[m
[31m-            {[m
[31m-                ImGui::Text("Select an instance to view properties");[m
[31m-            }[m
[31m-            else[m
[31m-            {[m
[31m-                ModelResource* model = _selectedModel.get();[m
[31m-[m
[31m-                ImGui::TextColored(ImVec4(1, 1, 0, 1), "Instance Properties");[m
[31m-                ImGui::Separator();[m
[31m-[m
[31m-                ImGui::Text("Name: %s", model->name().c_str());[m
[31m-[m
[31m-                if (model->uri().isSet())[m
[31m-                {[m
[31m-                    std::string modelURI = model->uri()->full();[m
[31m-                    ImGui::Text("URI: %s", modelURI.c_str());[m
[31m-[m
[31m-                    // Display file size[m
[31m-                    std::string filePath = resolveFilePath(modelURI);[m
[31m-                    std::string sizeStr;[m
[31m-                    bool fileExists = getFileSize(filePath, sizeStr);[m
[31m-                    if (fileExists)[m
[31m-                        ImGui::Text("File Size: %s", sizeStr.c_str());[m
[31m-                    else[m
[31m-                        ImGui::TextColored(ImVec4(1, 0, 0, 1), "File Size: File not found");[m
[31m-                }[m
[31m-                else[m
[31m-                {[m
[31m-                    ImGui::Text("URI: (none)");[m
[31m-                }[m
[31m-[m
[31m-                ImGui::Text("Type: %s", model->is2D() ? "2D" : "3D");[m
[31m-[m
[31m-                if (model->canScaleToFitXY().isSet())[m
[31m-                    ImGui::Text("Scale to Fit XY: %s", model->canScaleToFitXY().get() ? "Yes" : "No");[m
[31m-                else[m
[31m-                    ImGui::Text("Scale to Fit XY: Not specified");[m
[31m-[m
[31m-                if (model->canScaleToFitZ().isSet())[m
[31m-                    ImGui::Text("Scale to Fit Z: %s", model->canScaleToFitZ().get() ? "Yes" : "No");[m
[31m-                else[m
[31m-                    ImGui::Text("Scale to Fit Z: Not specified");[m
[31m-[m
[31m-                // Display tags if available[m
[31m-                if (!model->tagString().empty())[m
[31m-                {[m
[31m-                    ImGui::Separator();[m
[31m-                    ImGui::Text("Tags: %s", model->tagString().c_str());[m
[31m-                }[m
[31m-            }[m
[31m-[m
[31m-            ImGui::EndChild();[m
[31m-        }[m
[31m-[m
[31m-        const std::string& getGLString(int value)[m
[31m-        {[m
[31m-            return osgDB::Registry::instance()->getObjectWrapperManager()->getString("GL", value);[m
[31m-        }[m
[31m-[m
[31m-        std::string resolveFilePath(const std::string& uri) const[m
[31m-        {[m
[31m-            if (uri.empty())[m
[31m-                return uri;[m
[31m-[m
[31m-            // If it's already an absolute path, return as-is[m
[31m-            if (osgDB::isAbsolutePath(uri))[m
[31m-                return uri;[m
[31m-[m
[31m-            // If we have a library path, resolve relative to it[m
[31m-            if (!_libraryPath.empty())[m
[31m-            {[m
[31m-                std::string basePath = osgDB::getFilePath(_libraryPath);[m
[31m-                return osgDB::concatPaths(basePath, uri);[m
[31m-            }[m
[31m-[m
[31m-            return uri;[m
[31m-        }[m
[31m-[m
[31m-        bool getFileSize(const std::string& filePath, std::string& outSizeStr) const[m
[31m-        {[m
[31m-            if (filePath.empty())[m
[31m-                return false;[m
[31m-[m
[31m-            if (!osgDB::fileExists(filePath))[m
[31m-                return false;[m
[31m-[m
[31m-            // Get file size[m
[31m-            std::ifstream file(filePath, std::ifstream::ate | std::ifstream::binary);[m
[31m-            if (!file.is_open())[m
[31m-                return false;[m
[31m-[m
[31m-            std::streampos fileSize = file.tellg();[m
[31m-            file.close();[m
[31m-[m
[31m-            // Format file size with appropriate units[m
[31m-            double size = static_cast<double>(fileSize);[m
[31m-            if (size < 1024.0)[m
[31m-            {[m
[31m-                outSizeStr = std::to_string(static_cast<int>(size)) + " B";[m
[31m-            }[m
[31m-            else if (size < 1024.0 * 1024.0)[m
[31m-            {[m
[31m-                size /= 1024.0;[m
[31m-                char buf[64];[m
[31m-                snprintf(buf, sizeof(buf), "%.1f KB", size);[m
[31m-                outSizeStr = buf;[m
[31m-            }[m
[31m-            else if (size < 1024.0 * 1024.0 * 1024.0)[m
[31m-            {[m
[31m-                size /= (1024.0 * 1024.0);[m
[31m-                char buf[64];[m
[31m-                snprintf(buf, sizeof(buf), "%.1f MB", size);[m
[31m-                outSizeStr = buf;[m
[31m-            }[m
[31m-            else[m
[31m-            {[m
[31m-                size /= (1024.0 * 1024.0 * 1024.0);[m
[31m-                char buf[64];[m
[31m-                snprintf(buf, sizeof(buf), "%.1f GB", size);[m
[31m-                outSizeStr = buf;[m
[31m-            }[m
[31m-[m
[31m-            return true;[m
[31m-        }[m
[31m-[m
[31m-        //! Load settings from .ini file[m
[31m-        void load(const Config& conf) override[m
[31m-        {[m
[31m-            conf.get("LibraryPath", _libraryPath);[m
[31m-            if (!_libraryPath.empty())[m
[31m-            {[m
[31m-                loadLibrary();[m
[31m-            }[m
[31m-[m
[31m-            std::string name;[m
[31m-            if (conf.get("Name", name))[m
[31m-            {[m
[31m-                strncpy(_nameFilter, name.c_str(), sizeof(_nameFilter) - 1);[m
[31m-            }[m
[31m-            std::string tags;[m
[31m-            if (conf.get("Tags", tags))[m
[31m-            {[m
[31m-                strncpy(_tagFilter, tags.c_str(), sizeof(_tagFilter) - 1);[m
[31m-            }[m
[31m-        }[m
[31m-[m
[31m-        //! Save settings to .ini file[m
[31m-        void save(Config& conf) override[m
[31m-        {[m
[31m-            if (!_libraryPath.empty())[m
[31m-                conf.set("LibraryPath", _libraryPath);[m
[31m-            conf.set("Name", std::string(_nameFilter));[m
[31m-            conf.set("Tags", std::string(_tagFilter));[m
[31m-        }[m
[31m-[m
[31m-    private:[m
[31m-        osg::ref_ptr<ResourceLibrary> _library;[m
[31m-        std::string _libraryPath;[m
[31m-        osg::observer_ptr<SkinResource> _selectedSkin;[m
[31m-        osg::ref_ptr<osg::Image> _currentImage;[m
[31m-        osg::ref_ptr<osg::Texture2D> _currentTexture;[m
[31m-        osg::observer_ptr<ModelResource> _selectedModel;[m
[31m-        char _nameFilter[256];[m
[31m-        char _tagFilter[256];[m
[31m-    };[m
[31m-}[m
[31m-[m
[1mdiff --git a/src/osgEarthImGui/SceneGraphGUI b/src/osgEarthImGui/SceneGraphGUI[m
[1mindex 2aa4fe8ef..6ab4812b6 100644[m
[1m--- a/src/osgEarthImGui/SceneGraphGUI[m
[1m+++ b/src/osgEarthImGui/SceneGraphGUI[m
[36m@@ -1069,14 +1069,6 @@[m [mnamespace osgEarth[m
                 ImGui::TreePop();[m
             }[m
 [m
[31m-            if (ImGui::TreeNode("Preview"))[m
[31m-            {[m
[31m-                static ImGuiEx::ImGuiPreview preview;[m
[31m-                preview.setNode(getSelectedNode());[m
[31m-                preview.render(ri);[m
[31m-                ImGui::TreePop();[m
[31m-            }[m
[31m-[m
             osg::MatrixTransform* matrixTransform = dynamic_cast<osg::MatrixTransform*>(node);[m
             if (matrixTransform)[m
             {[m
[1mdiff --git a/src/osgEarthImGui/SystemGUI b/src/osgEarthImGui/SystemGUI[m
[1mindex 11e2f327b..04c8c2106 100644[m
[1m--- a/src/osgEarthImGui/SystemGUI[m
[1m+++ b/src/osgEarthImGui/SystemGUI[m
[36m@@ -138,11 +138,11 @@[m [mnamespace osgEarth[m
                     frame_times[f] = now - t_previous;[m
                     t_previous = now;[m
                     auto avg_timing_ms = 1e-6 * (float)get_average_timing_ns(&frame_times, 120, f);[m
[31m-                    snprintf( buf, sizeof(buf), "%.2f ms / %d fps", avg_timing_ms, (int)std::ceil(1000.0 / avg_timing_ms));[m
[32m+[m[32m                    sprintf(buf, "%.2f ms / %d fps", avg_timing_ms, (int)std::ceil(1000.0 / avg_timing_ms));[m
                     ImGuiLTable::PlotLines("Frame", get_timing_ms, &frame_times, frame_count, frame_num, buf, 0.0f, 32.0f);[m
 [m
                     total_jobs[f] = jobs::get_metrics()->total();[m
[31m-                    snprintf(buf, sizeof(buf), "%d", total_jobs[f]);[m
[32m+[m[32m                    sprintf(buf, "%d", total_jobs[f]);[m
                     ImGuiLTable::PlotLines("Jobs", get_counts, &total_jobs, frame_count, frame_num, buf, 0u, 100u);[m
 [m
                     auto pager = view(ri)->getDatabasePager();[m
[36m@@ -150,7 +150,7 @@[m [mnamespace osgEarth[m
                         auto ico = pager->getIncrementalCompileOperation();[m
                         if (ico) {[m
                             ico_jobs[f] = ico->getToCompile().size();[m
[31m-                            snprintf(buf, sizeof(buf), "%d", ico_jobs[f]);[m
[32m+[m[32m                            sprintf(buf, "%d", ico_jobs[f]);[m
                             ImGuiLTable::PlotLines("ICO", get_counts, &ico_jobs, frame_count, frame_num, buf, 0u, 4u);[m
                         }[m
                     }[m
[1mdiff --git a/src/osgEarthImGui/TerrainEditGUI b/src/osgEarthImGui/TerrainEditGUI[m
[1mindex 3dbecb904..17d530a96 100644[m
[1m--- a/src/osgEarthImGui/TerrainEditGUI[m
[1m+++ b/src/osgEarthImGui/TerrainEditGUI[m
[36m@@ -476,7 +476,7 @@[m [mnamespace osgEarth[m
                 _lifemapDecal->setBlendFuncs([m
                     GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,[m
                     GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,[m
[31m-                    GL_ZERO, GL_ONE, // Set lush to not blend as it affects asset placement.[m
[32m+[m[32m                    GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,[m
                     GL_ZERO, GL_ONE);[m
 [m
                 // If there is a LifeMapLayer, append to it as a Post, otherwise standalone decal.[m
[1mdiff --git a/src/osgEarthImGui/TerrainGUI b/src/osgEarthImGui/TerrainGUI[m
[1mindex 96c38ee6a..8aec5f828 100644[m
[1m--- a/src/osgEarthImGui/TerrainGUI[m
[1m+++ b/src/osgEarthImGui/TerrainGUI[m
[36m@@ -137,28 +137,28 @@[m [mnamespace osgEarth[m
                     // Expiration options[m
                     ImGui::Separator();[m
                     unsigned int minExpiryFrames = options.getMinExpiryFrames();[m
[31m-                    if (ImGuiLTable::InputScalar("Min expiry frames", ImGuiDataType_U32, &minExpiryFrames, &u32_one, nullptr, "%u"))[m
[32m+[m[32m                    if (ImGuiLTable::InputScalar("Min Expiry Frames", ImGuiDataType_U32, &minExpiryFrames, &u32_one, nullptr, "%u"))[m
                     {[m
                         options.setMinExpiryFrames(minExpiryFrames);[m
                         engine->dirtyTerrainOptions();[m
                     }[m
 [m
                     float minExpiryTime = options.getMinExpiryTime();[m
[31m-                    if (ImGuiLTable::InputFloat("Min expiry time (s)", &minExpiryTime))[m
[32m+[m[32m                    if (ImGuiLTable::InputFloat("Min Expiry Time", &minExpiryTime))[m
                     {[m
                         options.setMinExpiryTime(minExpiryTime);[m
                         engine->dirtyTerrainOptions();[m
                     }[m
 [m
                     float minExpiryRange = options.getMinExpiryRange();[m
[31m-                    if (ImGuiLTable::InputFloat("Min expiry range (m)", &minExpiryRange))[m
[32m+[m[32m                    if (ImGuiLTable::InputFloat("Min Expiry Range", &minExpiryRange))[m
                     {[m
                         options.setMinExpiryRange(minExpiryRange);[m
                         engine->dirtyTerrainOptions();[m
                     }[m
 [m
                     unsigned int maxTilesToUnloadPerFrame = options.getMaxTilesToUnloadPerFrame();[m
[31m-                    if (ImGuiLTable::InputScalar("Max unloads per frame", ImGuiDataType_U32, &maxTilesToUnloadPerFrame, &u32_one, nullptr, "%u"))[m
[32m+[m[32m                    if (ImGuiLTable::InputScalar("Max Tiles to Unload", ImGuiDataType_U32, &maxTilesToUnloadPerFrame, &u32_one, nullptr, "%u"))[m
                     {[m
                         options.setMaxTilesToUnloadPerFrame(maxTilesToUnloadPerFrame);[m
                         engine->dirtyTerrainOptions();[m
[36m@@ -183,13 +183,6 @@[m [mnamespace osgEarth[m
                             }[m
                             ImGuiLTable::EndCombo();[m
                         }[m
[31m-[m
[31m-                        bool gpuPaging = options.getGPUPaging();[m
[31m-                        if (ImGuiLTable::Checkbox("GPU paging", &gpuPaging))[m
[31m-                        {[m
[31m-                            options.setGPUPaging(gpuPaging);[m
[31m-                            engine->dirtyTerrainOptions();[m
[31m-                        }[m
                     }[m
 [m
                     if (options.getGPUTessellation())[m
[36m@@ -331,7 +324,7 @@[m [mnamespace osgEarth[m
                             dist_m += (vec[i] - vec[i - 1]).length();[m
                     }[m
                     char buf[64];[m
[31m-                    snprintf(buf, sizeof(buf), "%.1f m", dist_m);[m
[32m+[m[32m                    sprintf(buf, "%.1f m", dist_m);[m
                     _measureLabel->setText(buf);[m
                     GeoPoint labelPos = _measureFeature->getExtent().getCentroid();[m
                     labelPos.altitudeMode() = ALTMODE_RELATIVE;[m
[1mdiff --git a/src/osgEarthImGui/VegetationLayerGUI b/src/osgEarthImGui/VegetationLayerGUI[m
[1mindex 2d145eec7..b23be0f4a 100644[m
[1m--- a/src/osgEarthImGui/VegetationLayerGUI[m
[1m+++ b/src/osgEarthImGui/VegetationLayerGUI[m
[36m@@ -453,7 +453,7 @@[m [mnamespace osgEarth[m
                                     {[m
                                         for (auto& asset_ref : assets)[m
                                         {[m
[31m-                                            drawModelAsset(asset_ref->asset(), ri);[m
[32m+[m[32m                                            drawModelAsset(asset_ref->asset());[m
                                         }[m
                                         ImGui::TreePop();[m
                                     }[m
[36m@@ -471,33 +471,20 @@[m [mnamespace osgEarth[m
                         auto assets = bioman.getResidentAssetsIfNotLocked();[m
                         for (auto& asset : assets)[m
                         {[m
[31m-                            drawModelAsset(asset, ri);[m
[32m+[m[32m                            drawModelAsset(asset);[m
                         }[m
                     }[m
                 }[m
                 ImGui::End();[m
             }[m
 [m
[31m-            void drawModelAsset(const ModelAsset* asset, osg::RenderInfo& ri)[m
[32m+[m[32m            void drawModelAsset(const ModelAsset* asset)[m
             {[m
[31m-                static ImGuiEx::ImGuiPreview preview;[m
[31m-                static URI previewURI;[m
[31m-                static const ModelAsset* openAsset = nullptr;[m
[31m-[m
                 std::string name = asset->name();[m
                 if (asset->traits().empty() == false)[m
                     name += " (" + AssetTraits::toString(asset->traits()) + ")";[m
 [m
[31m-                ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_NoTreePushOnOpen;[m
[31m-                bool isOpen = (openAsset == asset);[m
[31m-[m
[31m-                ImGui::TreeNodeEx((void*)(std::intptr_t)asset, flags | (isOpen ? ImGuiTreeNodeFlags_Selected : 0), "%s", name.c_str());[m
[31m-                if (ImGui::IsItemClicked())[m
[31m-                {[m
[31m-                    openAsset = (openAsset == nullptr) ? asset : nullptr;[m
[31m-                }[m
[31m-[m
[31m-                if (isOpen)[m
[32m+[m[32m                if (ImGui::TreeNode(name.c_str()))[m
                 {[m
                     if (asset->modelURI().isSet())[m
                         ImGui::Text("Model: %s", asset->modelURI()->base().c_str());[m
[36m@@ -508,15 +495,7 @@[m [mnamespace osgEarth[m
                     if (asset->traits().empty() == false)[m
                         ImGui::Text("Traits: %s", AssetTraits::toString(asset->traits()).c_str());[m
 [m
[31m-                    if (asset->modelURI().isSet())[m
[31m-                    {[m
[31m-                        if (asset->modelURI().get() != previewURI)[m
[31m-                        {[m
[31m-                            preview.setNode(asset->modelURI()->getNode());[m
[31m-                            previewURI = asset->modelURI().get();[m
[31m-                        }[m
[31m-                        preview.render(ri);[m
[31m-                    }[m
[32m+[m[32m                    ImGui::TreePop();[m
                 }[m
             }[m
 [m
[1mdiff --git a/src/osgEarthProcedural/BiomeManager.cpp b/src/osgEarthProcedural/BiomeManager.cpp[m
[1mindex 641443163..219997a25 100644[m
[1m--- a/src/osgEarthProcedural/BiomeManager.cpp[m
[1m+++ b/src/osgEarthProcedural/BiomeManager.cpp[m
[36m@@ -53,13 +53,15 @@[m [mnamespace[m
         out->allocateImage(in->s(), in->t(), 1, GL_RG, GL_UNSIGNED_BYTE);[m
         out->setInternalTextureFormat(GL_RG8);[m
         osg::Vec4 v;[m
[32m+[m[32m        osg::Vec4 packed;[m
         ImageUtils::PixelReader read(in);[m
         ImageUtils::PixelWriter write(out);[m
         read.forEachPixel([&](auto& i)[m
             {[m
                 read(v, i);[m
                 osg::Vec3 normal(v.r()*2.0f - 1.0f, v.g()*2.0f - 1.0f, v.b()*2.0f - 1.0f);[m
[31m-                write(NormalMapGenerator::pack(normal), i);[m
[32m+[m[32m                NormalMapGenerator::pack(normal, packed);[m
[32m+[m[32m                write(packed, i);[m
             }[m
         );[m
 [m
[1mdiff --git a/src/osgEarthProcedural/BridgeLayer.cpp b/src/osgEarthProcedural/BridgeLayer.cpp[m
[1mindex 9cb4720a2..5d2cd17fa 100644[m
[1m--- a/src/osgEarthProcedural/BridgeLayer.cpp[m
[1m+++ b/src/osgEarthProcedural/BridgeLayer.cpp[m
[36m@@ -333,7 +333,7 @@[m [mnamespace[m
             if (line->size() < 2)[m
                 continue;[m
 [m
[31m-            osgEarth::Polygon* poly = new osgEarth::Polygon();[m
[32m+[m[32m            Polygon* poly = new Polygon();[m
             poly->resize(line->size() * 2);[m
 [m
             for (int i = 0; i < line->size(); ++i)[m
[36m@@ -1028,16 +1028,6 @@[m [mBridgeLayer::createTileImplementation(const TileKey& key, ProgressCallback* prog[m
     // This functor assembles the Network and clamps our road data.[m
     auto preprocess = [&](FeatureList& features, ProgressCallback* progress)[m
         {[m
[31m-            if (!_session.valid())[m
[31m-                return;[m
[31m-[m
[31m-            auto map = _session->getMap();[m
[31m-            if (!map || !map->getElevationPool())[m
[31m-                return;[m
[31m-[m
[31m-            if (progress && progress->isCanceled())[m
[31m-                return;[m
[31m-[m
             // build the network:[m
             for (auto& feature : features)[m
             {[m
[1mdiff --git a/src/osgEarthProcedural/LifeMapLayer b/src/osgEarthProcedural/LifeMapLayer[m
[1mindex e2c22ea49..2bd552c94 100644[m
[1m--- a/src/osgEarthProcedural/LifeMapLayer[m
[1m+++ b/src/osgEarthProcedural/LifeMapLayer[m
[36m@@ -9,7 +9,7 @@[m
 #include <osgEarth/ImageLayer>[m
 #include <osgEarth/ElevationPool>[m
 #include <osgEarth/LayerReference>[m
[31m-#include <osgEarth/TerrainOptions>[m
[32m+[m[32m#include <osgEarth/LandCoverLayer>[m
 [m
 namespace osgEarth { namespace Procedural[m
 {[m
[36m@@ -100,8 +100,6 @@[m [mnamespace osgEarth { namespace Procedural[m
 [m
         GeoImage applyPostLayer(const GeoImage&, const TileKey&, Layer*, ProgressCallback*) const override;[m
 [m
[31m-        void prepareForRendering(TerrainEngine* engine) override;[m
[31m-[m
     public:[m
 [m
         void addedToMap(const Map* map) override;[m
[36m@@ -113,8 +111,8 @@[m [mnamespace osgEarth { namespace Procedural[m
         osg::observer_ptr<const Map> _map;[m
         mutable ElevationPool::WorkingSet _workingSet;[m
         osg::ref_ptr<osg::Image> _noiseFunc;[m
[32m+[m
         LandCoverSample::Factory::Ptr _landCoverFactory;[m
[31m-        TerrainOptionsAPI _terrainOptions;[m
 [m
         void checkForLayerError(Layer*);[m
     };[m
[1mdiff --git a/src/osgEarthProcedural/LifeMapLayer.cpp b/src/osgEarthProcedural/LifeMapLayer.cpp[m
[1mindex 0498d2ee7..55d479219 100644[m
[1m--- a/src/osgEarthProcedural/LifeMapLayer.cpp[m
[1m+++ b/src/osgEarthProcedural/LifeMapLayer.cpp[m
[36m@@ -9,7 +9,12 @@[m
 #include <osgEarth/ElevationPool>[m
 #include <osgEarth/Math>[m
 #include <osgEarth/MetaTile>[m
[31m-#include <osgEarth/TerrainEngineNode>[m
[32m+[m[32m#include <osgEarth/rtree.h>[m
[32m+[m
[32m+[m[32m#include <osgDB/ReadFile>[m
[32m+[m[32m#include <osgDB/FileNameUtils>[m
[32m+[m[32m#include <osgDB/ReaderWriter>[m
[32m+[m
 #include <random>[m
 [m
 #define LC "[" << className() << "] \"" << getName() << "\" "[m
[36m@@ -169,12 +174,6 @@[m [mLifeMapLayer::closeImplementation()[m
     return super::closeImplementation();[m
 }[m
 [m
[31m-void[m
[31m-LifeMapLayer::prepareForRendering(TerrainEngine* terrain)[m
[31m-{[m
[31m-    _terrainOptions = terrain->getOptions();[m
[31m-}[m
[31m-[m
 void[m
 LifeMapLayer::checkForLayerError(Layer* layer)[m
 {[m
[36m@@ -395,15 +394,14 @@[m [mLifeMapLayer::createImageImplementation([m
         return GeoImage::INVALID;[m
 [m
     // collect the elevation data:[m
[31m-    osg::ref_ptr<ElevationTile> elevTile;[m
[32m+[m[32m    osg::ref_ptr<ElevationTexture> elevTile;[m
     ElevationPool* ep = map->getElevationPool();[m
     ep->getTile(key, true, elevTile, &_workingSet, progress);[m
 [m
     // ensure we have a normal map for slopes and curvatures:[m
     if (elevTile.valid() && getTerrainWeight() > 0.0f)[m
     {[m
[31m-        elevTile->generateNormalMap(map.get(), _terrainOptions.getNormalMapTileSize(),[m
[31m-            &_workingSet, progress);[m
[32m+[m[32m        elevTile->generateNormalMap(map.get(), &_workingSet, progress);[m
     }[m
 [m
     GeoExtent extent = key.getExtent();[m
[1mdiff --git a/src/osgEarthProcedural/RoadNetwork.cpp b/src/osgEarthProcedural/RoadNetwork.cpp[m
[1mindex bc33a294c..b35fa5295 100644[m
[1m--- a/src/osgEarthProcedural/RoadNetwork.cpp[m
[1m+++ b/src/osgEarthProcedural/RoadNetwork.cpp[m
[36m@@ -268,7 +268,7 @@[m [mnamespace[m
             if (line->size() < 2)[m
                 continue;[m
 [m
[31m-            osgEarth::Polygon* poly = new osgEarth::Polygon();[m
[32m+[m[32m            Polygon* poly = new Polygon();[m
             poly->resize(line->size() * 2);[m
 [m
             for (int i = 0; i < line->size(); ++i)[m
[1mdiff --git a/src/osgEarthProcedural/TextureSplattingMaterials.cpp b/src/osgEarthProcedural/TextureSplattingMaterials.cpp[m
[1mindex b1957b9d2..eba95a754 100644[m
[1m--- a/src/osgEarthProcedural/TextureSplattingMaterials.cpp[m
[1m+++ b/src/osgEarthProcedural/TextureSplattingMaterials.cpp[m
[36m@@ -137,8 +137,9 @@[m [mnamespace[m
                 {[m
                     readNormals(normal, iter.u(), iter.v());[m
 [m
[31m-                    if (normals->getPixelFormat() == GL_RG || normals->getPixelFormat() == GL_COMPRESSED_RED_GREEN_RGTC2_EXT)[m
[32m+[m[32m                    if (normals->getPixelFormat() == GL_COMPRESSED_RED_GREEN_RGTC2_EXT)[m
                     {[m
[32m+[m[32m                        //NormalMapGenerator::unpack(normal, normal3);[m
                         // do nothing[m
                         packed.x() = normal.x();[m
                         packed.y() = normal.y();[m
[36m@@ -150,12 +151,12 @@[m [mnamespace[m
                             normal_scale.y() * (normal.y() * 2.0 - 1.0),[m
                             normal_scale.z() * (normal.z() * 2.0 - 1.0));[m
 [m
[31m-                        packed = NormalMapGenerator::pack(normal3);[m
[32m+[m[32m                        NormalMapGenerator::pack(normal3, packed);[m
                     }[m
                 }[m
                 else[m
                 {[m
[31m-                    packed = NormalMapGenerator::pack(normal3);[m
[32m+[m[32m                    NormalMapGenerator::pack(normal3, packed);[m
                 }[m
 [m
                 if (roughness.valid())[m
[36m@@ -192,8 +193,6 @@[m [mnamespace[m
         //ImageUtils::compressImageInPlace(output.get(), "cpu");[m
         //ImageUtils::mipmapImageInPlace(output.get());[m
 [m
[31m-        //TODO: using BC7 compression should work here![m
[31m-[m
         return output;[m
     }[m
 }[m
[1mdiff --git a/src/osgEarthProcedural/VegetationLayer.cpp b/src/osgEarthProcedural/VegetationLayer.cpp[m
[1mindex 9024b57c2..30a85553a 100644[m
[1m--- a/src/osgEarthProcedural/VegetationLayer.cpp[m
[1m+++ b/src/osgEarthProcedural/VegetationLayer.cpp[m
[36m@@ -1645,15 +1645,11 @@[m [mVegetationLayer::getAssetPlacements([m
     std::vector<osg::Vec3d> map_points_culled;[m
     map_points_culled.reserve(result.size());[m
 [m
[31m-    // Use a NEW PRNG for the density thresholding, since we can't know[m
[31m-    // the state of the old one and we need this to be completely deterministic[m
[31m-    Random density_prng(key.getTileX() + key.getTileY());[m
[31m-[m
     for (int i = 0; i < result.size(); ++i)[m
     {[m
         Placement& p = result[i];[m
 [m
[31m-        if (density_prng.next() <= p.density())[m
[32m+[m[32m        if (RAND() <= p.density())[m
         {[m
             result_culled.emplace_back(std::move(p));[m
             map_points_culled.emplace_back(std::move(map_points[i]));[m
