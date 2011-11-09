#include "Geographic"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/Math>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Texture2D>

#include <osgEarth/ImageUtils>
#include <osgEarth/Notify>
#include <osgEarth/VerticalSpatialReference>
#include <osgEarth/TaskService>

#include "GeoPatch"
#include "MultiArray"

namespace seamless
{
using namespace std;
using namespace osg;
using namespace osgEarth;

typedef multi_array_ref<Vec3f, Vec3Array, 2> PatchArray;

Geographic::Geographic(const Map* map,
                       const osgEarth::Drivers::SeamlessOptions& options)
    : PatchSet(options, new PatchOptions), _profile(new EulerProfile),
      _eModel(new EllipsoidModel)
{
    setPrecisionFactor(8);
    setMap(map);
    {
        int maxLevel = 0;
        const ElevationLayerVector& elevations = _mapf->elevationLayers();
        for (ElevationLayerVector::const_iterator itr = elevations.begin(),
                 end = elevations.end();
             itr != end;
             ++itr)
        {
            const TerrainLayerOptions& options
                = (*itr)->getTerrainLayerOptions();
            if (options.maxLevel().isSet()
                && options.maxLevel().get() > maxLevel)
                maxLevel = options.maxLevel().get();
        }
        if (maxLevel > 0)
            setMaxLevel(maxLevel);

    }
    int serviceThreads = computeLoadingThreads(_options.loadingPolicy().get());
    _hfService = new TaskService("Height Field Service", serviceThreads);
    _imageService = new TaskService("Image Service", serviceThreads);
}

Geographic::Geographic(const Geographic& rhs, const osg::CopyOp& copyop)
    : PatchSet(rhs, copyop),
      _profile(static_cast<EulerProfile*>(copyop(rhs._profile.get()))),
      _eModel(static_cast<EllipsoidModel*>(copyop(rhs._eModel.get()))),
      _hfService(rhs._hfService), _imageService(rhs._imageService)
{
}

Geographic::~Geographic()
{
}

Node* Geographic::createPatchSetGraph(const std::string& filename)
{
    CoordinateSystemNode* csn = new CoordinateSystemNode;
    // Should these values come from the map profile?
    csn->setCoordinateSystem("EPSG:4326");
    csn->setFormat("WKT");
    csn->setEllipsoidModel(_eModel.get());
    for (int face = 0; face < 6; ++face)
    {
        double x = 0.0, y = 0.0;
        euler::faceToCube(x, y, face);
        PatchOptions* poptions = static_cast<PatchOptions*>(
            osg::clone(getPatchOptionsPrototype()));
        poptions->setPatchSet(this);
        poptions->setTileKey(_profile->createTileKey(x, y, 2));
        Node* node = createPatchGroup("foobar.osgearth_engine_seamless_patch",
                                      poptions);
        csn->addChild(node);
    }
    return csn;
}

namespace
{

GeoHeightField
mergeHeightFields(const GeoExtent& targetExtent, const GeoHeightFieldVector& hfs)
{
    if (hfs.size() != 4)
    {
        OE_FATAL << "mergeHeightFields expected 4 height fields\n";
        return GeoHeightField();
    }
    // List is in tile subkey quadrant order.
    // Assume the height fields all have the same dimensions
    unsigned targetCols = hfs[0].getHeightField()->getNumColumns() * 2 - 1;
    unsigned targetRows = hfs[0].getHeightField()->getNumRows() * 2 - 1;
    HeightField* targethf = new HeightField;
    targethf->allocate(targetCols, targetRows);
    GeoHeightField geo(targethf, targetExtent, 0);
    for (int i = 0; i < 4; ++i)
    {
        const HeightField* src = hfs[i].getHeightField();
        unsigned targetColumn
            = floor((hfs[i].getExtent().xMin() - targetExtent.xMin())
                    / targetExtent.width() * (targetCols - 1) + .5);
        unsigned targetRow
            = floor((hfs[i].getExtent().yMin() - targetExtent.yMin())
                    / targetExtent.height() * (targetRows - 1) + .5);
        for (unsigned sj = 0, tj = targetRow;
             sj < src->getNumRows() && tj < targetRows;
             ++sj, ++tj)
        {
            for (unsigned si = 0, ti = targetColumn;
             si < src->getNumColumns() && ti < targetCols;
             ++si, ++ti)
                targethf->setHeight(ti, tj, src->getHeight(si, sj));
        }
    }
    return geo;
}

GeoImage
mergeImages(const GeoExtent& targetExtent, const GeoImageVector& imgs)
{
    Image* targetImage = new Image;
    const Image* proto = imgs[0].getImage();
    targetImage->setInternalTextureFormat(proto->getInternalTextureFormat());
    int numRows = proto->s() * 2;
    int numCols = proto->t() * 2;
    targetImage->allocateImage(numRows, numCols, proto->r(),
                               proto->getPixelFormat(), proto->getDataType(),
                               proto->getPacking());
    for (GeoImageVector::const_iterator itr = imgs.begin(),
             end = imgs.end();
         itr != end;
         ++itr)
    {
        const GeoExtent& srcExtent = itr->getExtent();
        int dstx
            = floor((srcExtent.xMin() - targetExtent.xMin()) / targetExtent.width()
                    * numCols + .5);
        int dsty
            = floor((srcExtent.yMin() - targetExtent.yMin()) / targetExtent.height()
                    * numRows + .5);
        ImageUtils::copyAsSubImage(itr->getImage(), targetImage,
                                   dstx, dsty);
    }
    return GeoImage(targetImage, targetExtent);
}
}

// Create vertex arrays from the height field for a patch and install
// them in the patch.
void expandHeights(Geographic* gpatchset, const TileKey& key,
                   const GeoHeightField& hf, Vec3Array* verts,
                   Vec3Array* normals)
{
    int resolution = gpatchset->getResolution();
    const GeoExtent& patchExtent = key.getExtent();
    double centx, centy;
    patchExtent.getCentroid(centx, centy);
    Vec3d patchCenter = gpatchset->toModel(centx, centy, 0);
    const SpatialReference* srs = key.getProfile()->getSRS();
    const SpatialReference* geoSrs = srs->getGeographicSRS();
    // Populate cell
    ref_ptr<Patch::Data> data = new Patch::Data;
    int patchDim = resolution + 1;
    double xInc = (patchExtent.xMax() - patchExtent.xMin()) / resolution;
    double yInc = (patchExtent.yMax() - patchExtent.yMin()) / resolution;
    const EllipsoidModel* eModel = gpatchset->getEllipsoidModel();
    const float verticalScale = gpatchset->getVerticalScale();
    PatchArray mverts(*verts, patchDim);
    for (int j = 0; j < patchDim; ++j)
    {
        for (int i = 0; i < patchDim; i++)
        {
            Vec2d cubeCoord(patchExtent.xMin() + i * xInc,
                            patchExtent.yMin() + j * yInc);
            double lon, lat;
            srs->transform(cubeCoord.x(), cubeCoord.y(), geoSrs, lon, lat);
            float elevation;

            bool found = hf.getElevation(srs, cubeCoord.x(), cubeCoord.y(),
                                         INTERP_BILINEAR, 0, elevation);
            // Into ec coordinates
            if (!found)
            {
                OE_WARN << "Couldn't find height sample for cube coordinates "
                        << cubeCoord.x() << ", " << cubeCoord.y()
                        << " (lon lat " << lon << ", " << lat << ")\n";
                continue;
            }
            elevation *= verticalScale;
            Vec3d coord;
            eModel->convertLatLongHeightToXYZ(
                DegreesToRadians(lat), DegreesToRadians(lon), elevation,
                coord.x(), coord.y(), coord.z());
            mverts[j][i] = coord - patchCenter;
            if (fabs(mverts[j][i].z()) > 6000000)
                OE_WARN << "found huge coordinate.\n";
        }
    }
    // Normals. Average the normals of the triangles around the sample
    // point. We're not following the actual tessallation of the grid.
    for (int j = 0; j < patchDim; ++j)
    {
        for (int i = 0; i < patchDim; i++)
        {
            const Vec3& pt = (*verts)[j * patchDim + i];
            // A cross of points.
            Vec3 delta[4];      // Initialized to zero vectors
            for (int k = 0; k < 2; ++k)
            {
                int gridx = i + 2 * k - 1;
                if (gridx < patchDim && gridx >= 0)
                    delta[2 * k] = (*verts)[j * patchDim + gridx] - pt;
            }
            for (int k = 0; k < 2; ++k)
            {
                int gridy = j + 2 * k - 1;
                if (gridy < patchDim && gridy >= 0)
                    delta[2 * k + 1] = (*verts)[gridy * patchDim + i] - pt;
            }
            Vec3 normal;
            for (int k = 1; k <= 4; ++k)
            {
                int v1 = k - 1, v2 = k % 4;
                // If One or both of the deltas are 0, then the cross
                // product is 0 and won't contribute to the average.
                normal += delta[v1] ^ delta[v2];
            }
            normal.normalize();
            (*normals)[j * patchDim + i] = normal;
        }
    }
}

void installHeightField(GeoPatch* patch, const TileKey& key,
                        const GeoHeightField& hf)
{
    Geographic* gpatchset = patch->getGeographic();
    int resolution = gpatchset->getResolution();
    // Populate cell
    int patchDim = resolution + 1;
    Vec3Array* verts = new Vec3Array(patchDim * patchDim);
    verts->setDataVariance(Object::DYNAMIC);
    Vec3Array* normals = new Vec3Array(patchDim * patchDim);
    normals->setDataVariance(Object::DYNAMIC);
    Vec2Array* texCoords = new Vec2Array(patchDim * patchDim);
    expandHeights(gpatchset, key, hf, verts, normals);
    const float resinv = 1.0f / static_cast<float>(resolution);
    for (int j = 0; j < patchDim; ++j)
    {
        for (int i = 0; i < patchDim; i++)
        {
            (*texCoords)[j * patchDim +i] = Vec2(i * resinv, j * resinv);
        }
    }
    // Construct the patch and its transform.
    ref_ptr<Patch::Data> data = new Patch::Data;
    data->vertexData.array = verts;
    data->vertexData.binding = Geometry::BIND_PER_VERTEX;
    data->normalData.array = normals;
    data->normalData.binding = Geometry::BIND_PER_VERTEX;
    Vec4Array* colors = new Vec4Array(1);
    (*colors)[0] = Vec4(1.0, 1.0, 1.0, 1.0);
    data->colorData.array = colors;
    data->colorData.binding = Geometry::BIND_OVERALL;
    data->texCoordList
        .push_back(Geometry::ArrayData(texCoords, Geometry::BIND_PER_VERTEX));
    patch->setData(data);
}

// Create a patch and the transform that places it in the
// world. Install a height field if one is given.
MatrixTransform* createPatchAux(Geographic* gpatchset,
                                const TileKey& key,
                                const GeoHeightField& hf)
{
    GeoPatch* patch = new GeoPatch(key);
    patch->setGeographic(gpatchset);
    const GeoExtent& patchExtent = key.getExtent();
    double centx, centy;
    patchExtent.getCentroid(centx, centy);
    Vec3d patchCenter = gpatchset->toModel(centx, centy, 0);
    Matrixd patchMat = Matrixd::translate(patchCenter);
    installHeightField(patch, key, hf);
    MatrixTransform* result = new MatrixTransform;
    result->addChild(patch);
    result->setMatrix(patchMat);
    return result;
}


namespace
{
// Get a height field from the map, or an empty one if there is no
// data for this tile.
GeoHeightField getGeoHeightField(MapFrame& mapf, const TileKey& key,
                                 int resolution)
{
    osg::ref_ptr<HeightField> hf;
    mapf.getHeightField(key, true, hf, 0L, INTERP_BILINEAR);
    if  (!hf)
        hf = key.getProfile()->getVerticalSRS()
            ->createReferenceHeightField(key.getExtent(),
                                         resolution + 1, resolution + 1);
    return GeoHeightField(hf, key.getExtent(),
                          key.getProfile()->getVerticalSRS());
}

// Split up patch keys that cross the Date Line. The only patches
// that do are the the equatorial face with center at (-180, 0),
// and the poles faces.

inline bool crossesDateLine(const TileKey& key)
{
    int face = EulerProfile::getFace(key);
    const GeoExtent& keyExtent = key.getExtent();
    return ((face == 2 || face == 4 || face == 5)
            && keyExtent.xMax() - keyExtent.xMin() > .5);
}

struct HeightFieldRequest : public TaskRequest
{
    HeightFieldRequest(Geographic* gpatchset, const TileKey& key)

        : _gpatchset(gpatchset), _key(key), _mapf(gpatchset->getMapFrame())
    {
    }
    void operator()(ProgressCallback* progress)
    {
        const Map* map = _gpatchset->getMap();
        int resolution = _gpatchset->getResolution();
        GeoHeightField hf;
        if (crossesDateLine(_key))
        {
            GeoHeightFieldVector hfs;
            for (int child = 0; child < 4; ++child)
            {
                TileKey subCubeKey = _key.createChildKey(child);
                hfs.push_back(getGeoHeightField(_mapf, subCubeKey, resolution));
            }
            hf = mergeHeightFields(_key.getExtent(), hfs);
        }
        else
        {
            hf = getGeoHeightField(_mapf, _key, resolution);
        }
        int patchDim = resolution + 1;
        Vec3Array* verts = new Vec3Array(patchDim * patchDim);
        _result = verts;
        _normalResult = new Vec3Array(patchDim * patchDim);
        expandHeights(_gpatchset.get(), _key, hf,
                      verts, _normalResult.get());
    }
    ref_ptr<Geographic> _gpatchset;
    TileKey _key;
    // vertices are in _result;
    ref_ptr<Vec3Array> _normalResult;
    MapFrame _mapf;
};

struct ImageRequest : public TaskRequest
{
    ImageRequest(Geographic* gpatchset, const TileKey& key)
        : _gpatchset(gpatchset), _key(key), _mapf(gpatchset->getMapFrame())
    {
    }

    void operator()(ProgressCallback* progress)
    {
        GeoImage gimage;
        const ImageLayerVector& layers = _mapf.imageLayers();
        if (crossesDateLine(_key))
        {
            GeoImageVector gis;
            if (!layers.empty())
            {
                for (int child = 0; child < 4; ++child)
                {
                    TileKey subCubeKey = _key.createChildKey(child);
                    gis.push_back(layers[0]->createImage(subCubeKey));
                }
            }
            if (!gis.empty())
                gimage = mergeImages(_key.getExtent(), gis);
        }
        else
        {
            if (!layers.empty())
                gimage = layers[0]->createImage(_key);
        }
        _result = gimage.getImage();
    }
    ref_ptr<Geographic> _gpatchset;
    const TileKey _key;
    MapFrame _mapf;
};

// Update a patch node once map data is available
class GeoPatchUpdateCallback : public NodeCallback
{
public:
    GeoPatchUpdateCallback() {}
    GeoPatchUpdateCallback(HeightFieldRequest* hfRequest,
                           ImageRequest* imageRequest)
        : _hfRequest(hfRequest), _imageRequest(imageRequest)
    {
    }

    GeoPatchUpdateCallback(const GeoPatchUpdateCallback& nc,
                           const CopyOp& copyop)
        : NodeCallback(nc, copyop), _hfRequest(nc._hfRequest),
          _imageRequest(nc._imageRequest)
    {
    }

    META_Object(seamless, GeoPatchUpdateCallback);

    virtual void operator()(Node* node, NodeVisitor* nv);
    
    ref_ptr<HeightFieldRequest> _hfRequest;
    ref_ptr<ImageRequest> _imageRequest;
};
}

Transform* Geographic::createPatch(const std::string& filename,
                                   PatchOptions* poptions)
{
    const TileKey patchKey = poptions->getTileKey();
    // Dummy height field until data is available.
    const VerticalSpatialReference* vsrs
        = patchKey.getProfile()->getVerticalSRS();
    ref_ptr<HeightField> hf = vsrs->createReferenceHeightField(
        patchKey.getExtent(), _resolution + 1, _resolution + 1);
    GeoHeightField ghf(hf.get(), patchKey.getExtent(), vsrs);
    ref_ptr<MatrixTransform> transform = createPatchAux(this, patchKey, ghf);
    GeoPatch* patch = dynamic_cast<GeoPatch*>(transform->getChild(0));
    ref_ptr<HeightFieldRequest> hfr = new HeightFieldRequest(this, patchKey);
    ref_ptr<ImageRequest> ir = new ImageRequest(this, patchKey);
    patch->setUpdateCallback(new GeoPatchUpdateCallback(hfr.get(), ir.get()));
    _hfService->add(hfr.get());
    _imageService->add(ir.get());
    return transform.release();
}

namespace
{
ClusterCullingCallback*
createClusterCullingCallback(const Matrixd& transform, const Patch* patch,
                             const EllipsoidModel* et)
{
    //This code is a very slightly modified version of the
    //DestinationTile::createClusterCullingCallback in
    //VirtualPlanetBuilder.
    double globe_radius =  et->getRadiusPolar();
    Vec3 center_position(transform.getTrans());
    Vec3 center_normal(center_position);
    center_normal.normalize();

    // populate the vertex/normal/texcoord arrays from the grid.

    float min_dot_product = 1.0f;
    float max_cluster_culling_height = 0.0f;
    float max_cluster_culling_radius = 0.0f;
    const Vec3Array* verts = static_cast<const Vec3Array*>(
        patch->getData()->vertexData.array.get());
    for (Vec3Array::const_iterator itr = verts->begin(), end = verts->end();
         itr != end;
         ++itr)
    {
        Vec3d dv = *itr;
        Vec3d v = dv + center_position;
        double lat, lon, height;

        et->convertXYZToLatLongHeight(v.x(), v.y(), v.z(),
                                      lat, lon, height);

        double d = sqrt(dv.x()*dv.x() + dv.y()*dv.y() + dv.z()*dv.z());
        double theta = acos(globe_radius / (globe_radius + fabs(height)));
        double phi = 2.0 * asin (d*0.5 / globe_radius); // d/globe_radius;
        double beta = theta + phi;
        double sb = sin(beta);
        double cb = cos(beta);
        double cutoff = osg::PI_2 - 0.1;

        //log(osg::INFO,"theta="<<theta<<"\tphi="<<phi<<" beta "<<beta);
        if (phi<cutoff && beta<cutoff)
        {
            float local_dot_product = -sb;
            float local_m = globe_radius*( 1.0/ cb - 1.0);
            float local_radius = static_cast<float>(globe_radius * sb / cb); // beta*globe_radius;
            min_dot_product = osg::minimum(min_dot_product, local_dot_product);
            max_cluster_culling_height = osg::maximum(max_cluster_culling_height,local_m);
            max_cluster_culling_radius = osg::maximum(max_cluster_culling_radius,local_radius);
        }
        else
        {
            //log(osg::INFO,"Turning off cluster culling for wrap around tile.");
            return 0;
        }
    }

    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback;

    ccc->set(center_position + center_normal*max_cluster_culling_height ,
             center_normal,
             min_dot_product,
             max_cluster_culling_radius);

    return ccc;
}
}

Node* Geographic::createPatchGroup(const string& filename,
                                   PatchOptions* poptions)
{
    Node* result = PatchSet::createPatchGroup(filename, poptions);
    PatchGroup* pgroup = dynamic_cast<PatchGroup*>(result);
    // Make a cluster culling callback
    MatrixTransform* transform
        = dynamic_cast<MatrixTransform*>(pgroup->getChild(0));
    Patch* patch = dynamic_cast<Patch*>(transform->getChild(0));
    ClusterCullingCallback* ccc
        = createClusterCullingCallback(transform->getMatrix(), patch,
                                       _eModel.get());
    pgroup->setCullCallback(ccc);
    return pgroup;
}

Vec3d Geographic::toModel(double cubeX, double cubeY, double elevation)
{
    double faceX = cubeX, faceY = cubeY;
    int face;
    euler::cubeToFace(faceX, faceY, face);
    double lat_deg, lon_deg;
    euler::faceCoordsToLatLon(faceX, faceY, face, lat_deg, lon_deg);
    Vec3d result;
    _eModel->convertLatLongHeightToXYZ(
        DegreesToRadians(lat_deg), DegreesToRadians(lon_deg), elevation,
        result.x(), result.y(), result.z());
    return result;
}

Node* Geographic::createChild(const PatchOptions* parentOptions, int childNum)
{
    PatchOptions* poptions = static_cast<PatchOptions*>(
        osg::clone(parentOptions));
    poptions->setPatchLevel(parentOptions->getPatchLevel() + 1);
    poptions->setTileKey(parentOptions->getTileKey().createChildKey(childNum));
    return createPatchGroup("foobies.osgearth_engine_seamless_patch", poptions);

}

// A tile can be thought of lying between edges with integer
// coordinates at its LOD. With x going to the right and y going down,
// a tile between (tile_x, tile_y) and (tile_x + 1, tile_y + 1).
//
// edge order should be counter clockwise

struct GridEdge
{
    unsigned v[2][2];
    unsigned lod;
};

struct KeyIndex
{
    KeyIndex() : lod(0), x(0), y(0) {}
    KeyIndex(unsigned lod_, unsigned x_, unsigned y_)
        : lod(lod_), x(x_), y(y_)
    {
    }
    KeyIndex(const TileKey& key)
        : lod(key.getLevelOfDetail()), x(key.getTileX()), y(key.getTileY())
    {
    }
    bool operator==(const KeyIndex& rhs) const
    {
        return lod == rhs.lod && x == rhs.x && y == rhs.y;
    }
    unsigned lod;
    unsigned x;
    unsigned y;
};

bool containsTile(const KeyIndex& parent, const KeyIndex& child)
{
    if (parent.lod > child.lod)
        return false;
    if (parent.lod == child.lod)
        return parent.x == child.x && parent.y == child.y;
    int lodDiff = child.lod - parent.lod;
    if (child.x >> lodDiff == parent.x && child.y >> lodDiff == parent.y)
        return true;
    else
        return false;
}

// assume that tile is at a lower or equal LOD than neighbor
bool isNeighborTile(const KeyIndex& tile, const KeyIndex& neighbor)
{
    int lodDiff = neighbor.lod - tile.lod;
    int lodMult = 1 << lodDiff;
    unsigned tx = tile.x << lodDiff;
    unsigned ty = tile.y << lodDiff;
    if (tx == neighbor.x + 1 || tx + lodMult == neighbor.x)
        return neighbor.y >= ty && neighbor.y + 1 <= ty + lodMult;
    else if (ty == neighbor.y + 1 || ty + lodMult == neighbor.y)
        return neighbor.x >= tx && neighbor.x + 1 <= tx + lodMult;
    return false;
        
}

// Do tiles share a corner?
bool adjoinsTile(const KeyIndex& tile, const KeyIndex& neighbor)
{
    int lodDiff = neighbor.lod - tile.lod;
    int lodMult = 1 << lodDiff;
    unsigned tx = tile.x << lodDiff;
    unsigned ty = tile.y << lodDiff;
    if (tx == neighbor.x + 1 || tx + lodMult == neighbor.x)
        return ty == neighbor.y + 1 || ty + lodMult == neighbor.y;
    else
        return false;
}

PatchGroup* findFaceRoot(GeoPatch* patch, NodePath& pathList)
{
    // Get the patch's key
    Group* parent = patch->getParent(0);
    PatchGroup* parentPatch = dynamic_cast<PatchGroup*>(parent->getParent(0));
    if (!parentPatch)
        return 0;
    PatchOptions* parentOptions = parentPatch->getOptions();
    TileKey patchKey = parentOptions->getTileKey();
    int x = patchKey.getTileX() >> (patchKey.getLevelOfDetail() - 2);
    int y = patchKey.getTileY() >> (patchKey.getLevelOfDetail() - 2);
    
    for (NodePath::iterator itr = pathList.begin(), end = pathList.end();
         itr != end;
         ++itr)
    {
        PatchGroup* pg = dynamic_cast<PatchGroup*>(*itr);
        if (pg)
        {
            PatchOptions* poptions = pg->getOptions();
            if (poptions)
            {
                TileKey key = poptions->getTileKey();
                if (key.getLevelOfDetail() == 2 && x == key.getTileX()
                    && y == key.getTileY())
                return pg;
            }
        }
    }
    return 0;
}

typedef vector_ref<Vec3f, Vec3Array> EdgeRef;

EdgeRef makeEdgeRef(GeoPatch* gpatch, int edgeno, int mult)
{
    Vec3Array* verts
        = static_cast<Vec3Array*>(gpatch->getData()->vertexData.array.get());
    int patchDim = gpatch->getPatchSet()->getResolution() + 1;
    int shape = (patchDim - 1) / mult + 1;
    switch(edgeno)
    {
    case 0:
        return EdgeRef(*verts, mult, shape, 0);
    case 1:
        return EdgeRef(*verts, patchDim * mult, shape, patchDim - 1);
    case 2:
        return EdgeRef(*verts, mult, shape, (patchDim - 1) * patchDim);
    case 3:
        return EdgeRef(*verts, patchDim * mult, shape, 0);
    default:
        return EdgeRef(*verts, 0, 0, 0); // shouldn't happen
    }
}

struct ShareResult
{
    ShareResult()
        : numEdges(0)
    {
        for (int i = 0; i < 2; ++i)
            tile1[i] = tile2[i] = -1;
    }
    int numEdges;
    int tile1[2];
    int tile2[2];
};

// tile2 is at a higher LOD than tile1
ShareResult tilesShareEdges(const KeyIndex& tile1, const KeyIndex& tile2)
{
    ShareResult result;
    int lodDiff = tile2.lod - tile1.lod;
    int x = tile1.x << lodDiff;
    int xleft = (tile1.x + 1) << lodDiff;
    int y = tile1.y << lodDiff;
    int ybottom = (tile1.y + 1) << lodDiff;
    if (tile2.x >= x && tile2.x + 1 <= xleft
        && tile2.y >= y && tile2.y + 1 <= ybottom)
    {
        // tile1 contains tile2; do they share edges?
        if (x == tile2.x)
        {
            result.tile1[0] = 3;
            result.tile2[0] = 3;
            result.numEdges = 1;
        }
        else if (xleft == tile2.x + 1)
        {
            result.tile1[0] = 1;
            result.tile2[0] = 1;
            result.numEdges = 1;
        }
        if (y == tile2.y)
        {
            result.tile1[result.numEdges] = 2;
            result.tile2[result.numEdges] = 2;
            result.numEdges++;
        }
        else if (ybottom == tile2.y + 1)
        {
            result.tile1[result.numEdges] = 0;
            result.tile2[result.numEdges] = 0;
            result.numEdges++;
        }
    }
    else
    {
        // Tiles can share 1 edge.
        if (x == tile2.x + 1)
        {
            result.tile1[0] = 3;
            result.tile2[0] = 1;
            result.numEdges = 1;
        }
        else if (xleft == tile2.x)
        {
            result.tile1[0] = 1;
            result.tile2[0] = 3;
            result.numEdges = 1;
        }
        else if (y == tile2.y + 1)
        {
            result.tile1[0] = 2;
            result.tile2[0] = 0;
            result.numEdges = 1;
        }
        else if (ybottom == tile2.y)
        {
            result.tile1[0] = 0;
            result.tile2[0] = 2;
            result.numEdges = 1;
        }
    }
    return result;
}

void transferEdges(
    GeoPatch* toPatch, const Matrixd& toMat, const KeyIndex& toIdx,
    GeoPatch* fromPatch, const Matrixd& fromMat, const KeyIndex& fromIdx,
    const ShareResult& shared);

void safeCopy(Vec3f& dest, const Vec3f& src, const Matrixd& mat)
{
    Vec3f tmp = src * mat;
    if ((tmp - dest).length2() > 100000000)
        OE_WARN << "whoops!\n";
    dest = tmp;
}

class TileUpdater : public NodeVisitor
{
public:
    TileUpdater(GeoPatch* gpatch)
        : NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN), _gpatch(gpatch)
    {
        const MatrixTransform* trans
            = static_cast<const MatrixTransform*>(_gpatch->getParent(0));
        _tileMat = trans->getMatrix();
        const PatchGroup* pg
            = static_cast<const PatchGroup*>(trans->getParent(0));
        const PatchOptions* popt = pg->getOptions();
        _tileIndex = popt->getTileKey();
    }

    void apply(PagedLOD& node)
    {
        PatchGroup* pgrp = dynamic_cast<PatchGroup*>(&node);
        if (!pgrp)
            return;
        const PatchOptions* popt = pgrp->getOptions();
        if (!popt)
            return;
        KeyIndex idx = popt->getTileKey();
        if (idx == _tileIndex)
            return;

        if (containsTile(idx, _tileIndex) || isNeighborTile(idx, _tileIndex))
            copyTileEdges(pgrp, popt);
        else if (adjoinsTile(idx, _tileIndex))
            copyCorner(pgrp, popt);
        else
            return;
        if (node.getNumChildren() > 1)
            traverse(*node.getChild(1));
    }
protected:
    void copyTileEdges(PatchGroup* node, const PatchOptions* gopt)
    {
        // The tile to update
        MatrixTransform* trans
            = static_cast<MatrixTransform*>(node->getChild(0));
        GeoPatch* tpatch = static_cast<GeoPatch*>(trans->getChild(0));
        KeyIndex idx(gopt->getTileKey());
        ShareResult shared = tilesShareEdges(idx, _tileIndex);
        if (shared.numEdges != 0)
        {
            transferEdges(tpatch, trans->getMatrix(), idx,
                          _gpatch, _tileMat, _tileIndex, shared);
            tpatch->dirtyVertexData();
        }
    }
    void copyCorner(PatchGroup* node, const PatchOptions* gopt)
    {
        // The tile to update
        MatrixTransform* trans
            = static_cast<MatrixTransform*>(node->getChild(0));
        Matrixd toMat = trans->getMatrix();
        Matrixd transferMat =  _tileMat * Matrixd::inverse(toMat);
        GeoPatch* tpatch = static_cast<GeoPatch*>(trans->getChild(0));
        KeyIndex tidx(gopt->getTileKey());
        Geographic* gset = _gpatch->getGeographic();
        int patchDim = gset->getResolution() + 1;
        Vec3Array* verts = static_cast<Vec3Array*>(_gpatch->getData()
                                                   ->vertexData.array.get());
        PatchArray varray(*verts, patchDim);
        Vec3Array* tverts = static_cast<Vec3Array*>(tpatch->getData()
                                                    ->vertexData.array.get());
        PatchArray tarray(*tverts, patchDim);
        int lodDiff = _tileIndex.lod - tidx.lod;
        int lodMult = 1 << lodDiff;
        unsigned tx = tidx.x << lodDiff;
        unsigned ty = tidx.y << lodDiff;
        
        if (_tileIndex.x < tx)
        {
            if (_tileIndex.y == ty + lodMult)
                //tarray[0][0] = varray[patchDim - 1][patchDim - 1] * transferMat;
                safeCopy(tarray[0][0], varray[patchDim - 1][patchDim - 1], transferMat);
            else
                //tarray[patchDim - 1][0] = varray[0][patchDim - 1] * transferMat;
                safeCopy(tarray[patchDim - 1][0], varray[0][patchDim - 1], transferMat);
        }
        else
        {
            if (_tileIndex.y == ty + lodMult)
                // tarray[0][patchDim - 1] = varray[patchDim - 1][0] * transferMat;
                safeCopy(tarray[0][patchDim - 1], varray[patchDim - 1][0], transferMat);
            else
                // tarray[patchDim - 1][patchDim - 1] = varray[0][0] * transferMat;
                safeCopy(tarray[patchDim - 1][patchDim - 1], varray[0][0], transferMat);
        }
        tpatch->dirtyVertexData();
    }
    GeoPatch* _gpatch;
    KeyIndex _tileIndex;
    Matrixd _tileMat;
};

void transferEdges(
    GeoPatch* toPatch, const Matrixd& toMat, const KeyIndex& toIdx,
    GeoPatch* fromPatch, const Matrixd& fromMat, const KeyIndex& fromIdx,
    const ShareResult& shared)
{
    int resolution = toPatch->getPatchSet()->getResolution();
    int patchDim = resolution + 1;
    int lodDiff = fromIdx.lod - toIdx.lod;
    int detailMult = 1 << lodDiff;
    Matrixd transferMat = fromMat * Matrixd::inverse(toMat);
    for (int i = 0; i < shared.numEdges; ++i)
    {
        EdgeRef toEdge = makeEdgeRef(toPatch, shared.tile1[i], 1);
        EdgeRef fromEdge = makeEdgeRef(fromPatch, shared.tile2[i], detailMult);
        int toStart;
        if (shared.tile1[i] == 0 || shared.tile1[i] == 2)
            toStart = (fromIdx.x - (toIdx.x * detailMult)) * resolution / detailMult;
        else
            toStart = (detailMult - 1 - (fromIdx.y - (toIdx.y * detailMult)))
                * resolution / detailMult;
        for (int jt = toStart, jf = 0; jf < fromEdge.shape(); ++jt, ++jf)
        {
#if 0
            Vec3d vtx = Vec3d(fromEdge[jf]);
            vtx = vtx * transferMat;
            toEdge[jt] = Vec3f(vtx);
#endif
            safeCopy(toEdge[jt], fromEdge[jf], transferMat);
        }
    }
}

void GeoPatchUpdateCallback::operator()(Node* node, NodeVisitor* nv)
{
    GeoPatch* patch = dynamic_cast<GeoPatch*>(node);
    if (!patch)
        return;
    if (_hfRequest.valid() && _hfRequest->isCompleted())
    {
        Vec3Array* verts = dynamic_cast<Vec3Array*>(_hfRequest->getResult());
        Vec3Array* norms = _hfRequest->_normalResult.get();
        if (verts && norms)
        {
            Vec3Array* patchVerts = static_cast<Vec3Array*>(
                patch->getData()->vertexData.array.get());
            Vec3Array* patchNorms = static_cast<Vec3Array*>(
                patch->getData()->normalData.array.get());
            copy(verts->begin(), verts->end(), patchVerts->begin());
            patchVerts->dirty();
            copy(norms->begin(), norms->end(), patchNorms->begin());
            patchNorms->dirty();
        }
        _hfRequest = 0;
        PatchGroup* faceRoot = findFaceRoot(patch, nv->getNodePath());
        if (faceRoot)
        {
            TileUpdater tileUpdater(patch);
            faceRoot->accept(tileUpdater);
        }
    }
    if (_imageRequest.valid() && _imageRequest->isCompleted())
    {
        Image* image = dynamic_cast<Image*>(_imageRequest->getResult());
        if (image)
        {
            Texture2D* tex = new Texture2D();
            tex->setImage(image);
            tex->setWrap(Texture::WRAP_S, Texture::CLAMP_TO_EDGE);
            tex->setWrap(Texture::WRAP_T, Texture::CLAMP_TO_EDGE);
            tex->setFilter(Texture::MIN_FILTER,
                           Texture::LINEAR_MIPMAP_LINEAR);
            tex->setFilter(Texture::MAG_FILTER, Texture::LINEAR);
            StateSet* ss = patch->getOrCreateStateSet();
            ss->setTextureAttributeAndModes(0, tex, StateAttribute::ON);
        }
        _imageRequest = 0;
    }
    if (!_hfRequest.valid() && !_imageRequest.valid())
        node->setUpdateCallback(0);
}
}
