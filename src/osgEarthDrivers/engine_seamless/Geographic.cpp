#include "Geographic"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/Math>
#include <osg/NodeCallback>
#include <osg/Texture2D>

#include <osgEarth/ImageUtils>
#include <osgEarth/Notify>
#include <osgEarth/VerticalSpatialReference>
#include <osgEarth/TaskService>

#include "GeoPatch"

namespace seamless
{
using namespace std;
using namespace osg;
using namespace osgEarth;

// Hard-wire the patch resolution and screen-space polygon size.
Geographic::Geographic(Map* map)
    : PatchSet(64, new GeographicOptions), _map(map), _profile(new EulerProfile),
      _eModel(new EllipsoidModel)
{
    setPrecisionFactor(8);
    const MapLayerList& heightList = _map->getHeightFieldMapLayers();
    {
        int maxLevel = 0;
        Threading::ScopedReadLock lock(_map->getMapDataMutex());
        for (MapLayerList::const_iterator itr = heightList.begin(),
                 end = heightList.end();
             itr != end;
             ++itr)
            if ((*itr)->maxLevel().isSet()
                && (*itr)->maxLevel().get() > maxLevel)
                maxLevel = (*itr)->maxLevel().get();
        if (maxLevel > 0)
            setMaxLevel(maxLevel);
    }
    _hfService = new TaskService("Height Field Service");
    _imageService = new TaskService("Image Service");
}

Geographic::Geographic(const Geographic& rhs, const osg::CopyOp& copyop)
    : PatchSet(rhs, copyop), _map(static_cast<Map*>(copyop(rhs._map.get()))),
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
        GeographicOptions* goptions = static_cast<GeographicOptions*>(
            osg::clone(getPatchOptionsPrototype()));
        goptions->setPatchSet(this);
        goptions->setTileKey(_profile->createTileKey(x, y, 2));
        Node* node = createPatchGroup("foobar.tengpatch", goptions);
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
            (*verts)[j * patchDim + i] = coord - patchCenter;
            if (fabs((*verts)[j * patchDim + i].z()) > 6000000)
            {
                OE_WARN << "found huge coordinate.\n";
            }
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
    Geographic* gpatchset = static_cast<Geographic*>(patch->getPatchSet());
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
    patch->setPatchSet(gpatchset);
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
GeoHeightField getGeoHeightField(Map* map, const TileKey& key, int resolution)
{
    HeightField* hf = 0;
    {
        Threading::ScopedReadLock lock(map->getMapDataMutex());
        hf = map->createHeightField(key, true, INTERP_BILINEAR);
    }
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

        : _gpatchset(gpatchset), _key(key)
    {
    }
    void operator()(ProgressCallback* progress)
    {
        Map* map = _gpatchset->getMap();
        int resolution = _gpatchset->getResolution();
        GeoHeightField hf;
        if (crossesDateLine(_key))
        {
            GeoHeightFieldVector hfs;
            for (int child = 0; child < 4; ++child)
            {
                TileKey subCubeKey = _key.createChildKey(child);
                hfs.push_back(getGeoHeightField(map, subCubeKey, resolution));
            }
            hf = mergeHeightFields(_key.getExtent(), hfs);
        }
        else
        {
            hf = getGeoHeightField(map, _key, resolution);
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
};

struct ImageRequest : public TaskRequest
{
    ImageRequest(Geographic* gpatchset, const TileKey& key)
        : _gpatchset(gpatchset), _key(key)
    {
    }

    void operator()(ProgressCallback* progress)
    {
        GeoImage gimage;
        Map* map = _gpatchset->getMap();
        Threading::ScopedReadLock lock(map->getMapDataMutex());
        if (crossesDateLine(_key))
        {
            GeoImageVector gis;
            for (int child = 0; child < 4; ++child)
            {
                TileKey subCubeKey = _key.createChildKey(child);
                if (!map->getImageMapLayers().empty())
                    gis.push_back(map->getImageMapLayers()[0]
                                  ->createImage(subCubeKey));
            }
            if (!gis.empty())
                gimage = mergeImages(_key.getExtent(), gis);
        }
        else
        {
            if (!map->getImageMapLayers().empty())
                gimage = map->getImageMapLayers()[0]->createImage(_key);
        }
        _result = gimage.getImage();
    }
    ref_ptr<Geographic> _gpatchset;
    const TileKey _key;
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

    virtual void operator()(Node* node, NodeVisitor* nv)
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
    ref_ptr<HeightFieldRequest> _hfRequest;
    ref_ptr<ImageRequest> _imageRequest;
};
}

Transform* Geographic::createPatch(const std::string& filename,
                                   PatchOptions* poptions)
{
    GeographicOptions* goptions = static_cast<GeographicOptions*>(poptions);
    const TileKey patchKey = goptions->getTileKey();
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
    const GeographicOptions* parentgopt
        = static_cast<const GeographicOptions*>(parentOptions);
    GeographicOptions* goptions = osg::clone(parentgopt);
    goptions->setPatchLevel(parentgopt->getPatchLevel() + 1);
    goptions->setTileKey(parentgopt->getTileKey().createChildKey(childNum));
    return createPatchGroup("foobies.tengpatch", goptions);

}

GeographicOptions::GeographicOptions()
    : _tileKey(TileKey::INVALID)
{
}

GeographicOptions::GeographicOptions(string& str)
    : PatchOptions(str), _tileKey(TileKey::INVALID)
{
}

GeographicOptions::GeographicOptions(const GeographicOptions& rhs,
                                     const CopyOp& copyop)
    : PatchOptions(rhs, copyop),
      _tileKey(rhs._tileKey)
{

}

GeographicOptions::~GeographicOptions()
{
}

}
