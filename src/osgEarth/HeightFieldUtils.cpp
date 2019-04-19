
/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#include <osgEarth/HeightFieldUtils>
#include <osgEarth/CullingUtils>

using namespace osgEarth;


bool
HeightFieldUtils::validateSamples(float &a, float &b, float &c, float &d)
{
    // If ALL the sample points are NO_DATA_VALUE then we can't do anything.
    if (a == NO_DATA_VALUE && b == NO_DATA_VALUE && c == NO_DATA_VALUE && d == NO_DATA_VALUE)
    {
        return false;
    }

    // If any of the samples are valid but some are NO_DATA_VALUE we can replace the nodata with valid values.
    if (a == NO_DATA_VALUE ||
        b == NO_DATA_VALUE || 
        c == NO_DATA_VALUE ||
        d == NO_DATA_VALUE)
    {
        float validValue = a;
        if (validValue == NO_DATA_VALUE) validValue = b;
        if (validValue == NO_DATA_VALUE) validValue = c;
        if (validValue == NO_DATA_VALUE) validValue = d;

        if (a == NO_DATA_VALUE) a = validValue;
        if (b == NO_DATA_VALUE) b = validValue;
        if (c == NO_DATA_VALUE) c = validValue;
        if (d == NO_DATA_VALUE) d = validValue;
    }

    return true;
}

float
HeightFieldUtils::getHeightAtPixel(const osg::HeightField* hf, double c, double r, ElevationInterpolation interpolation)
{
    float result = 0.0;
    switch (interpolation)
    {
    case INTERP_BILINEAR:
    {
        //OE_INFO << "getHeightAtPixel: (" << c << ", " << r << ")" << std::endl;
        int rowMin = osg::maximum((int)floor(r), 0);
        int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(hf->getNumRows() - 1)), 0);
        int colMin = osg::maximum((int)floor(c), 0);
        int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(hf->getNumColumns() - 1)), 0);

        if (rowMin > rowMax) rowMin = rowMax;
        if (colMin > colMax) colMin = colMax;

        float urHeight = hf->getHeight(colMax, rowMax);
        float llHeight = hf->getHeight(colMin, rowMin);
        float ulHeight = hf->getHeight(colMin, rowMax);
        float lrHeight = hf->getHeight(colMax, rowMin);

        //Make sure not to use NoData in the interpolation
        if (!validateSamples(urHeight, llHeight, ulHeight, lrHeight))
        {
            return NO_DATA_VALUE;
        }

        //OE_INFO << "Heights (ll, lr, ul, ur) ( " << llHeight << ", " << urHeight << ", " << ulHeight << ", " << urHeight << std::endl;

        //Check for exact value
        if ((colMax == colMin) && (rowMax == rowMin))
        {
            //OE_NOTICE << "Exact value" << std::endl;
            result = hf->getHeight((int)c, (int)r);
        }
        else if (colMax == colMin)
        {
            //OE_NOTICE << "Vertically" << std::endl;
            //Linear interpolate vertically
            result = ((double)rowMax - r) * llHeight + (r - (double)rowMin) * ulHeight;
        }
        else if (rowMax == rowMin)
        {
            //OE_NOTICE << "Horizontally" << std::endl;
            //Linear interpolate horizontally
            result = ((double)colMax - c) * llHeight + (c - (double)colMin) * lrHeight;
        }
        else
        {
            //OE_NOTICE << "Bilinear" << std::endl;
            //Bilinear interpolate
            double r1 = ((double)colMax - c) * (double)llHeight + (c - (double)colMin) * (double)lrHeight;
            double r2 = ((double)colMax - c) * (double)ulHeight + (c - (double)colMin) * (double)urHeight;
            result = ((double)rowMax - r) * (double)r1 + (r - (double)rowMin) * (double)r2;
        }
        break;
    }
    case INTERP_AVERAGE:
    {
        //OE_INFO << "getHeightAtPixel: (" << c << ", " << r << ")" << std::endl;
        int rowMin = osg::maximum((int)floor(r), 0);
        int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(hf->getNumRows() - 1)), 0);
        int colMin = osg::maximum((int)floor(c), 0);
        int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(hf->getNumColumns() - 1)), 0);

        if (rowMin > rowMax) rowMin = rowMax;
        if (colMin > colMax) colMin = colMax;

        float urHeight = hf->getHeight(colMax, rowMax);
        float llHeight = hf->getHeight(colMin, rowMin);
        float ulHeight = hf->getHeight(colMin, rowMax);
        float lrHeight = hf->getHeight(colMax, rowMin);

        //Make sure not to use NoData in the interpolation
        if (!validateSamples(urHeight, llHeight, ulHeight, lrHeight))
        {
            return NO_DATA_VALUE;
        }

        //OE_INFO << "Heights (ll, lr, ul, ur) ( " << llHeight << ", " << urHeight << ", " << ulHeight << ", " << urHeight << std::endl;

        double x_rem = c - (int)c;
        double y_rem = r - (int)r;

        double w00 = (1.0 - y_rem) * (1.0 - x_rem) * (double)llHeight;
        double w01 = (1.0 - y_rem) * x_rem * (double)lrHeight;
        double w10 = y_rem * (1.0 - x_rem) * (double)ulHeight;
        double w11 = y_rem * x_rem * (double)urHeight;

        result = (float)(w00 + w01 + w10 + w11);
        break;
    }
    case INTERP_NEAREST:
    {
        //Nearest interpolation
        result = hf->getHeight((unsigned int)osg::round(c), (unsigned int)osg::round(r));
        break;
    }
    case INTERP_TRIANGULATE:
    {
        //Interpolation to make sure that the interpolated point follows the triangles generated by the 4 parent points
        int rowMin = osg::maximum((int)floor(r), 0);
        int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(hf->getNumRows() - 1)), 0);
        int colMin = osg::maximum((int)floor(c), 0);
        int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(hf->getNumColumns() - 1)), 0);

        if (rowMin == rowMax)
        {
            if (rowMin < (int)hf->getNumRows() - 1)
            {
                rowMax = rowMin + 1;
            }
            else if (rowMax > 0)
            {
                rowMin = rowMax - 1;
            }
        }

        if (colMin == colMax)
        {
            if (colMin < (int)hf->getNumColumns() - 1)
            {
                colMax = colMin + 1;
            }
            else if (colMax > 0)
            {
                colMin = colMax - 1;
            }
        }

        if (rowMin > rowMax) rowMin = rowMax;
        if (colMin > colMax) colMin = colMax;

        float urHeight = hf->getHeight(colMax, rowMax);
        float llHeight = hf->getHeight(colMin, rowMin);
        float ulHeight = hf->getHeight(colMin, rowMax);
        float lrHeight = hf->getHeight(colMax, rowMin);

        //Make sure not to use NoData in the interpolation
        if (!validateSamples(urHeight, llHeight, ulHeight, lrHeight))
        {
            return NO_DATA_VALUE;
        }


        //The quad consisting of the 4 corner points can be made into two triangles.
        //The "left" triangle is ll, ur, ul
        //The "right" triangle is ll, lr, ur

        //Determine which triangle the point falls in.
        osg::Vec3d v0, v1, v2;

        double dx = c - (double)colMin;
        double dy = r - (double)rowMin;

        if (dx > dy)
        {
            //The point lies in the right triangle
            v0.set(colMin, rowMin, llHeight);
            v1.set(colMax, rowMin, lrHeight);
            v2.set(colMax, rowMax, urHeight);
        }
        else
        {
            //The point lies in the left triangle
            v0.set(colMin, rowMin, llHeight);
            v1.set(colMax, rowMax, urHeight);
            v2.set(colMin, rowMax, ulHeight);
        }

        //Compute the normal
        osg::Vec3d n = (v1 - v0) ^ (v2 - v0);

        result = (n.x() * (c - v0.x()) + n.y() * (r - v0.y())) / -n.z() + v0.z();
        break;
    }
    }

    return result;
}

bool
HeightFieldUtils::getInterpolatedHeight(const osg::HeightField* hf, 
                                        unsigned c, unsigned r, 
                                        float& out_height,
                                        ElevationInterpolation interpolation)
{
    int count = 0;
    float total = 0.0f;
    if ( c > 0 ) {
        total += hf->getHeight(c-1, r);
        count++;
    }
    if ( c < hf->getNumColumns()-1 ) {
        total += hf->getHeight(c+1, r);
        count++;
    }
    if ( r > 0 ) {
        total += hf->getHeight(c, r-1);
        count++;
    }
    if ( r < hf->getNumRows()-1 ) {
        total += hf->getHeight(c, r+1);
        count++;
    }
    if ( count > 0 )
        total /= (float)count;
    else
        return false;

    out_height = total;
    return true;
}

float
HeightFieldUtils::getHeightAtLocation(const osg::HeightField* hf, double x, double y, double llx, double lly, double dx, double dy, ElevationInterpolation interpolation)
{
    //Determine the pixel to sample
    double px = osg::clampBetween( (x - llx) / dx, 0.0, (double)(hf->getNumColumns()-1) );
    double py = osg::clampBetween( (y - lly) / dy, 0.0, (double)(hf->getNumRows()-1) );
    return getHeightAtPixel(hf, px, py, interpolation);
}

osg::Vec3
HeightFieldUtils::getNormalAtLocation(const HeightFieldNeighborhood& hood, double x, double y, double llx, double lly, double dx, double dy, ElevationInterpolation interp)
{
    const osg::HeightField* hf = hood._center.get();
    if (!hf)
        return osg::Vec3(0,0,1);

    double xcells = (double)(hf->getNumColumns()-1);
    double ycells = (double)(hf->getNumRows()-1);

    double xres = 1.0/xcells;
    double yres = 1.0/ycells;

    double mPerDegAtEquator = 111000.0;
    double tIntervalMeters = hf->getYInterval() * mPerDegAtEquator; // TODO: account for Geo vs Proj (see convertToNormalMap)
    
    double s = osg::clampBetween( (x - llx) / dx, 0.0, xcells );
    double t = osg::clampBetween( (y - lly) / dy, 0.0, ycells );

    double lat = hf->getOrigin().y() + hf->getYInterval()*t;
    
    double sIntervalMeters = hf->getXInterval() * mPerDegAtEquator*cos(osg::DegreesToRadians(lat));

    float centerHeight = getHeightAtLocation(hf, x, y, llx, lly, dx, dy, interp);

    double nx = xres * s;
    double ny = yres * t;

    osg::Vec3 west(-sIntervalMeters, 0.0f, centerHeight);
    osg::Vec3 east( sIntervalMeters, 0.0f, centerHeight);
    osg::Vec3 south(0, -tIntervalMeters, centerHeight);
    osg::Vec3 north(0,  tIntervalMeters, centerHeight);

    bool clamped = false;

    if (!getHeightAtNormalizedLocation(hood, nx - xres, ny, west.z())) {
        west.x() = 0.0, 
        west.z() = centerHeight;
        clamped = true;
    }

    if (!getHeightAtNormalizedLocation(hood, nx + xres, ny, east.z())) {
        east.x() = 0.0, east.z() = centerHeight;
        clamped = true;
    }

    if (!getHeightAtNormalizedLocation(hood, nx, ny - yres, south.z())) {
        south.y() = 0.0, south.z() = centerHeight;
        clamped = true;
    }

    if (!getHeightAtNormalizedLocation(hood, nx, ny + yres, north.z())) {
        north.y() = 0.0, north.z() = centerHeight;
        clamped = true;
    }    

    // account for degenerate vectors
    if (east.x() == 0.0 && west.x() == 0.0)
        east.x() = sIntervalMeters;

    if (north.y() == 0.0 && south.y() == 0.0)
        north.y() = tIntervalMeters;

    osg::Vec3 n = (east - west) ^ (north-south);
    n.normalize();

    //if (clamped)
    //    n = osg::Vec3(1,0,0);

    //if (clamped && (n*osg::Vec3(0,0,1) < 0.1)) {
    //    OE_WARN << "H, normal = " << n.x() << ", " << n.y() << ", " << n.z() << "\n";
    //}

    // curvature:
    float D = (0.5*(west.z() + east.z()) - centerHeight) / (sIntervalMeters*sIntervalMeters);
    float E = (0.5*(south.z() + north.z()) - centerHeight) / (tIntervalMeters*tIntervalMeters);
    float curvature = osg::clampBetween(-2.0f*(D + E)*100.0f, -1.0f, 1.0f);
    
    return n; // TODO, include curv
}

float
HeightFieldUtils::getHeightAtNormalizedLocation(const osg::HeightField* input,
                                                double nx, double ny,
                                                ElevationInterpolation interp)
{
    double px = osg::clampBetween(nx, 0.0, 1.0) * (double)(input->getNumColumns() - 1);
    double py = osg::clampBetween(ny, 0.0, 1.0) * (double)(input->getNumRows() - 1);
    return getHeightAtPixel( input, px, py, interp );
}

bool
HeightFieldUtils::getHeightAtNormalizedLocation(const HeightFieldNeighborhood& hood,
                                                double nx, double ny,
                                                float& output,
                                                ElevationInterpolation interp)
{
    osg::HeightField* hf = 0L;
    double nx2, ny2;
    if ( hood.getNeighborForNormalizedLocation(nx, ny, hf, nx2, ny2) )
    {
        double px = osg::clampBetween(nx2, 0.0, 1.0) * (double)(hf->getNumColumns() - 1);
        double py = osg::clampBetween(ny2, 0.0, 1.0) * (double)(hf->getNumRows() - 1);
        output = getHeightAtPixel( hf, px, py, interp );
        return output != NO_DATA_VALUE;
    }
    return false;
}

void
HeightFieldUtils::scaleHeightFieldToDegrees( osg::HeightField* hf )
{
    if (hf)
    {
        //The number of degrees in a meter at the equator
        //TODO: adjust this calculation based on the actual EllipsoidModel.
        float scale = 1.0f/111319.0f;

        osg::HeightField::HeightList& heights = hf->getHeightList();
        for(unsigned i=0; i<heights.size(); ++i)
            heights[i] *= scale;
    }
    else
    {
        OE_WARN << "[osgEarth::HeightFieldUtils] scaleHeightFieldToDegrees heightfield is NULL" << std::endl;
    }
}


osg::HeightField*
HeightFieldUtils::createSubSample(const osg::HeightField* input,
                                  const GeoExtent& inputEx, 
                                  const GeoExtent& outputEx,
                                  osgEarth::ElevationInterpolation interpolation)
{
    double div = outputEx.width()/inputEx.width();
    if ( div >= 1.0f )
        return 0L;

    int numCols = input->getNumColumns();
    int numRows = input->getNumRows();

    double xInterval = inputEx.width()  / (double)(input->getNumColumns()-1);
    double yInterval = inputEx.height()  / (double)(input->getNumRows()-1);
    double dx = div * xInterval;
    double dy = div * yInterval;


    osg::HeightField* dest = new osg::HeightField();
    dest->allocate( numCols, numRows );
    dest->setXInterval( dx );
    dest->setYInterval( dy );
    dest->setBorderWidth( input->getBorderWidth() );

    // copy over the skirt height, adjusting it for relative tile size.
    dest->setSkirtHeight( input->getSkirtHeight() * div );

    double x, y;
    int col, row;

    for( x = outputEx.xMin(), col=0; col < numCols; x += dx, col++ )
    {
        for( y = outputEx.yMin(), row=0; row < numRows; y += dy, row++ )
        {
            float height = HeightFieldUtils::getHeightAtLocation( input, x, y, inputEx.xMin(), inputEx.yMin(), xInterval, yInterval, interpolation);
            dest->setHeight( col, row, height );
        }
    }

    osg::Vec3d orig( outputEx.xMin(), outputEx.yMin(), input->getOrigin().z() );
    dest->setOrigin( orig );

    return dest;
}

osg::HeightField*
HeightFieldUtils::resampleHeightField(osg::HeightField*      input,
                                      const GeoExtent&       extent,
                                      int                    newColumns, 
                                      int                    newRows,
                                      ElevationInterpolation interp)
{
    if ( newColumns <= 1 && newRows <= 1 )
        return 0L;

    if ( newColumns == input->getNumColumns() && newRows == (int)input->getNumRows() )
        return input;
        //return new osg::HeightField( *input, osg::CopyOp::DEEP_COPY_ALL );

    double spanX = extent.width(); //(input->getNumColumns()-1) * input->getXInterval();
    double spanY = extent.height(); //(input->getNumRows()-1) * input->getYInterval();
    const osg::Vec3& origin = input->getOrigin();

    double stepX = spanX/(double)(newColumns-1);
    double stepY = spanY/(double)(newRows-1);

    osg::HeightField* output = new osg::HeightField();
    output->allocate( newColumns, newRows );
    output->setXInterval( stepX );
    output->setYInterval( stepY );
    output->setOrigin( origin );
    
    for( int y = 0; y < newRows; ++y )
    {
        for( int x = 0; x < newColumns; ++x )
        {
            double nx = (double)x / (double)(newColumns-1);
            double ny = (double)y / (double)(newRows-1);
            float h = getHeightAtNormalizedLocation( input, nx, ny, interp );
            output->setHeight( x, y, h );
        }
    }

    return output;
}


osg::HeightField*
HeightFieldUtils::createReferenceHeightField(const GeoExtent& ex,
                                             unsigned         numCols,
                                             unsigned         numRows,
                                             unsigned         border,
                                             bool             expressAsHAE)
{
    osg::HeightField* hf = new osg::HeightField();

    hf->allocate( numCols + 2*border, numRows + 2*border );

    hf->setXInterval( ex.width() / (double)(numCols-1) );
    hf->setYInterval( ex.height() / (double)(numRows-1) );

    hf->setOrigin(osg::Vec3d(
        ex.xMin() - hf->getXInterval()*(double)border,
        ex.yMin() - hf->getYInterval()*(double)border,
        0.0 ) );

    const VerticalDatum* vdatum = ex.isValid() ? ex.getSRS()->getVerticalDatum() : 0L;

    if ( vdatum && expressAsHAE )
    {
        // need the lat/long extent for geoid queries:
        GeoExtent geodeticExtent = ex.getSRS()->isGeographic() ? ex : ex.transform( ex.getSRS()->getGeographicSRS() );
        double latMin = geodeticExtent.yMin();
        double lonMin = geodeticExtent.xMin();
        double lonInterval = geodeticExtent.width() / (double)(numCols-1);
        double latInterval = geodeticExtent.height() / (double)(numRows-1);

        double latStart = latMin - latInterval*(double)border;
        double lonStart = lonMin - lonInterval*(double)border;

        for( unsigned r=0; r<hf->getNumRows(); ++r )
        {            
            double lat = latStart + latInterval*(double)r;
            for( unsigned c=0; c<hf->getNumColumns(); ++c )
            {
                double lon = lonStart + lonInterval*(double)c;
                double offset = vdatum->msl2hae(lat, lon, 0.0);
                hf->setHeight( c, r, offset );
            }
        }
    }
    else
    {
        hf->getFloatArray()->assign(hf->getNumColumns()*hf->getNumRows(), 0.0f);
    }

    hf->setBorderWidth( border );

    return hf;
}

void
HeightFieldUtils::resolveInvalidHeights(osg::HeightField* grid,
                                        const GeoExtent&  ex,
                                        float             invalidValue,
                                        const Geoid*      geoid)
{
    if ( geoid )
    {
        // need the lat/long extent for geoid queries:
        unsigned numRows = grid->getNumRows();
        unsigned numCols = grid->getNumColumns();
        GeoExtent geodeticExtent = ex.getSRS()->isGeographic() ? ex : ex.transform( ex.getSRS()->getGeographicSRS() );
        double latMin = geodeticExtent.yMin();
        double lonMin = geodeticExtent.xMin();
        double lonInterval = geodeticExtent.width() / (double)(numCols-1);
        double latInterval = geodeticExtent.height() / (double)(numRows-1);

        for( unsigned r=0; r<numRows; ++r )
        {
            double lat = latMin + latInterval*(double)r;
            for( unsigned c=0; c<numCols; ++c )
            {
                double lon = lonMin + lonInterval*(double)c;
                if ( grid->getHeight(c, r) == invalidValue )
                {
                    grid->setHeight( c, r, geoid->getHeight(lat, lon) );
                }
            }
        }
    }
    else
    {
        osg::HeightField::HeightList& heights = grid->getHeightList();
        for(unsigned i=0; i<heights.size(); ++i)
        {
            if ( heights[i] == invalidValue )
                heights[i] = 0.0f;
        }
    }
}

osg::NodeCallback*
HeightFieldUtils::createClusterCullingCallback(const osg::HeightField*    grid, 
                                               const osg::EllipsoidModel* et, 
                                               float                      verticalScale )
{
    //This code is a very slightly modified version of the DestinationTile::createClusterCullingCallback in VirtualPlanetBuilder.
    if ( !grid || !et )
        return 0L;

    double globe_radius = et->getRadiusPolar();
    unsigned int numColumns = grid->getNumColumns();
    unsigned int numRows = grid->getNumRows();

    double midLong = grid->getOrigin().x()+grid->getXInterval()*((double)(numColumns-1))*0.5;
    double midLat = grid->getOrigin().y()+grid->getYInterval()*((double)(numRows-1))*0.5;
    double midZ = grid->getOrigin().z();

    double midX,midY;
    et->convertLatLongHeightToXYZ(osg::DegreesToRadians(midLat),osg::DegreesToRadians(midLong),midZ, midX,midY,midZ);

    osg::Vec3 center_position(midX,midY,midZ);
    osg::Vec3 center_normal(midX,midY,midZ);
    center_normal.normalize();

    osg::Vec3 transformed_center_normal = center_normal;

    unsigned int r,c;

    // populate the vertex/normal/texcoord arrays from the grid.
    double orig_X = grid->getOrigin().x();
    double delta_X = grid->getXInterval();
    double orig_Y = grid->getOrigin().y();
    double delta_Y = grid->getYInterval();
    double orig_Z = grid->getOrigin().z();


    float min_dot_product = 1.0f;
    float max_cluster_culling_height = 0.0f;
    float max_cluster_culling_radius = 0.0f;

    for(r=0;r<numRows;++r)
    {
        for(c=0;c<numColumns;++c)
        {
            double X = orig_X + delta_X*(double)c;
            double Y = orig_Y + delta_Y*(double)r;
            double Z = orig_Z + grid->getHeight(c,r) * verticalScale;
            double height = Z;

            et->convertLatLongHeightToXYZ(
                osg::DegreesToRadians(Y), osg::DegreesToRadians(X), Z,
                X, Y, Z);

            osg::Vec3d v(X,Y,Z);
            osg::Vec3 dv = v - center_position;
            double d = sqrt(dv.x()*dv.x() + dv.y()*dv.y() + dv.z()*dv.z());
            double theta = acos( globe_radius/ (globe_radius + fabs(height)) );
            double phi = 2.0 * asin (d*0.5/globe_radius); // d/globe_radius;
            double beta = theta+phi;
            double cutoff = osg::PI_2 - 0.1;

            //log(osg::INFO,"theta="<<theta<<"\tphi="<<phi<<" beta "<<beta);
            if (phi<cutoff && beta<cutoff)
            {
                float local_dot_product = -sin(theta + phi);
                float local_m = globe_radius*( 1.0/ cos(theta+phi) - 1.0);
                float local_radius = static_cast<float>(globe_radius * tan(beta)); // beta*globe_radius;
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
    }    

    osg::NodeCallback* ccc = ClusterCullingFactory::create(
        center_position + transformed_center_normal*max_cluster_culling_height ,
        transformed_center_normal, 
        min_dot_product,
        max_cluster_culling_radius);

    return ccc;
}


NormalMap*
HeightFieldUtils::convertToNormalMap(const HeightFieldNeighborhood& hood,
                                     const SpatialReference*        hoodSRS)
{
    const osg::HeightField* hf = hood._center.get();
    if ( !hf )
        return 0L;
    
    NormalMap* normalMap = new NormalMap(hf->getNumColumns(), hf->getNumRows());

    double xcells = (double)(hf->getNumColumns()-1);
    double ycells = (double)(hf->getNumRows()-1);
    double xres = 1.0/xcells;
    double yres = 1.0/ycells;

    // north-south interval in meters:
    double mPerDegAtEquator = (hoodSRS->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI)/360.0;
    double tIntervalMeters = 
        hoodSRS->isGeographic() ? hf->getYInterval() * mPerDegAtEquator :
        hf->getYInterval();

    for(int t=0; t<(int)hf->getNumRows(); ++t)
    {
        // east-west interval in meters (changes for each row):
        double lat = hf->getOrigin().y() + hf->getYInterval()*(double)t;
        double sIntervalMeters =
            hoodSRS->isGeographic() ? hf->getXInterval() * mPerDegAtEquator * cos(osg::DegreesToRadians(lat)) :
            hf->getXInterval();

        for(int s=0; s<(int)hf->getNumColumns(); ++s)
        {
            float centerHeight = hf->getHeight(s, t);

            double nx = xres*(double)s;
            double ny = yres*(double)t;

            osg::Vec3f west ( -sIntervalMeters, 0, centerHeight );
            osg::Vec3f east (  sIntervalMeters, 0, centerHeight );
            osg::Vec3f south( 0, -tIntervalMeters, centerHeight );
            osg::Vec3f north( 0,  tIntervalMeters, centerHeight );

            if ( !HeightFieldUtils::getHeightAtNormalizedLocation(hood, nx-xres, ny, west.z()) )
                west.x() = 0.0, 
                west.z() = centerHeight;

            if ( !HeightFieldUtils::getHeightAtNormalizedLocation(hood, nx+xres, ny, east.z()) )
                east.x() = 0.0, 
                east.z() = centerHeight;

            if ( !HeightFieldUtils::getHeightAtNormalizedLocation(hood, nx, ny-yres, south.z()) )
                south.y() = 0.0, 
                south.z() = centerHeight;

            if ( !HeightFieldUtils::getHeightAtNormalizedLocation(hood, nx, ny+yres, north.z()) )
                north.y() = 0.0, 
                north.z() = centerHeight;

            // account for degenerate vectors
            if (east.x() == 0.0 && west.x() == 0.0)
                east.x() = sIntervalMeters;

            if (north.y() == 0.0 && south.y() == 0.0)
                north.y() = tIntervalMeters;

            osg::Vec3f n = (east - west) ^ (north - south);
            n.normalize();

            // calculate and encode curvature (2nd derivative of elevation)
            //float L2inv = 1.0f/(sIntervalMeters*sIntervalMeters);
            float D = (0.5*(west.z()+east.z()) - centerHeight) / (sIntervalMeters*sIntervalMeters); //* L2inv;
            float E = (0.5*(south.z()+north.z()) - centerHeight) / (tIntervalMeters*tIntervalMeters); //* L2inv;
            float curvature = osg::clampBetween(-2.0f*(D+E)*100.0f, -1.0f, 1.0f);

            //// encode for RGBA [0..1]
            //osg::Vec4f enc( n.x(), n.y(), n.z(), curvature );
            //enc = (enc + osg::Vec4f(1.0,1.0,1.0,1.0))*0.5;

            //write(enc, s, t);

            normalMap->set(s, t, n, curvature);
        }
    }

    return normalMap;
}

//TODO: get rid of this. -gw
void
HeightFieldUtils::createNormalMap(const osg::Image* elevation,
                                  NormalMap* normalMap,
                                  const GeoExtent& extent)
{   
    ImageUtils::PixelReader readElevation(elevation);
    //ImageUtils::PixelWriter writeNormal(normalMap);

    int sMax = (int)elevation->s()-1;
    int tMax = (int)elevation->t()-1;
        
    // north-south interval in meters:
    double xInterval = extent.width() / (double)(sMax);
    double yInterval = extent.height() / (double)(tMax);

    const SpatialReference* srs = extent.getSRS();
    double mPerDegAtEquator = (srs->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
    double dy = srs->isGeographic() ? yInterval * mPerDegAtEquator : yInterval;

    for (int t = 0; t<(int)elevation->t(); ++t)
    {
        double lat = extent.yMin() + yInterval*(double)t;
        double dx = srs->isGeographic() ? xInterval * mPerDegAtEquator * cos(osg::DegreesToRadians(lat)) : xInterval;

        for(int s=0; s<(int)elevation->s(); ++s)
        {
            float h = readElevation(s, t).r();

            osg::Vec3f west ( s > 0 ? -dx : 0, 0, h );
            osg::Vec3f east ( s < sMax ?  dx : 0, 0, h );
            osg::Vec3f south( 0, t > 0 ? -dy : 0, h );
            osg::Vec3f north( 0, t < tMax ? dy : 0, h );

            west.z() = readElevation(osg::maximum(0, s - 1), t).r();
            east.z() = readElevation(osg::minimum(sMax, s + 1), t).r();
            south.z() = readElevation(s, osg::maximum(0, t - 1)).r();
            north.z() = readElevation(s, osg::minimum(tMax, t + 1)).r();

            osg::Vec3f n = (east-west) ^ (north-south);
            n.normalize();

            // calculate and encode curvature (2nd derivative of elevation)
            float D = (0.5*(west.z()+east.z()) - h) / (dx*dx);
            float E = (0.5*(south.z()+north.z()) - h) / (dy*dy);
            float curvature = osg::clampBetween(-2.0f*(D+E)*100.0f, -1.0f, 1.0f);

            // encode for RGBA [0..1]
            //osg::Vec4f NC( n.x(), n.y(), n.z(), curvature );
            //NC = (NC + osg::Vec4f(1.0,1.0,1.0,1.0))*0.5;

            normalMap->set(s, t, n, curvature);
            //writeNormal(NC, s, t);
        }
    }
}
