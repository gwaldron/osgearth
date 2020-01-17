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

#include <osgEarth/SimplexNoise>
#include <osgEarth/ImageUtils>
#include <algorithm>

#define POW2(x) ((double)(x==0 ? 1 : (2 << (x-1))))

using namespace osgEarth;

const SimplexNoise::Grad SimplexNoise::grad3[12] = {
    Grad(1, 1, 0), Grad(-1, 1, 0), Grad(1, -1, 0), Grad(-1, -1, 0),
    Grad(1, 0, 1), Grad(-1, 0, 1), Grad(1, 0, -1), Grad(-1, 0, -1),
    Grad(0, 1, 1), Grad(0, -1, 1), Grad(0, 1, -1), Grad(0, -1, -1)
};

const SimplexNoise::Grad SimplexNoise::grad4[32] =  {
    Grad(0, 1, 1, 1), Grad(0, 1, 1, -1), Grad(0, 1, -1, 1), Grad(0, 1, -1, -1),
    Grad(0, -1, 1, 1), Grad(0, -1, 1, -1), Grad(0, -1, -1, 1), Grad(0, -1, -1, -1),
    Grad(1, 0, 1, 1), Grad(1, 0, 1, -1), Grad(1, 0, -1, 1), Grad(1, 0, -1, -1),
    Grad(-1, 0, 1, 1), Grad(-1, 0, 1, -1), Grad(-1, 0, -1, 1), Grad(-1, 0, -1, -1),
    Grad(1, 1, 0, 1), Grad(1, 1, 0, -1), Grad(1, -1, 0, 1), Grad(1, -1, 0, -1),
    Grad(-1, 1, 0, 1), Grad(-1, 1, 0, -1), Grad(-1, -1, 0, 1), Grad(-1, -1, 0, -1),
    Grad(1, 1, 1, 0), Grad(1, 1, -1, 0), Grad(1, -1, 1, 0), Grad(1, -1, -1, 0),
    Grad(-1, 1, 1, 0), Grad(-1, 1, -1, 0), Grad(-1, -1, 1, 0), Grad(-1, -1, -1, 0)
};

const unsigned char SimplexNoise::perm[512] = {
    151, 160, 137, 91, 90, 15,
    131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23,
    190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33,
    88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166,
    77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244,
    102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196,
    135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123,
    5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42,
    223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9,
    129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228,
    251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107,
    49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
    138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180,

    151, 160, 137, 91, 90, 15,
    131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23,
    190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33,
    88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166,
    77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244,
    102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196,
    135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123,
    5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42,
    223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9,
    129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228,
    251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107,
    49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
    138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180
};

// Skewing and unskewing factors for 2, 3, and 4 dimensions
double const SimplexNoise::F2 = 0.5*(sqrt(3.0)-1.0);
double const SimplexNoise::G2 = (3.0-sqrt(3.0))/6.0;
double const SimplexNoise::F3 = 1.0/3.0;
double const SimplexNoise::G3 = 1.0/6.0;
double const SimplexNoise::F4 = (sqrt(5.0)-1.0)/4.0;
double const SimplexNoise::G4 = (5.0-sqrt(5.0))/20.0;

SimplexNoise::Grad::Grad(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = 1.0;
}

SimplexNoise::Grad::Grad(double x, double y, double z, double w)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

// This method is a *lot* faster than using (int)Math.floor(x)
int SimplexNoise::FastFloor(double x)
{
    int xi = (int)x;
    return x<xi ? xi-1 : xi;
}

double SimplexNoise::Dot(Grad const & g, double x, double y)
{
    return g.x*x + g.y*y;
}

double SimplexNoise::Dot(Grad const & g, double x, double y, double z)
{
    return g.x*x + g.y*y + g.z*z;
}

double SimplexNoise::Dot(Grad const & g, double x, double y, double z, double w)
{
    return g.x*x + g.y*y + g.z*z + g.w*w;
}


const double   SimplexNoise::DefaultFrequency    =  1.0;
const double   SimplexNoise::DefaultPersistence  =  0.5;
const double   SimplexNoise::DefaultLacunarity   =  2.0;
const double   SimplexNoise::DefaultRangeLow     = -1.0;
const double   SimplexNoise::DefaultRangeHigh    =  1.0;
const unsigned SimplexNoise::DefaultOctaves      =  10;
const bool     SimplexNoise::DefaultNormalize    =  false;


SimplexNoise::SimplexNoise() :
_octaves   ( DefaultOctaves ),
_freq      ( DefaultFrequency ),
_pers      ( DefaultPersistence ),
_lacunarity( DefaultLacunarity ),
_low       ( DefaultRangeLow ),
_high      ( DefaultRangeHigh ),
_normalize ( DefaultNormalize )
{
    for(unsigned int i=0; i<512; i++)
    {
        permMod12[i] = (unsigned char)(perm[i] % 12);
    }
}

SimplexNoise::SimplexNoise(const SimplexNoise& rhs) :
_octaves   ( rhs._octaves ),
_freq      ( rhs._freq ),
_pers      ( rhs._pers ),
_lacunarity( rhs._lacunarity ),
_low       ( rhs._low ),
_high      ( rhs._high ),
_normalize ( rhs._normalize )
{
    for(unsigned int i=0; i<512; i++)
    {
        permMod12[i] = (unsigned char)(perm[i] % 12);
    }
}

double SimplexNoise::getTiledValue(double x, double y) const
{
    const double TwoPI = 2.0 * osg::PI;
    double freq = _freq;
    double o = osg::maximum(1u, _octaves);
    double amp = 1.0;
    double maxamp = 0.0;
    double n = 0.0;

    // trick to create tiled noise (2 ortho circles)
    // http://www.gamedev.net/blog/33/entry-2138456-seamless-noise/
    
    double nx = cos(x*TwoPI)/TwoPI;
    double ny = cos(y*TwoPI)/TwoPI;
    double nz = sin(x*TwoPI)/TwoPI;
    double nw = sin(y*TwoPI)/TwoPI;

    for(unsigned i=0; i<o; ++i)
    {
        n += Noise(nx*freq, ny*freq, nz*freq, nw*freq) * amp;
        maxamp += amp;
        amp *= _pers;
        freq *= _lacunarity;
    }

    if ( _normalize )
    {
        n /= maxamp;
        n = n * (_high-_low)/2.0 + (_high+_low)/2.0;
    }
    return n;
}

double SimplexNoise::getTiledValueWithTurbulence(double x, double y, double F) const
{
    const double TwoPI = 2.0 * osg::PI;
    double freq = _freq;
    double o = osg::maximum(1u, _octaves);
    double amp = 1.0;
    double maxamp = 0.0;
    double n = 0.0;

    // trick to create tiled noise (2 ortho circles)
    // http://www.gamedev.net/blog/33/entry-2138456-seamless-noise/

    double nx = cos(x*TwoPI)/TwoPI;
    double ny = cos(y*TwoPI)/TwoPI;
    double nz = sin(x*TwoPI)/TwoPI;
    double nw = sin(y*TwoPI)/TwoPI;

    for(unsigned i=0; i<o; ++i)
    {
        float t = -0.5f, FF = F;
        for(; FF<127.0f; FF*=2.0)
            t += fabs(getValue(nx*freq/F, ny*freq/F, nz*freq/F, nw*freq/F));
        n += t * amp;
        maxamp += amp;
        amp *= _pers;
        freq *= _lacunarity;
    }

    if ( _normalize )
    {
        n /= maxamp;
        n = n * (_high-_low)/2.0 + (_high+_low)/2.0;
    }
    return n;
}

double SimplexNoise::getValue(double xin, double yin) const
{
    double freq = _freq;
    double o = osg::maximum(1u, _octaves);
    double amp = 1.0;
    double maxamp = 0.0;
    double n = 0.0;

    for(unsigned i=0; i<o; ++i)
    {
        n += Noise(xin*freq, yin*freq) * amp;
        maxamp += amp;
        amp *= _pers;
        freq *= _lacunarity;
    }
    if ( _normalize )
    {
        n /= maxamp;
        n = n * (_high-_low)/2.0 + (_high+_low)/2.0;
    }
    return n;
}

double SimplexNoise::getValue(double xin, double yin, double zin) const
{
    double freq = _freq;
    double o = osg::maximum(1u, _octaves);
    double amp = 1.0;
    double maxamp = 0.0;
    double n = 0.0;

    for(unsigned i=0; i<o; ++i)
    {
        n += Noise(xin*freq, yin*freq, zin*freq) * amp;
        maxamp += amp;
        amp *= _pers;
        freq *= _lacunarity;
    }
    
    if ( _normalize )
    {
        n /= maxamp;
        n = n * (_high-_low)/2.0 + (_high+_low)/2.0;
    }
    return n;
}

double SimplexNoise::getValue(double xin, double yin, double zin, double win) const
{
    double freq = _freq;
    double o = osg::maximum(1u, _octaves);
    double amp = 1.0;
    double maxamp = 0.0;
    double n = 0.0;

    for(unsigned i=0; i<o; ++i)
    {
        n += Noise(xin*freq, yin*freq, zin*freq, win*freq) * amp;
        maxamp += amp;
        amp *= _pers;
        freq *= _lacunarity;
    }
    
    if ( _normalize )
    {
        n /= maxamp;
        n = n * (_high-_low)/2.0 + (_high+_low)/2.0;
    }
    return n;
}


// 2D simplex noise
double SimplexNoise::Noise(double xin, double yin) const
{
    double n0, n1, n2; // Noise contributions from the three corners
    // Skew the input space to determine which simplex cell we're in
    double s = (xin+yin)*F2; // Hairy factor for 2D
    int i = FastFloor(xin+s);
    int j = FastFloor(yin+s);
    double t = (i+j)*G2;
    double X0 = i-t; // Unskew the cell origin back to (x,y) space
    double Y0 = j-t;
    double x0 = xin-X0; // The x,y distances from the cell origin
    double y0 = yin-Y0;
    // For the 2D case, the simplex shape is an equilateral triangle.
    // Determine which simplex we are in.
    int i1, j1; // Offsets for second (middle) corner of simplex in (i,j) coords
    if(x0>y0)
    {
        i1=1;    // lower triangle, XY order: (0,0)->(1,0)->(1,1)
        j1=0;
    } else
    {
        i1=0;    // upper triangle, YX order: (0,0)->(0,1)->(1,1)
        j1=1;
    }
    // A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
    // a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
    // c = (3-sqrt(3))/6
    double x1 = x0 - i1 + G2; // Offsets for middle corner in (x,y) unskewed coords
    double y1 = y0 - j1 + G2;
    double x2 = x0 - 1.0 + 2.0 * G2; // Offsets for last corner in (x,y) unskewed coords
    double y2 = y0 - 1.0 + 2.0 * G2;
    // Work out the hashed gradient indices of the three simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int gi0 = permMod12[ii+perm[jj]];
    int gi1 = permMod12[ii+i1+perm[jj+j1]];
    int gi2 = permMod12[ii+1+perm[jj+1]];
    // Calculate the contribution from the three corners
    double t0 = 0.5 - x0*x0-y0*y0;
    if(t0<0) n0 = 0.0;
    else
    {
        t0 *= t0;
        n0 = t0 * t0 * Dot(grad3[gi0], x0, y0);    // (x,y) of grad3 used for 2D gradient
    }
    double t1 = 0.5 - x1*x1-y1*y1;
    if(t1<0) n1 = 0.0;
    else
    {
        t1 *= t1;
        n1 = t1 * t1 * Dot(grad3[gi1], x1, y1);
    }
    double t2 = 0.5 - x2*x2-y2*y2;
    if(t2<0) n2 = 0.0;
    else
    {
        t2 *= t2;
        n2 = t2 * t2 * Dot(grad3[gi2], x2, y2);
    }
    // Add contributions from each corner to get the final noise value.
    // The result is scaled to return values in the interval [-1,1].
    return 70.0 * (n0 + n1 + n2);
}


// 3D simplex noise
double SimplexNoise::Noise(double xin, double yin, double zin) const
{
    double n0, n1, n2, n3; // Noise contributions from the four corners
    // Skew the input space to determine which simplex cell we're in
    double s = (xin+yin+zin)*F3; // Very nice and simple skew factor for 3D
    int i = FastFloor(xin+s);
    int j = FastFloor(yin+s);
    int k = FastFloor(zin+s);
    double t = (i+j+k)*G3;
    double X0 = i-t; // Unskew the cell origin back to (x,y,z) space
    double Y0 = j-t;
    double Z0 = k-t;
    double x0 = xin-X0; // The x,y,z distances from the cell origin
    double y0 = yin-Y0;
    double z0 = zin-Z0;
    // For the 3D case, the simplex shape is a slightly irregular tetrahedron.
    // Determine which simplex we are in.
    int i1, j1, k1; // Offsets for second corner of simplex in (i,j,k) coords
    int i2, j2, k2; // Offsets for third corner of simplex in (i,j,k) coords
    if(x0>=y0)
    {
        if(y0>=z0)
        {
            i1=1;    // X Y Z order
            j1=0;
            k1=0;
            i2=1;
            j2=1;
            k2=0;
        } else if(x0>=z0)
        {
            i1=1;    // X Z Y order
            j1=0;
            k1=0;
            i2=1;
            j2=0;
            k2=1;
        } else
        {
            i1=0;    // Z X Y order
            j1=0;
            k1=1;
            i2=1;
            j2=0;
            k2=1;
        }
    } else     // x0<y0
    {
        if(y0<z0)
        {
            i1=0;    // Z Y X order
            j1=0;
            k1=1;
            i2=0;
            j2=1;
            k2=1;
        } else if(x0<z0)
        {
            i1=0;    // Y Z X order
            j1=1;
            k1=0;
            i2=0;
            j2=1;
            k2=1;
        } else
        {
            i1=0;    // Y X Z order
            j1=1;
            k1=0;
            i2=1;
            j2=1;
            k2=0;
        }
    }
    // A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
    // a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z), and
    // a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z), where
    // c = 1/6.
    double x1 = x0 - i1 + G3; // Offsets for second corner in (x,y,z) coords
    double y1 = y0 - j1 + G3;
    double z1 = z0 - k1 + G3;
    double x2 = x0 - i2 + 2.0*G3; // Offsets for third corner in (x,y,z) coords
    double y2 = y0 - j2 + 2.0*G3;
    double z2 = z0 - k2 + 2.0*G3;
    double x3 = x0 - 1.0 + 3.0*G3; // Offsets for last corner in (x,y,z) coords
    double y3 = y0 - 1.0 + 3.0*G3;
    double z3 = z0 - 1.0 + 3.0*G3;
    // Work out the hashed gradient indices of the four simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;
    int gi0 = permMod12[ii+perm[jj+perm[kk]]];
    int gi1 = permMod12[ii+i1+perm[jj+j1+perm[kk+k1]]];
    int gi2 = permMod12[ii+i2+perm[jj+j2+perm[kk+k2]]];
    int gi3 = permMod12[ii+1+perm[jj+1+perm[kk+1]]];
    // Calculate the contribution from the four corners
    double t0 = 0.6 - x0*x0 - y0*y0 - z0*z0;
    if(t0<0) n0 = 0.0;
    else
    {
        t0 *= t0;
        n0 = t0 * t0 * Dot(grad3[gi0], x0, y0, z0);
    }
    double t1 = 0.6 - x1*x1 - y1*y1 - z1*z1;
    if(t1<0) n1 = 0.0;
    else
    {
        t1 *= t1;
        n1 = t1 * t1 * Dot(grad3[gi1], x1, y1, z1);
    }
    double t2 = 0.6 - x2*x2 - y2*y2 - z2*z2;
    if(t2<0) n2 = 0.0;
    else
    {
        t2 *= t2;
        n2 = t2 * t2 * Dot(grad3[gi2], x2, y2, z2);
    }
    double t3 = 0.6 - x3*x3 - y3*y3 - z3*z3;
    if(t3<0) n3 = 0.0;
    else
    {
        t3 *= t3;
        n3 = t3 * t3 * Dot(grad3[gi3], x3, y3, z3);
    }
    // Add contributions from each corner to get the final noise value.
    // The result is scaled to stay just inside [-1,1]
    return 32.0*(n0 + n1 + n2 + n3);
}

double SimplexNoise::Noise(double x, double y, double z, double w) const
{
    double n0, n1, n2, n3, n4; // Noise contributions from the five corners
    // Skew the (x,y,z,w) space to determine which cell of 24 simplices we're in
    double s = (x + y + z + w) * F4; // Factor for 4D skewing
    int i = FastFloor(x + s);
    int j = FastFloor(y + s);
    int k = FastFloor(z + s);
    int l = FastFloor(w + s);
    double t = (i + j + k + l) * G4; // Factor for 4D unskewing
    double X0 = i - t; // Unskew the cell origin back to (x,y,z,w) space
    double Y0 = j - t;
    double Z0 = k - t;
    double W0 = l - t;
    double x0 = x - X0;    // The x,y,z,w distances from the cell origin
    double y0 = y - Y0;
    double z0 = z - Z0;
    double w0 = w - W0;
    // For the 4D case, the simplex is a 4D shape I won't even try to describe.
    // To find out which of the 24 possible simplices we're in, we need to
    // determine the magnitude ordering of x0, y0, z0 and w0.
    // Six pair-wise comparisons are performed between each possible pair
    // of the four coordinates, and the results are used to rank the numbers.
    int rankx = 0;
    int ranky = 0;
    int rankz = 0;
    int rankw = 0;
    if(x0 > y0)
    {
        rankx++;
    } else
    {
        ranky++;
    }

    if(x0 > z0)
    {
        rankx++;
    } else
    {
        rankz++;
    }

    if(x0 > w0)
    {
        rankx++;
    } else
    {
        rankw++;
    }

    if(y0 > z0)
    {
        ranky++;
    } else
    {
        rankz++;
    }

    if(y0 > w0)
    {
        ranky++;
    } else
    {
        rankw++;
    }

    if(z0 > w0)
    {
        rankz++;
    } else
    {
        rankw++;
    }

    int i1, j1, k1, l1; // The integer offsets for the second simplex corner
    int i2, j2, k2, l2; // The integer offsets for the third simplex corner
    int i3, j3, k3, l3; // The integer offsets for the fourth simplex corner
    // simplex[c] is a 4-vector with the numbers 0, 1, 2 and 3 in some order.
    // Many values of c will never occur, since e.g. x>y>z>w makes x<z, y<w and x<w
    // impossible. Only the 24 indices which have non-zero entries make any sense.
    // We use a thresholding to set the coordinates in turn from the largest magnitude.
    // Rank 3 denotes the largest coordinate.
    i1 = rankx >= 3 ? 1 : 0;
    j1 = ranky >= 3 ? 1 : 0;
    k1 = rankz >= 3 ? 1 : 0;
    l1 = rankw >= 3 ? 1 : 0;
    // Rank 2 denotes the second largest coordinate.
    i2 = rankx >= 2 ? 1 : 0;
    j2 = ranky >= 2 ? 1 : 0;
    k2 = rankz >= 2 ? 1 : 0;
    l2 = rankw >= 2 ? 1 : 0;
    // Rank 1 denotes the second smallest coordinate.
    i3 = rankx >= 1 ? 1 : 0;
    j3 = ranky >= 1 ? 1 : 0;
    k3 = rankz >= 1 ? 1 : 0;
    l3 = rankw >= 1 ? 1 : 0;
    // The fifth corner has all coordinate offsets = 1, so no need to compute that.
    double x1 = x0 - i1 + G4; // Offsets for second corner in (x,y,z,w) coords
    double y1 = y0 - j1 + G4;
    double z1 = z0 - k1 + G4;
    double w1 = w0 - l1 + G4;
    double x2 = x0 - i2 + 2.0*G4; // Offsets for third corner in (x,y,z,w) coords
    double y2 = y0 - j2 + 2.0*G4;
    double z2 = z0 - k2 + 2.0*G4;
    double w2 = w0 - l2 + 2.0*G4;
    double x3 = x0 - i3 + 3.0*G4; // Offsets for fourth corner in (x,y,z,w) coords
    double y3 = y0 - j3 + 3.0*G4;
    double z3 = z0 - k3 + 3.0*G4;
    double w3 = w0 - l3 + 3.0*G4;
    double x4 = x0 - 1.0 + 4.0*G4; // Offsets for last corner in (x,y,z,w) coords
    double y4 = y0 - 1.0 + 4.0*G4;
    double z4 = z0 - 1.0 + 4.0*G4;
    double w4 = w0 - 1.0 + 4.0*G4;
    // Work out the hashed gradient indices of the five simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;
    int ll = l & 255;
    int gi0 = perm[ii+perm[jj+perm[kk+perm[ll]]]] % 32;
    int gi1 = perm[ii+i1+perm[jj+j1+perm[kk+k1+perm[ll+l1]]]] % 32;
    int gi2 = perm[ii+i2+perm[jj+j2+perm[kk+k2+perm[ll+l2]]]] % 32;
    int gi3 = perm[ii+i3+perm[jj+j3+perm[kk+k3+perm[ll+l3]]]] % 32;
    int gi4 = perm[ii+1+perm[jj+1+perm[kk+1+perm[ll+1]]]] % 32;
    // Calculate the contribution from the five corners
    double t0 = 0.6 - x0*x0 - y0*y0 - z0*z0 - w0*w0;
    if(t0<0) n0 = 0.0;
    else {
        t0 *= t0;
        n0 = t0 * t0 * Dot(grad4[gi0], x0, y0, z0, w0);
    }
    double t1 = 0.6 - x1*x1 - y1*y1 - z1*z1 - w1*w1;
    if(t1<0) n1 = 0.0;
    else {
        t1 *= t1;
        n1 = t1 * t1 * Dot(grad4[gi1], x1, y1, z1, w1);
    }
    double t2 = 0.6 - x2*x2 - y2*y2 - z2*z2 - w2*w2;
    if(t2<0) n2 = 0.0;
    else {
        t2 *= t2;
        n2 = t2 * t2 * Dot(grad4[gi2], x2, y2, z2, w2);
    }
    double t3 = 0.6 - x3*x3 - y3*y3 - z3*z3 - w3*w3;
    if(t3<0) n3 = 0.0;
    else {
        t3 *= t3;
        n3 = t3 * t3 * Dot(grad4[gi3], x3, y3, z3, w3);
    }
    double t4 = 0.6 - x4*x4 - y4*y4 - z4*z4 - w4*w4;
    if(t4<0) n4 = 0.0;
    else {
        t4 *= t4;
        n4 = t4 * t4 * Dot(grad4[gi4], x4, y4, z4, w4);
    }
    // Sum up and scale the result to cover the range [-1,1]
    return 27.0 * (n0 + n1 + n2 + n3 + n4);
}

osg::Image*
SimplexNoise::createSeamlessImage(unsigned dim) const
{
    if (dim == 0) return 0L;

    // Copy this generator and set a [0..1] range.
    SimplexNoise noise(*this);
    noise.setRange(0.0, 1.0);
    noise.setNormalize(true);

    osg::Image* image = new osg::Image();
    image->allocateImage(dim, dim, 1, GL_RED, GL_UNSIGNED_BYTE);
    ImageUtils::PixelWriter write(image);

    float minN =  FLT_MAX;
    float maxN = -FLT_MAX;

    // populate the image, tracking the min and max noise readings:
    osg::Vec4f value;
    for (unsigned s = 0; s < dim; ++s)
    {
        double u = (double)s / (double)dim;
        for (unsigned t = 0; t < dim; ++t)
        {
            double v = (double)t / (double)dim;
            value.r() = noise.getTiledValue(u, v);
            minN = osg::minimum(minN, value.r());
            maxN = osg::maximum(maxN, value.r());
            write(value, s, t);
        }        
    }

    if (getNormalize())
    {
        // Histogram stretch to [0..1]
        float scale = 1.0/(maxN-minN);
        float bias = -minN;

        OE_INFO << "minN=" << minN << "; maxN=" << maxN << "; scale=" << scale << "; bias=" << bias << "\n";

        ImageUtils::PixelReader read(image);
        read.setBilinear(false);

        for (unsigned s = 0; s < dim; ++s)
        {
            for (unsigned t = 0; t < dim; ++t)
            {
                value = read(s, t);
                value.r() = (value.r()+bias)*scale;
                write(value, s, t);
            }
        }
    }

    return image;
}
