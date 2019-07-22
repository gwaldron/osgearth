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

// "License": Public Domain
// I, Mathias Panzenböck, place this file hereby into the public domain. Use it at your own risk for whatever you like.
// In case there are jurisdictions that don't support putting things in the public domain you can also consider it to
// be "dual licensed" under the BSD, MIT and Apache licenses, if you want to. This code is trivial anyway. Consider it
// an example on how to get the endian conversion functions on different platforms.

#ifndef OSGEARTH_PORTABLE_ENDIAN_H__
#define OSGEARTH_PORTABLE_ENDIAN_H__

#if (defined(_WIN16) || defined(_WIN32) || defined(_WIN64)) && !defined(__WINDOWS__)

#   include <stdint.h>

#	define __WINDOWS__

#endif

#if defined(__linux__) || defined(__CYGWIN__)

#	include <endian.h>

#elif defined(__APPLE__)

#	include <libkern/OSByteOrder.h>

#	define htobe16(x) OSSwapHostToBigInt16(x)
#	define htole16(x) OSSwapHostToLittleInt16(x)
#	define be16toh(x) OSSwapBigToHostInt16(x)
#	define le16toh(x) OSSwapLittleToHostInt16(x)
 
#	define htobe32(x) OSSwapHostToBigInt32(x)
#	define htole32(x) OSSwapHostToLittleInt32(x)
#	define be32toh(x) OSSwapBigToHostInt32(x)
#	define le32toh(x) OSSwapLittleToHostInt32(x)
 
#	define htobe64(x) OSSwapHostToBigInt64(x)
#	define htole64(x) OSSwapHostToLittleInt64(x)
#	define be64toh(x) OSSwapBigToHostInt64(x)
#	define le64toh(x) OSSwapLittleToHostInt64(x)

#	define __BYTE_ORDER    BYTE_ORDER
#	define __BIG_ENDIAN    BIG_ENDIAN
#	define __LITTLE_ENDIAN LITTLE_ENDIAN
#	define __PDP_ENDIAN    PDP_ENDIAN

#elif defined(__OpenBSD__)

#	include <sys/endian.h>

#elif defined(__NetBSD__) || defined(__FreeBSD__) || defined(__DragonFly__)

#	include <sys/endian.h>

#	define be16toh(x) betoh16(x)
#	define le16toh(x) letoh16(x)

#	define be32toh(x) betoh32(x)
#	define le32toh(x) letoh32(x)

#	define be64toh(x) betoh64(x)
#	define le64toh(x) letoh64(x)

#elif defined(__WINDOWS__)

#	include <windows.h>

#	if BYTE_ORDER == LITTLE_ENDIAN

#		define htobe16(x) _byteswap_ushort(x)
#		define htole16(x) (x)
#		define be16toh(x) _byteswap_ushort(x)
#		define le16toh(x) (x)
 
#		define htobe32(x) _byteswap_ulong(x)
#		define htole32(x) (x)
#		define be32toh(x) _byteswap_ulong(x)
#		define le32toh(x) (x)

#		if defined(htonll) && defined(ntohll)
#			define htobe64(x) htonll(x)
#			define htole64(x) (x)
#			define be64toh(x) ntohll(x)
#			define le64toh(x) (x)
#		else
#			define htobe64(x) _byteswap_uint64(x)
#			define htole64(x) (x)
#			define be64toh(x) _byteswap_uint64(x)
#			define le64toh(x) (x)
#		endif

#	elif BYTE_ORDER == BIG_ENDIAN

		/* that would be xbox 360 */
#		define htobe16(x) (x)
#		define htole16(x) __builtin_bswap16(x)
#		define be16toh(x) (x)
#		define le16toh(x) __builtin_bswap16(x)
 
#		define htobe32(x) (x)
#		define htole32(x) __builtin_bswap32(x)
#		define be32toh(x) (x)
#		define le32toh(x) __builtin_bswap32(x)
 
#		define htobe64(x) (x)
#		define htole64(x) __builtin_bswap64(x)
#		define be64toh(x) (x)
#		define le64toh(x) __builtin_bswap64(x)

#	else

#		error byte order not supported

#	endif

#	define __BYTE_ORDER    BYTE_ORDER
#	define __BIG_ENDIAN    BIG_ENDIAN
#	define __LITTLE_ENDIAN LITTLE_ENDIAN
#	define __PDP_ENDIAN    PDP_ENDIAN

#else

//#	error platform not supported
#   define htobe16(X) X
#   define htobe32(X) X
#   define htobe64(X) X
#   define be16toh(X) X
#   define be32toh(X) X
#   define be64toh(X) X

#endif

#define OE_ENCODE_SHORT(X)  htobe16(X)
#define OE_ENCODE_INT(X)    htobe32(X)
#define OE_ENCODE_LONG(X)   htobe32(X)

inline uint32_t OE_ENCODE_FLOAT(float x) {
    return htobe32(*(uint32_t*)(&x));
}
inline uint64_t OE_ENCODE_DOUBLE(double x) {
    return htobe64(*(uint64_t*)(&x));
}

#define OE_DECODE_SHORT(X)  be16toh(X)
#define OE_DECODE_INT(X)    be32toh(X)
#define OE_DECODE_LONG(X)   be32toh(X)

inline float OE_DECODE_FLOAT(uint32_t x) {
    uint32_t temp = be32toh(x);
    float r = *(float*)(&temp);
    return r;
}
inline double OE_DECODE_DOUBLE(uint64_t x) {
    uint64_t temp = be64toh(x);
    double r = *(double*)(&temp);
    return r;
}

namespace osgEarth
{
    inline void byteSwapInPlace(uint32_t x) {
        unsigned char* p = (unsigned char*)&x;
        std::swap(p[0], p[3]);
        std::swap(p[1], p[2]);
    }
    inline void byteSwapInPlace(uint16_t x) {
        unsigned char* p = (unsigned char*)&x;
        std::swap(p[0], p[1]);
    }
}

#if __BYTE_ORDER == __LITTLE_ENDIAN
    #define OE_IS_LITTLE_ENDIAN
#else
    #define OE_IS_BIG_ENDIAN
#endif

#endif // OSGEARTH_PORTABLE_ENDIAN_H__
