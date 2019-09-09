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
#ifndef OSGEARTH_TEXTURE_COMPOSITOR_H
#define OSGEARTH_TEXTURE_COMPOSITOR_H 1

#include <osgEarth/Common>
#include <osgEarth/ThreadingUtils>
#include <osg/observer_ptr>

namespace osgEarth
{
    class Layer;
    class TextureImageUnitReservation;

    /**
     * Utility class that manages GPU resources for a terrain engine
     */
    class OSGEARTH_EXPORT TerrainResources : public osg::Referenced
    {
    public:
        /**
         * Constructs a new resource tracker object.
         */
        TerrainResources();

        /** dtor */
        virtual ~TerrainResources() { }

        /**
         * Requests a texture image unit that is not in use, and marks is as reserved.
         * You must release the reserved texture image unit by calling releaseTextureImageUnit().
         * @param out_unit  Reserved unit is written to this variable.
         * @param requestor Optional requestor string (for information purposes only)
         */
        bool reserveTextureImageUnit(
            int&        out_unit,
            const char* requestor =0L );
        
        /**
         * Requests a texture image unit that is not in use, and marks is as reserved.
         * The reserveration is released when the "reservation" object goes out of
         * scope and destructs.
         * @param reservation Reserved unit is written to this variable.
         * @param requestor   Optional requestor string (for information purposes only)
         */
        bool reserveTextureImageUnit(
            TextureImageUnitReservation& reservation,
            const char* requestor =0L);
        
        /**
         * Requests a texture image unit that is not in use, and marks is as reserved
         * for the specific Layer. The unit will still be available for other layers
         * (but not globally).
         * The reserveration is released when the "reservation" object goes out of
         * scope and destructs.
         * @param reservation Reserved unit is written to this variable.
         * @param requestor   Optional requestor string (for information purposes only)
         */
        bool reserveTextureImageUnitForLayer(
            TextureImageUnitReservation& reservation,
            const Layer* layer,
            const char* requestor =0L);

        /**
         * Releases a reserved texture image unit previously returned by reserveTextureImageUnit.
         */
        void releaseTextureImageUnit(int unit);

        /**
         * Releases a reserved texture image unit previously reserved by calling
         * reserveTextureImageUnit(int, Layer). 
         */
        void releaseTextureImageUnit(int unit, const Layer* layer);

        /**
         * Marks a specific texture image as reserved. Only call this if your application
         * is known to use a particular texture image unit and you don't want osgEarth
         * touching it.
         *
         * Returns true upon success, and false if the unit has already been reserved.
         */
        bool setTextureImageUnitOffLimits(int unit);

    private:
        Threading::Mutex _reservedUnitsMutex;

        typedef std::set<int> ReservedUnits;
        ReservedUnits _globallyReservedUnits;

        typedef std::map<const Layer*, ReservedUnits> PerLayerReservedUnits;
        PerLayerReservedUnits _perLayerReservedUnits;
    };

    class OSGEARTH_EXPORT TextureImageUnitReservation
    {
    public:
        TextureImageUnitReservation();
        virtual ~TextureImageUnitReservation();
        int unit() const { return _unit; }
        bool valid() const { return _unit >= 0; }
    private:
        int _unit;
        osg::observer_ptr<TerrainResources> _res;
        const Layer* _layer; // only need the raw ptr.
        friend class TerrainResources;
    };
}

#endif // OSGEARTH_TEXTURE_COMPOSITOR_H
