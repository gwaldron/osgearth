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
#ifndef OSGEARTH_TILE_RASTERIZER_H
#define OSGEARTH_TILE_RASTERIZER_H 1

#include <osgEarth/Common>
#include <osgEarth/GeoData>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/TileKey>
#include <osg/Camera>
#include <osg/BufferObject>
#include <osg/Texture2D>
#include <queue>

namespace osgEarth
{
    // placeholder - might go away -gw
    class OSGEARTH_EXPORT GeoNode : public osg::Object
    {
    public:
        META_Object(osgEarth, GeoNode);
        GeoNode() { }
        GeoNode(const GeoNode& rhs, const osg::CopyOp& copy) { }
        GeoNode(osg::Node* node, const GeoExtent& extent) : _node(node), _extent(extent) { }

        osg::ref_ptr<osg::Node> _node;
        GeoExtent _extent;
    };

    /**
     * Node that will render node graphs to textures, one at a time.
     */
    class OSGEARTH_EXPORT TileRasterizer : public osg::Camera
    {
    public:
        /** Construct a new tile rasterizer camera */
        TileRasterizer();

        /**
         * Schedule a rasterization to an osg::Image.
         * @param node Node to render to the image
         * @param size of the target image (both dimensions)
         * @param extent geospatial extent of the node to render.
         * @return Future image - blocks on .get()
         */
        Threading::Future<osg::Image> push(osg::Node* node, unsigned size, const GeoExtent& extent);

        /**
         * Schedule a rasterization to a texture.
         * @param node    Node to render to the texture
         * @param texture Texture to which to render the node
         * @param extent  Geographic extent of the output texture
         */
        void push(osg::Node* node, osg::Texture* texture, const GeoExtent& extent);

    public: // osg::Node

        void accept(osg::NodeVisitor&);

        void traverse(osg::NodeVisitor&);

    private:
        virtual ~TileRasterizer();

        // internal - image with custom readback
        struct ReadbackImage : public osg::Image
        {
            osg::RenderInfo* _ri;
            void readPixels(int x, int y, int width, int height, GLenum pixelFormat, GLenum type, int packing);
        };

        // internal - scheduled rasterization job
        struct Job
        {
            osg::ref_ptr<osg::Node> _node;
            GeoExtent _extent;
            osg::ref_ptr<osg::Texture> _texture;
            osg::ref_ptr<ReadbackImage> _image;
            osg::ref_ptr<osg::PixelBufferObject> _imagePBO;
            Threading::Promise<osg::Image> _imagePromise;
            GLuint _fragmentsWritten;
        };

        mutable Threading::Mutex _mutex;
        typedef std::queue<Job> JobQueue;
        mutable JobQueue _pendingJobs;  // queue for jobs waiting to render
        mutable JobQueue _readbackJobs; // queue for jobs waiting for rtt/glReadPixels to finish
        mutable JobQueue _finishedJobs; // queue for jobs waiting for the promise to resolve

        //osg::ref_ptr<osg::Uniform> _distortionU;
        mutable osg::buffered_object<GLuint> _samplesQuery;

    public: // internal

        void preDraw(osg::RenderInfo&) const;
        void postDraw(osg::RenderInfo&) const;
    };

} // namespace osgEarth

#endif // OSGEARTH_TILE_RASTERIZER_H
