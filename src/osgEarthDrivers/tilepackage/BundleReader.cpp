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
#include "BundleReader"

BundleReader::BundleReader(const std::string& bundleFile, unsigned int bundleSize) :
    _bundleFile(bundleFile),
    _lod(0),
    _colOffset(0),
    _rowOffset(0),
    _bundleSize(bundleSize)
{
    init();
}

void BundleReader::init()
{

    std::string base = osgDB::getNameLessExtension(_bundleFile);
    _indexFile = base + ".bundlx";

    // Open the bundle
    _in.open(_bundleFile.c_str(), std::ofstream::binary);

    // Read the index
    readIndex(_indexFile, _index);

    std::string baseName = osgDB::getSimpleFileName(base);

    _rowOffset = hexFromString(baseName.substr(1, 4));
    _colOffset = hexFromString(baseName.substr(6, 4));

    std::string path = osgDB::getFilePath(_bundleFile);

    std::string levelDir = osgDB::getSimpleFileName(path);
    _lod = as<unsigned int>(levelDir.substr(1, 2), 0);
}

/**
* Reads the index of a bundle file.
*/
void BundleReader::readIndex(const std::string& filename, std::vector<int>& index)
{
    std::ifstream input(filename.c_str(), std::ifstream::binary);
    char header[INDEX_HEADER_SIZE];
    input.read(header, INDEX_HEADER_SIZE);
    while (input.good()) {
        std::vector<char> buffer;
        buffer.resize(5);
        if (input.read(&buffer[0], INDEX_SIZE))
        {
            int offset = computeOffset(buffer);
            index.push_back(offset);
        }
    }
}

osg::Image* BundleReader::readImage(const TileKey& key)
{
    // Figure out the index for the tilekey
    unsigned int row = key.getTileX() - _colOffset;
    unsigned int i = key.getTileY() - _rowOffset + (row * _bundleSize);
    return readImage(i);
}

osg::Image* BundleReader::readImage(unsigned int index)
{
    if (index < 0 || index > _index.size()) return 0;

    _in.seekg(_index[index], std::ios::beg);
    std::vector<char> sizeBuffer;
    sizeBuffer.resize(4);
    _in.read(&sizeBuffer[0], 4);
    int size = computeOffset(sizeBuffer);
    if (size > 0)
    {
        std::string image;
        image.resize(size);
        _in.read(&image[0], size);
        std::stringstream ss(image);
        return ImageUtils::readStream(ss, 0);
    }

    return 0;
}
