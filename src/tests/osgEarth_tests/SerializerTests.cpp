#include <osgEarth/catch.hpp>

#include <osgDB/Registry>

#include <osgEarth/Registry>
#include <osgEarth/MapNode>

using namespace osgEarth;

void readAndWrite(osgDB::ReaderWriter* readerwriter, std::string filename)
{
    SECTION(std::string("Deserialize and re-serialize: ") + filename) {

	std::ifstream file(filename);
	REQUIRE(file.is_open());

	std::stringstream read_stream;
	read_stream << file.rdbuf();
	file.close();

	osgDB::ReaderWriter::ReadResult rr = readerwriter->readNode(read_stream, nullptr);
	REQUIRE(rr.success());

	osg::ref_ptr<osg::Node> node = rr.getNode();
	REQUIRE(node.valid());

	osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( node );
	REQUIRE(mapNode != nullptr);

	std::stringstream write_stream;
	osg::ref_ptr<osgDB::Options> opts = osgEarth::Registry::instance()->cloneOrCreateOptions(nullptr);
	osgDB::ReaderWriter::WriteResult wr = readerwriter->writeNode(*mapNode, write_stream, opts.get());
	REQUIRE(wr.success());

#if 1   // Used to generate comparable input files for this test.
	std::ofstream dbgfile;
	dbgfile.open(std::string("/tmp/dbg_")+filename, std::ofstream::out );
	if (dbgfile.is_open())
	{
	    dbgfile << write_stream.str();
	}
	dbgfile.close();
#endif

	// Check that the read input and output earth file from
	// deserialization/re-serializarion are equal.
	REQUIRE(read_stream.str() == write_stream.str());
    }
}

TEST_CASE( "Serializing and deserializing Earth files" ) {

    osgDB::ReaderWriter* readerwriter = osgDB::Registry::instance()->getReaderWriterForExtension("earth");
    REQUIRE(readerwriter != nullptr);

    readAndWrite(readerwriter, "../tests/serialized-simple.earth");
    readAndWrite(readerwriter, "../tests/serialized-extension.earth");
    readAndWrite(readerwriter, "../tests/serialized-external-extension.earth");
}
