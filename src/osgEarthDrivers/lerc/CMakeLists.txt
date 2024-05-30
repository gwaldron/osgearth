include_directories(BEFORE ../../third_party/lerc/include )

add_osgearth_plugin(
    TARGET osgdb_lerc
    SOURCES
        ReaderWriterLERC.cpp
        ../../third_party/lerc/src/LercLib/BitMask.cpp
        ../../third_party/lerc/src/LercLib/BitStuffer2.cpp
        ../../third_party/lerc/src/LercLib/Huffman.cpp
        ../../third_party/lerc/src/LercLib/Lerc.cpp
        ../../third_party/lerc/src/LercLib/Lerc_c_api_impl.cpp
        ../../third_party/lerc/src/LercLib/Lerc2.cpp
        ../../third_party/lerc/src/LercLib/RLE.cpp
        ../../third_party/lerc/src/LercLib/Lerc1Decode/BitStuffer.cpp
        ../../third_party/lerc/src/LercLib/Lerc1Decode/CntZImage.cpp )