INCLUDE_DIRECTORIES( ../../third_party/lerc )

SET(TARGET_SRC
    ReaderWriterLERC.cpp
    ../../third_party/lerc/BitMask.cpp
    ../../third_party/lerc/BitStuffer2.cpp
    ../../third_party/lerc/Huffman.cpp
    ../../third_party/lerc/Lerc.cpp
    ../../third_party/lerc/Lerc_c_api_impl.cpp
    ../../third_party/lerc/Lerc2.cpp
    ../../third_party/lerc/RLE.cpp
    ../../third_party/lerc/Lerc1Decode/BitStuffer.cpp
    ../../third_party/lerc/Lerc1Decode/CntZImage.cpp
)

#### end var setup  ###
SETUP_PLUGIN(lerc)