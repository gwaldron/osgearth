INCLUDE_DIRECTORIES( ../../third_party/lerc/include )

SET(TARGET_SRC
    ReaderWriterLERC.cpp
    ../../third_party/lerc/src/LercLib/BitMask.cpp
    ../../third_party/lerc/src/LercLib/BitStuffer2.cpp
    ../../third_party/lerc/src/LercLib/Huffman.cpp
    ../../third_party/lerc/src/LercLib/Lerc.cpp
    ../../third_party/lerc/src/LercLib/Lerc_c_api_impl.cpp
    ../../third_party/lerc/src/LercLib/Lerc2.cpp
    ../../third_party/lerc/src/LercLib/RLE.cpp
    ../../third_party/lerc/src/LercLib/Lerc1Decode/BitStuffer.cpp
    ../../third_party/lerc/src/LercLib/Lerc1Decode/CntZImage.cpp
)

#### end var setup  ###
SETUP_PLUGIN(lerc)