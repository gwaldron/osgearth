SET(TARGET_SRC
    dxt.cpp	
	FastDXTImageProcessor.cpp
	libdxt.cpp
	intrinsic.cpp
	util.cpp
)
SET(TARGET_H
    util.h
    dxt.h
	libdxt.h
)

SETUP_PLUGIN(fastdxt)