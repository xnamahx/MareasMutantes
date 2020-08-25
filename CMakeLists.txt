cmake_minimum_required(VERSION 3.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../min-api/script/min-pretarget.cmake)

include_directories( 
	"${C74_INCLUDES}"
)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/mi"
)

set(TIDES_SRC
       mi/tides2/poly_slope_generator.cc
       mi/tides2/ramp_extractor.cc
       mi/tides2/resources.cc
)

set(MI_COMMON_SRC
       mi/stmlib/dsp/units.cc
       mi/stmlib/dsp/atan.cc 
       mi/stmlib/utils/random.cc
)

set(MITIDESLIB_SRC 
    ${MI_COMMON_SRC}
    ${TIDES_SRC}
)

add_library(MITIDESLib ${MITIDESLIB_SRC} )

add_library( 
	${PROJECT_NAME} 
	MODULE
	${PROJECT_NAME}.cpp
)

set(GCC_COMPILE_FLAGS "-DTEST")

add_definitions(${GCC_COMPILE_FLAGS})

target_link_libraries(${PROJECT_NAME} MITIDESLib)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../min-api/script/min-posttarget.cmake)
