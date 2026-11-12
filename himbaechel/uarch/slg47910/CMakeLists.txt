set(SOURCES
    constids.inc
    slg47910.cc
)

add_nextpnr_himbaechel_microarchitecture(${uarch}
    CORE_SOURCES ${SOURCES}
)

set(HIMBAECHEL_SLG47910_DB "" CACHE STRING
    "Path to a Project Inkwell forgefpga database checkout")
if (NOT HIMBAECHEL_SLG47910_DB AND NOT IMPORT_BBA_FILES)
    message(FATAL_ERROR "HIMBAECHEL_SLG47910_DB must be set to a Project Inkwell forgefpga checkout")
endif()

add_bba_produce_command(
    TARGET  nextpnr-himbaechel-slg47910-bba
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/gen/slg47910_gen.py
        --db ${HIMBAECHEL_SLG47910_DB}
        --bba ${CMAKE_CURRENT_BINARY_DIR}/chipdb-slg47910.bba.new
    OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/chipdb-slg47910.bba
    INPUTS
        ${CMAKE_CURRENT_SOURCE_DIR}/gen/slg47910_gen.py
        ${CMAKE_CURRENT_SOURCE_DIR}/constids.inc
)

add_bba_compile_command(
    TARGET  nextpnr-himbaechel-slg47910-chipdb
    OUTPUT  himbaechel/slg47910/chipdb-slg47910.bin
    INPUT   ${CMAKE_CURRENT_BINARY_DIR}/chipdb-slg47910.bba
    MODE    binary
)
