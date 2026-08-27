cmake_minimum_required(VERSION 3.16)
project(usrp_burst_gui CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(UHD REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
pkg_check_modules(FFTW REQUIRED fftw3f)
if(NOT FFTW_FOUND)
    # Some distros only ship the double-precision fftw3.pc; fall back and
    # link fftw3f by name (header fftw3.h covers both via fftwf_* symbols
    # once linked against libfftw3f).
    pkg_check_modules(FFTW REQUIRED fftw3)
endif()

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
set(IMPLOT_DIR ${CMAKE_SOURCE_DIR}/third_party/implot)

add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${GLFW_INCLUDE_DIRS}
)
target_link_libraries(imgui PUBLIC ${GLFW_LIBRARIES} OpenGL::GL)
target_compile_options(imgui PUBLIC ${GLFW_CFLAGS_OTHER})

add_library(implot STATIC
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
)
target_include_directories(implot PUBLIC ${IMPLOT_DIR})
target_link_libraries(implot PUBLIC imgui)

# System 1: SDR Source Node + GUI + TCP Streamer
add_executable(usrp_burst_gui
    src/main.cpp
    src/usrp_manager.cpp
    src/burst_controller.cpp
    src/fft_processor.cpp
    src/net_streamer.cpp
    src/gui.cpp
)

target_include_directories(usrp_burst_gui PRIVATE
    ${UHD_INCLUDE_DIRS}
    ${FFTW_INCLUDE_DIRS}
)

target_link_directories(usrp_burst_gui PRIVATE ${FFTW_LIBRARY_DIRS})

target_link_libraries(usrp_burst_gui PRIVATE
    imgui
    implot
    ${UHD_LIBRARIES}
    fftw3f
    Threads::Threads
)

target_compile_options(usrp_burst_gui PRIVATE -Wall -Wextra -O2)

# System 2: Relay Node (sc16 -> float -> Demux x2/x3/x4 -> 8-Plot GUI -> Scaled sc16 -> S3)
add_executable(usrp_relay_gui
    src/relay_main.cpp
    src/multichannel_demux.cpp
    src/multichannel_gui.cpp
    src/synthetic_telemetry.cpp
    src/fft_processor.cpp
    src/net_streamer.cpp
    src/gui.cpp
)

target_include_directories(usrp_relay_gui PRIVATE
    ${FFTW_INCLUDE_DIRS}
)

target_link_directories(usrp_relay_gui PRIVATE ${FFTW_LIBRARY_DIRS})

target_link_libraries(usrp_relay_gui PRIVATE
    imgui
    implot
    fftw3f
    Threads::Threads
)

target_compile_options(usrp_relay_gui PRIVATE -Wall -Wextra -O2)

# System 3: Sink Node (Operator Console GUI with Sidebar Tabs & Focused Views)
add_executable(usrp_sink_gui
    src/sink_main.cpp
    src/operator_console_gui.cpp
    src/osm_slippy_map.cpp
    src/audio_player.cpp
    src/multichannel_demux.cpp
    src/multichannel_gui.cpp
    src/synthetic_telemetry.cpp
    src/fft_processor.cpp
    src/net_streamer.cpp
    src/gui.cpp
)

target_include_directories(usrp_sink_gui PRIVATE
    ${FFTW_INCLUDE_DIRS}
)

target_link_directories(usrp_sink_gui PRIVATE ${FFTW_LIBRARY_DIRS})

target_link_libraries(usrp_sink_gui PRIVATE
    imgui
    implot
    fftw3f
    asound
    Threads::Threads
)

target_compile_options(usrp_sink_gui PRIVATE -Wall -Wextra -O2)



