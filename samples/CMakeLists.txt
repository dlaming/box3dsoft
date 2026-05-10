# Samples app

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
# set(FETCHCONTENT_QUIET FALSE)

# glad for OpenGL API
set(GLAD_DIR ${CMAKE_SOURCE_DIR}/extern/glad)

add_library(
	glad STATIC
	${GLAD_DIR}/src/glad.c
	${GLAD_DIR}/include/glad/glad.h
	${GLAD_DIR}/include/KHR/khrplatform.h
)
target_include_directories(glad PUBLIC ${GLAD_DIR}/include)

# glfw for windowing and input
SET(GLFW_BUILD_DOCS OFF CACHE BOOL "GLFW Docs")
SET(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "GLFW Examples")
SET(GLFW_BUILD_TESTS OFF CACHE BOOL "GLFW Tests")
SET(GLFW_INSTALL OFF CACHE BOOL "GLFW Install")

FetchContent_Declare(
	glfw
	GIT_REPOSITORY https://github.com/glfw/glfw.git
	GIT_TAG master
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(glfw)

# imgui and glfw backend for GUI
# https://gist.github.com/jeffamstutz/992723dfabac4e3ffff265eb71a24cd9
# Modified to pin to a specific imgui release
FetchContent_Populate(imgui
	URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.5.zip
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/build/imgui
)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/build/imgui)

add_library(imgui STATIC
	${IMGUI_DIR}/imconfig.h
	${IMGUI_DIR}/imgui.h
	${IMGUI_DIR}/imstb_truetype.h

	${IMGUI_DIR}/imgui.cpp
	${IMGUI_DIR}/imgui_draw.cpp
	${IMGUI_DIR}/imgui_demo.cpp
	${IMGUI_DIR}/imgui_tables.cpp
	${IMGUI_DIR}/imgui_widgets.cpp

	${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
	${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_link_libraries(imgui PUBLIC glfw glad)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
# The sample app also uses stb_truetype and this keeps the symbols separate
target_compile_definitions(imgui PRIVATE IMGUI_STB_NAMESPACE=imgui_stb)
set_target_properties(imgui PROPERTIES
	CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# implot for drawing real time plots
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.17
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(implot)

add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
    ${implot_SOURCE_DIR}/implot.h
    ${implot_SOURCE_DIR}/implot_internal.h
)
target_link_libraries(implot PUBLIC imgui)
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})

set (BOX3D_SAMPLE_FILES
	camera.cpp
	camera.h
	earcut.h
	font.cpp
	font.h
	geo_buffer.cpp
	geo_buffer.h
	jsmn.h
	main.cpp
	mesh_loader.cpp
	mesh_loader.h
	renderer.cpp
	renderer.h
	sample.cpp
	sample.h
	scene.cpp
	scene.h
	shader.cpp
	shader.h
	ssao_buffer.cpp
	ssao_buffer.h
	stb_image_write.h
	stb_truetype.h
    tiny_obj_loader.h
	utility.h

	sample_benchmark.cpp
	sample_bodies.cpp
	sample_character.cpp
	sample_collision.cpp
	sample_compound.cpp
	sample_continuous.cpp
	sample_determinism.cpp
	sample_events.cpp
	sample_geometry.cpp
	sample_issues.cpp
	sample_joint.cpp
	sample_manifold.cpp
	sample_mesh.cpp
	sample_ragdoll.cpp
	sample_robustness.cpp
	sample_shapes.cpp
	sample_stacking.cpp
	sample_tree.cpp
)

add_executable(samples ${BOX3D_SAMPLE_FILES})
target_include_directories(samples PRIVATE ${CMAKE_SOURCE_DIR}/data)
target_include_directories(samples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(samples PRIVATE box3d shared glfw imgui implot glad)

set_target_properties(samples PROPERTIES
	CMAKE_COMPILE_WARNING_AS_ERROR ON
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${BOX3D_SAMPLE_FILES})

message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")
