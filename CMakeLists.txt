cmake_minimum_required(VERSION 3.13)

project(RayCaster
	VERSION 0.0.1
	DESCRIPTION "Simple ray caster for development purposes"
	LANGUAGES CXX
)

# a few settings
# set(Trilinos_MINIMUM_VERSION 13.0)

# define main target
add_executable(caster)

# exports compile_command.json with relevant parths
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the properties of the target (might be moved to another file)
set_target_properties(caster
           PROPERTIES CXX_STANDARD 17
		   CXX_STANDARD_REQUIRED TRUE
)

# set compilation flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	target_compile_options(caster
		PRIVATE -march=native
		PRIVATE -Wall
		PRIVATE -faligned-new
		PRIVATE -fmessage-length=0
	)
else()
	message(WARNING "RayCaster: Flags have not been implemented for the current compiler! (${CMAKE_CXX_COMPILER_ID})")
endif()

# set(CMAKE_CXX_EXTENSIONS OFF CACHE BOOL
# 	"Disbled CXX extensions" FORCE
# )

# locate packages
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
	set_target_properties(OpenMP::OpenMP_CXX
		PROPERTIES IMPORTED_GLOBAL TRUE
	)
endif()

find_package(MPI REQUIRED)
if(MPI_CXX_FOUND)
	set_target_properties(MPI::MPI_CXX
		PROPERTIES IMPORTED_GLOBAL TRUE
	)
endif()

find_package(DaRe REQUIRED)
if(DaRe_FOUND)
    set_target_properties(DaRe::dare
        PROPERTIES IMPORTED_GLOBAL TRUE
    )
endif()

# find_package(Trilinos ${Trilinos_MINIMUM_VERSION} REQUIRED)
# if(Trilinos_FOUND)
# 	# update user
# 	message(STATUS "Found Trilinos version ${Trilinos_VERSION} at ${Trilinos_DIR} - Generating custom target")

# 	# create interface library
# 	add_library(trilinos INTERFACE)

# 	set_target_properties(trilinos PROPERTIES
# 		INTERFACE_LINK_LIBRARIES "${Trilinos_LIBRARIES}"
# 		INTERFACE_INCLUDE_DIRECTORIES "${Trilinos_INCLUDE_DIRS}"
# 	)
# endif()

# add target sources
target_sources(caster
	PUBLIC main.cpp
    PUBLIC RayTracer.cpp
)

# Specify the include directories
target_include_directories(caster PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
)

# link libraries
target_link_libraries(caster
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC MPI::MPI_CXX
    PUBLIC DaRe::dare
	# PUBLIC trilinos
)
