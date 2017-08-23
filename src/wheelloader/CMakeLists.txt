#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib64" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")
endif("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

message(STATUS "==== Wheel Loader programs ====")



SET(MODELS
#  cascading_packing_condition
#  central_pivot_mech
#  demo_IRR_fel
#  demo_PAR_friction
#  demo_VEH_ArticulatedVehicle
  demo_PAR_WheelLoader
  demo_SEQ_WheelLoader
#  demo_VEH_wheelloader
  hydraulic_force
  iArmlinkage
  test_granular_PAR
#  test_granular_PAR_cyl
#  test_granular_PAR_funnel
#  test_granular_PAR_layer
  test_HYDR_actuator # future
  test_mesh
  test_normal
  test_normal_PAR
  test_PAR_rolling_friction
  test_scalability_sandpile
  test_SEQ_rolling_friction
  utest_PAR_linactuator
  zBarlinkage
  zBarlinkage_PAR
  zBarSandpile
  )

  SET(MODEL_FILES
    subsystems/Articulated_Front.h
    subsystems/Articulated_Front.cpp
    subsystems/Articulated_Rear.h
    subsystems/Articulated_Rear.cpp
	subsystems/WL_Driveline4WD.h
	subsystems/WL_Driveline4WD.cpp
subsystems/WL_SimpleDriveline4WD.h
subsystems/WL_SimpleDriveline4WD.cpp
subsystems/WL_SimpleMapPowertrain.h
subsystems/WL_SimpleMapPowertrain.cpp
	subsystems/WL_ShaftsPowertrain.h
	subsystems/WL_ShaftsPowertrain.cpp
	subsystems/WL_FialaTire.h
	subsystems/WL_FialaTire.cpp
)
 SET(UTILITIES
 utilities/UtilityFunctions.h
 utilities/ZBarMechanism.h
 utilities/Pneumatics.h
  )

SOURCE_GROUP("utilities" FILES ${UTILITIES})
SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${MODELS}.cpp ${MODEL_FILES})
SET(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}")

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_parallel
    ChronoEngine_opengl
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR})
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})
INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})





# Disable some warnings triggered by Irrlicht (Windows only)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(COMPILE_FLAGS "${COMPILE_FLAGS} /wd4275")
endif()


foreach(PROGRAM ${MODELS})

  message(STATUS "...add ${PROGRAM}")

#  if (${PROGRAM} MATCHES "demo_PAR_WheelLoader")
		add_executable(${PROGRAM}  "${PROGRAM}.cpp"  ${UTILITIES} ${MODEL_FILES})
#  else()
#		add_executable(${PROGRAM}  "${PROGRAM}.cpp"  ${UTILITIES})

#  endif()
 
  source_group(""  FILES "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
  )

  target_link_libraries(${PROGRAM} ${CHRONO_LIBRARIES} ${LIBRARIES})

endforeach(PROGRAM)
