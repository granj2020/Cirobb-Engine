# Variable INCLUDE_FILES
set(CIROBB_INCLUDE_FILES

  CbMath.h
  Collision.h 
  RigidBody.h 
  Manifold.h
  Scene.h
  Shape.h
)


# Variable SOURCE_FILES
set(CIROBB_SOURCE_FILES

  Collision.cpp 
  RigidBody.cpp
  Manifold.cpp
  Scene.cpp
  Shape.cpp
)


add_library(cirobb STATIC

  ${CIROBB_INCLUDE_FILES} 
  ${CIROBB_SOURCE_FILES}
)


# Esto es usado para crear carpetas de visual studio.
source_group(include FILES ${CIROBB_INCLUDE_FILES})
source_group(src FILES ${CIROBB_SOURCE_FILES})

