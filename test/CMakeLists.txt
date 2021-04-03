add_definitions(-DFREEGLUT_EXPORTS -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS)


# Variable INCLUDE_FILES
set(INCLUDE_FILES

  Render.h
)


# Variable SOURCE_FILES
set(SOURCE_FILES

  Render.cpp
  Main.cpp
)


# Esto es usado para crear carpetas de visual studio.
source_group(include FILES ${INCLUDE_FILES})
source_group(src FILES ${SOURCE_FILES})


# Proyecto ejecutable con el archivo fuente VAR INCLUDE_FILES Y VAR SOURCE_FILES
add_executable(test ${INCLUDE_FILES} ${SOURCE_FILES})


target_link_libraries(

  test
  freeglut_static
  cirobb
  ${OPENGL_LIBRARIES}
)