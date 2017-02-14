file(REMOVE_RECURSE
  "libglfw.pdb"
  "libglfw.dylib"
  "libglfw.3.2.dylib"
  "libglfw.3.dylib"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/glfw.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
