MACRO(APPEND_LINK_FLAG OPTION_NAME FLAG_TO_ADD)
  get_target_property(EXISTING_${OPTION_NAME} ${PROJECT_NAME} ${OPTION_NAME})
  if(EXISTING_${OPTION_NAME} STREQUAL "EXISTING_${OPTION_NAME}-NOTFOUND")
      set(EXISTING_${OPTION_NAME})
  endif()
  set_target_properties(${PROJECT_NAME} PROPERTIES ${OPTION_NAME} "${EXISTING_${OPTION_NAME}} ${FLAG_TO_ADD}")
ENDMACRO()
MACRO(APPLY_COMPILER_OPTIONS_MSVC TARGET_NAME)
  target_compile_options(${TARGET_NAME} PRIVATE
    /wd4266 # 'declaration'      : no override available for virtual member function from base 'class'; function is hidden
    /wd4350 # behavior change: 'type' called instead of 'type2'
    /wd4512 # 'class' : assignment operator could not be generated
    /wd4514 # 'function' : unreferenced inline function has been removed
    /wd4571 # Informational: catch(...) semantics changed since Visual C++ 7.1; structured exceptions (SEH) are no longer caught
    /wd4611 # interaction between 'function' and C++ object destruction is non-portable
    /wd4623 # 'type': default constructor was implicitly defined as deleted
    /wd4625 # 'type': copy constructor was implicitly defined as deleted
    /wd4626 # 'type': assignment operator was implicitly defined as deleted
    /wd4668 # 'name' is not defined as a preprocessor macro, replacing with '0' for 'value'
    /wd4702 # unreachable code
    /wd4710 # 'function' : function not inlined
    /wd4711 # function 'function' selected for inline expansion
    /wd4774 # 'description' : format string expected in argument number is not a string literal
    /wd4820 # 'type': 'number' bytes padding added after type 'type'
  )
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00) # MSVC 2013
    target_compile_options(${TARGET_NAME} PRIVATE
      /wd4056 # overflow in floating-point constant arithmetic (-INIFINITY)
      /wd4127 # conditional expression is constant
      /wd4738 # storing 32-bit float result in memory, possible loss of performance
      /wd4756 # overflow in constant arithmetic (INFINITY - by design)
    )
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 19.19) # MSVC 2019
    target_compile_options(${TARGET_NAME} PRIVATE
      /wd4746 # volatile access of 'variable' is subject to /volatile:<iso|ms> setting; consider using __iso_volatile_load/store intrinsic functions
      /wd4865 # the underlying type of 'enum' will change when '/Zc:enumTypes' is specified on the command line
      /wd5026 # 'type': move constructor was implicitly defined as deleted
      /wd5027 # 'type': move assignment operator was implicitly defined as deleted
      /wd5039 # 'function': pointer or reference to potentially throwing function passed to extern C function under -EHc. Undefined behavior may occur if this function throws an exception.
      /wd5045 # Compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
      /wd5220 # 'name': a non-static data member with a volatile qualified type no longer implies%$N that compiler generated copy/move constructors and copy/move assignment operators are not trivial
    )
  endif()
  # this object file does not define anay previously undefined public symbols, so it will not be used by any link operation that consumes this library
  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    APPEND_LINK_FLAG(STATIC_LIBRARY_FLAGS /ignore:4221)
    APPEND_LINK_FLAG(LINK_FLAGS /ignore:4221)
  endif()
ENDMACRO()
