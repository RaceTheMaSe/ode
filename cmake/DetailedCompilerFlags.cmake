MACRO(APPLY_COMPILER_OPTIONS_GNU TARGET_NAME)
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${TARGET_NAME} PRIVATE
        -Wno-float-equal
        -Wno-format-nonliteral
        -Wno-padded
        -Wcast-qual
        -Wno-long-long
        -Wno-unused-macros
    )
endif()
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${TARGET_NAME} PRIVATE
        # -Weverything:
        -Wno-used-but-marked-unused # libccd
        -Wno-duplicate-enum
        -Wno-global-constructors
        -Wno-exit-time-destructors
        -Wno-old-style-cast
        -Wno-cast-align # pointer arithmetics on dContact array - asserts added to check stride
        -Wno-zero-as-null-pointer-constant
        -Wno-switch-default # contradicting option -switch-enum
        $<$<COMPILE_LANGUAGE:CXX>:
        -Wno-c++98-compat-pedantic
        -Wno-c++11-long-long
        >
        -Wno-deprecated-declarations # MacOS
    )
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9)
        target_compile_options(${TARGET_NAME} PRIVATE -Wno-alloca)
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11)
        target_compile_options(${TARGET_NAME} PRIVATE -Wno-reserved-identifier)
    else()
        target_compile_options(${TARGET_NAME} PRIVATE -Wno-reserved-id-macro)
    endif()
    # Clang 20 - argument unused during compilation: '-c'
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 20 AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 22)
    target_compile_options(${TARGET_NAME} PRIVATE
        -Wno-unused-command-line-argument
    )
    endif()
    # emits incorrect C++11 suggestions in C++98 mode
    if(CMAKE_CXX_STANDARD STREQUAL 98 AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11)
        target_compile_options(${TARGET_NAME} PRIVATE
            -Wno-suggest-destructor-override
            -Wno-suggest-override
        )
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16)
        target_compile_options(${TARGET_NAME} PRIVATE
            -Wno-unsafe-buffer-usage
        )
    endif()
endif()
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(${TARGET_NAME} PRIVATE 
        -Wcast-align
    )
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0.0)
        target_compile_options(${TARGET_NAME} PRIVATE 
            -Wno-suggest-attribute=format
            -Wno-clobbered
        $<$<COMPILE_LANGUAGE:CXX>:
            -Wno-class-memaccess
            -Wno-deprecated-copy
            >
        $<$<COMPILE_LANGUAGE:C>:
            -Wno-traditional                # lots of indented #define
            -Wno-traditional-conversion     # libccd
            -Wno-unsuffixed-float-constants # libccd
            >)
    endif()
endif()

ENDMACRO()
