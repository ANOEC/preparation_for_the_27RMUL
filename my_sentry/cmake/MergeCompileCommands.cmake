foreach(input IN ITEMS "${INPUT_A}" "${INPUT_B}")
    if(NOT EXISTS "${input}")
        message(FATAL_ERROR "Compilation database does not exist: ${input}")
    endif()
endforeach()

file(READ "${INPUT_A}" content_a)
file(READ "${INPUT_B}" content_b)

string(STRIP "${content_a}" content_a)
string(STRIP "${content_b}" content_b)
string(REGEX REPLACE "^\\[" "" content_a "${content_a}")
string(REGEX REPLACE "\\]$" "" content_a "${content_a}")
string(REGEX REPLACE "^\\[" "" content_b "${content_b}")
string(REGEX REPLACE "\\]$" "" content_b "${content_b}")
string(STRIP "${content_a}" content_a)
string(STRIP "${content_b}" content_b)

set(separator "")
if(NOT content_a STREQUAL "" AND NOT content_b STREQUAL "")
    set(separator ",")
endif()

file(WRITE "${OUTPUT}"
    "[\n${content_a}${separator}\n${content_b}\n]\n"
)
