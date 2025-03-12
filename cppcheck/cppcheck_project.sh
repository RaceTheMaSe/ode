#! /bin/bash
# https://unix.stackexchange.com/a/725492
# %pattern* strips all after pattern, #*pattern strips all before pattern, doubling ## or %% selects last match
currFolder=${PWD##*/}
fullPath=${PWD}
pathWithoutCurrFolder="${fullPath%/${currFolder}*}"
projectFolder="${pathWithoutCurrFolder##*/}"
filter=*/${projectFolder}/src/*
cppcheck --project=../build-msvc22/compile_commands.json --enable=all --platform=win64 -D_MSC_VER -D_WIN32 --std=c++11 --std=c11 --suppressions-list=cppcheck_suppressions.txt --max-ctu-depth=4 --check-level=exhaustive --output-file=cppcheck_result_colorcoded.txt
cat cppcheck_result_colorcoded.txt | ansi2txt > cppcheck_result.txt
rm cppcheck_result_colorcoded.txt
echo 'Result in cppcheck_result.txt'
