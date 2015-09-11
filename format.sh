#!/bin/sh
exec clang-format-3.5 -style=file -i $(find src -name "*.h" -or -name "*.cpp")
