# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(foxglove_sdk-populate NONE)



include(ExternalProject)
ExternalProject_Add(foxglove_sdk-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.19.0/foxglove-v0.19.0-cpp-x86_64-unknown-linux-gnu.zip" "URL_HASH" "SHA256=f3d2f6a8ba97fedeae13b978f5e8a51dec4803eddfd23b198a292bf9ab6675ff"
                    SOURCE_DIR          "/home/lzh/VMC_Quadruped/src/build/foxglove_bridge/_deps/foxglove_sdk-src"
                    BINARY_DIR          "/home/lzh/VMC_Quadruped/src/build/foxglove_bridge/_deps/foxglove_sdk-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


