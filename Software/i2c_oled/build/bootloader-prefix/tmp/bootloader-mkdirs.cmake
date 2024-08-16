# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/liwka/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "C:/Projetos/i2c_oled/build/bootloader"
  "C:/Projetos/i2c_oled/build/bootloader-prefix"
  "C:/Projetos/i2c_oled/build/bootloader-prefix/tmp"
  "C:/Projetos/i2c_oled/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Projetos/i2c_oled/build/bootloader-prefix/src"
  "C:/Projetos/i2c_oled/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Projetos/i2c_oled/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Projetos/i2c_oled/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
