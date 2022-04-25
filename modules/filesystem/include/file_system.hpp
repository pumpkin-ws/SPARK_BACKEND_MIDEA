#ifndef FILE_SYSTEM_HPP_
#define FILE_SYSTEM_HPP_

#include <vector>
#include <iostream>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <limits.h>
#include "filesystem/include/file_system.hpp"
#include "common.hpp"

namespace spark_filesystem{
    bool checkTailValid(std::string tail);
    std::vector<std::string> getAllFileName(std::string path, std::string tail);
    bool checkDirectoryExists(std::string dir_name);
    bool createDirectory(std::string dir_name);

    bool copyAndReplaceFile(const std::string& src, const std::string& dest);

    bool copyAndAppendFile(const std::string& src, const std::string& dest);
}

#endif