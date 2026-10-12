#include "file.h"
#include <stdio.h>
#include <sys/stat.h>
#include <sstream>
#include <unistd.h>
#include <errno.h>

namespace leju {
namespace hw {

bool FileUtils::Exists(const std::string &filepath){
    return (access(filepath.c_str(), F_OK) == 0);
}

bool FileUtils::ReadToString(const std::string& file_path, std::string& out_content) {
    std::ifstream ifs(file_path, std::ios_base::in);
    if (!ifs.good()) {
        return false;
    }

    std::stringstream buffer;
    out_content.clear();
    buffer << ifs.rdbuf();
    out_content = std::move(buffer.str());

    // Remove trailing newline if present
    if (!out_content.empty() && out_content.back() == '\n') {
        out_content.pop_back();
    }

    return true;
}

bool FileUtils::IsDirectory(const std::string &dirpath) {
    struct stat statbuf;
    if (stat(dirpath.c_str(), &statbuf) != 0) {
        return false;
    }
    return S_ISDIR(statbuf.st_mode);
}

} // namespace hw
} // namespace leju