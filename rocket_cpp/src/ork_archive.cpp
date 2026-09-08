#include "ork_archive.hpp"
#include <zip.h>
#include <stdexcept>
#include <vector>

std::string readOrkXml(const std::string& ork_path) {
    int err = 0;
    zip_t* archive = zip_open(ork_path.c_str(), ZIP_RDONLY, &err);
    if (!archive) {
        throw std::runtime_error("Could not open .ork file as a zip archive: " + ork_path);
    }

    zip_int64_t index = zip_name_locate(archive, "rocket.ork", 0);
    if (index < 0) {
        zip_close(archive);
        throw std::runtime_error("No 'rocket.ork' entry found inside " + ork_path);
    }

    zip_stat_t stat;
    zip_stat_index(archive, index, 0, &stat);

    zip_file_t* entry = zip_fopen_index(archive, index, 0);
    if (!entry) {
        zip_close(archive);
        throw std::runtime_error("Could not open the 'rocket.ork' entry inside " + ork_path);
    }

    std::vector<char> buffer(stat.size);
    zip_fread(entry, buffer.data(), stat.size);
    zip_fclose(entry);
    zip_close(archive);

    return std::string(buffer.begin(), buffer.end());
}
