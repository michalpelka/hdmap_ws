#include <vector>
#include <ostream>
#include <fstream>
#include "iostream"
#include <cstring>
template<typename T>
inline bool save_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
    std::ofstream ofs(file_name, std::ios::binary);
    if (!ofs.good()) {
        return false;
    }
    ofs.write(reinterpret_cast<char*>(vector_data.data()), vector_data.size()* sizeof(T));
    return true;
}

template<typename T>
inline bool load_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
    std::basic_ifstream<char> vd_str(file_name, std::ios::binary);
    if (!vd_str.good()) {
        return false;
    }
    std::vector<char> data((std::istreambuf_iterator<char>(vd_str)), std::istreambuf_iterator<char>());
    std::vector<T> v(data.size() / sizeof(T));
    std::memcpy(v.data(), data.data(), data.size());
    vector_data = v;
    return true;
}

namespace structs {
    struct Point {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        uint32_t intensity = 0;
        double time = 0.0;
    };
    struct ChunkFile {
        int filename;
        double time_begin_inclusive;
        double time_end_inclusive;
    };
}


int main(){


    const std::string fn {"/media/michal/ext/LivoxAviaMapa2022-10-16T16:34:21.577162/data/"};
    const std::string chunks_bin_fn = fn + "/" + "chunks.bin";
    std::vector<structs::ChunkFile> chunk_files;

    load_vector_data<structs::ChunkFile>(chunks_bin_fn, chunk_files);
    std::ofstream txt ("/tmp/txt.asc");

    for (const auto &ch : chunk_files){
        std::vector<structs::Point> file;
        const std::string fn {"/media/michal/ext/LivoxAviaMapa2022-10-16T16:34:21.577162/data/"+std::to_string(ch.filename)+".bin"};
        load_vector_data<structs::Point>(chunks_bin_fn, file);
        //std::cout << ch.filename <<" " <<std::fixed<< ch.time_begin_inclusive<<" " << ch.time_end_inclusive << std::endl;

    }

}

