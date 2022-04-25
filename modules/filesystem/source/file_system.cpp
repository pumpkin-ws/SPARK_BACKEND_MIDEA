#include "file_system.hpp"


bool spark_filesystem::checkTailValid (std::string tail) {
    bool valid = false;
    // check empty
    if(tail.empty()){
        std::cout << "empty extend name!" << std::endl;
    }
    else if(tail.at(0) != '.'){
        std::cout << "invalid extend name!" << std::endl;
    }
    else{
        valid = true;
    }
    return valid;
};

std::vector<std::string> spark_filesystem::getAllFileName(std::string path, std::string tail){
    std::vector<std::string> fileNames;
    // TODO : needs to first check if path exists!
    // check valid extend name
    if(checkTailValid(tail) == false){
        return fileNames;
    }
    // check path empty 
    if(path.empty()){  
        char buff[PATH_MAX];
        getcwd(buff, PATH_MAX);
        path = buff;
        std::cout << "empty path! default pwd:" << path << " will be used to load file names" << std::endl;
    }
    
    // search file in directory
    for ( boost::filesystem::directory_iterator it(path); it != boost::filesystem::directory_iterator(); ++it ){
        if ( boost::filesystem::is_regular_file( it->status() ) && it->path().extension() == tail ){
            std::string fileName( it->path().filename().string() );
            fileNames.push_back(fileName);
        }
    }
    return fileNames;
};

bool spark_filesystem::checkDirectoryExists(std::string dir_name){
    boost::filesystem::path p(dir_name);
    return boost::filesystem::exists(p);
}

bool spark_filesystem::createDirectory(std::string dir_name) {
    if(checkDirectoryExists(dir_name) == true) {
        LOG_WARNING("Directory already exists! Will not create another one.");
        return false;
    } else {
        try {
            boost::filesystem::path p(dir_name);
            boost::filesystem::create_directory(p);
            if(checkDirectoryExists(dir_name)) {
                printf("Directory was created\n");
            }
            return true;
        } catch (std::exception &e) {
            printf("Exception thrown, unable to create directory!\n");
            LOG_ERROR(e.what());
            return false;
        }
    }
};

bool spark_filesystem::copyAndReplaceFile(const std::string& src, const std::string& dest) {
    std::ifstream ifs(src);
    std::ofstream ofs(dest);
    if (!ifs.is_open()) {
        LOG_ERROR("Unable to open source file.");
        return false;
    }
    if (!ofs.is_open()) {
        LOG_ERROR("Unable to open destination file.");
        return false;
    }
    std::string current_line;
    while (getline(ifs, current_line)) {
        ofs << current_line << std::endl;
    }
    ifs.close();
    ofs.close();
    return true;
}

bool spark_filesystem::copyAndAppendFile(const std::string& src, const std::string& dest) {
    std::ifstream ifs(src);
    std::ofstream ofs(dest, std::ofstream::app);
    if (!ifs.is_open()) {
        LOG_ERROR("Unable to open source file.");
        return false;
    }
    if (!ofs.is_open()) {
        LOG_ERROR("Unable to open destination file.");
        return false;
    }
    std::string current_line;
    while (getline(ifs, current_line)){
        ofs << current_line << std::endl;
    }
    ifs.close();
    ofs.close();
    return true;
}