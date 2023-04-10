#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <time.h>
#include <unordered_map>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
using namespace std;

int fileNameFilter(const struct dirent *cur)
{
	std::string str(cur->d_name);
	if (str.find(".bin") != std::string::npos || str.find(".pcd") != std::string::npos || str.find(".png") != std::string::npos || str.find(".jpg") != std::string::npos || str.find(".txt") != std::string::npos)
	{
		return 1;
	}
	return 0;
}

bool get_all_files(const string &dir_in, std::vector<std::string> &files)
{

    if (dir_in.empty())
    {
        return false;
    }
    struct stat s;
    stat(dir_in.c_str(), &s);
    if (!S_ISDIR(s.st_mode))
    {
        return false;
    }
    DIR *open_dir = opendir(dir_in.c_str());
    if (NULL == open_dir)
    {
        std::exit(EXIT_FAILURE);
    }
    dirent *p = nullptr;
    while ((p = readdir(open_dir)) != nullptr)
    {
        struct stat st;
        if (p->d_name[0] != '.')
        {

            std::string name = dir_in + std::string("/") + std::string(p->d_name);
            stat(name.c_str(), &st);
            if (S_ISDIR(st.st_mode))
            {
                get_all_files(name, files);
            }
            else if (S_ISREG(st.st_mode))
            {
                boost::char_separator<char> sepp{"."};
                tokenizer tokn(std::string(p->d_name), sepp);
                vector<string> filename_sep(tokn.begin(), tokn.end());
                string type_ = "." + filename_sep[1];
                break;
            }
        }
    }

    struct dirent **namelist;
    int n = scandir(dir_in.c_str(), &namelist, fileNameFilter, alphasort);
    if (n < 0)
    {
        return false;
    }
    for (int i = 0; i < n; ++i)
    {
        std::string filePath(namelist[i]->d_name);
        files.push_back(filePath);
        free(namelist[i]);
    };
    free(namelist);
    closedir(open_dir);
    return true;
}

template <class Type>
Type stringToNum(const string &str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}


#endif