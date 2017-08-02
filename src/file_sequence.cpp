#include "skill_transfer/file_sequence.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdexcept>

FileSequence::FileSequence(std::string directory_path) : dir_path_(directory_path),
                                                         current_file_index_(0)
{
  readDirectory();
}

std::string FileSequence::getNextFileContents()
{
  if (!hasNextFile())
    throw new std::out_of_range("No file left to read");

  return readNextFile();
}

bool FileSequence::hasNextFile()
{
  return current_file_index_ < filenames_.size();
}

std::vector<std::string> FileSequence::getFilenames()
{
  return filenames_;
}

void FileSequence::readDirectory()
{
  for (boost::filesystem::directory_entry &f :
       boost::filesystem::directory_iterator(dir_path_))
  {
    filenames_.push_back(f.path().filename().string());
  }

  std::sort(filenames_.begin(), filenames_.end());
}

std::string FileSequence::readNextFile()
{
  auto path = dir_path_ / filenames_[current_file_index_];
  boost::filesystem::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();

  current_file_index_ += 1;

  return buffer.str();
}