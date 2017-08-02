#ifndef FILE_SEQUENCE_H
#define FILE_SEQUENCE_H

#include <vector>
#include <string>
#include <boost/filesystem.hpp>

class FileSequence
{
public:
  FileSequence(std::string directory_path);

  std::string getNextFileContents();
  bool hasNextFile();
  std::vector<std::string> getFilenames();

private:
  boost::filesystem::path dir_path_;
  std::vector<std::string> filenames_;
  unsigned int current_file_index_;

  void readDirectory();
  std::string readNextFile();
};

#endif