#pragma once
#include <algorithm>
#include <iostream>
#include <vector>

class InputParser {
public:
  InputParser(int& argc, char** argv) {
    for (int i = 1; i < argc; ++i)
      this->tokens_.push_back(std::string(argv[i]));
  }

  inline const std::string& getCmdOption(const std::string& option) const {
    std::vector<std::string>::const_iterator itr;
    itr = std::find(this->tokens_.begin(), this->tokens_.end(), option);
    if (itr != this->tokens_.end() && ++itr != this->tokens_.end()) {
      return *itr;
    }
    static const std::string empty_string("");
    return empty_string;
  }

  inline int getInt(const std::string& option) const {
    std::vector<std::string>::const_iterator itr;
    itr = std::find(this->tokens_.begin(), this->tokens_.end(), option);
    if (itr != this->tokens_.end() && ++itr != this->tokens_.end()) {
      return std::stoi(*itr);
    }
    return 0;
  }

  inline float getFloat(const std::string& option) const {
    std::vector<std::string>::const_iterator itr;
    itr = std::find(this->tokens_.begin(), this->tokens_.end(), option);
    if (itr != this->tokens_.end() && ++itr != this->tokens_.end()) {
      return std::stof(*itr);
    }
    return 0.f;
  }

  bool cmdOptionExists(const std::string& option) const {
    return std::find(this->tokens_.begin(), this->tokens_.end(), option) != this->tokens_.end();
  }

private:
  std::vector<std::string> tokens_;
};
