#pragma once

inline std::string readFile(const std::string& filename) {
    std::ifstream file;
    file.open(filename);

    std::stringstream stream;
    stream << file.rdbuf();
    return stream.str();
}


class Exception : public std::exception {
public:
    Exception(const std::string& message) : mMessage(message) {}

    const char* what() const noexcept {
        return mMessage.c_str();
    }

private:
    std::string mMessage;
};
