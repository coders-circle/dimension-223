#pragma once


struct Area {
    int x1, y1, x2, y2;

    Area(int x1, int x2, int y1, int y2)
    : x1(x1), x2(x2), y1(y1), y2(y2)
    {}

    Area() {}

    bool contains(int x, int y) const {
        return x >= x1 && x <= x2 && y >= y1 && y <= y2;
    }
};


inline std::string readFile(const std::string& filename) {
    std::ifstream file;
    file.open(filename);

    std::stringstream stream;
    stream << file.rdbuf();
    return stream.str();
}

inline std::string getFilename(const std::string& path)
{
    std::string temp = path;

    size_t s = temp.find_last_of("/");
    if (s != std::string::npos)
        temp = temp.substr(s+1, temp.size()-s-1);

    return temp;
}

inline std::string getFolder(const std::string& path)
{
    std::string temp = path;

    size_t s = temp.find_last_of("/");
    if (s != std::string::npos)
        temp = temp.substr(0, s);

    return temp;
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


inline btVector3 glmToBullet(const glm::vec3& v) {
    return btVector3(v.x, v.y, v.z);
}


inline btQuaternion glmToBullet(const glm::quat& v) {
    return btQuaternion(v.x, v.y, v.z, v.w);
}


inline glm::vec3 bulletToGlm(const btVector3& v) {
    return glm::vec3(v.getX(), v.getY(), v.getZ());
}


inline std::ostream& operator << (std::ostream& out, const glm::vec3& v) {
    out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return out;
}

void blur(std::vector<glm::vec3>& points, int width, int height);
