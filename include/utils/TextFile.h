#pragma once

#include <fstream>
#include <sstream>


/**
 * A class to handle loading string from a text file.
 */
class TextFile
{
public:
    /**
     * Load text from a file.
     * @param filename  Path of text file to load.
     */
    TextFile(const std::string& filename) : m_filename(filename)
    {
        std::ifstream inFile;
        inFile.open(filename);

        std::stringstream strStream;
        strStream << inFile.rdbuf();
        m_text = strStream.str();
    }

    /**
     * Get loaded text.
     * @return String representing complete text from the laoded file.
     */
    std::string GetText() const { return m_text; }

private:
    std::string m_filename;
    std::string m_text;

};
