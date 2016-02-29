#include <exception>

/**
 * Class representing errors specific to dimension-223.
 */
class Exception: public std::exception
{
public:
    /**
     * Construct a new exception.
     * @param errorMsg  Error message for this exception.
     */
    Exception(const std::string& errorMsg) : m_errorMsg(errorMsg) {}

    /**
     * Get error message for this exception
     * @return Error message as C-pointer string.
     */
    const char* what() const noexcept
    {
        return m_errorMsg.c_str();
    }

protected:
    std::string m_errorMsg;
};
