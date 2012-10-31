#ifndef __INPUT_DEVICE__
#define __INPUT_DEVICE__

#include <string>

namespace obvious {

class InputDevice
{
public:
    InputDevice(const std::string& name) : m_name(name) { }
    virtual ~InputDevice(void) { };

    const std::string& name(void) const { return m_name; }
    void setName(const std::string& name) { m_name = name; }

    virtual bool grab(void) = 0;

private:
    std::string m_name;
};

} // end namespace obvious

#endif
