#ifndef POINT_CLOUD_REGISTRATION_OUTPUT_STREAM_HPP
#define POINT_CLOUD_REGISTRATION_OUTPUT_STREAM_HPP

#include <iostream>

namespace point_cloud_registration {
class OutputStream
{
private:
    bool verbose_;
public:
    OutputStream(bool verbose): verbose_(verbose) {}

    template <typename T>
    OutputStream &operator<<(T &&t)
    {
        if (verbose_) {
            std::cout << t;
        }
        return *this;

    }
};
} // namespace point_cloud_registration

#endif
