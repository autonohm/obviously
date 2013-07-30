#ifndef __GPU_ORGANIZED__
#define __GPU_ORGANIZED__

#include "obgpu/search/Search.h"

namespace obvious { namespace gpu { namespace search {

class Organized : public Search
{
public:
    Organized(void) : Search() { }

    virtual void radiusSearch(const float radius);
};

} // end namespace search

} // end namespace gpu

} // end namespace obvious

#endif
