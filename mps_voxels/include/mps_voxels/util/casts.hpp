#ifndef MPS_CASTS_HPP
#define MPS_CASTS_HPP

#include <memory>
#include <cassert>

// From Boost polymorphic_downcast
template <class Target, class Source>
inline Target down_cast(Source* x)
{
	assert(dynamic_cast<Target>(x) == x);  // detect logic error
	return static_cast<Target>(x);
}

template <class Target, class Source>
inline std::shared_ptr<Target> down_pointer_cast(const std::shared_ptr<Source>& x)
{
	assert(std::dynamic_pointer_cast<Target>(x) == x);  // detect logic error
	return std::static_pointer_cast<Target>(x);
}

#endif //PROJECT_CASTS_HPP