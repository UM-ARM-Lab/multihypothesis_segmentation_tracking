//
// Created by pricear on 2020-04-01.
//

#ifndef MPS_CONVERSION_HPP
#define MPS_CONVERSION_HPP

#include <cstddef>
#include <type_traits>

namespace mps
{

template <typename T>
struct ros_message_conversion{};

template <typename T>
struct ros_message_type_of {};

/**
 * @tparam T Main type to be converted to message
 * @tparam Aux Auxiliary type containing any extra info (particularly headers)
 * @param t Object to convert
 * @param a Aux data object (defaults to null pointer)
 * @return
 */
template <typename T, typename Aux = std::nullptr_t>
auto toMessage(T t, Aux a = nullptr)
{
	return ros_message_conversion<std::remove_cv_t<std::remove_reference_t<T>>>::toMessage(t, a);
}

template <typename T>
auto fromMessage(T t)
{
	return ros_message_conversion<typename ros_message_type_of<std::remove_cv_t<std::remove_reference_t<T>>>::CppType>::fromMessage(t);
}

#define MESSAGE_CONVERSION_BOILERPLATE_AUX(CPPTYPE, ROSTYPE, AUXTYPE) \
template<> \
struct ros_message_type_of<ROSTYPE> { using CppType = CPPTYPE; }; \
 \
template<> \
struct ros_message_conversion<CPPTYPE> \
{ \
	using CppType = CPPTYPE; \
	using RosType = ROSTYPE; \
	using AuxType = AUXTYPE; \
 \
	static \
	RosType toMessage(const CppType& t, const std::nullptr_t); /* NOLINT(readability-avoid-const-params-in-decls)*/ \
 \
	static \
	RosType toMessage(const CppType& t, const AuxType& h); \
 \
	static \
	CppType fromMessage(const RosType& t); \
};

#define MESSAGE_CONVERSION_BOILERPLATE(CPPTYPE, ROSTYPE) \
template<> \
struct ros_message_type_of<ROSTYPE> { using CppType = CPPTYPE; }; \
 \
template<> \
struct ros_message_conversion<CPPTYPE> \
{ \
	using RosType = ROSTYPE; \
	using CppType = CPPTYPE; \
 \
	static \
	RosType toMessage(const CppType& t, const std::nullptr_t); \
 \
	static \
	CppType fromMessage(const RosType& t); \
};

}

#endif //MPS_CONVERSION_HPP
