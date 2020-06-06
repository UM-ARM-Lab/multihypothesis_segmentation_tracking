/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
	RosType toMessage(const CppType& t, const std::nullptr_t); /* NOLINT(readability-avoid-const-params-in-decls)*/ \
 \
	static \
	CppType fromMessage(const RosType& t); \
};

}

#endif //MPS_CONVERSION_HPP
