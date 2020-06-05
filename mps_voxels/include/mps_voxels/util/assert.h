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

#ifndef MPS_ASSERT_H
#define MPS_ASSERT_H

#include <iostream>

#ifndef MPS_ENABLE_ASSERTS
#define MPS_ENABLE_ASSERTS true
#endif

#define MPS_UNUSED(x) do { (void)sizeof(x); } while(false)
#define MPS_HALT() goto exit

#if MPS_ENABLE_ASSERTS
	#define MPS_ASSERT(cond) do { if (!(cond)) { throw std::logic_error(__FILE__ + std::string(":") + std::to_string(__LINE__) + std::string("    MPS_ASSERT Failed:\n") + std::string(#cond)); } } while(false)
#else
	#define MPS_ASSERT(cond) MPS_UNUSED(cond)
#endif


template <typename Index, typename Container>
inline void assert_in_range(const Index& idx, const Container& ctr)
{
	MPS_ASSERT(0 <= idx && idx < static_cast<Index>(ctr.size()));
	MPS_UNUSED(idx);
	MPS_UNUSED(ctr); // Suppress compiler warnings when in release mode
}

#endif //MPS_ASSERT_H
