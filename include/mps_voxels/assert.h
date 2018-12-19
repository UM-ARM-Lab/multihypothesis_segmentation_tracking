//
// Created by arprice on 12/18/18.
//

#ifndef MPS_ASSERT_H
#define MPS_ASSERT_H

#include <iostream>

#ifndef MPS_ENABLE_ASSERTS
#define MPS_ENABLE_ASSERTS true
#endif

#define MPS_UNUSED(x) do { (void)sizeof(x); } while(false)
#define MPS_HALT() goto exit

#if MPS_ENABLE_ASSERTS
	#define MPS_ASSERT(cond) do { if (!(cond)) { throw std::logic_error(""); } } while(false)
#else
	#define MPS_ASSERT(cond) MPS_UNUSED(cond)
#endif

#endif //MPS_ASSERT_H
