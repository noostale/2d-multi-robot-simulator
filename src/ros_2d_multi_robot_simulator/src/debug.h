#pragma once

#ifdef ENABLE_DEBUG
#define DEBUG_PRINT(msg) std::cout << msg << std::endl
#else
#define DEBUG_PRINT(msg)
#endif