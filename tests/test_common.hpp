#pragma once

#include <iostream>
#include <fstream>
#include <cstdlib>

#define NAV_TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "\n[ASSERTION FAILED] " << #cond << " : " << (msg) \
                      << "\nLocation: " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)
