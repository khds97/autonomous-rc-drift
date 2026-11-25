// test_common.hpp
// Minimal check macro so the tests need no test framework.
#pragma once

#include <cstdio>
#include <cstdlib>

static int g_failures = 0;

#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      std::printf("FAIL %s:%d  %s  ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
      ++g_failures; \
    } \
  } while (0)

#define TEST_RESULT() \
  (g_failures == 0 ? (std::printf("PASS\n"), 0) : (std::printf("%d check(s) failed\n", g_failures), 1))
