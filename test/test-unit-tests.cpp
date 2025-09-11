/* sources:
https://piolabs.com/blog/insights/unit-testing-part-1.html
https://piolabs.com/blog/insights/unit-testing-part-2.html
https://piolabs.com/blog/insights/unit-testing-part-3.html

      - uses: actions/cache@v4 what does this do in .yml

      build - fails native, works portenta
      test - works native, fails portenta
*/

// #include <Arduino.h>
#include <unity.h>

void setUp(void) { // the tutorial said these 2 funcs are needed?
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_load_sd_card()
{
  bool stuff {};
    stuff = true; // did some stuff

  TEST_ASSERT_TRUE(stuff);
}

void test_clear_sd_card()
{
  TEST_ASSERT_TRUE(true);
}

int main()
{
  UNITY_BEGIN();

  RUN_TEST(test_load_sd_card);
  RUN_TEST(test_clear_sd_card);

  UNITY_END();
  return 0;
}