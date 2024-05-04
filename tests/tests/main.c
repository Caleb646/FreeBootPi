
#include "unity.h"
#include "mem.h"

void setup(void)
{
    // set stuff up here
}

void tear_down(void)
{
    // clean stuff up here
}

void test_malloc(void)
{
    TEST_ASSERT_TRUE(1 == 1);
}


// not needed when using generate_test_runner.rb
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_malloc);
    return UNITY_END();
}