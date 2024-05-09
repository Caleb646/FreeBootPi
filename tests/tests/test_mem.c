#include "unity.h"
//#include "mem.h"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_malloc(void)
{
    TEST_ASSERT_TRUE(1 == 1);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_malloc);
    return UNITY_END();
}