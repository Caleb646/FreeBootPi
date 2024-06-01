#include "mem.h"
#include "unity.h"

void setUp (void) {
    // set stuff up here
}

void tearDown (void) {
    // clean stuff up here
}

void test_malloc (void) {
    char test_buffer[1024];
    heap_t heap = { .start          = (char*)test_buffer,
                    .end            = &(test_buffer[1024]),
                    .cur_pos        = (char*)test_buffer,
                    .size           = 1024,
                    .free_list_head = NULL };
    TEST_ASSERT_TRUE (1 == 1);
}

int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_malloc);
    return UNITY_END ();
}