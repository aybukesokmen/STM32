#include <stdio.h>
#include <assert.h>

int sum(int a, int b) {
  int result = a + b;
  return result;
}


void test_sum() {
  int result = sum(2, 3);
  if(result == 5) {
    printf("Test OK\n");
  } else {
    printf("Test FAILED\n");
  }
}


int main()
{
    test_sum();

    return 0;
}

