
#include <stdio.h>
#include <stdint.h>
#include <math.h>

int main() {
    float dt = 0.0f;
    float err = 10.0f;
    float kd = 0.0f;
    float dout = kd * err / dt;
    float pout = 0.4f;
    float output = pout + dout;
    int16_t set = (int16_t)output;
    printf("dout: %f, out: %f, set: %d\n", dout, output, set);
    return 0;
}

