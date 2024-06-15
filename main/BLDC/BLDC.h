typedef struct {
    int GPIO;
    float throttle; // 0 to 1
    int channel;
} BLDC;


void BLDC_create(BLDC* bldc, int gpio, int channel);

void BLDC_set_throttle(BLDC* bldc, float throttle);
