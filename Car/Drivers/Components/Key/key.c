#include "key.h"

#define KEY_PRESSED         1
#define KEY_UNPRESSED       0

#define KEY_TIME_DOUBLE	    200
#define KEY_TIME_LONG       2000
#define KEY_TIME_REPEAT     100

#define KEY_COUNT           4

uint8_t Key_Flag[KEY_COUNT];

uint8_t Key_GetState(uint8_t n) 
{
    uint32_t pinStatus;
    pinStatus = DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_I_PIN | GPIO_KEY_II_PIN | GPIO_KEY_III_PIN | GPIO_KEY_IV_PIN);

    if (n == 0) {
        if (!(pinStatus & GPIO_KEY_I_PIN)) {
            return KEY_PRESSED;
        }
    } else if (n == 1){
        if (!(pinStatus & GPIO_KEY_II_PIN)) {
            return KEY_PRESSED;
        }
    } else if (n == 2) {
        if (!(pinStatus & GPIO_KEY_III_PIN)) {
            return KEY_PRESSED;
        }
    } else if (n == 3) {
        if (!(pinStatus & GPIO_KEY_IV_PIN)) {
            return KEY_PRESSED;
        }
    }

    return KEY_UNPRESSED;
}

uint8_t Key_Check(uint32_t Key_Pin, uint8_t Flag)
{
    uint8_t n;
    if (Key_Pin == GPIO_KEY_I_PIN) {
        n = 0;
    } else if (Key_Pin == GPIO_KEY_II_PIN) {
        n = 1;
    } else if (Key_Pin == GPIO_KEY_III_PIN) {
        n = 2;
    } else if (Key_Pin == GPIO_KEY_IV_PIN) {
        n = 3;
    } else {
        n = 0;
    }

    if (Key_Flag[n] & Flag) {
        if (Flag != KEY_HOLD) {
            Key_Flag[n] &= ~Flag;
        }
        return 1;
    }
    return 0;
}

void Key_Tick(void) 
{
    static uint8_t count, i;
    static uint8_t currState[KEY_COUNT], prevState[KEY_COUNT];
    static uint8_t S[KEY_COUNT];
    static uint16_t time[KEY_COUNT];

    for (i = 0; i < KEY_COUNT; i++) {
        if (time[i] > 0) {
            time[i] --;
        }
    }
    
    count++;
    if (count >= 20) {
        count = 0;
        
        for (i = 0; i < KEY_COUNT; i++) {
            prevState[i] = currState[i];
            currState[i] = Key_GetState(i);
            
            if (currState[i] == KEY_PRESSED) {
                Key_Flag[i] |= KEY_HOLD;
            } else {
                Key_Flag[i] &= ~KEY_HOLD;
            }

            if (currState[i] == KEY_PRESSED && prevState[i] == KEY_UNPRESSED) {
                Key_Flag[i] |= KEY_DOWN;
            }

            if (currState[i] == KEY_UNPRESSED && prevState[i] == KEY_PRESSED) {
                Key_Flag[i] |= KEY_UP;
            }

            if (S[i] == 0) {
                if (currState[i] == KEY_PRESSED) {
                    time[i] = KEY_TIME_LONG;
                    S[i] = 1;
                }
            } else if (S[i] == 1) {
                if (currState[i] == KEY_UNPRESSED) {
                    time[i] = KEY_TIME_DOUBLE;
                    S[i] = 2;
                } else if (time[i] == 0) {
                    time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_LONG;
                    S[i] = 4;
                }
            } else if (S[i] == 2) {
                if (currState[i] == KEY_PRESSED) {
                    Key_Flag[i] |= KEY_DOUBLE;
                    S[i] = 3;
                } else if (time[i] == 0) {
                    Key_Flag[i] |= KEY_SINGLE;
                    S[i] = 0;
                }
            } else if (S[i] == 3) {
                if (currState[i] == KEY_UNPRESSED) {
                    S[i] = 0;
                }
            } else if (S[i] == 4) {
                if (currState[i] == KEY_UNPRESSED) {
                    S[i] = 0;
                } else if (time[i] == 0) {
                    time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_REPEAT;
                    S[i] = 4;
                }
            }
        }
    }
}
