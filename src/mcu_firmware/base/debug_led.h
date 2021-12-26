#define LEDPIN                     8
#define LEDPIN_PINMODE             pinMode (LEDPIN, OUTPUT);
#define LEDPIN_OFF                 digitalWrite(LEDPIN, LOW);
#define LEDPIN_ON                  digitalWrite(LEDPIN, HIGH);