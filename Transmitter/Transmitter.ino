void setup() {
  pinMode(9, OUTPUT); 
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  TCCR1A |= (1 << COM1A1);
  ICR1 = 399;  
  OCR1A = 160; 
  TCCR1B |= (1 << CS10);
}