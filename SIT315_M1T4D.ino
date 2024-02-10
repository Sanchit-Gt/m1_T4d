// Define pin assignments
const int pirPin = 2;         // PIR sensor pin
const int buttonPin = 3;      // Button pin
int led1 = 13;                // LED 1 pin
int led2 = 12;                // LED 2 pin
int led3 = 4;                 // LED 3 pin

// Define variables and constants
uint8_t pirState, buttonState;    // Store PIR and button states
uint8_t prepir, prebutton = 0;    // Store previous states
const uint16_t timer = 0;         // Initial timer value
const uint16_t compare = 31250;   // Timer comparison value

// Setup function - runs once at startup
void setup() {
  Serial.begin(9600);        // Initialize serial communication
  pinMode(pirPin, INPUT);    // Set PIR pin as input
  pinMode(buttonPin, INPUT_PULLUP);  // Set button pin with internal pull-up resistor
  pinMode(led1, OUTPUT);     // Set LED 1 pin as output
  pinMode(led2, OUTPUT);     // Set LED 2 pin as output
  pinMode(led3, OUTPUT);     // Set LED 3 pin as output
  
  // Enable Pin Change Interrupts for pins 2 and 3
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  
  // Configure Timer1 settings
  TCCR1A = 0;                // Reset Timer1 Control Registers
  TCCR1B |= (1 << CS12);     // Set Timer1 prescaler to 256
  TCCR1B &= ~(1 << CS11);    // (prescaler is 256)
  TCCR1B &= ~(1 << CS10);    // (prescaler is 256)
  TCNT1 = timer;             // Set Timer1 count value
  OCR1A = compare;           // Set Timer1 comparison value
  TIMSK1 = (1 << OCIE1A);    // Enable Timer1 Output Compare A Match interrupt
  sei();                     // Enable global interrupts
}

// Loop function - runs repeatedly after setup
void loop() {
  // Check for changes in PIR sensor state
  if (prepir != pirState) {
    digitalWrite(led1, pirState);  // Set LED 1 based on PIR sensor
    Serial.println("PIR motion detected"); // Print PIR detection message
    prepir = pirState;  // Update previous PIR state
  }
  // Check for changes in button state
  if (prebutton != buttonState) {
    // Toggle LED 2 state
    if (digitalRead(led2) == LOW) {
      digitalWrite(led2, HIGH); // Turn on LED 2
    } else {
      digitalWrite(led2, LOW);  // Turn off LED 2
    }
    Serial.println("Button pressed"); // Print button press message
    prebutton = buttonState;  // Update previous button state
  }
  // Toggle LED 3 state using XOR operation
  PORTB ^= (1 << led3);
}

// Pin Change Interrupt Service Routine
ISR(PCINT2_vect) {
  // Update PIR and button states based on pin readings
  pirState = PIND & (1 << PD2);    // Read PIR pin state
  buttonState = PIND & (1 << PD3); // Read button pin state
}

// Timer1 Compare Match A Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
  TCNT1 = timer;                          // Reset Timer1 count value
  digitalWrite(13, digitalRead(13) ^ 1);  // Toggle built-in LED state
}
