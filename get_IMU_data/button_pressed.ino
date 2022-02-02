void button_pressed(int button_pin, int* buttonState, int* lastButtonState, unsigned long* lastDebounceTime, int led_pin, int* led_state) {
  unsigned int debounceDelay = 50;
  // read the state of the switch into a local variable:
  int reading = digitalRead(button_pin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != *lastButtonState) {
    // reset the debouncing timer
    *lastDebounceTime = millis();
  }

  if ((millis() - *lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != *buttonState) {
      *buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (*buttonState == HIGH) {
        *led_state = !(*led_state);

      }
    }
  }

  // set the LED:
  digitalWrite(led_pin, *led_state);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  *lastButtonState = reading;
}
