This is a library to control the Link Labs' Symphony Link evaluation board using an Arduino Due.

The Due controls the Symphony Link module (LL-RLP-20 or LL-RXR-27) on the evaluation board by connecting the Due's Serial1 interface (Arduino Due pins 18 and 19) to the module's UART interface. Useful debug information is printed to the Arduino IDE's serial monitor using the Serial interface (Arduino Due pins 0 and 1).

With Rev 6 of the Symphony Link evaluation board (LL3B60), the nRESET line was relocated from Arduino pin 2 to Arduino pin 7. If using evaluation board Rev 5 or earlier, define SYMPHONY_RESET_PIN in SymphonyLink.h as 2. If using evaluation board Rev 6 or later, define SYMPHONY_RESET_PIN in SymphonyLink.h as 7.

See the following links for hardware setup and product information:
http://docs.link-labs.com/m/52162/c/155625
http://docs.link-labs.com/m/52162/l/554388-connecting-the-evaluation-board-to-arduino-due

This Library is released into the public domain.

To download, click the DOWNLOAD ZIP button. Then, follow the instructions at https://www.arduino.cc/en/Guide/Libraries to install a zip library into the Arduino IDE.
