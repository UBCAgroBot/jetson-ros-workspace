/*
Reads Serial data from the USB and prints onto serial monitor.
To be used in conjunction with keyboard_controller_lrc.py
*/

void setup() {
  // Set up the serial communication at a baud rate of 9600
  Serial.begin(9600);
}

char buf[80];

void loop() {
    if (readline(Serial.read(), buf, 80) > 0) {
      Serial.print("Message Received: ");
      Serial.print(buf);
      Serial.println("\n");

      // Print the ASCII values of each character in the string
//      Serial.print("Message received: [");
//      for (int i = 0; i < sizeof(buf); i++) {
//        if (i > 0) {
//          Serial.print(", ");
//        }
//       Serial.print(int(buf[i]));
//      }
//      Serial.println("]");
    }
}

int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1 && readch > 0 && readch < 128) {
//                    Serial.println(readch);
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}
