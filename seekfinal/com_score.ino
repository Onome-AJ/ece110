/******************************************************************************
 * Messaging Protocol
 * 
 * 8-bit message is sent
 * _ _ _, _ _ _ _ _
 * MSB          LSB
 * 
 * 3 MSB are the robot_id, 5 LSB are the message
 * 
 * Robots will continuously broadcast their data; once a robot receives data
 * from all other robots, it will display the score on its LCD and continue
 * to broadcast its data.
 */

 //Comment out to disable ALL serial debug stream
#define DEBUG
#ifdef DEBUG
///Comment out to disable notification of generic message broadcasting
#define DEBUG_ALL_BROADCAST

///Comment out to disable notification of generic message receipt
#define DEBUG_ALL_RECEIPT
#endif

///Number of milliseconds to listen between each message broadcast
#define LISTEN_DELAY 500

// Special code for data which is NULL (invalid)
#define NULL_DATA -1

/**
 * Robot ID: the robot sending the data
 * Data: the robot-specific data to send
 */
char make_data_message(int robot_id, int data) {
  return (robot_id & 7) << 5 | (data & 31);
}

/**
 * Robot ID: the robot sending the data
 * Data: the robot-specific data to send
 */
void interpret_data_message(char message, int * robot_id, int * data) {
  if (robot_id) *robot_id = (message >> 5) & 7;
  if (data) *data = message & 31;
}

/**
 * Check that all data from other robots has been received
 */
boolean check_all_data(int * data) {
  return data[0] != NULL_DATA && data[1] != NULL_DATA && data[2] != NULL_DATA && data[3] != NULL_DATA && data[4] != NULL_DATA;
}

/**
 * Broadcast a message
 * Debugging print statements included
 */
void broadcast(int my_id, char message) {
  Serial2.print(message);

  #ifdef DEBUG_ALL_BROADCAST
  static char buf[32];
  int robot_id, data;
  interpret_data_message(message, &robot_id, &data);
  sprintf(buf, "[%d] Broadcast: Data %d", my_id, data);
  Serial.println(buf);
  #endif
}

/**
 * Wait until either of the following happens:
 *   - the Arduino's timer reaches a certain time
 *       RETURNS NULL_DATA
 *   - a message is received from the XBee
 *       RETURNS (char)message
 * Debugging print statements included
 */
char receive_until(int my_id, long t_end) {
  char message = NULL_DATA;
  while (millis() < t_end && message == NULL_DATA) {
    if (Serial2.available()) {
      message = Serial2.read();
    }
  }

  // Validate message
  if (message != NULL_DATA) {
    int robot_id, data;
    interpret_data_message(message, &robot_id, &data);
    if (robot_id < 0 || robot_id > 4) {
      message = NULL_DATA;
    }
  }

  #ifdef DEBUG_ALL_RECEIPT
  if (message != NULL_DATA) {
    static char buf[32];
    int robot_id, data;
    interpret_data_message(message, &robot_id, &data);
    sprintf(buf, "[%d] Received [%d]: Data %d", my_id, robot_id, data);
    Serial.println(buf);
  }
  #endif
  
  return message;
}

/**
 * Compute score from data received from robots
 */
int compute_score(int * data) {
  // Count number of aligned bits in Chaser 1 and Beater 1 (or 2) data
  // 10 points for each aligned bit, 150 if the snitch is found
  int beater_chaser_1 = data[ROBOT_ID_CHASER1] & data[ROBOT_ID_BEATER1];
  int beater_chaser_2 = data[ROBOT_ID_CHASER2] & data[ROBOT_ID_BEATER2];
  int seeker = !!data[ROBOT_ID_SEEKER]; // cast to 0 or 1
  return __builtin_popcount(beater_chaser_1) * 10 + __builtin_popcount(beater_chaser_2) * 10 + seeker * 150;
}

int communicate_score(int my_id, int my_data) {
  while (Serial2.available()) Serial2.read();

  static char buf[32];

  // Initialize data
  char my_message = make_data_message(my_id, my_data);

  // Data sent by other robots
  int robots_data[5] = {NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA};
  robots_data[my_id] = my_data;
  boolean all_data = false;

  // Broadcast and receive data
  // Loop stops once all data is received
  while (!all_data) {

    // Broadcast and listen when not broadcasting
    broadcast(my_id, my_message);

    long t_end = millis() + LISTEN_DELAY;
    char message;
    while ((message = receive_until(my_id, t_end)) != NULL_DATA) {

      // Signal received; change data
      int robot_id, data;
      interpret_data_message(message, &robot_id, &data);

      // Data received from a robot
      // If this is new data, check again if all data has been received
      char old_data = robots_data[robot_id];
      robots_data[robot_id] = data;
      if (robots_data[robot_id] != old_data) {
        //robots_data[robot_id] = data;
        all_data = check_all_data(robots_data);
        #ifdef DEBUG
        sprintf(buf, "[%d] Data from [%d]: %d", my_id, robot_id, data);
        Serial.println(buf);
        if (all_data) {
          sprintf(buf, "[%d] --- All Data ---", my_id);
          Serial.println(buf);
        }
        #endif
      }
    }
  }

  // Calculate score and display to LCD
  int final_score = compute_score(robots_data);
  sprintf(buf, "Score: %d", final_score);
  Serial3.println(final_score);
  #ifdef DEBUG
  sprintf(buf, "%d, %d, %d, %d, %d",
  robots_data[0],  robots_data[1],  robots_data[2],  robots_data[3],  robots_data[4]);
  Serial.println(buf);
  
  sprintf(buf, "**** Final Score: %d ****", final_score);
  Serial.println(buf);
  #endif

  // Broadcast data only
  // Loop continues forever
  while (true) {
    broadcast(my_id, my_message);
    delay(LISTEN_DELAY);
  }
}
