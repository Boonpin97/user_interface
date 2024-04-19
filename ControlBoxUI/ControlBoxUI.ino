// =======================================================================================================
// NOTES
// =======================================================================================================
//
   /* Save Button Coordinates as Struct Variables
    * Reduce the use of global variables to save program storage size?
    * (e.g. detect touch button based on the same coordinates on the display; 
    *       changing the position of the icon on the display also changes the positional boundaries for the touch)
    * Global Variables for each icon vs Raw Coordinates
    */
   
//
// =======================================================================================================
// INCLUDE LIBRARIES
// =======================================================================================================
//

//#include <SoftwareSerial.h>

#include <TouchScreen.h> //touch library
#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_KBV.h> //Hardware-specific library

#include "icons.h" // Icon Bitmaps saved in Notepad; Library

//
// =======================================================================================================
// VARIABLES 
// =======================================================================================================
//

// Serial Communication //
#define BUFFER_SIZE 64 // Adjust buffer size as needed 

char buffer[BUFFER_SIZE]; // Buffer to store received string
int index = 0; // Index to track position in buffer

typedef struct {
  int pump;
  double battery_current;
  double battery_voltage;
  double system_current;
  double system_power;
  double humidity;
  double temperature;
  double moisture;
} data_package;
data_package data;

// TFT LCD Display //
LCDWIKI_KBV lcd(ILI9486,A3,A2,A1,A0,A4); //model,cs,cd,wr,rd,reset

#define SCREEN_WIDTH   lcd.Get_Width()
#define SCREEN_HEIGHT  lcd.Get_Height()

// RGB565 //
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

// Touch Screen Coordinate System //
#define TS_MINX 906 // LEFT
#define TS_MAXX 116 // RIGHT

#define TS_MINY 92 // TOP
#define TS_MAXY 952 // BOTTOM
// We have a status line for like, is FONA working
#define STATUS_X 10
#define STATUS_Y 65

// Touch Screen Pressure Threshold //
#define MINPRESSURE 10
#define MAXPRESSURE 1000

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

int x,y; // Coordinate of Touchscreen

/* FLAGS - Icons Pressed */
bool PhaseChange = true; // Used for Changing Static Display of Specified Phase
bool startup = true;
bool power,sensors,actuators = false; // MENU of the Main Program Structure; Current Undergoing Panel

char pump[16], battery_current[16], battery_voltage[16], system_current[16], system_power[16], moisture[16], temperature[16], humidity[16];

//
// =======================================================================================================
// MAIN ARDUINO SETUP 
// =======================================================================================================
//

void setup() 
{
  Serial.begin(9600);

  //pinMode(50, INPUT);
  //pinMode(51, OUTPUT);
  //espSerial.begin(115200); // set the data rate for the SoftwareSerial port
  Serial1.begin(115200);
  
  lcd.Init_LCD();
  lcd.Fill_Screen(BLACK); 

  lcd.Set_Rotation(1); // 0: Vertical; 1: Landscape

  //displayScreenSize(); // Width: 480, Height: 320
}

//
// =======================================================================================================
// MAIN ARDUINO LOOP
// =======================================================================================================
//

void loop() 
{
  /*
  if (Serial1.available()>=sizeof(float))
  {
    byte incomingByte = Serial1.read();
    //Serial.print("Received byte from ESP32-C6: ");
    //Serial.println(incomingByte);

    // Check for termination character or buffer overflow
    if (incomingByte == '\0' || index >= BUFFER_SIZE - 1) {
      // Terminate the string
      buffer[index] = '\0';
      
      // Convert the string to float
      float floatValue = atof(buffer);
      
      // Check for conversion errors
      if (floatValue == 0 && buffer[0] != '0') {
        Serial.println("Error: Unable to convert string to float.");
      } else {
        // Print the received float value
        Serial.print("Received float value: ");
        Serial.println(floatValue);
      }
      
      // Reset index for next string
      index = 0;
    } else {
      // Check for buffer overflow
      if (index < BUFFER_SIZE - 1) {
        // Append the byte to the buffer
        buffer[index++] = incomingByte;
      } else {
        Serial.println("Error: Buffer overflow.");
        // Reset index for next string
        index = 0;
      }
    }

    // Keep Looping to Detect Any Changes Made due to Touch //
    detectTouchScreen(); 
    // Make Necessary Display Change if required //
    if (PhaseChange) // If touch screen detected, make changes to the display and switch back the 'change' to false
    {
      StaticDisplay(); // Revert Back to Original Static Display Configuration
    }
  }
  */
  
  // Check if data is available to read from ESP32-C6
  if (Serial1.available() >= sizeof(data_package)) 
  {
    Serial.println("RECEIVED!");
    // Read the incoming bytes into a buffer
    byte buffer[sizeof(data_package)];
    int bytesRead = Serial1.readBytes(buffer, sizeof(data_package));

    // Check if enough bytes are read
    if (bytesRead == sizeof(data_package)) 
    {
      // Deserialize the byte array into a struct
      memcpy(&data, buffer, sizeof(data_package));

      /*
      // Print the received struct members
      Serial.print("pump: ");
      Serial.println(receivedData.pump);
      //Serial.println(atof(receivedData.pump));
      
      Serial.print("battery_current: ");
      Serial.println(atof(receivedData.battery_current));
      Serial.print("battery_voltage: ");
      Serial.println(atof(receivedData.battery_voltage));
      Serial.print("system_current: ");
      Serial.println(atof(receivedData.system_current));
      Serial.print("system_power: ");
      Serial.println(atof(receivedData.system_power));
      
      Serial.print("humidity: ");
      Serial.println(receivedData.humidity);
      Serial.print("temperature: ");
      Serial.println(receivedData.temperature);
      Serial.print("moisture: ");
      Serial.println(receivedData.moisture);
      
      Serial.println(atof(receivedData.humidity));
      Serial.print("temperature: ");
      Serial.println(atof(receivedData.temperature));
      Serial.print("moisture: ");
      Serial.println(atof(receivedData.moisture));
      */
      // Keep Looping to Detect Any Changes Made due to Touch //
      detectTouchScreen(); 
      // Make Necessary Display Change if required //
      if (PhaseChange) // If touch screen detected, make changes to the display and switch back the 'change' to false
      {
        StaticDisplay(); // Revert Back to Original Static Display Configuration
      }
    }
    else
    {
      Serial.println("Error: Insufficient data received");
    }
  }
  else
  {
    Serial.println("NO DATA Received!");
  }
  
}

// Reconstruct the float value from the byte
float reconstructFloatValue(uint8_t byteValue) {
    // Create a union to interpret the bytes as a float
    union {
        float floatValue;
        uint8_t byteValues[sizeof(float)];
    } converter;

    // Initialize byte values
    converter.byteValues[0] = byteValue;
    converter.byteValues[1] = 0;  // Assuming little-endian byte order
    converter.byteValues[2] = 0;  // Assuming little-endian byte order
    converter.byteValues[3] = 0;  // Assuming little-endian byte order

    // Return the reconstructed float value
    return converter.floatValue;
}

//
// =======================================================================================================
// TOUCH SCREEN 
// =======================================================================================================
//

// CONTROL STRUCTURE // Startup, Power, Sensors, Actuators (Check the Graphic User Interface section for the function)
void StaticDisplay() // Display only ONCE in the main loop
{
  if (startup)
  {
    HomeInterface();
    PhaseChange = false;
  }
  if (power)
  {
    lcd.Fill_Screen(BLACK); 
    //ToggleMenu();
    PowerMenu();
    PhaseChange = false;
  }
  if (sensors)
  {
    lcd.Fill_Screen(BLACK);
    //ToggleMenu(); 
    SensorsMenu();
    PhaseChange = false;
  }
  if (actuators)
  {
    lcd.Fill_Screen(BLACK); 
    //ToggleMenu();
    ActuatorsMenu(); 
    PhaseChange = false;
  }
}

// CONTROL STRUCTURE //
void detectTouchScreen()
{
  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // Detect Touch Screen Pressed; Change Detected
  // If Released; it will not execute this if function
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) 
  {
    // CALIBRATE X/Y COORDINATE ON TOUCH SCREEN //
    // Touch Screen Coordinate System Calibration //
    // Check the configuration for p.x & p.y based on the Screen Orientation (e.g. lcd.Set_Rotation(1))
    /* (Horizontal)Bottom Left:0,0 ; Top Right:480,320 */
    /* (Vertical)Top Left:0,0 ; Bottom Right:480,320 */

    y = map(p.x, TS_MINX, TS_MAXX, 0, SCREEN_HEIGHT); // Raw ADC value has to be converted to Pixel Co-ordinates
    x = map(p.y, TS_MINY, TS_MAXY, SCREEN_WIDTH,0);
    // Coordinate Threshold -> Screen Size
    if(x<0)
      x = 0;
    else if(x>SCREEN_WIDTH)
      x = SCREEN_WIDTH;
    if(y<0)
      y = 0;
    else if(y>SCREEN_HEIGHT)
      y = SCREEN_HEIGHT;
    /*
    // Print X/Y Coordinates
    Serial.print("x-coordinate:");
    Serial.print(x);
    Serial.print("\ty-coordinate:");
    Serial.println(y);
    */

    /* Check Which Part of the Screen is Touched under the Specified Phase; Change Attempt Detected; Dynamic Value Change to Display */
    // HOME SCREEN //
    if (startup) /* Smart Farm */
      detectStartup(); // Start Button 
    // POWER MENU //
    if (power) /* Battery (Capacity, Current, Voltage); System (Current, Power) */
      detectControlPanels(); // Power Button
      //PowerMenu();
    // SENSORS MENU // 
    if (sensors) /* Humidity, Moisture, Temperature */
      detectControlPanels(); // Sensors Button
      //SensorsMenu();
    // ACTUATORS MENU //
    if (actuators) /* Pump */
      detectControlPanels(); // Actuators Button
      //ActuatorsMenu();
  }
}

// STARTUP
void detectStartup()
{
  if (buttonPressed(170,110,310,250,x,y)) // POWER button pressed
  {
    startup = false;
    sensors = false;
    actuators = false;
    
    power = true;
    PhaseChange = true;
  }
}

// POWER, SENSORS, ACTUATORS
void detectControlPanels() // Change the Control Panels to view the parameters
{ 
  if (buttonPressed(0,0,130,50,x,y)) // Power Panel Position
  {
    startup = false;
    sensors = false;
    actuators = false;
    
    power = true;
    PhaseChange = true;
  }
  if (buttonPressed(130,0,280,50,x,y)) // Sensors Panel Position
  {
    startup = false;
    power = false;
    actuators = false;
    
    sensors = true;
    PhaseChange = true;
  }
  if (buttonPressed(280,0,478,50,x,y)) // Actuators Panel Position
  {
    startup = false;
    power = false;
    sensors = false;
    
    actuators = true;
    PhaseChange = true;
  }
}

bool buttonPressed(int16_t x1,int16_t y1,int16_t x2,int16_t y2,int16_t px,int16_t py)
{
  if((px > x1 && px < x2) && (py > y1 && py < y2)) // Press Angle p.x & p.y within the button boundary
  {
    return true;  
  } 
  else
  {
    return false;  
  }
}

//
// =======================================================================================================
// GRAPHICS USER INTERFACE (Tree Menu) - Startup, Power, Sensors, Actuators
// =======================================================================================================
//

void showString(uint8_t *str,int16_t x,int16_t y,uint8_t csize,uint16_t fc, uint16_t bc,boolean mode)
{
  lcd.Set_Text_Mode(mode); // text overlap over the button
  lcd.Set_Text_Size(csize); // text size
  lcd.Set_Text_colour(fc); // font color
  lcd.Set_Text_Back_colour(bc); // background color
  lcd.Print_String(str,x,y); // print string
}

void showIcon(const uint8_t *color_buf,int16_t buf_size,int16_t x1,int16_t y1,int16_t x2,int16_t y2)
{
  lcd.Set_Addr_Window(x1, y1, x2, y2); // Size of the Picture (Rectangle)
  lcd.Push_Any_Color(color_buf, buf_size, 1, 1);
}

void displayScreenSize()
{
  /*(Vertical) Width:320; Height:480  */
  /*(Horizontal) Width:480; Height:320  */
  Serial.print("Screen_WIDTH:");
  Serial.print(lcd.Get_Display_Width()); 
  Serial.print("\tScreen_HEIGHT:");
  Serial.println(lcd.Get_Display_Height()); 
}

// START UP -> [DONE]
void HomeInterface() 
{
  // HEADING //
  showString("[ SMART FARM ]",(lcd.Get_Display_Width()/2)-120,10,3,WHITE,BLACK,1); // string,x,y,textsize,fontcolor,backgroundcolor,overlapmode
  
  // POWER BUTTON //
  lcd.Set_Draw_color(WHITE); 
  lcd.Fill_Circle(240,180,110); // Button FRAME
  lcd.Set_Draw_color(RED); 
  lcd.Fill_Circle(240,180,100); // Button BODY
  lcd.Set_Draw_color(WHITE); 
  lcd.Fill_Rectangle(230,140,250,220); // Slot
  lcd.Fill_Circle(240,140,10);
  lcd.Fill_Circle(240,220,10);
}

// TOGGLE MENU BUTTON 
void ToggleMenu() // header buttons to toggle the mode
{
  // HEADING //
  lcd.Set_Draw_color(RED);
  lcd.Fill_Round_Rectangle(0,0,130,50,8);
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(0,0,130,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("POWER",22,15,3,WHITE,BLACK,1);

  lcd.Set_Draw_color(BLUE);
  lcd.Fill_Round_Rectangle(130,0,280,50,8);
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(130,0,280,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("SENSORS",145,15,3,WHITE,BLACK,1);
  
  lcd.Set_Draw_color(MAGENTA);
  lcd.Fill_Round_Rectangle(280,0,478,50,8);
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(280,0,478,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("ACTUATORS",300,15,3,WHITE,BLACK,1);
}

// POWER
void PowerMenu()
{
  // Convert double to string
  //dtostrf(data.battery_current,10,2,battery_current);
  //dtostrf(data.battery_voltage,10,2,battery_voltage);
  //dtostrf(data.system_current,10,2,system_current);
  //dtostrf(data.system_power,10,2,system_power);
  
  // MENU SELECTION //
  lcd.Set_Draw_color(RED);
  lcd.Fill_Round_Rectangle(0,0,130,50,8);
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(0,0,130,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("POWER",22,15,3,WHITE,BLACK,1);

  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(130,0,280,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("SENSORS",145,15,3,WHITE,BLACK,1);
  
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(280,0,478,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("ACTUATORS",300,15,3,WHITE,BLACK,1);
  
  
  // HEADING //
  showString("[ BATTERY ]",(lcd.Get_Display_Width()/2)-220,(lcd.Get_Display_Height()/2)-100,2,WHITE,BLACK,1); // string,x,y,textsize,fontcolor,backgroundcolor,overlapmode
  showString("[ SYSTEM ]",(lcd.Get_Display_Width()/2)-220,(lcd.Get_Display_Height()/2)+25,2,WHITE,BLACK,1); // string,x,y,textsize,fontcolor,backgroundcolor,overlapmode

  // PARAMETERS - Battery //
  showString("Current: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  //showString(battery_current,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  showString("0.24A",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  showString("Voltage: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)-50,2,WHITE,BLACK,1);
  //showString(battery_voltage,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-50,2,WHITE,BLACK,1); 
  showString("3.74V",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-50,2,WHITE,BLACK,1); 

  // PARAMETERS - System //
  showString("Current: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)+50,2,WHITE,BLACK,1);
  //showString(system_current,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)+50,2,WHITE,BLACK,1); 
  showString("0.19A",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)+50,2,WHITE,BLACK,1); 
  showString("Power: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)+75,2,WHITE,BLACK,1); 
  //showString(system_power,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)+75,2,WHITE,BLACK,1); 
  showString("0.95W",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)+75,2,WHITE,BLACK,1);
}

// SENSORS
void SensorsMenu()
{
  // Convert double to string
  //dtostrf(data.humidity,10,2,humidity);
  //dtostrf(data.moisture,10,2,moisture);
  //dtostrf(data.temperature,10,2,temperature);
  
  // MENU SELECTION //
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(0,0,130,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("POWER",22,15,3,WHITE,BLACK,1);

  lcd.Set_Draw_color(BLUE);
  lcd.Fill_Round_Rectangle(130,0,280,50,8);
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(130,0,280,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("SENSORS",145,15,3,WHITE,BLACK,1);
  
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(280,0,478,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("ACTUATORS",300,15,3,WHITE,BLACK,1);
  

  // HEADING //
  showString("[ SENSORS ]",(lcd.Get_Display_Width()/2)-220,(lcd.Get_Display_Height()/2)-100,2,WHITE,BLACK,1); 
 
  // PARAMETERS //
  showString("Humidity: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  //showString(humidity,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  showString("77%",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  showString("Moisture: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)-50,2,WHITE,BLACK,1);
  //showString(moisture,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-50,2,WHITE,BLACK,1); 
  showString("0%",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-50,2,WHITE,BLACK,1); 
  showString("Temperature: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)-25,2,WHITE,BLACK,1);
  //showString(temperature,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-25,2,WHITE,BLACK,1);  
  showString("23 Degree Celsius",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-25,2,WHITE,BLACK,1); 

}

// ACTUATORS
void ActuatorsMenu()
{
  // Convert int to string
  //itoa(data.pump,pump,10);
  
  // MENU SELECTION //
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(0,0,130,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("POWER",22,15,3,WHITE,BLACK,1);

  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(130,0,280,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("SENSORS",145,15,3,WHITE,BLACK,1);
  
  lcd.Set_Draw_color(MAGENTA);
  lcd.Fill_Round_Rectangle(280,0,478,50,8);
  lcd.Set_Draw_color(WHITE);
  lcd.Draw_Round_Rectangle(280,0,478,50,8); // White Rectangular FRAME; Top Left to Bottom Right Corner, Radius
  showString("ACTUATORS",300,15,3,WHITE,BLACK,1);

  
  // HEADING //
  showString("[ PUMP ]",(lcd.Get_Display_Width()/2)-220,(lcd.Get_Display_Height()/2)-100,2,WHITE,BLACK,1); 
 
  // PARAMETERS //
  showString("Status: ",(lcd.Get_Display_Width()/2)-200,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  //showString(pump,(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
  showString("0",(lcd.Get_Display_Width()/2)-50,(lcd.Get_Display_Height()/2)-75,2,WHITE,BLACK,1); 
}
