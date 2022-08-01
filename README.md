# ESP32_AD7190

Example how to use AD7190 with ESP32 board.
It's been tested with AD7190 strain acquisition module:
https://github.com/gism/ESP32_AD7190/blob/main/Documentation/AD7190_Test_Board.pdf

//    Conection as follows:
//    
//    AD7190 board -- ESP32 board
//           GND   --  GND
//           SCLK  --  GPIO14
//           DIN   --  GPIO13 (MISO)
//           SYNC  --  Not connected
//       DOUT/RDY  --  GPIO12 (MOSI)
//           CS    --  GPIO15
//           VIN   --  3V3
