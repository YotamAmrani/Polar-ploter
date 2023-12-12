#include "Drawings.h"
#define NUMBER_OF_DRAWINGS 3

const static PROGMEM int16_t spiral_sphere[5][N_INSTRUCTIONS] = {{-50, 50, PEN_OFF},
                                                                    {50, 50, PEN_ON},
                                                                    {50, -50, PEN_ON},
                                                                    {-50,-50, PEN_ON},
                                                                    {-50, 50, PEN_ON}
                                                                    };

const static PROGMEM int16_t hexagon[7][N_INSTRUCTIONS] = {
                                                          {72, 0, PEN_OFF},
                                                          {36, 62, PEN_ON},
                                                          {-36, 62, PEN_ON},
                                                          {-72, 0, PEN_ON},
                                                          {-36, -62, PEN_ON},  // Closing the triangle
                                                          {36, -62, PEN_ON},  // Closing the triangle
                                                          {72, 0, PEN_ON}
                                                          };

const static PROGMEM int16_t triangle[4][N_INSTRUCTIONS] = {
                                                          {72, 0, PEN_OFF},
                                                          {-36, 62, PEN_ON},
                                                          {-36, -62, PEN_ON},
                                                          {72, 0, PEN_ON}
                                                          };






Drawing drawings[NUMBER_OF_DRAWINGS] = {Drawing(spiral_sphere, 5, 1500, true),
                                        Drawing(hexagon, 7, 1500, true),
                                        Drawing(triangle, 4, 1500, true)
                                        };
