struct map {
    int speed; // ticks per second
    int pwm;   // 0-255
    int mode;  // 0 = normal, 1 = coast
};

map lut[] = {
    {0,0,0},
    {60,20,0},
    {110,30,0},
    {160,40,0},
    {215,50,0},
    {265,60,0},
    {310,70,0},
    {360,80,0},
    {410,90,0},
    {455,100,0},
    {505,110,0},
    {540,120,0},
    {610,130,0},
    {650,140,0},
    {700,150,0},
    {770,160,0},
    {820,170,0},
    {850,180,0},
    {900,190,0},
    {920,200,0},
    {1000,210,0},
    {1050,220,0},
    {1100,230,0},
    {1150,240,0},
    {1200,250,0},
    {1250,255,0},


    
};