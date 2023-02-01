#pragma once

#define MOVE_SPD (10 * sqRes / 100)
#define ROTATE_SPD 3
#define VERTICAL_SPD (sqRes/25)

// Buttons
#define BUTTON1 35
#define BUTTON2 0

// bits of fptype >= (number of bits for sqRes) + fp + (1 bit for sign) + X
// where X = max((number of bits for mapSize), (number of bits for the integral part of tan/ctan))

const int fp = 14; // fixed point position
typedef int32_t fptype;

const int screenW = 240, screenH = 135, screenWh = screenW / 2, screenHh = screenH / 2;
const int around = 6 * screenW, aroundh = around / 2, aroundq = around / 4, around3q = 3 * aroundq; // FOV = 60 degs (6 FOVs = 360 degrees)

const int sqRes = (1 << 10), sqResh = sqRes / 2;
const fptype sqRes_fp = (fptype)sqRes << fp;
const int safety_dist = 3; // to wall

const int texRes = (1 << 7);

// viewer Current position, orientation and elevation_perc
extern int xC, yC, angleC;
extern int elevation_perc; // percentage

extern int showMap;
