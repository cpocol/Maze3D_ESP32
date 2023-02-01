#include <arduino.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include "Config.h"
#include "Controller.h"
#include "main.h"
#include "Map.h"

// jump/crunch
int maxJumpHeight = int(1.2 * sqResh); // jump this high
int maxCrunchHeight = -int(0.7 * sqResh); // crunch this low
float fFPS = 30; // approximate FPS
int acceleratedMotion[200];
int maxJump_idx, maxCrunch_idx;
int verticalAdvance = 0;
int jumping = 0, crunching = 0;
int z = 0; // same unit as sqRes

void move(int& x, int& y, int angle) {
    float rad = angle * 6.2831f / around;
    int xTest = x + int(MOVE_SPD * cos(rad));
    int yTest = y + int(MOVE_SPD * sin(rad));

    // check for wall collision
    int safetyX = ((aroundq < angle) && (angle < around3q)) ? +safety_dist : -safety_dist;
    int safetyY = (angle < aroundh) ? -safety_dist : +safety_dist;
    int adjXMap = ((aroundq < angle) && (angle < around3q)) ? -1 : 0;
    int adjYMap = (angle > aroundh) ? -1 : 0;

    int xWall, yWall;
    int wallID = Cast(angle, xWall, yWall);
    if (sq(x - xTest) + sq(y - yTest) >= sq(x - xWall) + sq(y - yWall)) { // inside wall
        if (wallID % 2 == 0) { // vertical wall ||
            x = xWall + safetyX;
            y = yTest;                          //                __
            if (Map[y / sqRes][x / sqRes] != 0) // it's a corner |
                y = (yTest / sqRes - adjYMap) * sqRes + safetyY;
        }
        else { // horizontal wall ==
            x = xTest;
            y = yWall + safetyY;                //                __
            if (Map[y / sqRes][x / sqRes] != 0) // it's a corner |
                x = (xTest / sqRes - adjXMap) * sqRes + safetyX;
        }
    }
    else // free cell
        x = xTest, y = yTest;
}

void rotate(int& angle, int dir, int around) {
    angle = (angle + dir * ROTATE_SPD + around) % around;
}

void initController() {
    float fDist = 0, fSpeed = 0, G = 10.f * sqRes; // G was empirically chosen as we don't have a proper world scale here
    for (int i = 0; i < 200; i++) {
        acceleratedMotion[i] = (int)fDist;

        fSpeed += G / fFPS;
        fDist += fSpeed / fFPS;
    }

    // search for the acceleratedMotion entry so that we'll decelerate to zero speed at max jump height
    for (maxJump_idx = 0; maxJump_idx < 200; maxJump_idx++)
        if (acceleratedMotion[maxJump_idx] > maxJumpHeight)
            break;

    if (maxJump_idx >= 200) maxJump_idx = 199;

    elevation_perc = 100 * z / sqResh; // as percentage from wall half height

	pinMode(BUTTON1, INPUT_PULLUP);
	pinMode(BUTTON2, INPUT_PULLUP);
}

void loopController(int& x, int& y, int& angle, int around) {
    if (digitalRead(BUTTON1) == LOW) // pedal forward
        move(x, y, angle);
    if (digitalRead(BUTTON2) == LOW) // pedal backward
        move(x, y, (angle + around / 2) % around);

	int touchRotL = touchRead(T2);
	int touchRotR = touchRead(T5);
	int touchJump1 = touchRead(T3);
	int touchJump2 = touchRead(T4);
	int touchStrafeL = touchRead(T9);
	int touchStrafeR = touchRead(T7);
	int touchCrunch = touchRead(T8);
	Serial.println("Touch2 = " + String(touchRotL) + " Touch3 = " + String(touchJump1)
        + " Touch4 = " + String(touchJump2) + " Touch5 = " + String(touchRotR)
        + " Touch9 = " + String(touchStrafeL) + " Touch8 = " + String(touchCrunch) + " Touch7 = " + String(touchStrafeR));
    int thRotL = 80, thJump1 = 80, thJump2 = 80, thRotR = 80; // chosen empirically
    int thStrafeL = 90, thCrunch = 90, thStrafeR = 90;

    bool bJump = false, bCrunch = false;
	if ((touchJump1 < thJump1) && (touchJump2 < thJump2))
        bJump = true;
    else
	if ((touchStrafeL < thStrafeL) && (touchStrafeR < thStrafeR) && (touchCrunch < thStrafeR))
        bCrunch = true;
    else {
        if (touchRotL < thRotL) // rotate left
            rotate(angle, -ROTATE_SPD, around);
        if (touchRotR < thRotR) // rotate right
            rotate(angle, +ROTATE_SPD, around);
        if (touchStrafeL < thStrafeL) // strafe left
            move(x, y, (angle - around / 4 + around) % around);
        if (touchStrafeR < thStrafeR) // strafe right
            move(x, y, (angle + around / 4 + around) % around);
    }

    // jump
    static int jump_idx;
    if (bJump && !jumping && !crunching) {
        jumping = 1;
        verticalAdvance = 1;
        jump_idx = maxJump_idx - 1;
        z = maxJumpHeight - acceleratedMotion[jump_idx];
    }
    else
    if (jumping) {
        if (verticalAdvance > 0) {
            if (jump_idx > 0) {
                jump_idx--;
                z = maxJumpHeight - acceleratedMotion[jump_idx];
            }
            else {
                verticalAdvance = -1;
                z = maxJumpHeight;
            }
        }
        else
        if (verticalAdvance < 0) {
            if (z > 0) {
                jump_idx++;
                z = max(0, maxJumpHeight - acceleratedMotion[jump_idx]);
            }
            else {
                verticalAdvance = 0;
                jumping = 0;
            }
        }
    }

    // crunch
    if (bCrunch && !jumping) {
        crunching = 1;
        if (z > maxCrunchHeight) {
            z -= VERTICAL_SPD;
            if (z < maxCrunchHeight)
                z = maxCrunchHeight;
        }
    }
    else
    if (crunching) {
        z += VERTICAL_SPD;
        if (z >= 0) {
            z = 0;
            crunching = 0;
        }
    }
    
    elevation_perc = 100 * z / sqResh; // as percentage from wall half height
}
