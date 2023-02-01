#include <math.h>
#include <TFT_eSPI.h>   // Board lib for ESP32 TTGO T-Display (ESP32 Dev Module)
#include "time.h"

#include "Config.h"
#include "Controller.h"
#include "Map.h"
#include "Texture.h"

#define sq(x) ((x)*(x))
#define min(a,b) ((a) < (b) ? (a) : (b))

// Buttons
#define BUTTON1 35
#define BUTTON2 0

// display
TFT_eSPI display = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h

uint16_t* screen = new uint16_t[screenW * screenH];

const int16_t mapSizeHeight = mapHeight * sqRes, mapSizeWidth = mapWidth * sqRes;
const fptype mapSizeWidth_fp = (((fptype)mapSizeWidth) << fp), mapSizeHeight_fp = (((fptype)mapSizeHeight) << fp);

int32_t Tan_fp[around]; // fp bits fixed point
int32_t CTan_fp[around];

//initial
int xC = 2.5 * sqRes;
int yC = 2.5 * sqRes;
int angleC = 1400;
int elevation_perc = 0; //as percentage from wall half height

float X2Rad(int X) {
	return (float)X * 3.1415f / aroundh;
}

void setup() {
	int16_t i, j;
	// precalculate
	for (int16_t a = 0; a < around; a++) {
		float angf = X2Rad(a);

        // tangent (theoretical range is [-inf..+inf], in practice (-128..+128) is fine)
        fptype maxTan = (((fptype)128) << fp) - 1;
		float temp = tan(angf) * (1 << fp);
        Tan_fp[a] = (fptype)temp;
        if (temp > maxTan)
            Tan_fp[a] = maxTan;
        if (temp < -maxTan)
            Tan_fp[a] = -maxTan;

		// cotangent
        temp = 1 / tanf(angf) * (((fptype)1) << fp);
        CTan_fp[a] = (fptype)temp;
        if (temp > maxTan)
            CTan_fp[a] = maxTan;
        if (temp < -maxTan)
            CTan_fp[a] = -maxTan;
	}

	display.init();
	display.setRotation(1);
	display.setSwapBytes(true);
	display.fillScreen(TFT_BLACK);

    initController();

    Serial.begin(115200);
    Serial.println("Start");
}

//returns wall ID (as map position and cell face)
int CastX(int16_t angle, fptype& xHit_fp, fptype& yHit_fp) { // hit vertical walls ||
    if ((angle == aroundq) || (angle == around3q))
        return -1; // CastY() will hit a wall correctly

    // prepare as for 1st or 4th quadrant
    int x = (xC / sqRes) * sqRes + sqRes;
    int dx = sqRes,   adjXMap = 0;
    fptype dy_fp = sqRes * Tan_fp[angle];
    // 2nd or 3rd quadrant
    if ((aroundq < angle) && (angle < around3q)) {
        x -= sqRes;
        adjXMap = -1;
        dx = -dx;
        dy_fp = -dy_fp;
    }
    yHit_fp = (((fptype)yC) << fp) + (x - xC) * Tan_fp[angle];

    while ((0 < x) && (x < mapSizeWidth) && (0 < yHit_fp) && (yHit_fp < mapSizeHeight_fp) &&
           (Map[(yHit_fp >> fp) / sqRes][x / sqRes + adjXMap] == 0)) {
        x += dx;
        yHit_fp += dy_fp;
    }

    xHit_fp = (fptype)x << fp;

    return int((yHit_fp / sqRes_fp) * mapWidth + (x / sqRes + adjXMap));
}

// returns wall ID (as map position and cell face)
int CastY(int16_t angle, fptype& xHit_fp, fptype& yHit_fp) { // hit horizontal walls ==
    if ((angle == 0) || (angle == aroundh))
        return -1; // CastX() will hit a wall correctly

    // prepare as for 1st or 2nd quadrant
    int y = (yC / sqRes) * sqRes + sqRes;
    int dy = sqRes,   adjYMap = 0;
    fptype dx_fp = sqRes * CTan_fp[angle];
    if (angle > aroundh) { // 3rd or 4th quadrants
        y -= sqRes;
        adjYMap = -1;
        dy = -dy;
        dx_fp = -dx_fp;
    }
    xHit_fp = (((fptype)xC) << fp) + (y - yC) * CTan_fp[angle];

    while ((0 < xHit_fp) && (xHit_fp < mapSizeWidth_fp) && (0 < y) && (y < mapSizeHeight) &&
           (Map[y / sqRes + adjYMap][(xHit_fp >> fp) / sqRes] == 0)) {
        xHit_fp += dx_fp;
        y += dy;
    }

    yHit_fp = (fptype)y << fp;

    return int((y / sqRes + adjYMap) * mapWidth + (xHit_fp / sqRes_fp));
}

// returns wall ID (as map position and cell face)
int Cast(int angle, int& xHit, int& yHit) {
    fptype xX_fp = 1000000000, yX_fp = xX_fp, xY_fp = xX_fp, yY_fp = xX_fp;
    int wallIDX = CastX(angle, xX_fp, yX_fp);
    int wallIDY = CastY(angle, xY_fp, yY_fp);
    // choose the nearest hit point
    if (llabs(((fptype)xC << fp) - xX_fp) < llabs(((fptype)xC << fp) - xY_fp)) { // vertical wall ||
        xHit = int(xX_fp >> fp);
        yHit = int(yX_fp >> fp);
        return 2 * wallIDX + 0;
    }
    else { // horizontal wall ==
        xHit = int(xY_fp >> fp);
        yHit = int(yY_fp >> fp);
        return 2 * wallIDY + 1;
    }
}

void RenderColumn(int col, int h, int textureColumn) {
    int32_t Dh_fp = (texRes << 22) / h; // 1 row in screen space is this many rows in texture space; use fixed point
    int32_t textureRow_fp = 0;
    //int minRow = screenHh - h / 2; // no elevation
    int minRow = ((100 - elevation_perc) * (2 * screenHh - h) / 2 + elevation_perc * screenHh) / 100;
    int maxRow = min(minRow + h, screenH);

    int minRowOrig = minRow;
    if (minRow < 0) { // clip
        textureRow_fp = -(minRow * Dh_fp);
        minRow = 0;
    }

	uint16_t* screenAddr = screen + minRow * screenW + col;
	//const uint16_t* textureAddr = Texture + textureColumn;
	const uint16_t* textureAddr = Texture + textureColumn * texRes; // huge speedup: 90 degs pre-rotated texture
    for (int row = minRow; row < maxRow; row++) {
		//*screenAddr = *(textureAddr + textureRow_fp / 1024 * texRes); // access texture column wise
		*screenAddr = *(textureAddr + (textureRow_fp >> 22)); // rotated texture - access it row wise (cache memory principles)
		textureRow_fp += Dh_fp;
		screenAddr += screenW;
	}
}

auto t_prev = millis();
void Render() {
	auto t_start = millis();

	memset(screen, 0, sizeof(uint16_t) * screenW * screenH);
	auto t_clear = millis();

	int32_t viewerToScreen_sq = sq(screenWh) * 3; // FOV = 60 degs => viewerToScreen = screenWh * sqrt(3)
	uint32_t textureColumn;
	for (int16_t col = 0; col < screenW; col++) {
		int16_t ang = (screenWh - col + angleC + around) % around;
        int xHit, yHit;
        Cast(ang, xHit, yHit);

		uint32_t textureColumn = ((xHit + yHit) % sqRes) * texRes / sqRes;

        int dist_sq = sq(xC - xHit) + sq(yC - yHit) + 1; // +1 avoids division by zero
        int h = int(sqRes * sqrt((viewerToScreen_sq + sq(screenWh - col)) / (float)dist_sq) + 0.5);

        RenderColumn(col, h, textureColumn);
	}

    // mirror image; we need this because the map's CS is left handed while the ray casting works right handed
    for (int row = 0; row < screenH; row++)
        for (int col = 0; col < screenWh; col++) {
            int16_t aux = *(screen + row * screenW + col);
            *(screen + row * screenW + col) = *(screen + row * screenW + screenW - 1 - col);
            *(screen + row * screenW + screenW - 1 - col) = aux;
        }
    
	auto t_render = millis();

	display.pushImage(0, 0, screenW, screenH, screen);
	auto t_show = millis();
	
	int y = 1;
	//display.setCursor(1, y);   display.printf("clear:  %2d", t_clear  - t_start);   y += 12;  - shows 0 ms
	//display.setCursor(1, y);   display.printf("render: %2d", t_render - t_start);   y += 12;
	//display.setCursor(1, y);   display.printf("show:   %2d", t_show   - t_render);  y += 12;
	//display.setCursor(1, y);   display.printf("between: %d", t_start  - t_prev);    y += 12;
	//display.setCursor(1, y);   display.printf("FPS:    %2d", 1000 / (t_show - t_prev));    y += 12;
	t_prev = t_show;
}

void loop() {
	Render();
    loopController(xC, yC, angleC, around);

    vTaskDelay(10); // limit the render FPS in order to reduce the render_refresh/scrren_refresh interferences
}
