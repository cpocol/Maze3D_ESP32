#include <math.h>
#include <TFT_eSPI.h>   // Board lib for ESP32 TTGO T-Display (ESP32 Dev Module)

#include "time.h"
#include "Map.h"
#include "Texture.h"

#define sq(x) ((x)*(x))
#define min(a,b) ((a) < (b) ? (a) : (b))

// Buttons
#define BUTTON1 35
#define BUTTON2 0

// display
TFT_eSPI display = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h

const int32_t screenW = 240, screenH = 135, screenWh = screenW / 2, screenHh = screenH / 2;
const int16_t around = 6 * screenW, aroundh = around / 2, aroundq = around / 4; //FOV = 60 degs (6 FOVs = 360 degrees)
uint16_t* screen = new uint16_t[screenW * screenH];

const uint16_t sqSide = 128; // must be the side of Texture
const int16_t mapSizeHeight = mapHeight * sqSide, mapSizeWidth = mapWidth * sqSide;

const int16_t TanFixPoint = 7;
int32_t Tan_fp[around]; // TanFixPoint bits fixed point
int32_t cTan_fp[around];

//initial
int16_t xC = 2.5 * sqSide;
int16_t yC = (mapHeight - 2.5) * sqSide; //flip vertically
int16_t angleC = 1400;

float X2Rad(int X)
{
	return (float)X * 3.1415f / aroundh;
}

void setup()
{
	int16_t i, j;
	// precalculate
	for (int16_t t = 0; t < around; t++)
	{
		float angf = X2Rad(t);

		// tangent
		float temp = tan(angf) * (1 << TanFixPoint);
		if (temp > (128 << TanFixPoint) - 1)
			Tan_fp[t] = (128 << TanFixPoint) - 1;
		else
		if (temp < (-128 << TanFixPoint) + 1)
			Tan_fp[t] = (-128 << TanFixPoint) + 1;
		else
			Tan_fp[t] = (int16_t)temp;

		// cotangent
		temp = 1 / tan(angf) * (1 << TanFixPoint);
		if (temp > 128 * (1 << TanFixPoint) - 1)
			cTan_fp[t] = 128 * (1 << TanFixPoint) - 1;
		else
		if (temp < -128 * (1 << TanFixPoint) + 1)
			cTan_fp[t] = -128 * (1 << TanFixPoint) + 1;
		else
			cTan_fp[t] = temp;
	}

	// vertically mirror the map (it's more natural to edit it this way)
	for (i = 0; i < mapHeight/2; i++)
		for (j = 0; j < mapWidth; j++)
		{
			int16_t aux = Map[i][j];
			Map[i][j] = Map[mapHeight - 1 - i][j];
			Map[mapHeight - 1 - i][j] = aux;
		}

	display.init();
	display.setRotation(1);
	display.setSwapBytes(true);
	display.fillScreen(TFT_BLACK);

	pinMode(BUTTON1, INPUT);
	pinMode(BUTTON2, INPUT);

    Serial.begin(115200);
    Serial.println("Start");
}

void CastX(int16_t ang, int16_t* xS, int16_t* yS)
{
	int16_t x, y, dx, dy;
	if ((ang > aroundq) && (ang < 3 * aroundq))
	{
		x = (xC / sqSide) * sqSide;
		x--;
		dx = -sqSide;
		dy = -Tan_fp[ang];
	}
	else
	{
		x = (xC / sqSide) * sqSide + sqSide;
		dx = sqSide;
		dy = Tan_fp[ang];
	}
	y = yC + (((int32_t)(x - xC) * Tan_fp[ang]) >> TanFixPoint);

	while ((x > 0) && (x < mapSizeWidth) && (y > 0) && (y < mapSizeHeight) &&
		   (Map[y / sqSide][x / sqSide] == 0))
	{
		x += dx;
		y += dy;
	}
	*xS = x;
	*yS = y;
}

void CastY(int16_t ang, int16_t* xS, int16_t* yS)
{
	int16_t x, y, dx, dy;
	if ((ang > 0) && (ang < aroundh))
	{
		y = (yC / sqSide) * sqSide + sqSide;
		dx = cTan_fp[ang];
		dy = sqSide;
	}
	else
	{
		y = (yC / sqSide) * sqSide;
		y--;
		dx = -cTan_fp[ang];
		dy = -sqSide;
	}
	x = xC + (((int32_t)(y - yC) * cTan_fp[ang]) >> TanFixPoint);
	
	while ((x > 0) && (x < mapSizeWidth) && (y > 0) && (y < mapSizeHeight) &&
		   (Map[y / sqSide][x / sqSide] == 0))
	{
		x += dx;
		y += dy;
	}
	*xS = x;
	*yS = y;
}

auto t_prev = millis();
void Render()
{
	auto t_start = millis();

	memset(screen, 0, sizeof(uint16_t) * screenW * screenH);
	auto t_clear = millis();

	int32_t viewerToScreen_sq = sq(screenWh) * 3; // FOV = 60 degs => viewerToScreen = screenWh * sqrt(3)
	int16_t xX, yX, xY, yY, xStroken, yStroken, h;
	uint32_t textureColumn;
	for (int16_t x = 0; x < screenW; x++)
	{
		int16_t ang = (screenWh - x + angleC + around) % around;
		CastX(ang, &xX, &yX);
		CastY(ang, &xY, &yY);
		if (abs(xC - xX) < abs(xC - xY)) // choose the nearest stroken point
		{
			xStroken = xX;
			yStroken = yX;
			textureColumn = yX % sqSide;
		}
		else
		{
			xStroken = xY;
			yStroken = yY;
			textureColumn = xY % sqSide;
		}
		int32_t dx = xC - xStroken;
		int32_t dy = yC - yStroken;
		int32_t dist_sq = sq(dx) + sq(dy);
		int32_t screen_dx = screenWh - x;
		if (dist_sq == 0)
			h = 10000;
		else
			h = sqSide * sqrt((viewerToScreen_sq + sq(screen_dx)) / (float)dist_sq);

		// 1 row in screen space is this many rows in texture space
		uint32_t Dh_fp = 1024 * sqSide / h; // 10 bits fixed point
		uint32_t textureRow_fp = 0;
		int16_t minY = screenHh - h / 2;
		if (minY < 0)
		{
			textureRow_fp = -minY * Dh_fp;
			minY = 0;
		}
		int16_t maxY = min(screenHh + h / 2, screenH);

		uint16_t* screenAddr = screen + minY * screenW + x;
		//const uint16_t* textureAddr = Texture + textureColumn;
		const uint16_t* textureAddr = Texture + textureColumn * sqSide; // speedup: 90 degs CCW pre-rotated texture
		for (int16_t y = minY; y < maxY; y++)
		{
			//*screenAddr = *(textureAddr + textureRow_fp / 1024 * sqSide);
			*screenAddr = *(textureAddr + textureRow_fp / 1024); // rotated texture
			textureRow_fp += Dh_fp;
			screenAddr += screenW;
		}
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

void loop()
{
	Render();

	// move viewer
	if (digitalRead(BUTTON1) == LOW)
	{
		xC += 20 * cos(X2Rad(angleC));
		yC += 20 * sin(X2Rad(angleC));
	}
	if (digitalRead(BUTTON2) == LOW)
	{
		xC -= 20 * cos(X2Rad(angleC));
		yC -= 20 * sin(X2Rad(angleC));
	}
	int touch2 = touchRead(T2);
	int touch5 = touchRead(T5);
	Serial.println("Touch2 = " + String(touch2) + " Touch5 = " + String(touch5));
	int touchTh = 80;
	if (touch2 < touchTh)
		angleC = (angleC + (touchTh - touch2) / 5) % around;
	if (touch5 < touchTh)
		angleC = (angleC - (touchTh - touch5) / 5) % around;

    vTaskDelay(10); // limit the render FPS in order to reduce the render_refresh/scrren_refresh interferences
}
