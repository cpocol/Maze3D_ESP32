const int16_t Map[][14] = {
{11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
{11,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0, 11, 11,  0, 11, 11, 11, 11,  0,  0,  0, 11},
{11,  0,  0, 11,  0,  0,  0,  0,  0, 11,  0,  0,  0, 11},
{11,  0,  0, 11,  0,  0,  0,  0,  0, 11,  0,  0,  0, 11},
{11,  0,  0, 11,  0,  0,  0,  0,  0, 11,  0,  0,  0, 11},
{11,  0,  0, 11,  0,  0,  0,  0,  0, 11,  0,  0,  0, 11},
{11, 11, 11, 11, 11,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11,  0,  0,  0, 11,  0,  0,  0,  0,  0,  0,  0,  0, 11},
{11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

const int16_t mapWidth = sizeof(Map[0])/sizeof(Map[0][0]);
const int16_t mapHeight = sizeof(Map)/sizeof(Map[0]);
