#pragma once
#include <stdio.h>
#include <stdlib.h>

#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "fontx.h"

// Supported Fonts:
extern FontxFile fx16G[2];
extern FontxFile fx24G[2];
extern FontxFile fx32G[2];
extern FontxFile fx32L[2];

extern FontxFile fx16M[2];
extern FontxFile fx24M[2];
extern FontxFile fx32M[2];

extern FontxFile fx20_SIO8859_1[2];

extern FontxFile fx24Comic[2];

void eduboard_init_spiffs(void);