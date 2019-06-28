#ifndef CONFIG_H
#define CONFIG_H

#include "../control/fast_loop.h"
#include "../control/main_loop.h"

struct ConfigItems {
    FastLoop &fast_loop;
    MainLoop &main_loop;
};

struct Config {
    Config();
    FastLoop &fast_loop;
    MainLoop &main_loop;
};

extern const Config config;
#endif
