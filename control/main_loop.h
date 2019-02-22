
#ifndef MAIN_LOOP_H
#define MAIN_LOOP_H

class LED;

class MainLoop {
 public:
    MainLoop();
    void update();
 private:
    LED *led_;
};

#endif
