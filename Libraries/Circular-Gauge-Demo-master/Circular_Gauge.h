#ifndef _Circular_Gauge_H_
#define _Circular_Gauge_H_

class Circular_Gauge {
    public:
        Circular_Gauge(int min, int max);
        Circular_Gauge(int min, int max, bool includeRenderTime);

        Adafruit_SH1106* getdisplay();

        bool ready;
        bool active;

        void begin();
        void stop();
        void drawGaugeData(float value);
};

#endif

