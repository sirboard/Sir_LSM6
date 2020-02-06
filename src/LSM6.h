#ifndef LSM6_h
#define LSM6_h

#include <Arduino.h>
#include "const.h"

class LSM6
{
  private:
    deviceType _device;
    uint8_t address;

    uint16_t io_timeout;
    bool did_timeout;

    int16_t testReg(uint8_t address, regAddr reg);

  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_DS33, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> g; // gyro readings

    uint8_t last_status; // status of last I2C transmission

    LSM6(void);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);


    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    void readAcc(void);
    void readGyro(void);
    void read(void);

    void setTimeout(uint16_t timeout);
    uint16_t getTimeout(void);
    bool timeoutOccurred(void);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);
};


template <typename Ta, typename Tb, typename To> void LSM6::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LSM6::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif