#ifndef GPS_DECODE_H
#define GPS_DECODE_H

typedef struct {
  char ok;
  double lat, lng, spd;
  unsigned char y,m,d, h,n,s;
  int error;
} gpsValue_t;

extern gpsValue_t gpsValue, gpsValueNew;

void gpsDecode(char *s);



#endif // GPS_DECODE_H
