#include "gps_decode.h"
#include <string.h>
#include <stdlib.h>

gpsValue_t gpsValue = {0}, gpsValueNew = {0};

int hexchar2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

int hex2int(char *c)
{
    int value;

    value = hexchar2int(c[0]);
    value = value << 4;
    value += hexchar2int(c[1]);

    return value;
}

int checksum_valid(char *string)
{
    char *checksum_str;
    int checksum;
    unsigned char calculated_checksum = 0;

    // Checksum is postcede by *
    checksum_str = strchr(string, '*');
    if (checksum_str != NULL){
        // Remove checksum from string
        *checksum_str = '\0';
        // Calculate checksum, starting after $ (i = 1)
        for (int i = 1; i < strlen(string); i++) {
            calculated_checksum = calculated_checksum ^ string[i];
        }
        checksum = hex2int((char *)checksum_str+1);
        //printf("Checksum Str [%s], Checksum %02X, Calculated Checksum %02X\r\n",(char *)checksum_str+1, checksum, calculated_checksum);
        if (checksum == calculated_checksum) {
            //printf("Checksum OK\r\n");
            return 1;
        }
    } else {
        //printf("Error: Checksum missing or NULL NMEA message\r\n");
        return 0;
    }
    return 0;
}

//split string
int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
   int i = 0;
   fields[i++] = string;

   while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
      *string = '\0';
      fields[i++] = ++string;
   }

   return --i;
}

/* декодирует строку с GPS-приемника (только $GNRMC)
   0      1      2      3     4      5      6   7   8   9      12
$GNRMC,120921.00,A,5539.03336,N,03739.84341,E,0.073,,051120,,,D*6A
          |      |      |     |      |      |   |   |   |   ||| └контрольная сумма(1)
          |      |      |     |      |      |   |   |   |   ||└индикатор(2)          
          |      |      |     |      |      |   |   |   |   └└Магнитное склонение(?)      
          |      |      |     |      |      |   |   |   └UTC-дата(ддммгг)          
          |      |      |     |      |      |   |   └курс(градусы)
          |      |      |     |      |      |   └скорость(узлы)   
          |      |      |     |      |      └восточное полушарие (E/W)               
          |      |      |     |      └долгота(гггмм.ммммм)            
          |      |      |     └северное полушарие (N/S)             
          |      |      └широта(гггмм.ммммм)             
          |      └статус(A=Ok / V=wrong)             
          └UTC-время(ччммсс.сс)             
--
1 - контрольная сумма от $ до *, XOR кодов символов
2 - N=нет, A-автономный, D-дифференциальный, E-оценка */
int gpsDecode_(gpsValue_t *g, char *buffer) {

    char *field[20]={0};
    char tempbuf[5];
    char *s;
    int n;
    
    if (checksum_valid(buffer)) {
        if ((strncmp(buffer, "$GP", 3) == 0) ||
            (strncmp(buffer, "$GN", 3) == 0)) {
                
            if (strncmp(&buffer[3], "RMC", 3) == 0) {
                
                n = parse_comma_delimited_str(buffer, field, 20);
                //debug_print_fields(n, field);
              //кол-во параметров неверно
                if (n != 12) return 2;
                
              //статус не "А"
                if (strlen(field[2]) != 1 || field[2][0] != 'A') return 3;
                
              //дата или время неверной длины    (так будет, если сообщение пустое)                    
                if (strlen(field[1]) != 9 || strlen(field[9]) != 6) return 4;                
                
              //year, month, day        
                s = field[9];
                strncpy(tempbuf, (char *)s, 2);
                tempbuf[2] = '\0';
                g->d = atoi(tempbuf);
                
                strncpy(tempbuf, (char *)s+2, 2);
                g->m = atoi(tempbuf);
                
                strncpy(tempbuf, (char *)s+4, 2);
                g->y = atoi(tempbuf);
                
              //hours, minutes, seconds  120921
                s = field[2];
                strncpy(tempbuf, (char *)s, 2);
                g->h = atoi(tempbuf);
                
                strncpy(tempbuf, (char *)s+2, 2);
                g->n = atoi(tempbuf);
                
                strncpy(tempbuf, (char *)s+4, 2);
                g->s = atoi(tempbuf);
                
              //latitude  5539.03336
                s = field[3];
                strncpy(tempbuf, (char *)s, 2);
                g->lat = atoi(tempbuf);

                strncpy(tempbuf, (char *)s+2, 2);
                g->lat += (float)atoi(tempbuf) / 60;
    
                strncpy(tempbuf, (char *)s+5, 5);                
                g->lat += (float)atoi(tempbuf) / 6000000;
              //на южном полушарии со знаком "-"
                if (field[4][0] == 'S') g->lat = -(g->lat);
        
              //latitude  03739.84341
                s = field[5];
                strncpy(tempbuf, (char *)s, 3);
                tempbuf[3] = '\0';
                g->lng = atoi(tempbuf);
                
                strncpy(tempbuf, (char *)s+3, 2);
                tempbuf[2] = '\0';
                g->lng += (float)atoi(tempbuf) / 60;
                
                strncpy(tempbuf, (char *)s+6, 5);
                g->lng += (float)atoi(tempbuf) / 6000000;
              //западное полушарие со знаком "-"
                if (field[6][0] == 'W') g->lng = -(g->lng);
                
              //speed  
                s = field[7];
                g->spd = atoi(s); //???
                //knots -> km/h
                g->spd *= 1.852;
                
                g->ok = 1;
                return 0;
            }                
        }    
    }
    return 1;
}

void gpsDecode(char *s) {
  int t = gpsDecode_(&gpsValueNew, s);
  gpsValueNew.error = t;
}
