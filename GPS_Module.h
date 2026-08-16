#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#ifdef ENABLE_GPS
#include <Arduino.h>

static char gps_line[100];
static uint8_t gps_line_index = 0;
static bool gps_has_fix = false;
static double gps_lat = 0.0;
static double gps_lon = 0.0;
static uint8_t gps_sats = 0;
static float gps_hdop = 0.0f;
static unsigned long gps_last_sentence_ms = 0;
static unsigned long gps_last_fix_ms = 0;
static unsigned long gps_last_report_ms = 0;
static unsigned long gps_chars = 0;

static bool gps_get_field(const char *sentence, uint8_t target, char *out, uint8_t out_len) {
  uint8_t field = 0;
  uint8_t pos = 0;
  out[0] = '\0';
  for (const char *p = sentence; ; ++p) {
    char c = *p;
    if (c == ',' || c == '*' || c == '\0' || c == '\r' || c == '\n') {
      if (field == target) {
        out[pos] = '\0';
        return true;
      }
      field++;
      pos = 0;
      if (c == '\0' || c == '*' || c == '\r' || c == '\n') return false;
      continue;
    }
    if (field == target && pos + 1 < out_len) out[pos++] = c;
  }
}

static double gps_coord_to_decimal(const char *value, const char *hemisphere) {
  if (!value || !value[0] || !hemisphere || !hemisphere[0]) return 0.0;
  double raw = atof(value);
  int degrees = (int)(raw / 100.0);
  double decimal = degrees + (raw - degrees * 100.0) / 60.0;
  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') decimal = -decimal;
  return decimal;
}

static void gps_parse_gga(const char *sentence) {
  char lat[16], ns[3], lon[16], ew[3], fix[4], sats[4], hdop[8];
  gps_get_field(sentence, 2, lat, sizeof(lat));
  gps_get_field(sentence, 3, ns, sizeof(ns));
  gps_get_field(sentence, 4, lon, sizeof(lon));
  gps_get_field(sentence, 5, ew, sizeof(ew));
  gps_get_field(sentence, 6, fix, sizeof(fix));
  gps_get_field(sentence, 7, sats, sizeof(sats));
  gps_get_field(sentence, 8, hdop, sizeof(hdop));
  gps_sats = (uint8_t)atoi(sats);
  gps_hdop = atof(hdop);
  gps_has_fix = atoi(fix) > 0 && lat[0] && lon[0];
  if (gps_has_fix) {
    gps_lat = gps_coord_to_decimal(lat, ns);
    gps_lon = gps_coord_to_decimal(lon, ew);
    gps_last_fix_ms = millis();
  }
}

static void gps_parse_rmc(const char *sentence) {
  char status[3], lat[16], ns[3], lon[16], ew[3];
  gps_get_field(sentence, 2, status, sizeof(status));
  gps_get_field(sentence, 3, lat, sizeof(lat));
  gps_get_field(sentence, 4, ns, sizeof(ns));
  gps_get_field(sentence, 5, lon, sizeof(lon));
  gps_get_field(sentence, 6, ew, sizeof(ew));
  gps_has_fix = status[0] == 'A' && lat[0] && lon[0];
  if (gps_has_fix) {
    gps_lat = gps_coord_to_decimal(lat, ns);
    gps_lon = gps_coord_to_decimal(lon, ew);
    gps_last_fix_ms = millis();
  }
}

static void gps_parse_sentence(const char *sentence) {
  gps_last_sentence_ms = millis();
  if (strstr(sentence, "GGA") == sentence + 3) gps_parse_gga(sentence);
  else if (strstr(sentence, "RMC") == sentence + 3) gps_parse_rmc(sentence);
}

static void gps_print_status() {
  unsigned long now = millis();
  Serial.print(F("G fix="));
  Serial.print(gps_has_fix ? 1 : 0);
  if (gps_has_fix) {
    Serial.print(F(" lat=")); Serial.print(gps_lat, 6);
    Serial.print(F(" lon=")); Serial.print(gps_lon, 6);
  }
  Serial.print(F(" sats=")); Serial.print(gps_sats);
  Serial.print(F(" hdop=")); Serial.print(gps_hdop, 1);
  Serial.print(F(" age_ms=")); Serial.print(gps_last_fix_ms ? now - gps_last_fix_ms : 0);
  Serial.print(F(" sentence_age_ms=")); Serial.print(gps_last_sentence_ms ? now - gps_last_sentence_ms : 0);
  Serial.print(F(" chars=")); Serial.println(gps_chars);
}

static void gps_initialize() {
  GPS_SERIAL.begin(GPS_BAUD_RATE);
  gps_line_index = 0;
  gps_has_fix = false;
  gps_last_sentence_ms = 0;
  gps_last_fix_ms = 0;
  gps_last_report_ms = 0;
  gps_chars = 0;
  Serial.print(F("GPS INIT Serial3 @ "));
  Serial.println(GPS_BAUD_RATE);
}

static void gps_update() {
  while (GPS_SERIAL.available()) {
    char c = (char)GPS_SERIAL.read();
    gps_chars++;
    if (c == '\n' || c == '\r') {
      if (gps_line_index > 6) {
        gps_line[gps_line_index] = '\0';
        gps_parse_sentence(gps_line);
      }
      gps_line_index = 0;
    } else if (gps_line_index + 1 < sizeof(gps_line)) {
      gps_line[gps_line_index++] = c;
    } else {
      gps_line_index = 0;
    }
  }
  unsigned long now = millis();
  if (now - gps_last_report_ms >= GPS_REPORT_INTERVAL_MS) {
    gps_last_report_ms = now;
    gps_print_status();
  }
}

#endif
#endif
