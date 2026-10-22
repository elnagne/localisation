#define main localization_application_main
#include "../EV3_Localization.c"
#undef main
#include <assert.h>

int main(int argc,char **argv)
{
 assert(argc==3||argc==4);
 assert(load_colour_calibration(argv[1])==0);
 assert(hsv_black_value_threshold>0.0);
 const int known[][4]={
  {1,200,180,190},{6,750,750,840},{5,710,373,397},
  {3,320,428,310},{4,740,720,420},{2,300,450,650},
  // Fresh dark, red-tinted readings observed on the user's black street.
  {1,156,102,107},{1,156,100,109},{1,154,102,109},
  {1,158,102,107},{1,153,102,107}
 };
 for (const auto &row:known) assert(classify_sensor_rgb(row+1)==row[0]);
 assert(load_colour_calibration(argv[2])==0);
 assert(hsv_black_value_threshold<0.0); // Overlap must disable the added gate.
 if (argc==4)
 {
  assert(load_colour_calibration(argv[3])==0);
  for (int sample=0;sample<CALIBRATION_SAMPLES_PER_COLOUR;sample++)
  {
   assert(classify_sensor_rgb(colour_calibration_samples[1][sample])==3);
   assert(classify_sensor_rgb(colour_calibration_samples[5][sample])==6);
  }
  puts("Run-5 green/white samples all classify correctly.");
 }
 puts("11 colour/black regression cases and overlapping-calibration guard passed.");
 return 0;
}
