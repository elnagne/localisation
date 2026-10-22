/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
 * a sequence that allows reliable localization.
 *
 * This source keeps the assignment's map parser and localization interfaces while
 * adding incremental EV3 robot control and sensor handling.
 */

#include "EV3_Localization.h"
#include <time.h>
#include <limits.h>

extern int *socket_id;
static int verify_nxt_colour_sensor(void);
static int align_to_street(void);
static long elapsed_ms(const struct timespec *start);
static int follow_black_street(void);
static int wiggle_to_black(void);
static int center_on_alignment_marker(int marker_colour);
static int center_on_alignment_marker_or_yellow(int marker_colour,int accept_yellow);
static int window_wipe_for_colour(int target_colour);
static int sample_alignment_colour(void);
static int read_wheel_degrees(int *right,int *left);
static int continuous_black_sweep(int *readings,int sample_count,int *final_angle);
static int intersection_black_sweep(int *readings,int sample_count,int *final_angle);
static int center_turn_to_heading(int target);
static int continuous_recovery_sweep(void);
static MoveResult turn_in_place_alignment(int angle_degrees,int turn_direction,int timeout_ms);
static int next_follow_alignment_colour=1;
// A scan leaves the axle over yellow; a street arrival leaves only the sensor there.
static int intersection_axle_centered=0;
// Only the first intersection search may accept yellow without a black departure.
static int acquiring_first_intersection=0;
static int street_follow_power=5;
static int street_test_main(int argc,char **argv);
static int geometry_check_main(int argc,char **argv);

#define SENSOR_TO_AXLE_OFFSET_REV 0.27
#define ALIGNMENT_AXIS_ADVANCE_REV SENSOR_TO_AXLE_OFFSET_REV
#define ALIGN_SWEEP_STEP_DEGREES 10
#define ALIGN_SWEEP_TOTAL_DEGREES 360
#define ALIGN_OPPOSITE_TOLERANCE_DEGREES 35
#define ALIGN_SWEEP_POWER 10
#define ALIGN_SWEEP_TIMEOUT_MS 60000
#define ALIGN_TURN_MIN_POWER 15
#define ALIGN_TURN_MAX_POWER 30
#define RECOVERY_SWEEP_RIGHT_DEGREES 40
#define RECOVERY_SWEEP_LEFT_DEGREES 80
#define ROAD_CENTER_STEP_DEGREES 5
#define ROAD_CENTER_MAX_SWEEP_DEGREES 30
#define INTERSECTION_SWEEP_STEP_DEGREES 5
#define INTERSECTION_SWEEP_COARSE_STEP_DEGREES 10
#define INTERSECTION_SWEEP_FINE_WINDOW_DEGREES 20
#define INTERSECTION_CROSSING_TOLERANCE_DEGREES 25
#define INTERSECTION_SWEEP_MAX_DEGREES 405
#define INTERSECTION_SWEEP_TIMEOUT_MS 180000
#define FIND_STREET_MAX_REV 1.1
#define FIND_STREET_TIMEOUT_MS 22000
#define ALIGNMENT_TURN_OVERSHOOT_DEGREES 0
#define COLOUR_CLASSIFICATION_MAX_DISTANCE_RATIO 0.85
#define MAX_RELOCALIZATIONS 2

int map[400][4];
int sx,sy;
double beliefs[400][4];

int main(int argc,char *argv[])
{
 char mapname[1024];
 int dest_x,dest_y,rx,ry;
 unsigned char *map_image;
 memset(&map[0][0],0,sizeof(map));
 sx=0;
 sy=0;

 if (argc>1 && (strcmp(argv[1],"--test-street")==0||strcmp(argv[1],"--test-turn-street")==0))
  return street_test_main(argc,argv);
 if (argc>1 && strcmp(argv[1],"--check-geometry")==0) return geometry_check_main(argc,argv);

 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y calibration_directory [--read-colours]\n");
  fprintf(stderr,"Use -1 -1 for calibration; --read-colours monitors the NXT sensor without movement.\n");
  fprintf(stderr,"Diagnostics: --test-street calibration_directory [power] or --check-geometry sensor_offset_mm wheel_diameter_mm\n");
  return 1;
 }
 snprintf(mapname,sizeof(mapname),"%s",argv[1]);
 char *end;
 errno=0;
 long coordinate=strtol(argv[2],&end,10);
 if (errno!=0||end==argv[2]||*end!='\0'||coordinate<INT_MIN||coordinate>INT_MAX)
 {
  fprintf(stderr,"Destination x must be an integer.\n");
  return 1;
 }
 dest_x=(int)coordinate;
 errno=0;
 coordinate=strtol(argv[3],&end,10);
 if (errno!=0||end==argv[3]||*end!='\0'||coordinate<INT_MIN||coordinate>INT_MAX)
 {
  fprintf(stderr,"Destination y must be an integer.\n");
  return 1;
 }
 dest_y=(int)coordinate;

 if (dest_x==-1&&dest_y==-1)
 {
  if (BT_open(HEXKEY)!=0) return 1;
  int status=calibrate_sensor();
  BT_close();
  return status==0 ? 0 : 1;
 }
 if (argc<5||load_colour_calibration(argv[4])!=0)
 {
  fprintf(stderr,"Pass a valid NXT calibration directory.\n");
  return 1;
 }

 map_image=readPPMimage(mapname,&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to read map image.\n");
  return 1;
 }
 if (parse_map(map_image,rx,ry)==0)
 {
  fprintf(stderr,"Unable to parse map image.\n");
  free(map_image);
  return 1;
 }
 if (sx<=0||sy<=0||sx>400/sy||dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination must be within the map: x=0..%d, y=0..%d.\n",sx-1,sy-1);
  free(map_image);
  return 1;
 }
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open EV3 Bluetooth connection.\n");
  free(map_image);
  return 1;
 }
 if (verify_nxt_colour_sensor()!=0)
 {
  BT_close();
  free(map_image);
  return 1;
 }

 if (argc>5&&strcmp(argv[5],"--read-colours")==0)
 {
  printf("Reading NXT colours; press Ctrl+C to stop.\n");
  for (;;)
  {
   if (get_sensor_color_robust()<0)
   {
    BT_close();
    free(map_image);
    return 1;
   }
   usleep(200000);
  }
 }

 fprintf(stderr,"All set, ready to go.\n");
 int status=beep()==0&&run_mission(dest_x,dest_y,MAX_RELOCALIZATIONS);
 if (status)
 {
  printf("Mission complete: arrived at (%d, %d).\n",dest_x,dest_y);
  beep();
  usleep(150000);
  beep();
 }
 else fprintf(stderr,"Mission stopped before arrival; localization or navigation failed.\n");
 BT_close();
 free(map_image);
 return status ? 0 : 1;
}

static int is_street_marker(MoveResult result)
{
 return result.reason==STOP_COLOUR&&(result.colour==1||result.colour==4);
}

static int circular_angle_distance(int first,int second)
{
 int distance=abs(first-second)%360;
 return distance>180 ? 360-distance : distance;
}

static int nearest_black_interval_center(const int *readings,int sample_count,int current_angle)
{
 int start_index=-1;
 for (int index=0;index<sample_count;index++)
  if (readings[index]!=1)
  {
   start_index=index;
   break;
  }
 if (start_index<0) return -1;

 int interval_centers[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES];
 int interval_count=0;
 int offset=1;
 while (offset<=sample_count)
 {
  if (readings[(start_index+offset)%sample_count]!=1)
  {
   offset++;
   continue;
  }
  int interval_start=offset;
  while (offset<=sample_count&&readings[(start_index+offset)%sample_count]==1) offset++;
  int interval_end=offset-1;
  int center_index=(start_index+(interval_start+interval_end)/2)%sample_count;
  interval_centers[interval_count++]=center_index*ALIGN_SWEEP_STEP_DEGREES;
 }

 if (interval_count==0) return -1;
 int selected_angle=interval_centers[0];
 int best_isolation=-1;
 int smallest_correction=361;
 for (int first=0;first<interval_count;first++)
 {
  int nearest_other=181;
  for (int second=0;second<interval_count;second++)
  {
   if (first==second) continue;
   int distance=circular_angle_distance(interval_centers[first],interval_centers[second]);
   if (distance<nearest_other) nearest_other=distance;
  }
  int correction=abs(circular_angle_distance(interval_centers[first],current_angle));
  if (nearest_other>best_isolation||
      (nearest_other==best_isolation&&correction<smallest_correction))
  {
   best_isolation=nearest_other;
   smallest_correction=correction;
   selected_angle=interval_centers[first];
  }
 }
 return selected_angle;
}

const char *get_colour_name(int colour)
{
 static const char *const names[]={"unknown","black","blue","green","yellow","red","white"};
 if (colour==-1) return "read error";
 if (colour<0||colour>6) return "unknown";
 return names[colour];
}

int beep(void)
{
 int tones[50][3]={{880,200,25},{-1,-1,-1}};
 return BT_play_tone_sequence(tones);
}

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!).
  *
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  *
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow.
  *
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */
  if (turn_direction!=0&&turn_direction!=1) return 0;
  MoveResult turn=turn_in_place(90,turn_direction,10000);
  if (turn.reason!=STOP_ANGLE)
  {
   report_move_result("intersection turn",turn);
   return 0;
  }
  return 1;
}

  static int align_to_street(void)
  {
   int initial_colour=sample_alignment_colour();
   if (initial_colour!=1&&initial_colour!=4)
   {
    fprintf(stderr,"Cannot align from %s; expected black or yellow.\n",get_colour_name(initial_colour));
    initial_colour=continuous_recovery_sweep();
    if (initial_colour!=1&&initial_colour!=4)
    {
     fprintf(stderr,"Recovery sweep did not reacquire black or yellow.\n");
     return 0;
    }
   }
   MoveResult advance=move_forward_until(NULL,0,ALIGNMENT_AXIS_ADVANCE_REV,5000);
   if (advance.reason!=STOP_DISTANCE)
   {
    report_move_result("align: position axle",advance);
    return 0;
   }

   const int sample_count=ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES;
   int readings[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES]={0};
   int final_angle;
  if (continuous_black_sweep(readings,sample_count,&final_angle)!=0) return 0;

   // Fill brief unknown gaps inside yellow sectors, then aim for the widest sector midpoint.
   for (int index=0;index<sample_count;index++)
   {
    if (readings[index]!=0) continue;
    int gap=0;
    while (gap<3&&readings[(index+gap)%sample_count]==0) gap++;
    if (gap<=2&&readings[(index+sample_count-1)%sample_count]==4&&
        readings[(index+gap)%sample_count]==4)
     for (int fill=0;fill<gap;fill++) readings[(index+fill)%sample_count]=4;
   }

   for (int index=0;index<sample_count;index++)
   {
    if (readings[index]!=0) continue;
    int gap=0;
    while (gap<3&&readings[(index+gap)%sample_count]==0) gap++;
    if (gap<=2&&readings[(index+sample_count-1)%sample_count]==1&&
        readings[(index+gap)%sample_count]==1)
     for (int fill=0;fill<gap;fill++) readings[(index+fill)%sample_count]=1;
   }

   int target_angle=-1,target_width=0,target_colour=4;
   for (int index=0;index<sample_count;index++)
   {
    int previous=(index+sample_count-1)%sample_count;
    if (readings[index]!=4||readings[previous]==4) continue;
    int width=0;
    while (width<sample_count&&readings[(index+width)%sample_count]==4) width++;
    if (width<2) continue;
    if (width>target_width)
    {
     target_width=width;
     target_angle=((index+(width-1)/2)%sample_count)*ALIGN_SWEEP_STEP_DEGREES;
    }
   }

   if (target_angle<0)
   {
    target_colour=1;
    target_angle=nearest_black_interval_center(readings,sample_count,final_angle);
   }
   if (target_angle<0)
   {
    fprintf(stderr,"Alignment sweep found no yellow sector or black interval.\n");
    return 0;
   }

     int current_angle=final_angle%360;
     int correction=current_angle-target_angle;
     if (correction>180) correction-=360;
     if (correction< -180) correction+=360;
     int turn_direction=correction>=0 ? 1 : 0;
     printf("Alignment target %s at %d degrees; turning %s by %d degrees.\n",
       get_colour_name(target_colour),target_angle,
       turn_direction==1 ? "counterclockwise" : "clockwise",abs(correction));
     if (correction!=0)
   {
      int commanded=abs(correction)+ALIGNMENT_TURN_OVERSHOOT_DEGREES;
      if (commanded>360) commanded=360;
      MoveResult center=turn_in_place_alignment(commanded,turn_direction,30000);
    if (center.reason!=STOP_ANGLE)
    {
     report_move_result("align: turn to target sector",center);
     return 0;
    }
   }
   int verified=sample_alignment_colour();
   if (verified!=target_colour)
   {
    fprintf(stderr,"Alignment target was %s, but final reading is %s.\n",
       get_colour_name(target_colour),get_colour_name(verified));
    int recovered=target_colour==1 ? wiggle_to_black() :
      window_wipe_for_colour(target_colour)==target_colour;
    if (!recovered)
    {
     fprintf(stderr,"Alignment recovery did not reacquire %s.\n",get_colour_name(target_colour));
     return 0;
    }
    printf("Recovery sweep reacquired the %s alignment target.\n",get_colour_name(target_colour));
   }
   int gyro_angle,gyro_rate;
   if (BT_read_gyro(GYRO_SENSOR_PORT,1,&gyro_angle,&gyro_rate)!=1) return 0;
  next_follow_alignment_colour=target_colour;
   printf("Aligned to %s sector center at sweep angle %d degrees; gyro reset.\n",
     get_colour_name(target_colour),target_angle);
   return 1;
  }

  static int turn_at_red_and_search(const int *stop_colours)
  {
    MoveResult turn=turn_in_place(180,0,15000);
   if (turn.reason!=STOP_ANGLE) return 0;
   MoveResult search=move_forward_until(stop_colours,3,FIND_STREET_MAX_REV,FIND_STREET_TIMEOUT_MS);
  if (is_street_marker(search)) return 1;
  if (search.reason!=STOP_DISTANCE&&search.reason!=STOP_TIMEOUT) return 0;
  int recovered=continuous_recovery_sweep();
  return recovered==1||recovered==4;
  }

  // Initial street acquisition may turn at red. Boundary recovery has already
  // turned 180 degrees and must not trigger another untracked turnaround.
  static int find_street_internal(int allow_red_turnaround)
  {
   const int stop_colours[]={1,4,5};
   MoveResult result=move_forward_until(stop_colours,3,FIND_STREET_MAX_REV,FIND_STREET_TIMEOUT_MS);
   report_move_result("find street",result);
   if (is_street_marker(result)) return 1;
   if (result.reason==STOP_COLOUR&&result.colour==5)
    return allow_red_turnaround ? turn_at_red_and_search(stop_colours) : 0;
  if (result.reason!=STOP_DISTANCE&&result.reason!=STOP_TIMEOUT) return 0;
   fprintf(stderr,"No road or intersection found within the bounded search.\n");
  int recovered=continuous_recovery_sweep();
  if (recovered==1||recovered==4) return 1;
  if (recovered==5)
   return allow_red_turnaround ? turn_at_red_and_search(stop_colours) : 0;
  return 0;
  }

  int find_street(void)
  {
   return find_street_internal(1);
  }

  int drive_along_street(void)
  {
   return follow_black_street();
  }

  int scan_intersection(int *tl,int *tr,int *br,int *bl)
  {
   intersection_axle_centered=0;
   if (tl==NULL||tr==NULL||br==NULL||bl==NULL) return 0;
   *tl=*tr=*br=*bl=-1;
   int initial_colour=sample_alignment_colour();
   if (initial_colour!=4)
   {
    fprintf(stderr,"Intersection scan expected yellow, got %s; starting recovery sweep.\n",
      get_colour_name(initial_colour));
    if (continuous_recovery_sweep()!=4)
    {
     fprintf(stderr,"Recovery sweep did not reacquire yellow.\n");
     return 0;
    }
   }
  if (!center_on_alignment_marker(4))
  {
   fprintf(stderr,"Could not center on the yellow intersection before advancing.\n");
   return 0;
  }
   MoveResult advance=move_forward_until(NULL,0,ALIGNMENT_AXIS_ADVANCE_REV,5000);
   if (advance.reason!=STOP_DISTANCE)
   {
    report_move_result("intersection scan: position axle",advance);
    return 0;
   }
  if (!wiggle_to_black())
  {
   fprintf(stderr,"Could not align with black street before intersection sweep.\n");
   return 0;
  }
   int gyro_angle,gyro_rate;
   if (BT_read_gyro(GYRO_SENSOR_PORT,1,&gyro_angle,&gyro_rate)!=1) return 0;
   const int sample_count=ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES;
   int readings[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES]={0};
   int final_angle;
  if (intersection_black_sweep(readings,sample_count,&final_angle)!=0) return 0;
   const int headings[]={315,45,135,225};
   const char *labels[]={"TL","TR","BR","BL"};
   int *outputs[]={tl,tr,br,bl};
   for (int corner=0;corner<4;corner++)
   {
    int counts[7]={0};
    for (int index=0;index<sample_count;index++)
     if (circular_angle_distance(index*ALIGN_SWEEP_STEP_DEGREES,headings[corner])<=25&&
      readings[index]>=1&&readings[index]<=6) counts[readings[index]]++;
    int selected=0,selected_count=0,runner_up=0;
    for (int colour=2;colour<=6;colour++)
    {
     if (colour==4||colour==5) continue;
     if (counts[colour]>selected_count)
     {
      runner_up=selected_count;
      selected=colour;
      selected_count=counts[colour];
     }
     else if (counts[colour]>runner_up) runner_up=counts[colour];
    }
    printf("Intersection %s at %d degrees: %s (%d/sector)\n",labels[corner],
      headings[corner],get_colour_name(selected),selected_count);
    if (selected_count>0&&selected_count>runner_up) *outputs[corner]=selected;
   }
   printf("Intersection sweep complete; starting street verified after four black crossings.\n");
   if (BT_read_gyro(GYRO_SENSOR_PORT,1,&gyro_angle,&gyro_rate)!=1) return 0;
   printf("Intersection relative readings: TL=%s TR=%s BL=%s BR=%s; gyro zero reset.\n",
     get_colour_name(*tl),get_colour_name(*tr),get_colour_name(*bl),get_colour_name(*br));
   intersection_axle_centered=1;
   // Success describes the physical sweep and restored street heading. Missing
   // or tied building colours stay unknown and do not invalidate that heading.
   return 1;
  }

static int recover_from_red_boundary(void)
{
 printf("Red boundary: turning around and reacquiring the street.\n");
 MoveResult turn=turn_in_place(180,0,15000);
 if (turn.reason!=STOP_ANGLE)
 {
  report_move_result("boundary turnaround",turn);
  return 0;
 }
 if (!find_street_internal(0))
 {
  fprintf(stderr,"Street reacquisition failed after boundary turnaround; stopping without another turn.\n");
  return 0;
 }
 int colour=sample_alignment_colour();
 if (colour<0||BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return 0;
 if (colour==4)
 {
  int confirmation=sample_alignment_colour();
  if (confirmation==4)
  {
   printf("Confirmed yellow during boundary return; using it as the returned intersection.\n");
   return 2;
  }
 }
 int recovery=continuous_recovery_sweep();
 if (recovery==4)
 {
  if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0||sample_alignment_colour()!=4)
  {
   fprintf(stderr,"Yellow found during boundary return did not remain confirmed while stopped.\n");
   return 0;
  }
  printf("Confirmed yellow during boundary recovery sweep; using it as the returned intersection.\n");
  return 2;
 }
 if (recovery==1&&center_on_alignment_marker(1)) return 1;
 fprintf(stderr,"Black or yellow not reacquired during boundary return; stopping without another alignment sweep.\n");
 return 0;
}

static int drive_to_next_intersection(int *direction,int *boundary_return)
{
 if (direction==NULL||boundary_return==NULL) return 0;
 *boundary_return=0;
 int recovered_boundary=0;
 for (int attempt=0;attempt<2;attempt++)
 {
  int status=drive_along_street();
  if (status==1)
  {
   *boundary_return=recovered_boundary;
   return 1;
  }
  if (status!=2)
  {
   fprintf(stderr,"Street following failed before reaching an intersection.\n");
   return 0;
  }
  if (attempt==1)
  {
   fprintf(stderr,"Could not recover from the red boundary.\n");
   return 0;
  }
  int recovery=recover_from_red_boundary();
  if (recovery==0)
  {
   fprintf(stderr,"Could not recover from the red boundary.\n");
   return 0;
  }
  *direction=(*direction+2)%4;
  recovered_boundary=1;
  if (recovery==2)
  {
   *boundary_return=1;
   return 1;
  }
 }
 return 0;
}

static int window_wipe_for_colour(int target_colour)
{
 int colour=sample_alignment_colour();
 if (colour==5)
 {
  int confirmation=sample_alignment_colour();
  if (confirmation!=5) colour=confirmation==1||confirmation==4 ? confirmation : 0;
 }
 if (colour<0) return -1;
 if ((target_colour==0&&(colour==1||colour==4||colour==5))||colour==target_colour) return colour;

 printf("Gentle recovery search: right then left in %d-degree steps, up to %d degrees per side.\n",
   ROAD_CENTER_STEP_DEGREES,ROAD_CENTER_MAX_SWEEP_DEGREES);
 for (int direction=0;direction<2;direction++)
 {
  int turned=0;
  while (turned<ROAD_CENTER_MAX_SWEEP_DEGREES)
  {
   MoveResult turn=turn_in_place(ROAD_CENTER_STEP_DEGREES,direction,10000);
   if (turn.reason!=STOP_ANGLE) return -1;
   turned+=ROAD_CENTER_STEP_DEGREES;
   colour=sample_alignment_colour();
   if (colour==5)
   {
    int confirmation=sample_alignment_colour();
    if (confirmation!=5) colour=confirmation==1||confirmation==4 ? confirmation : 0;
   }
   if (colour<0)
   {
    if (turned>0)
    {
     MoveResult restore=turn_in_place(turned,direction==1 ? 0 : 1,15000);
     if (restore.reason!=STOP_ANGLE) return -1;
    }
    return -1;
   }
   printf("Recovery probe %s %d degrees: %s (%d)\n",
     direction==1 ? "left" : "right",turned,get_colour_name(colour),colour);
   if ((target_colour==0&&(colour==1||colour==4||colour==5))||colour==target_colour)
    return colour;
  }
  if (turned>0)
  {
   MoveResult restore=turn_in_place(turned,direction==1 ? 0 : 1,15000);
   if (restore.reason!=STOP_ANGLE) return -1;
  }
 }
 return 0;
}

static int wiggle_to_black(void)
{
 int result=window_wipe_for_colour(1);
 if (result!=1)
 {
  fprintf(stderr,"Window wipe did not reacquire black road.\n");
  return 0;
 }
 return center_on_alignment_marker(1);
}

double score_intersection_scan(const int map_corners[4], const int observed[4], int heading)
{
  if (map_corners == NULL || observed == NULL || heading < 0 || heading > 3)
  {
    return -1.0;
  }

  double score = 1.0;

  for (int corner = 0; corner < 4; corner++)
  {
    int measured = observed[corner];

    if (measured != 2 && measured != 3 && measured != 6)
      continue;
    
    int expected = map_corners[(corner + heading) % 4];

    if (measured == expected)
    {
      score *= 0.8;
    }
    else
    {
      score *= 0.1;
    }
  }
  
  return score;
}

int update_beliefs_from_scan(const int observed[4])
{
  double updated[400][4];
  double total = 0.0;

  if (observed == NULL || sx <= 0 || sy <= 0 || sx > 400 / sy)
    return 0;

  for (int index = 0; index < sx * sy; index++)
  {
    for (int heading = 0; heading < 4; heading++)
    {
      double score = score_intersection_scan(map[index], observed, heading);
      if (score < 0.0)
        return 0;

      updated[index][heading] = beliefs[index][heading] * score;
      total += updated[index][heading];
    }
  }

  if (!isfinite(total) || total <= 0.0)
    return 0;

  // Mix in a small uniform distribution so zero-weight states can recover.
  const double recovery_weight = 1e-6;
  double uniform = 1.0 / (sx * sy * 4);

  for (int index = 0; index < sx * sy; index++)
  {
    for (int heading = 0; heading < 4; heading++)
    {
      double normalized = updated[index][heading] / total;
      beliefs[index][heading] =
          (1.0 - recovery_weight) * normalized + recovery_weight * uniform;
    }
  }

  return 1;
}

static void print_belief_matrix(void)
{
 printf("Belief matrix by intersection; each cell is [up, right, down, left]\n");
 printf("       ");
 for (int x=0;x<sx;x++) printf("       x=%-2d                  ",x);
 printf("\n");
 for (int y=0;y<sy;y++)
 {
  printf("y=%-2d  ",y);
  for (int x=0;x<sx;x++)
  {
   int index=y*sx+x;
   printf("[%0.3f,%0.3f,%0.3f,%0.3f] ",
     beliefs[index][0],beliefs[index][1],beliefs[index][2],beliefs[index][3]);
  }
  printf("\n");
 }
 fflush(stdout);
}

int update_beliefs_after_turn(int turn_direction)
{
  if ((turn_direction != 0 && turn_direction != 1) ||
      sx <= 0 || sy <= 0 || sx > 400 / sy)
    return 0;

  double updated[400][4] = {{0.0}};
  double total = 0.0;
  int shift = turn_direction == 0 ? 1 : 3;

  for (int index = 0; index < sx * sy; index++)
  {
    for (int heading = 0; heading < 4; heading++)
    {
      int intended = (heading + shift) % 4;
      double probability = beliefs[index][heading];

      // Initial assumptions: intended turn, no turn, or a half turn.
      // These discrete heading probabilities need validation on the robot.
      updated[index][intended] += 0.96 * probability;
      updated[index][heading] += 0.02 * probability;
      updated[index][(heading + 2) % 4] += 0.02 * probability;
    }
  }

  for (int index = 0; index < sx * sy; index++)
    for (int heading = 0; heading < 4; heading++)
      total += updated[index][heading];

  if (!isfinite(total) || total <= 0.0)
    return 0;

  const double recovery_weight = 1e-6;
  double uniform = 1.0 / (sx * sy * 4);

  for (int index = 0; index < sx * sy; index++)
  {
    for (int heading = 0; heading < 4; heading++)
    {
      beliefs[index][heading] =
          (1.0 - recovery_weight) * updated[index][heading] / total
          + recovery_weight * uniform;
    }
  }

  return 1;
}

int update_beliefs_after_forward(int boundary_return)
{
  if ((boundary_return != 0 && boundary_return != 1) ||
      sx <= 0 || sy <= 0 || sx > 400 / sy)
    return 0;

  const int dx[4] = {0, 1, 0, -1};
  const int dy[4] = {-1, 0, 1, 0};
  double updated[400][4] = {{0.0}};
  double total = 0.0;

  for (int index = 0; index < sx * sy; index++)
  {
    for (int heading = 0; heading < 4; heading++)
    {
      int next_x = index % sx + dx[heading];
      int next_y = index / sx + dy[heading];
      int outside = next_x < 0 || next_x >= sx || next_y < 0 || next_y >= sy;
      double probability = beliefs[index][heading];

      // Condition on the completed trip's red/yellow outcome. Outward-facing
      // edge states cannot explain a direct yellow arrival, and interior states
      // cannot explain red followed by a return to the same intersection.
      if (boundary_return)
      {
        if (!outside) continue;
        // Initial turnaround model: reversed heading, or a quarter-turn error.
        updated[index][(heading + 2) % 4] += 0.96 * probability;
        updated[index][(heading + 1) % 4] += 0.02 * probability;
        updated[index][(heading + 3) % 4] += 0.02 * probability;
      }
      else
      {
        if (outside) continue; // Never wrap across a map edge.
        int next_index = next_y * sx + next_x;
        // Initial model: one cell forward, or a false arrival/no advance.
        updated[next_index][heading] += 0.94 * probability;
        updated[index][heading] += 0.06 * probability;
      }
    }
  }

  for (int index = 0; index < sx * sy; index++)
    for (int heading = 0; heading < 4; heading++)
      total += updated[index][heading];

  // Leave existing beliefs intact if no state can explain this outcome.
  if (!isfinite(total) || total <= 0.0)
    return 0;

  const double recovery_weight = 1e-6;
  double uniform = 1.0 / (sx * sy * 4);
  for (int index = 0; index < sx * sy; index++)
  {
    for (int heading = 0; heading < 4; heading++)
      beliefs[index][heading] =
          (1.0 - recovery_weight) * updated[index][heading] / total
          + recovery_weight * uniform;
  }
  return 1;
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 if (robot_x==NULL||robot_y==NULL||direction==NULL) return 0;
 *robot_x=-1;
 *robot_y=-1;
 *direction=0;

 acquiring_first_intersection=0;
 if (!find_street()) return 0;
 int initial_colour=sample_alignment_colour();
 if (initial_colour<0) return 0;
 int initial_arrival=0;
 if (initial_colour==4)
 {
  if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return 0;
  int confirmation=sample_alignment_colour();
  if (confirmation<0) return 0;
  if (confirmation==4)
  {
   initial_arrival=1;
   printf("Initial yellow confirmed; scanning here without another street search.\n");
  }
 }
 int boundary_return=0;
 if (!initial_arrival)
 {
  if (!align_to_street()) return 0;
  *direction=0;
  acquiring_first_intersection=1;
  initial_arrival=drive_to_next_intersection(direction,&boundary_return);
  acquiring_first_intersection=0;
  if (!initial_arrival) return 0;
 }
 printf("First yellow intersection establishes relative orientation 0.\n");

 const int max_scans=24;
 const double minimum_ratio=10.0;
 const double minimum_probability=0.70;
 const double probability_backed_ratio=5.0;
 for (int intersection=0;intersection<max_scans;intersection++)
 {
  int tl=-1,tr=-1,br=-1,bl=-1;
  int heading_verified=scan_intersection(&tl,&tr,&br,&bl);
  if (!heading_verified)
   fprintf(stderr,"Intersection %d heading unverified; using available corner readings without confirming a pose.\n",
     intersection+1);
  printf("Intersection %d relative readings: TL=%s TR=%s BL=%s BR=%s\n",
    intersection+1,get_colour_name(tl),get_colour_name(tr),
    get_colour_name(bl),get_colour_name(br));

  int observed[4]={tl,tr,br,bl};
  if (!update_beliefs_from_scan(observed))
  {
   fprintf(stderr,"Belief update failed.\n");
   return 0;
  }
  print_belief_matrix();

  int best_index=0,best_heading=0;
  double best_probability=-1.0,second_probability=0.0,total=0.0;
  for (int index=0;index<sx*sy;index++)
  {
   for (int heading=0;heading<4;heading++)
   {
    double probability=beliefs[index][heading];
    total+=probability;
    if (probability>best_probability)
    {
     second_probability=best_probability<0.0 ? 0.0 : best_probability;
     best_probability=probability;
     best_index=index;
     best_heading=heading;
    }
    else if (probability>second_probability) second_probability=probability;
   }
  }

  printf("Best candidate: (%d, %d), heading %d, probability %.6f\n",
    best_index%sx,best_index/sx,best_heading,best_probability);
  printf("Total belief: %.12f\n",total);
  double ratio=second_probability>0.0 ? best_probability/second_probability : HUGE_VAL;
  printf("Runner-up: %.6f; best/runner-up ratio: %.2f\n",second_probability,ratio);

  int valid_corners=0;
  for (int corner=0;corner<4;corner++)
   if (observed[corner]==2||observed[corner]==3||observed[corner]==6) valid_corners++;
  printf("Scan heading: %s; valid building corners: %d/4.\n",
    heading_verified ? "verified" : "unverified",valid_corners);
  int clear_lead=ratio>=minimum_ratio||
    (best_probability>minimum_probability&&ratio>=probability_backed_ratio);
  int confident=heading_verified&&best_probability>0.0&&clear_lead;
  // Use accumulated position/heading belief, including evidence from earlier scans.
  // Missing colours do not block confirmation; an unverified physical heading does.
  if (confident)
  {
   *robot_x=best_index%sx;
   *robot_y=best_index/sx;
   *direction=best_heading;
   printf("Relative confidence confirmed with verified heading (%d/4 building corners).\n",valid_corners);
   return 1;
  }

  if (intersection+1==max_scans) break;

  int route_step=intersection%3;
  int turn_direction=route_step==0 ? 0 : route_step==1 ? 1 : -1;
  if (turn_direction==0) printf("Route step: turn right.\n");
  else if (turn_direction==1) printf("Route step: turn left.\n");
  else printf("Route step: continue straight.\n");

  if (turn_direction>=0)
  {
   if (!turn_at_intersection(turn_direction)) return 0;
   if (!update_beliefs_after_turn(turn_direction))
   {
    fprintf(stderr,"Turn belief update failed.\n");
    return 0;
   }
   *direction=(*direction+(turn_direction==0 ? 1 : 3))%4;
  }
  if (!drive_to_next_intersection(direction,&boundary_return)) return 0;
  if (!update_beliefs_after_forward(boundary_return))
  {
   fprintf(stderr,"Forward belief update failed.\n");
   return 0;
  }
 }

 fprintf(stderr,"Localization did not achieve sufficient belief with a verified heading within %d scans.\n",
         max_scans);
 return 0;
}

// Choose a Manhattan route: horizontal first, then vertical.
// Caller must validate coordinates against sx/sy. -1 means already at target.
int heading_toward_target(int x, int y, int target_x, int target_y)
{
 if (target_x>x) return 1;
 if (target_x<x) return 3;
 if (target_y>y) return 2;
 if (target_y<y) return 0;
 return -1;
}

static int prepare_navigation_intersection(void)
{
 // Successful scans already positioned the axle. Ordinary arrivals have not.
 if (intersection_axle_centered) return 1;
 if (sample_alignment_colour()!=4||!center_on_alignment_marker(4))
 {
  fprintf(stderr,"Navigation requires a verified yellow intersection before positioning the axle.\n");
  return 0;
 }
 MoveResult advance=move_forward_until(NULL,0,ALIGNMENT_AXIS_ADVANCE_REV,5000);
 if (advance.reason!=STOP_DISTANCE)
 {
  report_move_result("navigation: position axle over yellow",advance);
  return 0;
 }
 intersection_axle_centered=1;
 printf("Navigation axle positioned over the yellow intersection.\n");
 return 1;
}

NavigationStepResult navigate_one_step(int *robot_x, int *robot_y, int *direction,
                                      int target_x, int target_y)
{
 if (robot_x==NULL||robot_y==NULL||direction==NULL||
     robot_x==robot_y||robot_x==direction||robot_y==direction||
     sx<=0||sy<=0||sx>400/sy||
     *robot_x<0||*robot_x>=sx||*robot_y<0||*robot_y>=sy||
     *direction<0||*direction>3||
     target_x<0||target_x>=sx||target_y<0||target_y>=sy)
 {
  fprintf(stderr,"Invalid navigation pose or destination.\n");
  return NAV_STEP_ERROR;
 }

 int desired=heading_toward_target(*robot_x,*robot_y,target_x,target_y);
 if (desired<0) return NAV_STEP_ARRIVED;
 int turn=(desired-*direction+4)%4;
 const int dx[4]={0,1,0,-1},dy[4]={-1,0,1,0};
 int next_x=*robot_x+dx[desired],next_y=*robot_y+dy[desired];
 // Check the planned edge before issuing any motor commands.
 if (next_x<0||next_x>=sx||next_y<0||next_y>=sy) return NAV_STEP_ERROR;

 if (!prepare_navigation_intersection()) goto navigation_lost;
 if (turn==1||turn==3)
 {
  if (!turn_at_intersection(turn==1 ? 0 : 1)) goto navigation_lost;
 }
 else if (turn==2)
 {
  MoveResult result=turn_in_place(180,0,15000);
  if (result.reason!=STOP_ANGLE)
  {
   report_move_result("navigation turnaround",result);
   goto navigation_lost;
  }
 }
 // Use the direct street follower: unexpected red must not trigger exploration's
 // boundary turnaround or be counted as arrival at the planned intersection.
 // It verifies outgoing black before driving; a gyro turn alone does not commit the pose.
 next_follow_alignment_colour=1;
 if (drive_along_street()!=1) goto navigation_lost;

 *robot_x=next_x;
 *robot_y=next_y;
 *direction=desired;
 printf("Navigation reached (%d, %d), heading %d.\n",next_x,next_y,desired);
 return next_x==target_x&&next_y==target_y ? NAV_STEP_ARRIVED : NAV_STEP_MOVED;

navigation_lost:
 intersection_axle_centered=0;
 // A failed turn or trip can leave the robot between discrete map states.
 *robot_x=-1;
 *robot_y=-1;
 *direction=-1;
 fprintf(stderr,"Navigation stopped; pose uncertain, relocalization required.\n");
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return NAV_STEP_ERROR;
 return NAV_STEP_LOST;
}

static NavigationStepResult navigate_to_target(int robot_x, int robot_y, int direction,
                                                int target_x, int target_y)
{
 // Preserve LOST vs ERROR so the mission retries only recoverable navigation loss.
 if (sx<=0||sy<=0||sx>400/sy||
     robot_x<0||robot_x>=sx||robot_y<0||robot_y>=sy||direction<0||direction>3||
     target_x<0||target_x>=sx||target_y<0||target_y>=sy)
 {
  fprintf(stderr,"Invalid navigation pose or destination.\n");
  return NAV_STEP_ERROR;
 }

 // Every traversal must reduce the Manhattan distance by one. Bound the loop
 // by that initial distance; an already-reached target requires no movement.
 int max_steps=abs(target_x-robot_x)+abs(target_y-robot_y);
 int steps=0;
 do
 {
  NavigationStepResult result=navigate_one_step(&robot_x,&robot_y,&direction,target_x,target_y);
  if (result==NAV_STEP_ARRIVED) return NAV_STEP_ARRIVED;
  if (result!=NAV_STEP_MOVED)
  {
   fprintf(stderr,"Navigation interrupted.\n");
   return result;
  }
  steps++;
 } while (steps<max_steps);

 fprintf(stderr,"Navigation exceeded the planned route length.\n");
 return NAV_STEP_ERROR;
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 // Keep the existing public 1=arrived, 0=failed contract.
 return navigate_to_target(robot_x,robot_y,direction,target_x,target_y)==NAV_STEP_ARRIVED;
}

int run_mission(int target_x, int target_y, int max_relocalizations)
{
 if (sx<=0||sy<=0||sx>400/sy||target_x<0||target_x>=sx||
     target_y<0||target_y>=sy||max_relocalizations<0)
 {
  fprintf(stderr,"Invalid mission destination, map size, or retry limit.\n");
  return 0;
 }
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0)
 {
  fprintf(stderr,"Unable to brake before localization; mission stopped.\n");
  return 0;
 }

 int retries_left=max_relocalizations;
 for (;;)
 {
  // A lost pose invalidates the entire old distribution, including its heading.
  // Also discard colour confirmations and the previous street-alignment marker.
  for (int index=0;index<sx*sy;index++)
   for (int heading=0;heading<4;heading++)
    beliefs[index][heading]=1.0/(double)(sx*sy*4);
  reset_sensor_color_filter();
  next_follow_alignment_colour=1;
  intersection_axle_centered=0;
  int robot_x=-1,robot_y=-1,direction=-1;
  if (robot_localization(&robot_x,&robot_y,&direction))
  {
   printf("Localized at (%d, %d), heading %d; navigating to (%d, %d).\n",
          robot_x,robot_y,direction,target_x,target_y);
   NavigationStepResult result=navigate_to_target(robot_x,robot_y,direction,target_x,target_y);
   if (result==NAV_STEP_ARRIVED) return 1;
   if (result==NAV_STEP_ERROR)
   {
    fprintf(stderr,"Navigation error; automatic retry disabled for this failure.\n");
    return 0;
   }
  }
  else fprintf(stderr,"Localization attempt failed.\n");

  // Confirm braking before another search, and also on the final failed attempt.
  if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0)
  {
   fprintf(stderr,"Unable to brake after pose loss; mission stopped.\n");
   return 0;
  }
  if (retries_left==0)
  {
   fprintf(stderr,"Relocalization retry limit reached; mission stopped.\n");
   return 0;
  }
  retries_left--;
  printf("Restarting localization with uniform beliefs; %d further retries available.\n",retries_left);
 }
}

static int calibration_command(const unsigned char *operations, size_t operation_size,
           unsigned char *payload, size_t payload_size)
{
 // Frame one direct command, transfer the complete reply, validate its ID/status/size, and copy globals; return 0 or -1.
 unsigned char packet[64], response[64];
 unsigned int message_id=(unsigned int)message_id_counter++ & 0xffff;
 size_t packet_size=7+operation_size;
 if (packet_size>sizeof(packet)||payload_size>sizeof(response)-3) return -1;
 // Header: little-endian length excluding itself, message ID, reply request, and global-memory allocation.
 packet[0]=(unsigned char)(packet_size-2);
 packet[1]=0;
 packet[2]=(unsigned char)message_id;
 packet[3]=(unsigned char)(message_id>>8);
 packet[4]=0;
 packet[5]=(unsigned char)payload_size;
 packet[6]=0;
 memcpy(packet+7,operations,operation_size);
 // Partial transfers are legal; retry interrupted calls without resending bytes already delivered.
 size_t transferred=0;
 while (transferred<packet_size)
 {
  ssize_t count=send(*socket_id,packet+transferred,packet_size-transferred,MSG_NOSIGNAL);
  if (count<0&&errno==EINTR) continue;
  if (count<=0) return -1;
  transferred+=(size_t)count;
 }
 // RFCOMM is a byte stream: read the length prefix before collecting the complete reply body.
 unsigned char length_bytes[2];
 transferred=0;
 while (transferred<2)
 {
  ssize_t count=recv(*socket_id,length_bytes+transferred,2-transferred,0);
  if (count<0&&errno==EINTR) continue;
  if (count<=0) return -1;
  transferred+=(size_t)count;
 }
 size_t response_size=length_bytes[0]+256u*length_bytes[1];
 // Three reply bytes hold the message ID and status; the rest must match the requested globals.
 if (response_size!=3+payload_size) return -1;
 transferred=0;
 while (transferred<response_size)
 {
  ssize_t count=recv(*socket_id,response+transferred,response_size-transferred,0);
  if (count<0&&errno==EINTR) continue;
  if (count<=0) return -1;
  transferred+=(size_t)count;
 }
 // Never interpret data from another request or a firmware error as sensor measurements.
 if (response[0]!=(unsigned char)message_id||response[1]!=(unsigned char)(message_id>>8)||response[2]!=2)
  return -1;
 memcpy(payload,response+3,payload_size);
 return 0;
}

static int wait_for_calibration_enter(void)
{
 // Consume a whole terminal line so typed text cannot leak into the next colour prompt; return 0 on Enter, -1 on EOF.
 int character;
 while ((character=getchar())!=EOF)
  if (character=='\n') return 0;
 return -1;
}

static int verify_nxt_colour_sensor(void)
{
 unsigned char identify[]={opINPUT_DEVICE,GET_TYPEMODE,0,COLOUR_SENSOR_PORT,0x60,0x61};
 unsigned char device[2];
 if (calibration_command(identify,sizeof(identify),device,2)!=0||device[0]!=4)
 {
  fprintf(stderr,"Calibration requires an NXT colour sensor on input %d.\n",COLOUR_SENSOR_PORT+1);
  return -1;
 }
 return 0;
}

#define CALIBRATION_SAMPLES_PER_COLOUR 30
static const char *const calibration_colours[]={"red","green","blue","yellow","black","white"};
static const int calibration_map_ids[]={5,3,2,4,1,6};
static int colour_calibration_samples[6][CALIBRATION_SAMPLES_PER_COLOUR][3];
static int colour_calibration_loaded=0;
static double hsv_neutral_saturation_threshold=0.0;
static double hsv_black_white_value_threshold=0.0;
static double hsv_black_value_threshold=-1.0;
static int confirmed_colour=0;               // Last colour confirmed by three consecutive readings.
static int candidate_colour=0;               // Colour currently accumulating a confirmation streak.
static int candidate_count=0;                // Consecutive matches, capped at three.

void reset_sensor_color_filter(void)
{
 // Discard all temporal history, such as when switching calibration or starting a new scan.
 confirmed_colour=0;
 candidate_colour=0;
 candidate_count=0;
}

static void rgb_to_hsv(const int channels[3],double *hue,double *saturation,double *value)
{
 double red=fmax(0.0,(double)channels[0]);
 double green=fmax(0.0,(double)channels[1]);
 double blue=fmax(0.0,(double)channels[2]);
 double maximum=fmax(red,fmax(green,blue));
 double minimum=fmin(red,fmin(green,blue));
 double delta=maximum-minimum;

 *value=maximum/32767.0;
 *saturation=maximum>0.0 ? delta/maximum : 0.0;
 if (delta==0.0)
  *hue=0.0;
 else if (maximum==red)
  *hue=60.0*fmod((green-blue)/delta,6.0);
 else if (maximum==green)
  *hue=60.0*((blue-red)/delta+2.0);
 else
  *hue=60.0*((red-green)/delta+4.0);
 if (*hue<0.0) *hue+=360.0;
}

int load_colour_calibration(const char *directory)
{
 // Load six complete NXT captures for calibrated HSV classification.
 colour_calibration_loaded=0;
 hsv_black_value_threshold=-1.0;
 reset_sensor_color_filter();
 for (int colour_index=0;colour_index<6;colour_index++)
 {
  char filename[1024],header[128];
  int length=snprintf(filename,sizeof(filename),"%s/%s.txt",directory,calibration_colours[colour_index]);
  if (length<0||(size_t)length>=sizeof(filename)) return -1;
  FILE *input=fopen(filename,"r");
  if (input==NULL) return -1;
  if (fgets(header,sizeof(header),input)==NULL||strcmp(header,"sensor NXT RGB\n")!=0||
      fgets(header,sizeof(header),input)==NULL||strcmp(header,"sample R G B ambient\n")!=0)
  {
   fclose(input);
   return -1;
  }
  // Reject EV3 files, incomplete captures, and impossible corrected NXT readings.
  int failed=0;
  for (int sample=1;sample<=CALIBRATION_SAMPLES_PER_COLOUR;sample++)
  {
   int index,red,green,blue,ambient;
   if (fscanf(input,"%d %d %d %d %d",&index,&red,&green,&blue,&ambient)!=5||index!=sample||
    red< -32767||red>32767||green< -32767||green>32767||blue< -32767||blue>32767||ambient<0||ambient>32767)
   {
    failed=1;
    break;
   }
   colour_calibration_samples[colour_index][sample-1][0]=red;
   colour_calibration_samples[colour_index][sample-1][1]=green;
   colour_calibration_samples[colour_index][sample-1][2]=blue;
  }
  // Allow trailing whitespace, but reject extra rows or text from an incompatible file format.
  int character;
  while (!failed&&(character=fgetc(input))!=EOF)
   if (!isspace((unsigned char)character)) failed=1;
  if (ferror(input)) failed=1;
  fclose(input);
  if (failed) return -1;
 }

 // Learn the achromatic saturation gate and black/white brightness boundary from these captures.
 double neutral_saturation_max=0.0;
 double chromatic_saturation_floor=1.0;
 double black_value_max=0.0;
 double nonblack_value_min=HUGE_VAL;
 double white_value_min=1.0;
 for (int colour_index=0;colour_index<6;colour_index++)
  for (int sample=0;sample<CALIBRATION_SAMPLES_PER_COLOUR;sample++)
  {
   double hue,saturation,value;
   rgb_to_hsv(colour_calibration_samples[colour_index][sample],&hue,&saturation,&value);
   int colour=calibration_map_ids[colour_index];
   if (colour!=1&&value<nonblack_value_min) nonblack_value_min=value;
   if (colour==1||colour==6)
   {
    if (saturation>neutral_saturation_max) neutral_saturation_max=saturation;
    if (colour==1&&value>black_value_max) black_value_max=value;
    if (colour==6&&value<white_value_min) white_value_min=value;
   }
  }
 for (int colour=2;colour<=5;colour++)
 {
  double saturations[CALIBRATION_SAMPLES_PER_COLOUR];
  int count=0;
  for (int colour_index=0;colour_index<6;colour_index++)
   if (calibration_map_ids[colour_index]==colour)
    for (int sample=0;sample<CALIBRATION_SAMPLES_PER_COLOUR;sample++)
    {
     double hue,saturation,value;
     rgb_to_hsv(colour_calibration_samples[colour_index][sample],&hue,&saturation,&value);
     saturations[count++]=saturation;
    }
  for (int index=1;index<count;index++)
  {
   double value=saturations[index];
   int position=index;
   while (position>0&&saturations[position-1]>value)
   {
    saturations[position]=saturations[position-1];
    position--;
   }
   saturations[position]=value;
  }
  if (count>0&&saturations[count/10]<chromatic_saturation_floor)
   chromatic_saturation_floor=saturations[count/10];
 }
 hsv_neutral_saturation_threshold=(neutral_saturation_max+chromatic_saturation_floor)/2.0;
 hsv_black_white_value_threshold=(black_value_max+white_value_min)/2.0;
 // Low NXT readings can have a strong channel tint even on black. Use brightness
 // before hue/saturation only when the captures demonstrate a black/nonblack gap.
 if (black_value_max<nonblack_value_min)
  hsv_black_value_threshold=(black_value_max+nonblack_value_min)/2.0;

 // Enable classification only after all six NXT captures and their HSV thresholds validate.
 colour_calibration_loaded=1;
 return 0;
}

static int classify_sensor_rgb(const int channels[3])
{
 // Use saturation and value for black/white, then compare chromatic samples by circular hue and HSV distance.
 if (!colour_calibration_loaded) return 0;
 double hue,saturation,value;
 rgb_to_hsv(channels,&hue,&saturation,&value);
 if (hsv_black_value_threshold>=0.0&&value<hsv_black_value_threshold) return 1;
 if (saturation<=hsv_neutral_saturation_threshold)
  return value<hsv_black_white_value_threshold ? 1 : 6;

 double colour_distances[7];
 for (int colour=0;colour<7;colour++) colour_distances[colour]=HUGE_VAL;
 for (int colour_index=0;colour_index<6;colour_index++)
 {
  int colour=calibration_map_ids[colour_index];
  if (colour==1||colour==6) continue;
  for (int sample=0;sample<CALIBRATION_SAMPLES_PER_COLOUR;sample++)
  {
   double sample_hue,sample_saturation,sample_value;
   rgb_to_hsv(colour_calibration_samples[colour_index][sample],
     &sample_hue,&sample_saturation,&sample_value);
   double hue_difference=fabs(hue-sample_hue);
   if (hue_difference>180.0) hue_difference=360.0-hue_difference;
   hue_difference=hue_difference/180.0*fmin(saturation,sample_saturation);
   double saturation_difference=saturation-sample_saturation;
   double value_difference=value-sample_value;
   double distance=hue_difference*hue_difference+
     saturation_difference*saturation_difference+value_difference*value_difference;
   if (distance<colour_distances[colour]) colour_distances[colour]=distance;
  }
 }

 double nearest_distance=HUGE_VAL,second_distance=HUGE_VAL;
 int nearest_colour=0;
 for (int colour=2;colour<=5;colour++)
 {
  if (colour_distances[colour]<nearest_distance)
   {
   second_distance=nearest_distance;
   nearest_distance=colour_distances[colour];
   nearest_colour=colour;
   }
  else if (colour_distances[colour]<second_distance)
   {
   second_distance=colour_distances[colour];
   }
 }
 if (nearest_distance>=second_distance*COLOUR_CLASSIFICATION_MAX_DISTANCE_RATIO) return 0;
 return nearest_colour;
}

static int read_sensor_colour_unfiltered(void)
{
 if (!colour_calibration_loaded) return -1;
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
   setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0) return -1;
 int rgb[3],ambient;
 if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT,&rgb[0],&rgb[1],&rgb[2],&ambient)!=1||
   rgb[0]< -32767||rgb[0]>32767||rgb[1]< -32767||rgb[1]>32767||rgb[2]< -32767||rgb[2]>32767||
   ambient<0||ambient>32767) return -1;
 return classify_sensor_rgb(rgb);
}

static int sample_colour_agreement_mode(int required_votes,int early_exit)
{
 int counts[7]={0};
 int samples_taken=0;
 for (int sample=0;sample<5;sample++)
 {
  int colour=read_sensor_colour_unfiltered();
  if (colour<0) return -1;
  if (colour>=1&&colour<=6) counts[colour]++;
  samples_taken++;
  // Four matching votes already satisfy the four-of-five scan rule; the
  // remaining vote cannot change the result. Keep errors on reads we do take.
  if (early_exit&&colour>=1&&colour<=6&&counts[colour]>=required_votes) break;
  if (sample<4) usleep(20000);
 }
 int best_colour=0;
 for (int colour=1;colour<=6;colour++)
  if (counts[colour]>counts[best_colour]) best_colour=colour;
 printf("Alignment sample: %s (%d), %d/%d\n",get_colour_name(best_colour),best_colour,counts[best_colour],samples_taken);
 fflush(stdout);
 return counts[best_colour]>=required_votes ? best_colour : 0;
}

static int sample_colour_agreement(int required_votes)
{
 return sample_colour_agreement_mode(required_votes,0);
}

static int sample_stable_colour(void)
{
 return sample_colour_agreement(3);
}

static int sample_street_event_colour(void)
{
 int colour=sample_colour_agreement(4);
 if (colour<0) return -1;
 if (colour==5)
 {
  if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return -1;
  int confirmation=sample_colour_agreement(3);
  if (confirmation!=5)
  {
   printf("Stationary red-boundary confirmation failed; treating it as unknown.\n");
   colour=0;
  }
 }
 printf("Street sample: %s (%d)\n",get_colour_name(colour),colour);
 fflush(stdout);
 return colour;
}

static int sample_alignment_colour(void)
{
 return sample_colour_agreement(4);
}

static int sample_sweep_colour(void)
{
 int counts[7]={0};
 for (int sample=0;sample<5;sample++)
 {
  int colour=read_sensor_colour_unfiltered();
  if (colour<0) return -1;
  if (colour>=1&&colour<=6) counts[colour]++;
  if (sample<4) usleep(20000);
 }
 int best_colour=0,best_count=0,runner_up=0;
 for (int colour=1;colour<=6;colour++)
 {
  if (counts[colour]>best_count)
  {
   runner_up=best_count;
   best_colour=colour;
   best_count=counts[colour];
  }
  else if (counts[colour]>runner_up) runner_up=counts[colour];
 }
 int required_votes=(best_colour==1||best_colour==4) ? 2 : 4;
 printf("Sweep sample: %s (%d), %d/5\n",get_colour_name(best_colour),best_colour,best_count);
 fflush(stdout);
 return best_count>=required_votes&&best_count>runner_up ? best_colour : 0;
}

static int sample_intersection_colour(void)
{
 return sample_colour_agreement_mode(4,1);
}

typedef struct
{
 int crossings,in_black,nonblack_run;
 int entry_angle,last_black_angle;
} IntersectionCrossings;

static int intersection_sweep_step(int angle,const IntersectionCrossings *state)
{
 int offset=angle%90;
 int distance=offset<45 ? offset : 90-offset;
 int next_offset=(angle+INTERSECTION_SWEEP_COARSE_STEP_DEGREES)%90;
 int next_distance=next_offset<45 ? next_offset : 90-next_offset;
 // Keep fine probes around each road and throughout a black strip. Once the
 // final road is found, retain fine resolution until its exit is confirmed.
 if (state->in_black||state->crossings==4||distance<=INTERSECTION_SWEEP_FINE_WINDOW_DEGREES||
     next_distance<=INTERSECTION_SWEEP_FINE_WINDOW_DEGREES)
  return INTERSECTION_SWEEP_STEP_DEGREES;
 return INTERSECTION_SWEEP_COARSE_STEP_DEGREES;
}

static int observe_intersection_crossing(IntersectionCrossings *state,int angle,int colour)
{
 // Unknown readings cannot establish departure from a street. Two confirmed
 // nonblack stops are needed before another black interval can be counted.
 if (colour==0) {state->nonblack_run=0;return 0;}
 if (colour==1)
 {
  state->nonblack_run=0;
  if (!state->in_black)
  {
   state->in_black=1;
   state->entry_angle=angle;
   int expected=(state->crossings+1)*90;
   if (state->crossings<4&&abs(angle-expected)<=INTERSECTION_CROSSING_TOLERANCE_DEGREES)
    state->crossings++;
  }
  state->last_black_angle=angle;
  return 0;
 }
 if (state->in_black&&++state->nonblack_run>=2)
 {
  state->in_black=0;
  state->nonblack_run=0;
  return state->crossings==4;
 }
 return 0;
}

static int intersection_black_sweep(int *readings,int sample_count,int *final_angle)
{
 const int bins=ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES;
 if (readings==NULL||final_angle==NULL||sample_count!=bins) return -1;
 int votes[ALIGN_SWEEP_TOTAL_DEGREES/ALIGN_SWEEP_STEP_DEGREES][7]={{0}};
 for (int index=0;index<sample_count;index++) readings[index]=0;
 int success=0,angle,rate;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 IntersectionCrossings state={0,1,0,0,0};
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1) goto done;
 if (sample_intersection_colour()!=1)
 {
  fprintf(stderr,"Counted intersection sweep must start on confirmed black.\n");
  goto done;
 }
 readings[0]=1;
 votes[0][1]=1;
 for (int target=INTERSECTION_SWEEP_STEP_DEGREES;target<=INTERSECTION_SWEEP_MAX_DEGREES;
      target+=intersection_sweep_step(target,&state))
 {
  if (elapsed_ms(&start)>=INTERSECTION_SWEEP_TIMEOUT_MS)
  {
   fprintf(stderr,"Counted intersection sweep timed out.\n");
   break;
  }
  // Absolute targets retain the scan's gyro reference across every small turn.
  // Stop to sample, so Bluetooth latency cannot sweep past a narrow street.
  if (!center_turn_to_heading(target)||BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) break;
  int colour=sample_intersection_colour();
  if (colour<0) break;
  int index=(int)lround((double)angle/ALIGN_SWEEP_STEP_DEGREES)%sample_count;
  if (index<0) break;
  if (colour>=1&&colour<=6) votes[index][colour]++;
  int best=0,best_count=0,runner_up=0;
  for (int candidate=1;candidate<=6;candidate++)
  {
   int count=votes[index][candidate];
   if (count>best_count) {runner_up=best_count;best_count=count;best=candidate;}
   else if (count>runner_up) runner_up=count;
  }
  readings[index]=best_count>runner_up ? best : 0;
  printf("Intersection sweep: gyro %d degrees, %s (%d)\n",angle,get_colour_name(colour),colour);
  int previous=state.crossings;
  int complete=observe_intersection_crossing(&state,angle,colour);
  if (state.crossings!=previous)
   printf("Black street crossing %d/4 at %d degrees.\n",state.crossings,angle);
  if (!complete) continue;
  int midpoint=(state.entry_angle+state.last_black_angle)/2;
  if (abs(midpoint-360)>INTERSECTION_CROSSING_TOLERANCE_DEGREES) break;
  printf("Fourth black street spans %d..%d degrees; returning to midpoint %d.\n",
    state.entry_angle,state.last_black_angle,midpoint);
  if (!center_turn_to_heading(midpoint)||BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1||
      abs(angle-360)>INTERSECTION_CROSSING_TOLERANCE_DEGREES||sample_intersection_colour()!=1) break;
  *final_angle=angle;
  success=1;
  break;
 }
done:
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return -1;
 if (!success)
  fprintf(stderr,"Intersection sweep failed to verify the starting street after four crossings (%d/4).\n",
    state.crossings);
 return success ? 0 : -1;
}

static int continuous_black_sweep(int *readings,int sample_count,int *final_angle)
{
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1) return -1;
 int initial_colour=sample_sweep_colour();
 if (initial_colour<0) return -1;
 readings[0]=initial_colour;
 if (BT_turn(MOTOR_C,ALIGN_SWEEP_POWER,MOTOR_B,-ALIGN_SWEEP_POWER)!=0)
 {
  BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
  return -1;
 }

 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 struct timespec last_progress_time;
 clock_gettime(CLOCK_MONOTONIC,&last_progress_time);
 int last_progress_angle=0;
 int next_sample_angle=ALIGN_SWEEP_STEP_DEGREES;
 int read_error=0;
 while (next_sample_angle<ALIGN_SWEEP_TOTAL_DEGREES)
 {
  if (elapsed_ms(&start)>=ALIGN_SWEEP_TIMEOUT_MS)
  {
   fprintf(stderr,"Continuous alignment sweep timed out.\n");
   read_error=1;
   break;
  }
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   read_error=1;
   break;
  }
  if (angle!=last_progress_angle)
  {
   last_progress_angle=angle;
   clock_gettime(CLOCK_MONOTONIC,&last_progress_time);
  }
  else if (elapsed_ms(&last_progress_time)>=5000)
  {
   fprintf(stderr,"Alignment sweep stalled at gyro %d degrees before target %d; stopping motors.\n",
     angle,next_sample_angle);
   read_error=1;
   break;
  }
  if (angle<next_sample_angle)
  {
   usleep(10000);
   continue;
  }

  int colour=sample_sweep_colour();
  if (colour<0)
  {
   read_error=1;
   break;
  }
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   read_error=1;
   break;
  }
  int sample_index=(int)lround((double)angle/ALIGN_SWEEP_STEP_DEGREES)%sample_count;
  readings[sample_index]=colour;
  printf("Sweep: gyro %d degrees, %s (%d)\n",angle,get_colour_name(colour),colour);
  fflush(stdout);
  next_sample_angle=(angle/ALIGN_SWEEP_STEP_DEGREES+1)*ALIGN_SWEEP_STEP_DEGREES;
 }

 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return -1;
 if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return -1;
 if (read_error||angle<ALIGN_SWEEP_TOTAL_DEGREES-ALIGN_SWEEP_STEP_DEGREES)
 {
  fprintf(stderr,"Alignment sweep ended at %d degrees, before a complete turn.\n",angle);
  return -1;
 }
 int overshoot=angle-ALIGN_SWEEP_TOTAL_DEGREES;
 if (overshoot>3)
 {
  if (overshoot>45)
  {
   fprintf(stderr,"Alignment sweep overshot by %d degrees; refusing a large correction.\n",overshoot);
   return -1;
  }
  printf("Sweep overshot by %d degrees; correcting back to 360.\n",overshoot);
  MoveResult correction=turn_in_place(overshoot,1,10000);
  if (correction.reason!=STOP_ANGLE)
  {
   report_move_result("alignment sweep overshoot correction",correction);
   return -1;
  }
  *final_angle=ALIGN_SWEEP_TOTAL_DEGREES;
 }
 else
  *final_angle=angle;
 return 0;
}

static int update_sensor_color_filter(int colour)
{
 // Debounce colour changes across calls: retain the confirmed ID until a new candidate appears three times in a row.
 if (colour<=0)
 {
  // Unknown breaks the streak but holds the confirmed colour; errors also break it and remain visible as -1.
  candidate_colour=0;
  candidate_count=0;
  return colour<0 ? -1 : confirmed_colour;
 }
 if (colour!=candidate_colour)
 {
  // A different candidate must start its own streak, not inherit matches from the previous colour.
  candidate_colour=colour;
  candidate_count=0;
 }
 if (candidate_count<3) candidate_count++;
 if (candidate_count==3) confirmed_colour=colour;
 return confirmed_colour;
}

int get_sensor_color_robust(void)
{
 // Read NXT corrected RGB, classify against saved samples, confirm three times, and print every request.
 int colour;
 if (!colour_calibration_loaded) colour=-1;
 else
 {
  struct timeval timeout={5,0};
  if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
      setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0)
   colour=update_sensor_color_filter(-1);
  else
  {
    int rgb[3],ambient;
    if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT,&rgb[0],&rgb[1],&rgb[2],&ambient)!=1||
       rgb[0]< -32767||rgb[0]>32767||rgb[1]< -32767||rgb[1]>32767||rgb[2]< -32767||rgb[2]>32767||ambient<0||ambient>32767)
     colour=update_sensor_color_filter(-1);
    else
     colour=update_sensor_color_filter(classify_sensor_rgb(rgb));
  }
 }
 printf("Seeing colour: %s (%d)\n",get_colour_name(colour),colour);
 fflush(stdout);
 return colour;
}

int calibrate_sensor(void)
{
 // On an open EV3 connection, save 30 NXT RGB+A readings per colour across three spots.
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
     setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0)
 {
  perror("Unable to set calibration communication timeout");
  return -1;
 }
 if (verify_nxt_colour_sensor()!=0) return -1;
 if (mkdir("calibration",0755)!=0&&errno!=EEXIST)
 {
  perror("Unable to create calibration directory");
  return -1;
 }
 // Use the first unused numeric folder and preserve all earlier calibration runs.
 char directory[64];
 int run_number;
 for (run_number=1;run_number<1000000;run_number++)
 {
  snprintf(directory,sizeof(directory),"calibration/run-%d",run_number);
  if (mkdir(directory,0755)==0) break;
  if (errno!=EEXIST) {perror("Unable to create calibration session");return -1;}
 }
 if (run_number==1000000)
 {
  fprintf(stderr,"No unused calibration run number available.\n");
  return -1;
 }
 printf("Saving raw calibration readings in %s\n",directory);
 for (size_t colour_index=0;colour_index<6;colour_index++)
 {
  char filename[256];
   snprintf(filename,sizeof(filename),"%s/%s.txt",directory,calibration_colours[colour_index]);
  FILE *output=fopen(filename,"wx");
  if (output==NULL) {perror("Unable to create colour file");return -1;}
    int failed=fprintf(output,"sensor NXT RGB\nsample R G B ambient\n")<0;
    for (int sample=1;sample<=CALIBRATION_SAMPLES_PER_COLOUR&&!failed;sample++)
  {
   if ((sample-1)%10==0)
   {
        printf("Now reading %s: place sensor at distinct solid-colour spot %d/3 and press Enter.\n",
          calibration_colours[colour_index],(sample-1)/10+1);
    fflush(stdout);
    if (wait_for_calibration_enter()!=0) {failed=1;break;}
   }
  int channels[3],ambient;
  if (BT_read_colour_RGBraw_NXT(COLOUR_SENSOR_PORT,&channels[0],&channels[1],&channels[2],&ambient)!=1)
   {
    fprintf(stderr,"Sensor read failed; %s is incomplete.\n",filename);
    failed=1;
    break;
   }
  for (int channel=0;channel<3;channel++)
   if (channels[channel]< -32767||channels[channel]>32767) failed=1;
  if (ambient<0||ambient>32767) failed=1;
  failed=fprintf(output,"%d %d %d %d %d\n",sample,channels[0],channels[1],channels[2],ambient)<0||failed;
   if (fflush(output)!=0) failed=1;
  printf("%s %d/30: R=%d G=%d B=%d A=%d\n",calibration_colours[colour_index],sample,channels[0],channels[1],channels[2],ambient);
   if (sample%10!=0) usleep(100000);
   else if (!failed&&beep()!=0) failed=1;
  }
  // Signal completion only after all rows have been written and the file has closed successfully.
  if (fclose(output)!=0) failed=1;
  if (failed) {fprintf(stderr,"Calibration stopped; check %s.\n",filename);return -1;}
  if (beep()!=0) {fprintf(stderr,"Completion beep failed.\n");return -1;}
  printf("Saved 30 NXT RGB+A readings to %s\n",filename);
 }
 printf("NXT calibration capture complete.\n");
 return 0;
}

// ---- Smooth movement: gyro-held straight drives and gyro-controlled turns ----
#define MOVE_CRUISE_POWER 5
#define MOVE_MIN_POWER 5
#define MOVE_RAMP_UP_MS 500
#define MOVE_RAMP_DOWN_DEGREES 90.0
#define MOVE_HEADING_GAIN 0.5
#define MOVE_HEADING_LIMIT 8
#define TURN_MIN_POWER 10
#define TURN_MAX_POWER 15
#define TURN_GAIN 0.6
#define TURN_TOLERANCE_DEGREES 3
#define TURN_SETTLE_US 150000
#define TURN_DEFAULT_TIMEOUT_MS 10000

enum {ROTATE_BOTH=0, ROTATE_LEFT_WHEEL=1, ROTATE_RIGHT_WHEEL=2};

static long elapsed_ms(const struct timespec *start)
{
 struct timespec now;
 clock_gettime(CLOCK_MONOTONIC,&now);
 return (now.tv_sec-start->tv_sec)*1000L+(now.tv_nsec-start->tv_nsec)/1000000L;
}

static MoveResult stop_with(StopReason reason,int colour)
{
 MoveResult result={reason,colour};
 // Always brake, including on errors, so a failed read can never leave the wheels running.
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) result.reason=STOP_ERROR;
 return result;
}

static int movement_setup(const int *colours,int count)
{
 if (count<0||(count>0&&colours==NULL)) return -1;
 for (int i=0;i<count;i++)
  if (colours[i]<1||colours[i]>6) return -1;
 struct timeval timeout={5,0};
 if (setsockopt(*socket_id,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))!=0||
     setsockopt(*socket_id,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout))!=0) return -1;
 // Require fresh confirmations so a colour seen before the move cannot trigger an immediate stop.
 reset_sensor_color_filter();
 return 0;
}

static int read_wheel_degrees(int *right,int *left)
{
 // Output indexes: A=0, B=1 (right wheel), C=2 (left wheel); counts are 32-bit little endian.
 const unsigned char operations[]={opOUTPUT_GET_COUNT,LC0(0),LC0(1),GV0(0),opOUTPUT_GET_COUNT,LC0(0),LC0(2),GV0(4)};
 unsigned char payload[8];
 if (calibration_command(operations,sizeof(operations),payload,sizeof(payload))!=0) return -1;
 *right=(int32_t)((uint32_t)payload[0]|(uint32_t)payload[1]<<8|(uint32_t)payload[2]<<16|(uint32_t)payload[3]<<24);
 *left=(int32_t)((uint32_t)payload[4]|(uint32_t)payload[5]<<8|(uint32_t)payload[6]<<16|(uint32_t)payload[7]<<24);
 return 0;
}

static int hit_stop_colour(const int *colours,int count)
{
 // Returns the matching colour ID, 0 for no match, -1 on a read error.
 int colour=get_sensor_color_robust();
 if (colour<0) return -1;
 for (int i=0;i<count;i++)
  if (colours[i]==colour) return colour;
 return 0;
}

static int ramp_power(int target_power,long elapsed)
{
 // Linear rise from the minimum power so the wheels never jerk up to speed.
 if (elapsed>=MOVE_RAMP_UP_MS) return target_power;
 return MOVE_MIN_POWER+(int)((target_power-MOVE_MIN_POWER)*elapsed/MOVE_RAMP_UP_MS);
}

static int clamp_power(int power)
{
 return power>100 ? 100 : (power<-100 ? -100 : power);
}

static MoveResult drive_straight(int direction,const int *stop_colours,int colour_count,double max_revolutions,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (movement_setup(stop_colours,colour_count)!=0||(max_revolutions<=0&&timeout_ms<=0)) return failed;
 int angle,rate,right0,left0;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1||read_wheel_degrees(&right0,&left0)!=0) return failed;
 double limit=max_revolutions>0 ? max_revolutions*360.0 : 0;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 for (;;)
 {
  long elapsed=elapsed_ms(&start);
  if (timeout_ms>0&&elapsed>=timeout_ms) return stop_with(STOP_TIMEOUT,0);
  int right,left;
  if (read_wheel_degrees(&right,&left)!=0) return stop_with(STOP_ERROR,0);
  double travelled=(abs(right-right0)+abs(left-left0))/2.0;
  if (limit>0&&travelled>=limit) return stop_with(STOP_DISTANCE,0);
  if (colour_count>0)
  {
   int colour=get_sensor_color_robust();
   if (colour<0) return stop_with(STOP_ERROR,0);
   for (int stop_index=0;stop_index<colour_count;stop_index++)
    if (colour==stop_colours[stop_index]) return stop_with(STOP_COLOUR,colour);
  }
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return stop_with(STOP_ERROR,0);
  int magnitude=ramp_power(MOVE_CRUISE_POWER,elapsed);
  if (limit>0&&limit-travelled<MOVE_RAMP_DOWN_DEGREES)
  {
   // Slow smoothly over the final stretch of a distance-limited move.
   int tapered=MOVE_MIN_POWER+(int)((MOVE_CRUISE_POWER-MOVE_MIN_POWER)*(limit-travelled)/MOVE_RAMP_DOWN_DEGREES);
   if (tapered<magnitude) magnitude=tapered;
  }
  // Gyro angle grows clockwise; a positive angle is drift to the right, so speed up the right wheel.
  int correction=(int)(MOVE_HEADING_GAIN*angle);
  if (correction>MOVE_HEADING_LIMIT) correction=MOVE_HEADING_LIMIT;
  if (correction<-MOVE_HEADING_LIMIT) correction=-MOVE_HEADING_LIMIT;
  int base=direction*magnitude;
  if (BT_turn(MOTOR_C,clamp_power(base-correction),MOTOR_B,clamp_power(base+correction))!=0)
   return stop_with(STOP_ERROR,0);
 }
}

static MoveResult rotate_gyro_to_target(int wheels,int target,int reset_gyro,
                                                const int *stop_colours,int colour_count,
                                                int timeout_ms,int min_power,int max_power)
{
 MoveResult failed={STOP_ERROR,0};
 if ((reset_gyro&&(abs(target)<1||abs(target)>360))||movement_setup(stop_colours,colour_count)!=0) return failed;
 // A turn always has a bound, so a dead gyro cannot spin the robot forever.
 if (timeout_ms<=0) timeout_ms=TURN_DEFAULT_TIMEOUT_MS;
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,reset_gyro,&angle,&rate)!=1) return failed;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 for (;;)
 {
  long elapsed=elapsed_ms(&start);
  if (elapsed>=timeout_ms) return stop_with(STOP_TIMEOUT,0);
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return stop_with(STOP_ERROR,0);
  if (colour_count>0)
  {
   int colour=hit_stop_colour(stop_colours,colour_count);
   if (colour<0) return stop_with(STOP_ERROR,0);
   if (colour>0) return stop_with(STOP_COLOUR,colour);
  }
  int error=target-angle;
  if (abs(error)<=TURN_TOLERANCE_DEGREES)
  {
   // Brake, let momentum settle, and accept only if the heading is still on target.
   MoveResult braked=stop_with(STOP_ANGLE,0);
   if (braked.reason!=STOP_ANGLE) return braked;
   usleep(TURN_SETTLE_US);
   if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return failed;
   if (abs(target-angle)<=TURN_TOLERANCE_DEGREES) return braked;
   continue;
  }
  // Power tapers with the remaining error; a negative error (overshoot) reverses the wheels to correct.
  int magnitude=(int)(TURN_GAIN*abs(error));
  int ceiling=elapsed>=MOVE_RAMP_UP_MS ? max_power :
    min_power+(max_power-min_power)*elapsed/MOVE_RAMP_UP_MS;
  if (magnitude<min_power) magnitude=min_power;
  if (magnitude>ceiling) magnitude=ceiling;
  int direction=error>0 ? 1 : -1;
  // Clockwise = left wheel forward and/or right wheel backward.
  int left=wheels!=ROTATE_RIGHT_WHEEL ? direction*magnitude : 0;
  int right=wheels!=ROTATE_LEFT_WHEEL ? -direction*magnitude : 0;
  if (BT_turn(MOTOR_C,left,MOTOR_B,right)!=0) return stop_with(STOP_ERROR,0);
 }
}

static MoveResult rotate_gyro_with_power_limits(int wheels,int turn_right,int angle_degrees,
                                                const int *stop_colours,int colour_count,
                                                int timeout_ms,int min_power,int max_power)
{
 MoveResult failed={STOP_ERROR,0};
 if (angle_degrees<1||angle_degrees>360) return failed;
 return rotate_gyro_to_target(wheels,turn_right ? angle_degrees : -angle_degrees,1,
   stop_colours,colour_count,timeout_ms,min_power,max_power);
}

static int center_turn_to_heading(int target)
{
 MoveResult result=rotate_gyro_to_target(ROTATE_BOTH,target,0,NULL,0,10000,
   TURN_MIN_POWER,TURN_MAX_POWER);
 return result.reason==STOP_ANGLE;
}

static MoveResult rotate_gyro(int wheels,int turn_right,int angle_degrees,const int *stop_colours,
                              int colour_count,int timeout_ms)
{
 return rotate_gyro_with_power_limits(wheels,turn_right,angle_degrees,stop_colours,colour_count,
                                      timeout_ms,TURN_MIN_POWER,TURN_MAX_POWER);
}

MoveResult move_forward_until(const int *stop_colours,int colour_count,double max_revolutions,int timeout_ms)
{
 return drive_straight(1,stop_colours,colour_count,max_revolutions,timeout_ms);
}

MoveResult move_backward_until(const int *stop_colours,int colour_count,double max_revolutions,int timeout_ms)
{
 return drive_straight(-1,stop_colours,colour_count,max_revolutions,timeout_ms);
}

MoveResult pivot_single_wheel(int angle_degrees,int turn_direction,int reverse,const int *stop_colours,int colour_count,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 int turn_right=turn_direction==0;
 // Left wheel drives a right-forward or left-reverse pivot; the right wheel drives the other two.
 int wheels=turn_right!=(reverse!=0) ? ROTATE_LEFT_WHEEL : ROTATE_RIGHT_WHEEL;
 return rotate_gyro(wheels,turn_right,angle_degrees,stop_colours,colour_count,timeout_ms);
}

MoveResult turn_in_place(int angle_degrees,int turn_direction,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 return rotate_gyro(ROTATE_BOTH,turn_direction==0,angle_degrees,NULL,0,timeout_ms);
}

static MoveResult turn_in_place_alignment(int angle_degrees,int turn_direction,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 return rotate_gyro_with_power_limits(ROTATE_BOTH,turn_direction==0,angle_degrees,NULL,0,
                                      timeout_ms,ALIGN_TURN_MIN_POWER,ALIGN_TURN_MAX_POWER);
}

MoveResult turn_in_place_until(int angle_degrees,int turn_direction,const int *stop_colours,int colour_count,int timeout_ms)
{
 MoveResult failed={STOP_ERROR,0};
 if (turn_direction!=0&&turn_direction!=1) return failed;
 return rotate_gyro(ROTATE_BOTH,turn_direction==0,angle_degrees,stop_colours,colour_count,timeout_ms);
}

int report_move_result(const char *label,MoveResult result)
{
 // Prints why a move ended; returns 1 for an error so callers can abort a sequence.
 switch (result.reason)
 {
  case STOP_TIMEOUT: printf("%s: stopped by timeout\n",label); break;
  case STOP_DISTANCE: printf("%s: stopped by distance limit\n",label); break;
  case STOP_COLOUR: printf("%s: stopped by colour %s (%d)\n",label,get_colour_name(result.colour),result.colour); break;
  case STOP_ANGLE: printf("%s: angle reached\n",label); break;
  default: fprintf(stderr,"%s: error, motors braked\n",label); return 1;
 }
 return 0;
}

static int continuous_recovery_sweep(void)
{
 return window_wipe_for_colour(0);
}

static int recover_black_heading(void)
{
 printf("Line lost; stopping, checking right before left within the bounded recovery range.\n");
 int result=continuous_recovery_sweep();
 if (result==1) result=center_on_alignment_marker_or_yellow(1,1);
 if (result==4)
 {
  if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0||sample_alignment_colour()!=4)
  {
   fprintf(stderr,"Recovery yellow did not remain confirmed while stopped.\n");
   return 0;
  }
  printf("Yellow intersection found during recovery wiggle; stopping there.\n");
  return 4;
 }
 if (result==5)
 {
  printf("Red boundary found during recovery sweep.\n");
  return 5;
 }
 if (result!=1)
 {
  fprintf(stderr,"Black line not found in the bounded recovery sweep.\n");
  return 0;
 }
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1) return 0;
 printf("Black reacquired; heading reset. Resuming forward.\n");
 return 1;
}

static int sample_marker_alignment_colour(int accept_yellow)
{
 int colour=sample_alignment_colour();
 if (!accept_yellow||colour!=4) return colour;
 // Driving recovery may reach yellow while probing for black. Confirm while
 // stopped rather than sweeping away from a possible intersection.
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return -1;
 return sample_alignment_colour();
}

static int measure_marker_edge(int turn_direction,int marker_colour,int origin,int *edge_degrees,
                               int accept_yellow)
{
 int turned=0;
 int last_marker=0,nonblack_count=0;
 while (turned<ROAD_CENTER_MAX_SWEEP_DEGREES)
 {
  turned+=ROAD_CENTER_STEP_DEGREES;
  if (!center_turn_to_heading(origin+(turn_direction==0 ? turned : -turned))) return -1;
  int angle,rate;
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return -1;
  int colour=sample_marker_alignment_colour(accept_yellow);
  if (colour<0) return -1;
  if (accept_yellow&&colour==4) return 4;
  if (colour==marker_colour)
  {
   nonblack_count=0;
   last_marker=abs(angle-origin);
   continue;
  }
  nonblack_count++;
  if (nonblack_count==2)
  {
   *edge_degrees=last_marker;
   return 1;
  }
 }
 return 0;
}

static int restore_alignment_heading(int origin,int marker_colour,int accept_yellow)
{
 if (!center_turn_to_heading(origin)) return 0;
 int colour=sample_marker_alignment_colour(accept_yellow);
 if (accept_yellow&&colour==4) return 4;
 if (colour!=marker_colour)
 {
  fprintf(stderr,"Restored heading did not verify %s; stopping alignment.\n",
    get_colour_name(marker_colour));
  return 0;
 }
 return 1;
}

// 1=centered, 0=failed, 4=confirmed yellow during driving recovery only.
static int center_on_alignment_marker_or_yellow(int marker_colour,int accept_yellow)
{
 int origin,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,0,&origin,&rate)!=1) return 0;
 int colour=sample_marker_alignment_colour(accept_yellow);
 if (colour<0) return 0;
 if (accept_yellow&&colour==4) return 4;
 if (colour!=marker_colour)
 {
  printf("Searching gently for %s from %s before measuring its edges.\n",
    get_colour_name(marker_colour),get_colour_name(colour));
  int acquired=0;
  for (int direction=0;direction<2&&!acquired;direction++)
  {
   int turned=0;
   while (turned<ROAD_CENTER_MAX_SWEEP_DEGREES)
   {
    turned+=ROAD_CENTER_STEP_DEGREES;
    if (!center_turn_to_heading(origin+(direction==0 ? turned : -turned))) return 0;
    colour=sample_marker_alignment_colour(accept_yellow);
    if (colour<0) return 0;
    if (accept_yellow&&colour==4) return 4;
    if (colour==marker_colour)
    {
     acquired=1;
     printf("Found %s after %s %d degrees.\n",get_colour_name(marker_colour),
       direction==1 ? "left" : "right",turned);
     break;
    }
   }
   if (!acquired&&turned>0)
   {
    if (!center_turn_to_heading(origin)) return 0;
   }
  }
  if (!acquired)
  {
   fprintf(stderr,"Could not find %s within the gentle search range.\n",
     get_colour_name(marker_colour));
   return 0;
  }
 }

 // Keep one gyro reference throughout: resetting it for each small relative
 // turn accumulates tolerance error and can leave the robot off the street.
 if (BT_read_gyro(GYRO_SENSOR_PORT,0,&origin,&rate)!=1) return 0;
 int right_edge=0;
 int right_status=measure_marker_edge(0,marker_colour,origin,&right_edge,accept_yellow);
 if (right_status==4) return 4;
 if (right_status<0) return 0;
 int restored=restore_alignment_heading(origin,marker_colour,accept_yellow);
 if (restored!=1) return restored;
 if (right_status==0)
 {
  printf("Could not find the right edge of %s within %d degrees; keeping current heading.\n",
    get_colour_name(marker_colour),ROAD_CENTER_MAX_SWEEP_DEGREES);
  return 1;
 }

 int left_edge=0;
 int left_status=measure_marker_edge(1,marker_colour,origin,&left_edge,accept_yellow);
 if (left_status==4) return 4;
 if (left_status!=1)
 {
  if (left_status<0) return 0;
  restored=restore_alignment_heading(origin,marker_colour,accept_yellow);
  if (restored!=1) return restored;
  printf("Could not find the left edge of %s within %d degrees; keeping current heading.\n",
    get_colour_name(marker_colour),ROAD_CENTER_MAX_SWEEP_DEGREES);
  return 1;
 }

 int center_offset=(right_edge-left_edge)/2;
 printf("%s bounds: left -%d, right +%d degrees; midpoint offset %d degrees.\n",
   get_colour_name(marker_colour),left_edge,right_edge,center_offset);
 if (!center_turn_to_heading(origin+center_offset)) return 0;
 colour=sample_marker_alignment_colour(accept_yellow);
 if (colour<0) return 0;
 if (accept_yellow&&colour==4) return 4;
 if (colour!=marker_colour)
 {
  fprintf(stderr,"Marker midpoint did not verify %s; restoring entry heading.\n",
    get_colour_name(marker_colour));
  return restore_alignment_heading(origin,marker_colour,accept_yellow);
 }
 else
  printf("Centered on %s marker.\n",get_colour_name(marker_colour));
 return 1;
}

static int center_on_alignment_marker(int marker_colour)
{
 return center_on_alignment_marker_or_yellow(marker_colour,0)==1;
}

static int follow_black_street(void)
{
 // Leaving an intersection invalidates the axle placement, even if the trip fails.
 intersection_axle_centered=0;
 const double max_forward_revolutions=2;
 const int timeout_ms=60000;
 const int follow_power=street_follow_power>2 ? 2 : street_follow_power;
 const int steering_correction_limit=follow_power<8 ? follow_power/2 : 4;
 int angle,rate,right_previous,left_previous;
 int alignment_colour=next_follow_alignment_colour;
 next_follow_alignment_colour=1;
 int alignment_result=acquiring_first_intersection ?
   center_on_alignment_marker_or_yellow(alignment_colour,1) :
   center_on_alignment_marker(alignment_colour);
 if (alignment_result==4)
 {
  printf("Initial yellow confirmed during alignment; using it as the first intersection.\n");
  return 1;
 }
 if (alignment_result!=1) return 0;
 if (movement_setup(NULL,0)!=0||BT_read_gyro(GYRO_SENSOR_PORT,1,&angle,&rate)!=1||
   read_wheel_degrees(&right_previous,&left_previous)!=0) return 0;
 double forward_revolutions=0;
 int last_healthy_colour=alignment_colour;
 int observed_black=0;
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 printf("Following black street until yellow or red.\n");

 for (;;)
 {
  if (elapsed_ms(&start)>=timeout_ms)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   fprintf(stderr,"Street following timed out before a marker.\n");
   return 0;
  }
  int right,left;
  if (read_wheel_degrees(&right,&left)!=0||BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   return 0;
  }
  forward_revolutions+=(abs(right-right_previous)+abs(left-left_previous))/720.0;
  right_previous=right;
  left_previous=left;
  if (forward_revolutions>=max_forward_revolutions)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   fprintf(stderr,"No intersection or boundary within one cell.\n");
   return 0;
  }

  int colour=sample_street_event_colour();
  // Later trips require a black departure. The initial search needs only a
  // confirmed yellow anchor; it does not claim an intersection-to-intersection move.
  if (colour==1) observed_black=1;
  if (colour==0)
  {
   colour=last_healthy_colour;
   printf("Unknown street reading; retaining last healthy colour %s.\n",
     get_colour_name(last_healthy_colour));
  }
  if (colour==4&&(observed_black||acquiring_first_intersection))
  {
   if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return 0;
   if (acquiring_first_intersection&&sample_alignment_colour()!=4)
   {
    fprintf(stderr,"Initial yellow did not remain confirmed while stopped.\n");
    return 0;
   }
    printf("Intersection reached; stopped on yellow.\n");
    return 1;
  }
  if (colour==5)
  {
    MoveResult stopped=stop_with(STOP_COLOUR,colour);
    if (stopped.reason!=STOP_COLOUR)
    {
     fprintf(stderr,"Unable to brake at the red boundary; stopping street follow.\n");
     return 0;
    }
   printf("Red boundary reached.\n");
   return 2;
  }
  if (colour==1||colour==4)
  {
    last_healthy_colour=colour;
    if (colour==4) printf("Leaving starting yellow; waiting for black before accepting an intersection.\n");
    // Stay on the gyro-held heading while the sensor confirms the road.
  }
  else
  {
   if (colour<0)
   {
    fprintf(stderr,"Colour sensor read failed while following street.\n");
    BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
    return 0;
   }
    if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return 0;
    printf("Black lost (%s); starting bounded left/right wiggle.\n",get_colour_name(colour));
    int recovery= recover_black_heading();
    if (recovery==4)
    {
     if (observed_black||acquiring_first_intersection) return 1;
     fprintf(stderr,"Recovery found yellow before a black street departure; trip unconfirmed.\n");
     return 0;
    }
    if (recovery==5) return 2;
    if (recovery!=1) return 0;
    if (read_wheel_degrees(&right_previous,&left_previous)!=0)
    {
     BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
     return 0;
    }
    continue;
  }

  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   return 0;
  }
    int correction=(int)(MOVE_HEADING_GAIN*angle);
    if (correction>steering_correction_limit) correction=steering_correction_limit;
    if (correction< -steering_correction_limit) correction=-steering_correction_limit;
    int left_power=follow_power-correction;
    int right_power=follow_power+correction;
    if (left_power>2) left_power=2;
    if (right_power>2) right_power=2;
    if (BT_turn(MOTOR_C,left_power,MOTOR_B,right_power)!=0)
  {
   BT_motor_port_stop(MOTOR_B|MOTOR_C,1);
   return 0;
  }
 }
}

// One-segment diagnostics use the real street follower, with no exploration or
// boundary turnaround. Yellow is a sensor outcome; the operator must verify
// it is the intended next intersection rather than the starting intersection.
static int street_test_preflight(void)
{
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) return 0;
 int angle,rate,minimum=INT_MAX,maximum=INT_MIN;
 for (int sample=0;sample<5;sample++)
 {
  if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1) return 0;
  if (angle<minimum) minimum=angle;
  if (angle>maximum) maximum=angle;
  if (sample<4) usleep(200000);
 }
 printf("Stationary gyro spread: %lld degrees.\n",(long long)maximum-minimum);
 if ((long long)maximum-minimum>2)
 {
  fprintf(stderr,"STREET_RESULT,gyro_unstable\n");
  return 0;
 }
 int initial_colour=sample_alignment_colour();
 if (initial_colour!=1)
 {
  fprintf(stderr,"STREET_RESULT,invalid_start\nStart on black just beyond yellow; saw %s.\n",get_colour_name(initial_colour));
  return 0;
 }
 return 1;
}

static int run_street_segment_test(void)
{
 if (!street_test_preflight()) return 0;
 int right0,left0,right1,left1;
 if (read_wheel_degrees(&right0,&left0)!=0) return 0;
 printf("Street test: power %d; sensor offset %.6f wheel revolutions (configured, not measured).\n",
        street_follow_power,(double)SENSOR_TO_AXLE_OFFSET_REV);
 fflush(stdout);
 struct timespec start;
 clock_gettime(CLOCK_MONOTONIC,&start);
 int result=follow_black_street();
 long duration=elapsed_ms(&start);
 int brake_ok=BT_motor_port_stop(MOTOR_B|MOTOR_C,1)==0;
 int encoders_ok=read_wheel_degrees(&right1,&left1)==0;
 int final_colour=brake_ok ? sample_alignment_colour() : -1;
 if (encoders_ok)
  printf("STREET_ENCODERS,%lld,%lld\n",(long long)right1-right0,(long long)left1-left0);
 printf("Wheel deltas include initial centering and recovery turns; they are not straight-line distance.\n");
 printf("STREET_DURATION_MS,%ld\n",duration);
 const char *outcome=!brake_ok ? "brake_failed" : !encoders_ok ? "encoder_error" :
   result==2 ? "red_border" : result==1&&final_colour==4 ? "yellow_detected" :
   result==1 ? "yellow_unverified" : "following_failed";
 printf("STREET_RESULT,%s\n",outcome);
 fflush(stdout);
 return brake_ok&&encoders_ok&&result==1&&final_colour==4;
}

static int run_turn_street_test(int direction)
{
 if (!street_test_preflight()) return 0;
 printf("Turn test: %s 90 degrees from a manually centered axle.\n",direction==0 ? "right" : "left");
 fflush(stdout);
 MoveResult result=turn_in_place(90,direction,10000);
 int brake_ok=BT_motor_port_stop(MOTOR_B|MOTOR_C,1)==0;
 if (result.reason!=STOP_ANGLE||!brake_ok)
 {
  printf("TURN_RESULT,%s\n",brake_ok ? "turn_failed" : "brake_failed");
  return 0;
 }
 int angle,rate;
 if (BT_read_gyro(GYRO_SENSOR_PORT,0,&angle,&rate)!=1)
 {
  puts("TURN_RESULT,gyro_error");
  return 0;
 }
 printf("TURN_GYRO_DEGREES,%d\n",angle);
 // Inspect the result before allowing any centering/recovery to hide a bad turn.
 int colour=sample_alignment_colour();
 printf("Turn exit sensor: %s (%d).\n",get_colour_name(colour),colour);
 if (colour!=1)
 {
  puts("TURN_RESULT,not_on_black");
  return 0;
 }
 puts("TURN_RESULT,black_aligned");
 return run_street_segment_test();
}

static int street_test_main(int argc,char **argv)
{
 int turning=strcmp(argv[1],"--test-turn-street")==0;
 int required=turning ? 4 : 3;
 if (argc!=required&&argc!=required+1)
 {
  fprintf(stderr,"Usage: EV3_Localization --test-street calibration_directory [power 1..20]\n");
  fprintf(stderr,"       EV3_Localization --test-turn-street calibration_directory left|right [power 1..20]\n");
  return 1;
 }
 int direction=-1;
 if (turning)
 {
  if (strcmp(argv[3],"right")==0) direction=0;
  else if (strcmp(argv[3],"left")==0) direction=1;
  else return 1;
 }
 if (argc==required+1)
 {
  char *end;
  errno=0;
  long power=strtol(argv[required],&end,10);
  if (errno||end==argv[required]||*end||power<1||power>20) return 1;
  street_follow_power=(int)power;
 }
 if (load_colour_calibration(argv[2])!=0)
 {
  fprintf(stderr,"Invalid calibration directory.\n");
  return 1;
 }
 if (BT_open(HEXKEY)!=0) return 1;
 int setup_ok=movement_setup(NULL,0)==0&&verify_nxt_colour_sensor()==0;
 int success=setup_ok&&(turning ? run_turn_street_test(direction) : run_street_segment_test());
 if (BT_motor_port_stop(MOTOR_B|MOTOR_C,1)!=0) success=0;
 BT_close();
 return success ? 0 : 1;
}

static int geometry_check_main(int argc,char **argv)
{
 if (argc!=4)
 {
  fprintf(stderr,"Usage: EV3_Localization --check-geometry sensor_offset_mm wheel_diameter_mm\n");
  return 1;
 }
 double measurements[2];
 for (int index=0;index<2;index++)
 {
  char *end;
  errno=0;
  measurements[index]=strtod(argv[index+2],&end);
  if (errno||end==argv[index+2]||*end||!isfinite(measurements[index])||measurements[index]<=0.0) return 1;
 }
 double revolutions=measurements[0]/(acos(-1.0)*measurements[1]);
 if (!isfinite(revolutions)||revolutions<=0.0) return 1;
 printf("Measured sensor-to-axle advance: %.6f wheel revolutions.\n",revolutions);
 printf("Configured SENSOR_TO_AXLE_OFFSET_REV: %.6f.\n",(double)SENSOR_TO_AXLE_OFFSET_REV);
 printf("Calculation only: no connection, motion, or configuration change.\n");
 return 0;
}

// Map/PPM parsing is independent of robot control. All pixel offsets use size_t.
static int map_pixel_is_yellow(const unsigned char *image,int width,int x,int y)
{
 const unsigned char *pixel=image+((size_t)y*(size_t)width+(size_t)x)*3;
 return pixel[0]==255&&pixel[1]==255&&pixel[2]==0;
}

static int map_axis_intersections(const unsigned char *image,int rx,int ry,int fixed,
                                  int horizontal,int starts[400],int sizes[400],int *count)
{
 int limit=horizontal ? rx : ry;
 int position=0;
 *count=0;
 while (position<limit)
 {
  if (!map_pixel_is_yellow(image,rx,horizontal ? position : fixed,horizontal ? fixed : position))
  {position++;continue;}
  int first=position;
  do {position++;}
  while (position<limit&&map_pixel_is_yellow(image,rx,
         horizontal ? position : fixed,horizontal ? fixed : position));
  if (*count>=400) return 0;
  starts[*count]=first;
  sizes[*count]=position-first;
  (*count)++;
 }
 return *count>0;
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 // Yellow markers must form a complete grid. Corners are sampled clockwise
 // TL,TR,BR,BL, at the same offsets used by the assignment's original parser.
 // Commit the map only after all geometry, capacity and colour checks pass.
 sx=sy=0;
 if (map_img==NULL||rx<=0||ry<=0||(size_t)ry>((size_t)-1)/3/(size_t)rx)
 {
  fprintf(stderr,"Invalid map image dimensions.\n");
  return 0;
 }
 int first_x=-1,first_y=-1;
 for (int y=0;y<ry&&first_y<0;y++)
  for (int x=0;x<rx;x++)
   if (map_pixel_is_yellow(map_img,rx,x,y)) {first_x=x;first_y=y;break;}
 if (first_y<0)
 {
  fprintf(stderr,"Map contains no yellow intersections.\n");
  return 0;
 }

 int columns[400],column_widths[400],rows[400],row_heights[400],nx,ny;
 if (!map_axis_intersections(map_img,rx,ry,first_y,1,columns,column_widths,&nx)||
     !map_axis_intersections(map_img,rx,ry,first_x,0,rows,row_heights,&ny))
 {
  fprintf(stderr,"Invalid intersection sizes, spacing, or axis capacity.\n");
  return 0;
 }
 if (nx>400/ny)
 {
  fprintf(stderr,"Map exceeds the 400-intersection capacity.\n");
  return 0;
 }

 // The supplied map has unequal marker sizes and one-pixel edge irregularities.
 // Require solid marker interiors and reject yellow outside a one-pixel margin
 // around the inferred rectangles; do not require exact spacing or edge pixels.
 int row=0;
 for (int y=0;y<ry;y++)
 {
  while (row<ny&&y>rows[row]+row_heights[row]) row++;
  int in_row=row<ny&&y>=rows[row]-1;
  int column=0;
  for (int x=0;x<rx;x++)
  {
   while (column<nx&&x>columns[column]+column_widths[column]) column++;
   int allowed=in_row&&column<nx&&x>=columns[column]-1;
   int required=allowed&&x>=columns[column]+1&&x<columns[column]+column_widths[column]-1&&
                y>=rows[row]+1&&y<rows[row]+row_heights[row]-1;
   int yellow=map_pixel_is_yellow(map_img,rx,x,y);
   if ((yellow&&!allowed)||(!yellow&&required))
   {
    fprintf(stderr,"Yellow intersections do not form a complete rectangular grid.\n");
    return 0;
   }
  }
 }

 int parsed[400][4]={{0}};
 const int corner_x[4]={-1,1,1,-1},corner_y[4]={-1,-1,1,1};
 for (int y=0;y<ny;y++) for (int x=0;x<nx;x++)
 {
  int wx=column_widths[x],wy=row_heights[y];
  if (!map_pixel_is_yellow(map_img,rx,columns[x]+wx/2,rows[y]+wy/2))
  {
   fprintf(stderr,"Missing yellow intersection at (%d,%d).\n",x,y);
   return 0;
  }
  for (int corner=0;corner<4;corner++)
  {
   // Use wider arithmetic until bounds are checked, including huge dimensions.
   long long px=(long long)columns[x]+wx/2+(long long)corner_x[corner]*wx;
   long long py=(long long)rows[y]+wy/2+(long long)corner_y[corner]*wy;
   if (px<0||px>=rx||py<0||py>=ry)
   {
    fprintf(stderr,"Building sample outside image at intersection (%d,%d).\n",x,y);
    return 0;
   }
   const unsigned char *pixel=map_img+((size_t)py*(size_t)rx+(size_t)px)*3;
   int colour=0;
   if (pixel[0]==0&&pixel[1]==0&&pixel[2]==255) colour=2;
   else if (pixel[0]==0&&pixel[1]==255&&pixel[2]==0) colour=3;
   else if (pixel[0]==255&&pixel[1]==255&&pixel[2]==255) colour=6;
   if (colour==0)
   {
    fprintf(stderr,"Invalid building colour at intersection (%d,%d), corner %d.\n",x,y,corner);
    return 0;
   }
   parsed[y*nx+x][corner]=colour;
  }
 }

 memset(map,0,sizeof(map));
 memcpy(map,parsed,(size_t)nx*(size_t)ny*4*sizeof(int));
 sx=nx;sy=ny;
 fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);
 return 1;
}

static int ppm_header_token(FILE *input,char *token,size_t capacity,int *separator)
{
 int character;
 for (;;)
 {
  character=fgetc(input);
  if (character==EOF) return 0;
  if (isspace((unsigned char)character)) continue;
  if (character!='#') break;
  do {character=fgetc(input);} while (character!=EOF&&character!='\n'&&character!='\r');
 }
 size_t length=0;
 do
 {
  if (character=='\0') return 0;
  if (length+1>=capacity) return 0;
  token[length++]=(char)character;
  character=fgetc(input);
 } while (character!=EOF&&!isspace((unsigned char)character));
 token[length]='\0';
 *separator=character;
 return 1;
}

static int ppm_positive_integer(const char *text,int *value)
{
 // Reject signs, suffixes, overflow, and empty/zero dimensions.
 if (*text=='\0') return 0;
 for (const char *p=text;*p;p++) if (*p<'0'||*p>'9') return 0;
 errno=0;
 char *end;
 long parsed=strtol(text,&end,10);
 if (errno!=0||*end!='\0'||parsed<1||parsed>INT_MAX) return 0;
 *value=(int)parsed;
 return 1;
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Single binary P6 image, 8-bit RGB. Header comments and CRLF are supported;
 // after maxval, consume only the separator, never whitespace-valued pixels.
 if (rx==NULL||ry==NULL||rx==ry) return NULL;
 *rx=*ry=0;
 if (filename==NULL) return NULL;
 FILE *input=fopen(filename,"rb");
 if (input==NULL)
 {
  fprintf(stderr,"Unable to open map image for reading.\n");
  return NULL;
 }
 char token[64];
 int separator,width,height,maxval;
 if (!ppm_header_token(input,token,sizeof(token),&separator)||strcmp(token,"P6")!=0||
     !ppm_header_token(input,token,sizeof(token),&separator)||!ppm_positive_integer(token,&width)||
     !ppm_header_token(input,token,sizeof(token),&separator)||!ppm_positive_integer(token,&height)||
     !ppm_header_token(input,token,sizeof(token),&separator)||!ppm_positive_integer(token,&maxval)||
     maxval!=255||separator==EOF||(size_t)height>((size_t)-1)/3/(size_t)width)
 {
  fprintf(stderr,"Invalid P6 PPM header or image dimensions.\n");
  fclose(input);
  return NULL;
 }
 size_t bytes=(size_t)width*(size_t)height*3;
 // Check the actual file length before allocating from untrusted dimensions.
 // Map files contain exactly one image; truncated or extra payload is rejected.
 long raster_start=ftell(input);
 int length_ok=raster_start>=0&&fseek(input,0,SEEK_END)==0;
 long file_end=length_ok ? ftell(input) : -1;
 // Distinguish a CRLF separator from a CR separator followed by a pixel whose
 // first byte is LF. File length tells us whether the extra LF is header data.
 if (length_ok&&separator=='\r'&&file_end>raster_start&&
     (unsigned long long)(file_end-raster_start-1)==(unsigned long long)bytes)
 {
  if (fseek(input,raster_start,SEEK_SET)!=0||fgetc(input)!='\n') length_ok=0;
  else raster_start++;
 }
 if (!length_ok||file_end<raster_start||
     (unsigned long long)(file_end-raster_start)!=(unsigned long long)bytes||
     fseek(input,raster_start,SEEK_SET)!=0)
 {
  fprintf(stderr,"PPM raster length does not match its dimensions.\n");
  fclose(input);
  return NULL;
 }
 unsigned char *image=(unsigned char *)malloc(bytes);
 if (image==NULL)
 {
  fprintf(stderr,"Unable to allocate map image.\n");
  fclose(input);
  return NULL;
 }
 int read_ok=fread(image,1,bytes,input)==bytes&&!ferror(input);
 int close_ok=fclose(input)==0;
 if (!read_ok||!close_ok)
 {
  fprintf(stderr,"Unable to read complete PPM raster.\n");
  free(image);
  return NULL;
 }
 *rx=width;*ry=height;
 return image;
}
