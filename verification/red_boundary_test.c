/* Tests the real boundary control helpers with scripted hardware outcomes. */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define MOTOR_B 2
#define MOTOR_C 4
typedef enum {STOP_ERROR=-1,STOP_TIMEOUT=0,STOP_DISTANCE=1,STOP_COLOUR=2,STOP_ANGLE=3} StopReason;
typedef struct {StopReason reason;int colour;} MoveResult;
#define FIND_STREET_MAX_REV 1.1
#define FIND_STREET_TIMEOUT_MS 22000

int sx=3,sy=3;
double beliefs[400][4];
static int drive_results[3],drive_count,drive_calls;
static MoveResult search_results[4];
static int search_count,search_calls;
static StopReason turn_results[3];
static int turn_count,turn_calls;
static int sweep_colour,reacquired_colour,brake_calls;
static int center_calls,sweep_calls,sample_calls,sample_sequence[8],sample_sequence_count;
static int physical_heading,initial_heading,commands_valid;

static MoveResult turn_in_place(int degrees,int direction,int timeout)
{
 if (degrees!=180||direction!=0||timeout!=15000) commands_valid=0;
 StopReason reason=turn_calls<turn_count ? turn_results[turn_calls] : STOP_ERROR;
 turn_calls++;
 if (reason==STOP_ANGLE) physical_heading=(physical_heading+2)%4;
 MoveResult result={reason,0};return result;
}

static MoveResult move_forward_until(const int *colours,int count,double distance,int timeout)
{
 if (!colours||count!=3||colours[0]!=1||colours[1]!=4||colours[2]!=5||
     fabs(distance-FIND_STREET_MAX_REV)>1e-12||timeout!=FIND_STREET_TIMEOUT_MS)
  commands_valid=0;
 MoveResult failure={STOP_ERROR,0};
 MoveResult result=search_calls<search_count ? search_results[search_calls] : failure;
 search_calls++;return result;
}

static int drive_along_street(void)
{
 int result=drive_calls<drive_count ? drive_results[drive_calls] : 0;
 drive_calls++;return result;
}
static int center_on_alignment_marker(int colour) {center_calls++;return colour==1;}
static int continuous_recovery_sweep(void) {sweep_calls++;return sweep_colour;}
static int sample_alignment_colour(void)
{
 int index=sample_calls++;
 return index<sample_sequence_count ? sample_sequence[index] : reacquired_colour;
}
static int BT_motor_port_stop(int ports,int brake)
{if (ports!=(MOTOR_B|MOTOR_C)||brake!=1) commands_valid=0;brake_calls++;return 0;}
static int report_move_result(const char *label,MoveResult result)
{(void)label;(void)result;return 0;}

// Generated directly from EV3_Localization.c by test_red_boundary.py.
#include "boundary_under_test.inc"

#define CHECK(condition) do {if (!(condition)) { \
 fprintf(stderr,"Check failed: %s, line %d; turns=%d starting-heading=%d physical-heading=%d\n", \
 #condition,__LINE__,turn_calls,initial_heading,physical_heading);return 0;}} while (0)

static void reset(int heading)
{
 initial_heading=physical_heading=heading;
 drive_calls=search_calls=turn_calls=center_calls=sweep_calls=sample_calls=brake_calls=0;
 sample_sequence_count=0;
 drive_count=2;drive_results[0]=2;drive_results[1]=1;
 turn_count=1;turn_results[0]=STOP_ANGLE;
 search_count=1;search_results[0].reason=STOP_COLOUR;search_results[0].colour=1;
 sweep_colour=1;reacquired_colour=1;commands_valid=1;
 sx=sy=3;
 memset(beliefs,0,sizeof(beliefs));
}

static int direct_yellow(void)
{
 drive_results[0]=1;
 int heading=initial_heading,boundary=-1;
 CHECK(drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&heading==initial_heading&&turn_calls==0&&drive_calls==1);
 return 1;
}

static int red_return(void)
{
 int heading=initial_heading,boundary=-1;
 CHECK(drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==1&&heading==(initial_heading+2)%4&&heading==physical_heading);
 CHECK(turn_calls==1&&drive_calls==2&&search_calls==1&&sweep_calls>=1&&
   center_calls==1&&commands_valid);
 const int edge[4]={1,5,7,3};
 beliefs[edge[initial_heading]][initial_heading]=1;
 CHECK(update_beliefs_after_forward(boundary));
 double sum=0;
 for (int i=0;i<9;i++) for (int h=0;h<4;h++)
 {CHECK(isfinite(beliefs[i][h])&&beliefs[i][h]>0);sum+=beliefs[i][h];}
 CHECK(fabs(sum-1)<1e-12);
 CHECK(beliefs[edge[initial_heading]][heading]>0.95);
 // Probability stays at the original edge intersection, not one cell inward.
 CHECK(beliefs[4][heading]<0.00001);
 return 1;
}

static int yellow_during_reacquisition(void)
{
 search_results[0].colour=4;
 reacquired_colour=4;
 int heading=initial_heading,boundary=-1;
 CHECK(drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==1&&heading==(initial_heading+2)%4&&heading==physical_heading);
 CHECK(turn_calls==1&&drive_calls==1&&search_calls==1&&sweep_calls==0&&sample_calls==2);
 const int edge[4]={1,5,7,3};
 beliefs[edge[initial_heading]][initial_heading]=1;
 CHECK(update_beliefs_after_forward(boundary));
 CHECK(beliefs[edge[initial_heading]][heading]>0.95);
 return 1;
}

static int yellow_during_return_wipe(void)
{
 sweep_colour=4;reacquired_colour=1;
 sample_sequence[0]=1;sample_sequence[1]=4;sample_sequence[2]=4;sample_sequence_count=3;
 int heading=initial_heading,boundary=-1;
 CHECK(drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==1&&heading==(initial_heading+2)%4&&heading==physical_heading);
 CHECK(turn_calls==1&&drive_calls==1&&search_calls==1&&sweep_calls==1&&sample_calls==2);
 const int edge[4]={1,5,7,3};
 beliefs[edge[initial_heading]][initial_heading]=1;
 CHECK(update_beliefs_after_forward(boundary));
 CHECK(beliefs[edge[initial_heading]][heading]>0.95);
 return 1;
}

static int black_reacquired_after_yellow_miss(void)
{
 reacquired_colour=6;int heading=initial_heading,boundary=-1;
 CHECK(drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==1&&heading==physical_heading&&center_calls==1&&sweep_calls==1&&turn_calls==1);
 return 1;
}

static int failed_alignment(void)
{
 sweep_colour=0;reacquired_colour=6;int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&drive_calls==1&&sweep_calls==1);
 return 1;
}

static int failed_turn(void)
{
 turn_results[0]=STOP_TIMEOUT;int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&heading==initial_heading&&search_calls==0&&drive_calls==1);
 return 1;
}

static int failed_search(void)
{
 search_results[0].reason=STOP_ERROR;int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&sweep_calls==0&&drive_calls==1);
 return 1;
}

static int timed_search_recovered(void)
{search_results[0].reason=STOP_TIMEOUT;return red_return();}

static int timed_search_failed(void)
{
 search_results[0].reason=STOP_TIMEOUT;sweep_colour=0;
 int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&sweep_calls==1&&drive_calls==1);
 return 1;
}

static int failed_return_trip(void)
{
 drive_results[1]=0;int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&drive_calls==2&&turn_calls==1);
 return 1;
}

static int repeated_red_is_bounded(void)
{
 drive_results[1]=2;int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&drive_calls==2&&turn_calls==1);
 return 1;
}

static int nested_red_preserves_heading(void)
{
 // Red during reacquisition must stop, rather than perform a second half-turn.
 search_count=2;search_results[0].colour=5;
 search_results[1].reason=STOP_COLOUR;search_results[1].colour=1;
 turn_count=2;turn_results[1]=STOP_ANGLE;
 int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&turn_calls==1&&search_calls==1&&drive_calls==1);
 CHECK(physical_heading==(initial_heading+2)%4);
 return 1;
}

static int red_during_recovery_sweep_stops(void)
{
 search_results[0].reason=STOP_TIMEOUT;sweep_colour=5;
 int heading=initial_heading,boundary=-1;
 CHECK(!drive_to_next_intersection(&heading,&boundary));
 CHECK(boundary==0&&turn_calls==1&&search_calls==1&&drive_calls==1&&sweep_calls==1);
 return 1;
}

static int initial_red_still_turns(void)
{
 search_count=2;search_results[0].colour=5;
 search_results[1].reason=STOP_COLOUR;search_results[1].colour=1;
 CHECK(find_street());
 CHECK(turn_calls==1&&search_calls==2&&physical_heading==(initial_heading+2)%4);
 return 1;
}

static int initial_sweep_red_still_turns(void)
{
 search_count=2;search_results[0].reason=STOP_TIMEOUT;
 search_results[1].reason=STOP_COLOUR;search_results[1].colour=1;
 sweep_colour=5;
 CHECK(find_street());
 CHECK(turn_calls==1&&search_calls==2&&sweep_calls==1&&physical_heading==(initial_heading+2)%4);
 return 1;
}

static int null_arguments(void)
{
 int heading=0,boundary=-1;
 CHECK(!drive_to_next_intersection(NULL,&boundary));
 CHECK(!drive_to_next_intersection(&heading,NULL));
 CHECK(drive_calls==0&&turn_calls==0);
 return 1;
}

int main(void)
{
 struct Test {const char *name;int (*run)(void);} tests[]={
  {"direct_yellow",direct_yellow},{"red_return_same_intersection",red_return},
  {"yellow_during_reacquisition",yellow_during_reacquisition},
  {"yellow_during_return_wipe",yellow_during_return_wipe},
    {"black_reacquired_after_yellow_miss",black_reacquired_after_yellow_miss},
    {"failed_black_recovery_stops",failed_alignment},
  {"turn_timeout",failed_turn},{"sensor_search_error",failed_search},
  {"search_timeout_recovered",timed_search_recovered},{"search_timeout_failed",timed_search_failed},
  {"return_trip_failure",failed_return_trip},{"repeated_red_bounded",repeated_red_is_bounded},
  {"nested_red_heading_consistency",nested_red_preserves_heading},
  {"red_during_recovery_sweep_stops",red_during_recovery_sweep_stops},
  {"initial_red_still_turns",initial_red_still_turns},
  {"initial_sweep_red_still_turns",initial_sweep_red_still_turns},
  {"null_arguments",null_arguments}
 };
 int failures=0;
 for (unsigned i=0;i<sizeof(tests)/sizeof(tests[0]);i++) for (int h=0;h<4;h++)
 {
  reset(h);int passed=tests[i].run();
  printf("TEST,%s,%d,%s\n",tests[i].name,h,passed ? "PASS" : "FAIL");
  failures+=!passed;
 }
 return failures ? 1 : 0;
}
