#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define MOTOR_B 2
#define MOTOR_C 4
#define GYRO_SENSOR_PORT 1
#define MOVE_HEADING_GAIN 1.0
typedef enum {STOP_ERROR=-1,STOP_COLOUR=2} StopReason;
typedef struct {StopReason reason;int colour;} MoveResult;
static int intersection_axle_centered,next_follow_alignment_colour,street_follow_power=10;
static int acquiring_first_intersection;
static int events[8],event_count,event_index,motor_calls,brake_calls,wheel_calls;
static int center_failure,recovery_colour,center_colour;
static int initial_alignment_result,stationary_colour,brake_failure;
static long elapsed_ms(const struct timespec *start) {(void)start;return 0;}
static int center_on_alignment_marker(int colour) {center_colour=colour;return !center_failure;}
static int center_on_alignment_marker_or_yellow(int colour,int accept_yellow)
{assert(accept_yellow==1);center_colour=colour;return initial_alignment_result;}
static int sample_alignment_colour(void) {return stationary_colour;}
static int movement_setup(const int *colours,int count) {assert(colours==NULL&&count==0);return 0;}
static int BT_read_gyro(int port,int reset,int *angle,int *rate)
{assert(port==1&&(reset==0||reset==1));*angle=*rate=0;return 1;}
static int read_wheel_degrees(int *right,int *left) {*right=*left=++wheel_calls*18;return 0;}
static int BT_motor_port_stop(int ports,int brake)
{assert(ports==6&&brake==1);return ++brake_calls==brake_failure ? -1 : 0;}
static int BT_turn(int first,int power1,int second,int power2)
{assert(first==4&&second==2&&power1==2&&power2==2);motor_calls++;return 0;}
static int sample_street_event_colour(void)
{int index=event_index++;return events[index<event_count ? index : event_count-1];}
static MoveResult stop_with(StopReason reason,int colour)
{
 assert(reason==STOP_COLOUR&&(colour==4||colour==5));
 MoveResult result={BT_motor_port_stop(MOTOR_B|MOTOR_C,1)==0 ? reason : STOP_ERROR,colour};
 return result;
}
static const char *get_colour_name(int colour) {(void)colour;return "fixture";}
static int recover_black_heading(void) {return recovery_colour;}
#include "street_departure_under_test.inc"
static void reset(const int *script,int count)
{
 assert(count>0&&count<=8);
 for (int index=0;index<count;index++) events[index]=script[index];
 event_count=count;event_index=motor_calls=brake_calls=wheel_calls=0;
 center_failure=0;recovery_colour=1;center_colour=0;
 next_follow_alignment_colour=1;intersection_axle_centered=1;
 acquiring_first_intersection=0;initial_alignment_result=1;stationary_colour=4;brake_failure=0;
}
int main(void)
{
 const int normal[]={1,1,4};reset(normal,3);
 assert(follow_black_street()==1&&event_index==3&&motor_calls==2&&!intersection_axle_centered);
 const int starting_yellow[]={4,4,1,4};reset(starting_yellow,4);next_follow_alignment_colour=4;
 assert(follow_black_street()==1&&event_index==4&&motor_calls==3&&center_colour==4);
 const int only_yellow[]={4};reset(only_yellow,1);
 assert(follow_black_street()==0&&motor_calls>0&&brake_calls==1);
 const int unknown_first[]={0,4};reset(unknown_first,2);
 assert(follow_black_street()==0&&event_index>2); /* Retained black cannot arm arrival. */
 const int noisy_departure[]={4,0,4,1,0,4};reset(noisy_departure,6);
 assert(follow_black_street()==1&&event_index==6);
 const int red[]={5};reset(red,1);
 assert(follow_black_street()==2&&motor_calls==0&&brake_calls==1);
 reset(red,1);brake_failure=1;
 assert(follow_black_street()==0&&motor_calls==0&&brake_calls==1);
 const int error[]={-1};reset(error,1);
 assert(follow_black_street()==0&&motor_calls==0&&brake_calls==1);
 const int lost_before_departure[]={6};reset(lost_before_departure,1);recovery_colour=4;
 assert(follow_black_street()==0&&motor_calls==0&&brake_calls==1);
 const int lost_after_departure[]={1,6};reset(lost_after_departure,2);recovery_colour=4;
 assert(follow_black_street()==1&&motor_calls==1&&brake_calls==1);
 reset(normal,3);center_failure=1;
 assert(follow_black_street()==0&&motor_calls==0&&event_index==0);
 /* Only initial acquisition may use yellow before travelling along black. */
 reset(only_yellow,1);acquiring_first_intersection=1;
 assert(follow_black_street()==1&&motor_calls==0&&event_index==1&&brake_calls==1);
 reset(lost_before_departure,1);acquiring_first_intersection=1;recovery_colour=4;
 assert(follow_black_street()==1&&motor_calls==0&&brake_calls==1);
 reset(only_yellow,1);acquiring_first_intersection=1;initial_alignment_result=4;
 assert(follow_black_street()==1&&motor_calls==0&&event_index==0);
 reset(normal,3);acquiring_first_intersection=1;initial_alignment_result=0;
 assert(follow_black_street()==0&&motor_calls==0&&event_index==0);
 reset(only_yellow,1);acquiring_first_intersection=1;stationary_colour=0;
 assert(follow_black_street()==0&&motor_calls==0&&brake_calls==1);
 reset(only_yellow,1);acquiring_first_intersection=1;brake_failure=1;
 assert(follow_black_street()==0&&motor_calls==0&&brake_calls==1);
 reset(red,1);acquiring_first_intersection=1;
 assert(follow_black_street()==2&&motor_calls==0&&brake_calls==1);
 reset(normal,3);brake_failure=1;
 assert(follow_black_street()==0&&brake_calls==1); /* Arrival requires successful braking. */
 puts("19/19 initial-yellow, later-departure, arrival, red-brake and failure cases passed.");
 return 0;
}
