#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "math.h"

#define PI 3.14
// Variable related to control
#define Gain_z     -0.2       // gain for z position
#define Gain_x     0.2       // gain for x position
#define Gain_p     0.2       // gain for heaidng
#define Dsrd_z     0.5       // desired z position
#define Dsrd_x     0.0       // desired x position
#define Dsrd_p     0.0       // desired heading


bool m_bNew_msg;
int m_nIter = 0 ;
float m_fMarker_Pos[3] ;
float m_fMarker_quadAtt[4] ;
float m_fMarker_eulerAtt[3] ;

geometry_msgs::Twist cmd_vel ;

/* ========================================================== */
/* =================== chatterCallback ====================== */
/* ========================================================== */
void chatterCallback( const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
   // Position
   if (msg->markers.size() > 0){

   // iteration flag
   m_nIter = m_nIter + 1 ; 

   // flag for listen
   m_bNew_msg = true;

   m_fMarker_Pos[0] = msg->markers[0].pose.pose.position.x ;
   m_fMarker_Pos[1] = msg->markers[0].pose.pose.position.y ;
   m_fMarker_Pos[2] = msg->markers[0].pose.pose.position.z ;

   // Quaternion Attitude
   m_fMarker_quadAtt[0] = msg->markers[0].pose.pose.orientation.x ;
   m_fMarker_quadAtt[1] = msg->markers[0].pose.pose.orientation.y ;
   m_fMarker_quadAtt[2] = msg->markers[0].pose.pose.orientation.z ;
   m_fMarker_quadAtt[3] = msg->markers[0].pose.pose.orientation.w ;
   }
}

/* ========================================================== */
/* =================== gen_eulerAng ========================= */
/* ========================================================== */
void gen_eulerAng()
{
	tf::Quaternion quat(m_fMarker_quadAtt[0], m_fMarker_quadAtt[1], m_fMarker_quadAtt[2], m_fMarker_quadAtt[3]) ;
        tf::Matrix3x3 m(quat) ;

        double roll, pitch, yaw ;    // return radian value
        m.getRPY(roll, pitch, yaw) ;  
        m_fMarker_eulerAtt[0] = roll ;
	m_fMarker_eulerAtt[1] = pitch ;
	m_fMarker_eulerAtt[2] = yaw ;

        ROS_INFO("Pitch: [%.13f]", m_fMarker_eulerAtt[1] ) ;
}

/* ========================================================== */
/* ===================== gen_cmdVel ========================= */
/* ========================================================== */
void gen_cmdVel()
{
        double temp_err_z ;
	double temp_err_x ;
	double temp_err_p ;

        temp_err_z = Dsrd_z - m_fMarker_Pos[2] ;
	temp_err_x = Dsrd_x - m_fMarker_Pos[0] ;
	temp_err_p = Dsrd_p*PI/180 - m_fMarker_eulerAtt[1] ;

   	cmd_vel.linear.x = Gain_z*temp_err_z ;
	cmd_vel.linear.y = 0.0 ;
	cmd_vel.linear.z = 0.0 ;
        
        cmd_vel.angular.x = 0.0 ;
        cmd_vel.angular.y = 0.0 ;
        cmd_vel.angular.z = Gain_x*temp_err_x + Gain_p*temp_err_p ;
}

/* ========================================================== */
/* ===================== disable_cmdVel ===================== */
/* ========================================================== */
void disable_cmdVel()
{
    	cmd_vel.linear.x = 0.0 ;
	cmd_vel.linear.y = 0.0 ;
	cmd_vel.linear.z = 0.0 ;
        
        cmd_vel.angular.x = 0.0 ;
        cmd_vel.angular.y = 0.0 ;
        cmd_vel.angular.z = 0.0 ;
}

/* ========================================================== */
/* ===================== main =============================== */
/* ========================================================== */
int main(int argc, char **argv)
{

   ros::init( argc, argv, "irobot_pose_con") ;
   ros::NodeHandle n ;

   ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1000, chatterCallback) ;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

   while(true) {
      ros::spinOnce();       //Ready to listen the publish message
      
       if (m_bNew_msg){
  		gen_eulerAng() ;
                gen_cmdVel() ;
        
                pub.publish(cmd_vel) ;
		m_bNew_msg = false;
		ros::Duration(0.5).sleep(); // sleep for half a second
       }
       else {
                disable_cmdVel() ;        
                pub.publish(cmd_vel) ;
	        ros::Duration(0.5).sleep(); // sleep for half a second
       }      
    }

   return 0 ;
}
