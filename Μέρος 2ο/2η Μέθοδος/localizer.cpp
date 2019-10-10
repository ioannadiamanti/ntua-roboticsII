#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

#include <math.h>

#include <sstream>



/******************************************************************************** ΥΛΟΠΟΙΗΣΗ ΓΝΩΡΙΖΟΝΤΑΣ ΤΗΝ ΕΙΣΟΔΟ *****************************************************************************************/




// Global variables::
// Sonars:
float sonarF_val, sonarFL_val, sonarFR_val, sonarL_val, sonarR_val;
// IMU:
double imuRoll, imuPitch, imuYaw; // orientation
double imuAngVelX, imuAngVelY, imuAngVelZ; // angular velocity
double imuLinAccX, imuLinAccY, imuLinAccZ; // linear acceleration
double cmd_vel_x, cmd_vel_y, cmd_vel_z;

void sonarFrontCallback(const sensor_msgs::Range& msg){

   sonarF_val = msg.range;
   ROS_INFO_STREAM("Front Sonar's indication: "<<sonarF_val);
}

void sonarFrontLeftCallback(const sensor_msgs::Range& msg){

   sonarFL_val = msg.range;
   ROS_INFO_STREAM("Front-Left Sonar's indication: "<<sonarFL_val);
}

void sonarFrontRightCallback(const sensor_msgs::Range& msg){

   sonarFR_val = msg.range;
   ROS_INFO_STREAM("Front-Right Sonar's indication: "<<sonarFR_val);
}

void sonarLeftCallback(const sensor_msgs::Range& msg){

   sonarL_val = msg.range;
   ROS_INFO_STREAM("Left Sonar's indication: "<<sonarL_val);
}

void sonarRightCallback(const sensor_msgs::Range& msg){

   sonarR_val = msg.range;
   ROS_INFO_STREAM("Right Sonar's indication: "<<sonarR_val);
}

void imuCallback(const sensor_msgs::Imu& msg){

  // orientation:: quaternion to RPY (roll, pitch, yaw)
  // yaw belongs to (-pi,pi]
  tf::Quaternion q(
  msg.orientation.x,
  msg.orientation.y,
  msg.orientation.z,
  msg.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(imuRoll, imuPitch, imuYaw);

  // angular velocity
  imuAngVelX = msg.angular_velocity.x;
  imuAngVelY = msg.angular_velocity.y;
  imuAngVelZ = msg.angular_velocity.z;

  // linear acceleration
  imuLinAccX = msg.linear_acceleration.x;
  imuLinAccY = msg.linear_acceleration.y;
  imuLinAccZ = msg.linear_acceleration.z;
}

void cmdCallback(const geometry_msgs::Twist& msg){

   cmd_vel_x = msg.linear.x;
   cmd_vel_y = msg.linear.y;
   cmd_vel_z = msg.linear.z;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "localizer");
  ros::NodeHandle n;
  ros::Subscriber sonarFront_sub = n.subscribe("sonarFront_scan", 1, &sonarFrontCallback);
  ros::Subscriber sonarFrontLeft_sub = n.subscribe("sonarFrontLeft_scan", 1, &sonarFrontLeftCallback);
  ros::Subscriber sonarFrontRight_sub = n.subscribe("sonarFrontRight_scan", 1, &sonarFrontRightCallback);
  ros::Subscriber sonarLeft_sub = n.subscribe("sonarLeft_scan", 1, &sonarLeftCallback);
  ros::Subscriber sonarRight_sub = n.subscribe("sonarRight_scan", 1, &sonarRightCallback);
  ros::Subscriber imu_sub = n.subscribe("imu_data", 1, &imuCallback);
  ros::Subscriber velocity_sub = n.subscribe("cmd_vel", 1, &cmdCallback);
  ros::Publisher ekf_pub = n.advertise<nav_msgs::Odometry>("ekf_estimation", 1);
  ros::Rate loop_rate(10);

  nav_msgs::Odometry ekf_estimation_msg;
  ekf_estimation_msg.header.frame_id = "odom";
  ekf_estimation_msg.child_frame_id = "chassis";

  double estimRoll = 0.0; // always zero value in 2D navigation
  double estimPitch = 0.0; // always zero value in 2D navigation
  double estimYaw;
  double t, x_pred, y_pred, yaw_pred, x_vel_pred, x_est, y_est, yaw_est, x_vel_est, Dt, Cw [3] [3], sw, sa, dx, dy, f1, f2, f3, f4, pi, s_r, s_fr, s_f, s_fl, s_l, m_x = 0, m_y = 0, H [3] [3], H_T [3] [3], h [3] [1], z [3] [1], theta1, theta2, A [3] [3], Cv [3] [3], ss, syaw, P_k [3] [3], P_kk [3] [3], A_T [3] [3], L1 [3] [3], L2 [3] [3], L3 [3] [3], L4 [3] [3], L5 [3] [3], c [3] [3], Kalman_gain [3] [3], r [3] [1], I [3] [3], M [3] [3], det, p1, p2, p3, p4, error_corr, Cov1 [6] [6], y_vel_est, y_vel_pred;

  t = 0;              /** Χρόνος **/
  x_est = 0;          /** Βέλτιστη εκτίμηση στον χ **/
  y_est = 0;          /** Βέλτιστη εκτίμηση στον y **/
  yaw_est = 0;        /** Βέλτιστη εκτίμηση γωνίας **/
  x_vel_est = 0;      /** Εκτίμηση ταχύτητας **/
  Dt = 0.1;           /** Περίοδος δειγματοληψίας **/
  sw = 2 * 0.001;     /** Σφάλμα της γωνιακής ταχύτητας **/
  sa = 2 * 0.001;     /** Σφάλμα γραμμικής επιτάχυνσης **/
  pi = 3.14159265359; /** Η σταθερά π **/
  ss = 0.01;          /** Σφάλμα από τα sonars **/
  syaw = 0.002;       /** Σφάλμα για τη μέτρηση της γωνίας **/
  error_corr = 0;     /** Μία σταθερά που χρησιμοποιούμε για να κάνουμε μια μικρή διόρθωση για την βέλτιστη εκτίμηση στον y **/
  y_vel_est = 0;      /** Εκτίμηση ταχύτητας στον y **/

  /** Αρχικοποίηση της μήτρας συνδιακύμανσης του state **/
  for (int i = 0; i < 3; i++) {
   for (int j = 0; j < 3; j++) {
    P_k [i] [j] = 0;
    }
   }

  /** Δημιουργούμε μοναδιαίο πίνακα **/
  for (int i = 0; i < 3; i++) {
   for (int j = 0; j < 3; j++) {
    if (i == j) {
     I [i] [j] = 1;
     }
    else { 
     I [i] [j] = 0;
     }
    }
   }


  while (ros::ok())
  {

    ekf_estimation_msg.header.seq++;
    ekf_estimation_msg.header.stamp = ros::Time::now();

  /** Φιλτράρισμα της μέτρησης της γραμμικής επιτάχυνσης από το IMU **/
  if (imuLinAccX < 0.1 && imuLinAccX > -0.125) {
   imuLinAccX = 0;
  } 
  else if (imuLinAccX < -3) {
   imuLinAccX = -3;
  }
  else if (imuLinAccX > 3) {
   imuLinAccX = 3;
  }   

 /** Αρχικοποίση πινάκων για τις πράξεις **/
  for (int i = 0; i < 3; i++) {
   for (int j = 0; j < 3; j++) {
    L1 [i] [j] = 0;
    L5 [i] [j] = 0;
    P_kk [i] [j] = 0;
    M [i] [j] = 0;
    L2 [i] [j] = 0;
    Kalman_gain [i] [j] = 0;
    L3 [i] [j] = 0;
    c [i] [j] = 0;
    L4 [i] [j] =0;
    }
   }

 /** Δημιουργούμε τη μήτρα διακύμανσης 6 επί 6 της πόζας για να την βάλουμε στην έξοδο του kalman **/
  Cov1 [0] [0] = P_k [0] [0];
  Cov1 [0] [1] = P_k [0] [1];
  Cov1 [0] [2] = 0;
  Cov1 [0] [3] = 0;
  Cov1 [0] [4] = 0;
  Cov1 [0] [5] = P_k [0] [2];
  Cov1 [1] [0] = P_k [1] [0];
  Cov1 [1] [1] = P_k [1] [1];
  Cov1 [1] [2] = 0;
  Cov1 [1] [3] = 0;
  Cov1 [1] [4] = 0;
  Cov1 [1] [5] = P_k [1] [2];
  Cov1 [2] [0] = 0;
  Cov1 [2] [1] = 0;
  Cov1 [2] [2] = 0;
  Cov1 [2] [3] = 0;
  Cov1 [2] [4] = 0;
  Cov1 [2] [5] = 0;
  Cov1 [3] [0] = 0;
  Cov1 [3] [1] = 0;
  Cov1 [3] [2] = 0;
  Cov1 [3] [3] = 0;
  Cov1 [3] [4] = 0;
  Cov1 [3] [5] = 0;
  Cov1 [4] [0] = 0;
  Cov1 [4] [1] = 0;
  Cov1 [4] [2] = 0;
  Cov1 [4] [3] = 0;
  Cov1 [4] [4] = 0;
  Cov1 [4] [5] = 0;
  Cov1 [5] [0] = P_k [2] [0];
  Cov1 [5] [1] = P_k [2] [1];
  Cov1 [5] [2] = 0;
  Cov1 [5] [3] = 0;
  Cov1 [5] [4] = 0;
  Cov1 [5] [5] = P_k [2] [2];


  /** Μετασχηματίζουμε τη μέτρηση της γωνίας του IMU να έχει ως αναφορά το 0 και όχι τον αρχικό προσανατολισμό της ομάδας μας **/ 
  if (imuYaw + 2.57522205 > pi) {
   imuYaw = - 2 * pi + imuYaw + 2.57522205;
  }
  else if (imuYaw + 2.57522205 <= -pi) {
   imuYaw = 2 * pi + imuYaw + 2.57522205; 
  }
  else {
   imuYaw = imuYaw + 2.57522205;
  }

    if (t != 0) {
     dx = cos(yaw_pred) * 0.1;  /** Πόσο μετατοπισμένο είναι στον χ το σημείο αναφοράς από το οποίο παίρνουμε τις μετρήσεις των sonars σε σχέση με το κέντρο μάζας **/
     dy = sin(yaw_pred) * 0.1;  /** Πόσο μετατοπισμένο είναι στον y το σημείο αναφοράς από το οποίο παίρνουμε τις μετρήσεις των sonars σε σχέση με το κέντρο μάζας **/

      p1 = yaw_pred + pi / 2;   /** Η γωνία του αριστερού sonar **/
      p2 = yaw_pred + pi / 4;   /** Η γωνία του μπροστά αριστερού sonar **/
      p3 = yaw_pred - pi / 4;   /** Η γωνία του μπροστά δεξιού sonar **/
      p4 = yaw_pred - pi / 2;   /** Η γωνία του δεξιού sonar **/


    /** Μετασχηματίζουμε τις γωνίες των sonars να είναι από το (-π,π] **/
    if (p1 > pi) {
     p1 = -2 * pi + p1;
     }
    if (p1 <= -pi) {
     p1 = 2 * pi + p1;
     } 

    if (p2 > pi) {
     p2 = -2 * pi + p2;
     }
    if (p2 <= -pi) {
     p2 = 2 * pi + p2;
     } 

    if (p3 > pi) {
     p3 = -2 * pi + p3;
     }
    if (p3 <= -pi) {
     p3 = 2 * pi + p3;
     } 

    if (p4 > pi) {
     p4 = -2 * pi + p4;
     }
    if (p4 <= -pi) {
     p4 = 2 * pi + p4;
     } 

    /** Υπολογίζουμε τις γωνίες του ρομπότ σε σχέση με τις 4 γωνίες του τετραγώνου **/ 
    f1 = atan((2 - dy - y_pred) / (2 - dx - x_pred));
    f2 = pi/2 + atan((2 + dx + x_pred) / (2 - dy - y_pred));
    f4 = - atan((2 + dy + y_pred) / (2 - dx - x_pred));
    f3 = - pi/2 - atan((2 + dx + x_pred) / (2 + dy + y_pred));

    /** Υπολογίζουμε προς ποιον τοίχο είναι προσανατολισμένο κάθε sonar. Το 0 δείχνει τον μπροστά τοίχο, το 1 τον αριστερά, το 2 τον πίσω, το 3 τον δεξιά **/
    if (yaw_pred <= f1 && yaw_pred >= f4) {
      s_f = 0;
    }
    else if (yaw_pred >= f1 && yaw_pred <= f2) {
      s_f = 1;
    }
    else if (yaw_pred >= f2 || yaw_pred <= f3) {
      s_f = 2;
    }
    else if (yaw_pred <= f4 && yaw_pred >= f3) {
      s_f = 3;
    }

    if (p1 <= f1 && p1 >= f4) {
      s_l = 0;
    }
    else if (p1 >= f1 && p1 <= f2) {
      s_l = 1;
    }
    else if (p1 >= f2 || p1 <= f3) {
      s_l = 2;
    }
    else if (p1 <= f4 && p1 >= f3) {
      s_l = 3;
    }

    if (p2 <= f1 && p2 >= f4) {
      s_fl = 0;
    }
    else if (p2 >= f1 && p2 <= f2) {
      s_fl = 1;
    }
    else if (p2 >= f2 || p2 <= f3) {
      s_fl = 2;
    }
    else if (p2 <= f4 && p2 >= f3) {
      s_fl = 3;
    }

    if (p3 <= f1 && p3 >= f4) {
      s_fr = 0;
    }
    else if (p3 >= f1 && p3 <= f2) {
      s_fr = 1;
    }
    else if (p3 >= f2 || p3 <= f3) {
      s_fr = 2;
    }
    else if (p3 <= f4 && p3 >= f3) {
      s_fr = 3;
    }

    if (p4 <= f1 && p4 >= f4) {
      s_r = 0;
    }
    else if (p4 >= f1 && p4 <= f2) {
      s_r = 1;
    }
    else if (p4 >= f2 || p4 <= f3) {
      s_r = 2;
    }
    else if (p4 <= f4 && p4 >= f3) {
      s_r = 3;
    }

     /** Αν το κάθε sonar είναι προσανατολισμένο προς τον τοίχο και τον βλέπει όντως (βλέπουμε στο sonar τιμή μικρότερη του 2) τότε φτιάχνουμε το μοντέλο μέτρησης **/ 
     if (s_f == 0 && sonarF_val < 2) {
      z [0] [0] = sonarF_val + 0.15;           /** Μέτρηση του sonar (μόνο για το front ως προς το κέντρο μάζας) **/ 
      h [0] [0] = (2 - x_pred) / cos(yaw_pred);  /** Τι δείχνει η πρόβλεψη **/
      /** Παίρνουμε το gradient (γραμμικοποίηση) **/  
      H [0] [0] = - 1 / cos(yaw_pred);
      H [0] [1] = 0;
      H [0] [2] = (2 - x_pred) * (sin(yaw_pred) / (cos(yaw_pred) * cos(yaw_pred)));
      }
     else if (s_fr == 0 && sonarFR_val < 2) {
      z [0] [0] = sonarFR_val + 0.07071; 
      h [0] [0] = (2 - dx - x_pred) / cos(yaw_pred - (pi / 4)); 
      H [0] [0] = - 1 / cos(yaw_pred - (pi / 4));
      H [0] [1] = 0;
      H [0] [2] = 0.1 * sin(pi/4) / (cos(yaw_pred - (pi/4)) * cos(yaw_pred - (pi/4))) + (2 - x_pred) * sin(yaw_pred - (pi/4)) / (cos(yaw_pred - (pi/4)) * cos(yaw_pred - (pi/4)));
      }
     else if (s_r == 0 && sonarR_val < 2) {
      z [0] [0] = sonarR_val + 0.05; 
      h [0] [0] = (2 - dx - x_pred) / cos(yaw_pred - (pi / 2)); 
      H [0] [0] = - 1 / cos(yaw_pred - (pi / 2));
      H [0] [1] = 0;
      H [0] [2] = 0.1 * sin(pi/2) / (cos(yaw_pred - (pi/2)) * cos(yaw_pred - (pi/2))) + (2 - x_pred) * sin(yaw_pred- (pi/2)) / (cos(yaw_pred - (pi/2)) * cos(yaw_pred - (pi/2)));
     }
     else if (s_fl == 0 && sonarFL_val < 2) {
      z [0] [0] = sonarFL_val + 0.07071; 
      h [0] [0] = (2 - dx - x_pred) / cos(yaw_pred + (pi / 4)); 
      H [0] [0] = - 1 / cos((pi/4) + yaw_pred);
      H [0] [1] = 0;
      H [0] [2] = 0.1 * sin(-pi/4) / (cos(yaw_pred + (pi/4)) * cos(yaw_pred + (pi/4))) + (2 - x_pred) * sin(yaw_pred + (pi/4)) / (cos(yaw_pred + (pi/4)) * cos(yaw_pred + (pi/4)));
      }
     else if (s_l == 0 && sonarL_val < 2) {
      z [0] [0] = sonarL_val + 0.05; 
      h [0] [0] = (2 - dx - x_pred) / cos(yaw_pred + (pi / 2)); 
      H [0] [0] = - 1 / cos((pi/2) + yaw_pred);
      H [0] [1] = 0;
      H [0] [2] = 0.1 * sin(-pi/2) / (cos(yaw_pred + (pi/2)) * cos(yaw_pred + (pi/2))) + (2 - x_pred) * sin(yaw_pred + (pi/2)) / (cos(yaw_pred + (pi/2)) * cos(yaw_pred + (pi/2)));
      }
     else if (s_f == 2 && sonarF_val < 2) {
      z [0] [0] = sonarF_val + 0.15; 
      h [0] [0] = (2 + x_pred) / cos(yaw_pred + pi); 
      H [0] [0] =  1 / cos(yaw_pred + pi);
      H [0] [1] = 0;
      H [0] [2] = (2 + x_pred) * (sin(yaw_pred + pi) / (cos(yaw_pred + pi) * cos(yaw_pred + pi)));
      }
     else if (s_fr == 2 && sonarFR_val < 2) {
      z [0] [0] = sonarFR_val + 0.07071; 
      h [0] [0] = (2 + dx + x_pred) / cos(yaw_pred + 3 * pi/4); 
      H [0] [0] =  1 / cos(yaw_pred + 3 * pi/4);
      H [0] [1] = 0;
      H [0] [2] = - 0.1 * sin(- 3 * (pi/4)) / (cos(yaw_pred + 3 * (pi/4)) * cos(yaw_pred + 3 * (pi/4))) + (2 + x_pred) * sin(yaw_pred + 3 * (pi/4)) / (cos(yaw_pred + 3 * (pi/4)) * cos(yaw_pred + 3 * (pi/4)));
      }
     else if (s_r == 2 && sonarR_val < 2) {
      z [0] [0] = sonarR_val + 0.05; 
      h [0] [0] = (2 + dx + x_pred) / cos(yaw_pred + (pi/2)); 
      H [0] [0] =  1 / cos(yaw_pred + (pi/2));
      H [0] [1] = 0;
      H [0] [2] = - 0.1 * sin(-pi/2) / (cos(yaw_pred + (pi/2)) * cos(yaw_pred + (pi/2))) + (2 + x_pred) * sin(yaw_pred + (pi/2)) / (cos(yaw_pred + (pi/2)) * cos(yaw_pred + (pi/2)));
      }
     else if (s_fl == 2 && sonarFL_val < 2) {
      z [0] [0] = sonarFL_val + 0.07071; 
      h [0] [0] = (2 + dx + x_pred) / cos((pi/4) + yaw_pred + pi); 
      H [0] [0] =  1 / cos((pi/4) + yaw_pred + pi);
      H [0] [1] = 0;
      H [0] [2] =  - 0.1 * sin(- 5 * pi/4) / (cos(yaw_pred + pi + (pi/4)) * cos(yaw_pred + pi + (pi/4))) + (2 + x_pred) * sin(yaw_pred + pi + (pi/4)) / (cos(yaw_pred + pi + (pi/4)) * cos(yaw_pred + pi + (pi/4)));
      }
     else if (s_l == 2 && sonarL_val < 2) {
      z [0] [0] = sonarL_val + 0.05; 
      h [0] [0] = (2 + dx + x_pred) / cos((pi/2) +  yaw_pred + pi); 
      H [0] [0] =  1 / cos((pi/2) +  yaw_pred + pi);
      H [0] [1] = 0;
      H [0] [2] = - 0.1 * sin(- 3 * pi/2) / (cos(yaw_pred + pi + (pi/2)) * cos(yaw_pred + pi + (pi/2))) + (2 + x_pred) * sin(yaw_pred + pi + (pi/2)) / (cos(yaw_pred + pi + (pi/2)) * cos(yaw_pred + pi + (pi/2)));
      }
     else {
      z [0] [0] = 0;
      h [0] [0] = 0;
      H [0] [0] = 0;
      H [0] [1] = 0;
      H [0] [2] = 0;
      }

    if (s_f == 1 && sonarF_val < 2) {
      error_corr = 1;
      z [1] [0] = sonarF_val + 0.15; 
      h [1] [0] = (2 - y_pred) / cos(yaw_pred - pi/2); 
      H [1] [0] = 0;
      H [1] [1] = - 1 / cos(yaw_pred - pi/2);
      H [1] [2] = (2 - y_pred) * (sin(yaw_pred - pi/2) / (cos(yaw_pred - pi/2) * cos(yaw_pred - pi/2)));
      }
     else if (s_fl == 1 && sonarFL_val < 2) {
      error_corr = 1;
      z [1] [0] = sonarFL_val + 0.07071; 
      h [1] [0] = (2 - dy - y_pred) / cos(yaw_pred - (pi/4)); 
      H [1] [0] = 0;
      H [1] [1] = - 1 / cos(yaw_pred - (pi/4));
      H [1] [2] = - 0.1 * cos (pi/4) / (cos(yaw_pred - (pi/4)) * cos(yaw_pred - (pi/4))) + (2 - y_pred) * sin(yaw_pred - (pi/4)) / (cos(yaw_pred - (pi/4)) * cos(yaw_pred - (pi/4)));
      }
     else  if (s_l == 1 && sonarL_val < 2) {
      error_corr = 1;
      z [1] [0] = sonarL_val + 0.05; 
      h [1] [0] = (2 - dy - y_pred) / cos(yaw_pred);
      H [1] [0] = 0;
      H [1] [1] = - 1 / cos(yaw_pred);
      H [1] [2] = - 0.1 / (cos(yaw_pred) * cos(yaw_pred)) + (2 - y_pred) * sin(yaw_pred) / (cos(yaw_pred) * cos(yaw_pred));;
      }
     else if (s_fr == 1 && sonarFR_val < 2) {
      error_corr = 1;
      z [1] [0] = sonarFR_val + 0.07071; 
      h [1] [0] = (2 - dy - y_pred) / cos(yaw_pred - 3 * (pi/4)); 
      H [1] [0] = 0;
      H [1] [1] = - 1 / cos(yaw_pred - 3 * (pi/4));
      H [1] [2] = - 0.1 * cos (3 * pi/4) / (cos(yaw_pred - 3 * (pi/4)) * cos(yaw_pred - 3 * (pi/4))) + (2 - y_pred) * sin(yaw_pred - 3 * (pi/4)) / (cos(yaw_pred - 3 * (pi/4)) * cos(yaw_pred - 3 * (pi/4)));
      }
     else if (s_r == 1 && sonarR_val < 2) {
      error_corr = 1;
      z [1] [0] = sonarR_val + 0.05; 
      h [1] [0] = (2 - dy - y_pred) / cos(yaw_pred - pi); 
      H [1] [0] = 0;
      H [1] [1] = - 1 / cos(yaw_pred - pi);
      H [1] [2] = - 0.1 * cos (pi) / (cos(yaw_pred - pi) * cos(yaw_pred - pi)) + (2 - y_pred) * sin(yaw_pred - pi) / (cos(yaw_pred - pi) * cos(yaw_pred - pi));
      }
     else if (s_f == 3 && sonarF_val < 2) {
      error_corr = -1;
      z [1] [0] = sonarF_val + 0.15; 
      h [1] [0] = (2 + y_pred) / cos(yaw_pred + pi/2); 
      H [1] [0] = 0;
      H [1] [1] = 1 / cos(yaw_pred + pi/2);
      H [1] [2] = (2 + y_pred) * sin(yaw_pred + pi/2) / (cos(yaw_pred + pi/2) * cos(yaw_pred + pi/2));
      }
     else if (s_fl == 3 && sonarFL_val < 2) {     
      error_corr = -1;
      z [1] [0] = sonarFL_val + 0.07071; 
      h [1] [0] = (2 + dy + y_pred) / cos(yaw_pred + 3 * (pi/4)); 
      H [1] [0] = 0;
      H [1] [1] = 1 / cos(yaw_pred + 3 * (pi/4));
      H [1] [2] = 0.1 * cos (-3 * pi/4) / (cos(yaw_pred + 3 * (pi/4)) * cos(yaw_pred + 3 * (pi/4))) + (2 + y_pred) * sin(yaw_pred + 3 * (pi/4)) / (cos(yaw_pred + 3 * (pi/4)) * cos(yaw_pred + 3 * (pi/4)));
      }
     else if (s_l == 3 && sonarL_val < 2) {
      error_corr = -1;
      z [1] [0] = sonarL_val + 0.05; 
      h [1] [0] = (2 + dy + y_pred) / cos(yaw_pred + pi); 
      H [1] [0] = 0;
      H [1] [1] = 1 / cos(yaw_pred + pi);
      H [1] [2] = 0.1 * cos(-pi) / (cos(yaw_pred + pi) * cos(yaw_pred + pi)) + (2 + y_pred) * sin(yaw_pred + pi) / (cos(yaw_pred + pi) * cos(yaw_pred + pi)); ;
      }
     else if (s_fr == 3 && sonarFR_val < 2) {
      error_corr = -1;
      z [1] [0] = sonarFR_val + 0.07071; 
      h [1] [0] = (2 + dy + y_pred) / cos(yaw_pred + (pi/4)); 
      H [1] [0] = 0;
      H [1] [1] = 1 / cos(yaw_pred + (pi/4));
      H [1] [2] = 0.1 * cos  (-pi/4) / (cos(yaw_pred + (pi/4)) * cos(yaw_pred + (pi/4))) + (2 + y_pred) * sin(yaw_pred + (pi/4)) / (cos(yaw_pred + (pi/4)) * cos(yaw_pred + (pi/4)));
      }
     else if (s_r == 3 && sonarR_val < 2) {
      error_corr = -1;
      z [1] [0] = sonarR_val + 0.05; 
      h [1] [0] = (2 + dy + y_pred) / cos(yaw_pred); 
      H [1] [0] = 0;
      H [1] [1] = 1 / cos(yaw_pred);
      H [1] [2] = 0.1 / (cos(yaw_pred) * cos(yaw_pred)) + (2 + y_pred) * sin(yaw_pred) / (cos(yaw_pred) * cos(yaw_pred));
      }
     else {
      error_corr = 0;
      z [1] [0] = 0;
      h [1] [0] = 0;
      H [1] [0] = 0;
      H [1] [1] = 0;
      H [1] [2] = 0;
      }

   /** Μοντέλο μέτρησης για την ανανέωση κατάστασης της γωνίας **/
     z [2] [0] = imuYaw;
     h [2] [0] = yaw_pred;
     H [2] [0] = 0;
     H [2] [1] = 0;
     H [2] [2] = 1;

  /** Ανάστροφος του H **/ 
  for (int i = 0; i < 3; i++) {
   for (int j = 0; j < 3; j++) {
    H_T [j] [i] = H [i] [j];
    }
   }
  /** Μήτρα διακύμανσης του θορύβου από τα sonars και το IMU (μοντέλο μέτρησης) **/
     Cv [0] [0] = ss * ss;
     Cv [0] [1] = 0;
     Cv [0] [2] = 0;
     Cv [1] [0] = 0;
     Cv [1] [1] = ss * ss;
     Cv [1] [2] = 0;
     Cv [2] [0] = 0;
     Cv [2] [1] = 0;
     Cv [2] [2] = syaw * syaw;

    /** Α * P(K) **/
    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; ++k)
      {
       L1 [i] [j] += A [i] [k] * P_k [k] [j];
      }

    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
      {
       L5 [i] [j] += L1 [i] [k] * A_T [k] [j];
      }
    
    /** Αβεβαιότητα της πρόβλεψης **/ 
    for (int i = 0; i < 3; i++)
     for (int j = 0; j < 3; j++)
     {
      P_kk [i] [j] = L5 [i] [j] + Cw [i] [j];
     }

    /** P(k+1/k) * H ανάστροφο **/
    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
      {
       L2 [i] [j] += P_kk [i] [k] * H_T [k] [j];
      }

    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
      {
       L3 [i] [j] += H [i] [k] * L2 [k] [j];
      }

    for (int i = 0; i < 3; i++)
     for (int j = 0; j < 3; j++)
     {
      L3 [i] [j] = L3 [i] [j] + Cv [i] [j];
     }



    L4 [0] [0] = L3 [1] [1] * L3 [2] [2] - L3 [1] [2] * L3 [2] [1];               
    L4 [0] [1] = - 1 * (L3 [1] [0] * L3 [2] [2] - L3 [1] [2] * L3 [2] [0]);        
    L4 [0] [2] = L3 [1] [0] * L3 [2] [1] - L3[1] [1] * L3 [2] [0];                
    L4 [1] [0] = -1 * (L3 [0] [1] * L3 [2] [2] - L3 [2] [1] * L3 [0] [2]);        
    L4 [1] [1] = L3 [0] [0] * L3 [2] [2] - L3 [0] [2] * L3 [2] [0];               
    L4 [1] [2] = -1 * (L3 [0] [0] * L3 [2] [1] - L3 [0] [1] * L3 [2] [0]);        
    L4 [2] [0] = L3 [0] [1] * L3 [1] [2] -L3 [0] [2] * L3 [1] [1];                
    L4 [2] [1] = -1 * (L3 [0] [0] * L3 [1] [2] - L3 [0] [2] * L3 [1] [0]);        
    L4 [2] [2] = L3 [0] [0] * L3 [1] [1] - L3 [0] [1] * L3 [1] [0];                

    det = L3 [0] [0] * L4 [0] [0] + L3 [0] [1] * L4 [0] [1] + L3 [0] [2] * L4 [0] [2];

    for(int i=0;i<3; i++) //For storing adjoint in another 2-D Array
    {
    for(int j=0;j<3; j++) {
    c[i][j]=L4[j][i];
    }
    }

    for(int i=0;i<3;i++) //For calculating inverse of matrix (2-D Array)
      {
     for(int j=0;j<3;j++)
      c [i] [j] = c [i] [j] / det;
     }
 
    /** Kalman gain **/
    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
      {
       Kalman_gain [i] [j] += L2 [i] [k] * c [k] [j];
      }

     /** Διαφορά της μέτρησης από το αναμενόμενο **/
     r [0] [0] = z [0] [0] - h [0] [0]; 
     r [1] [0] = z [1] [0] - h [1] [0];
     r [2] [0] = z [2] [0] - h [2] [0];

     /** Ανανέωση κατάστασης **/
     x_est = x_pred + Kalman_gain [0] [0] * r [0] [0] + Kalman_gain [0] [1] * r [1] [0] + Kalman_gain [0] [2] * r [2] [0];  
     y_est = y_pred + Kalman_gain [1] [0] * r [0] [0] + Kalman_gain [1] [1] * r [1] [0] + Kalman_gain [1] [2] * r [2] [0];  
     yaw_est = yaw_pred + Kalman_gain [2] [0] * r [0] [0] + Kalman_gain [2] [1] * r [1] [0] + Kalman_gain [2] [2] * r [2] [0]; 

     /** Διορθώσαμε λίγο με το μάτι την τιμή της εκτίμησης του y **/
     if (error_corr == 1) {
      y_est = y_est + 0.07;
     }
     else if (error_corr == -1) {
      y_est = y_est - 0.07;
     } 

    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
      {
       M [i] [j] +=  Kalman_gain [i] [k] * H [k] [j];
      }

    for (int i = 0; i < 3; i++)
     for (int j = 0; j < 3; j++)
     {
      M [i] [j] = I [i] [j] - M [i] [j];
     }

   for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
     P_k [i] [j] = 0;
     }
    }

    /** Νέα μήτρα διακύμανσης **/
    for (int i = 0; i < 3; ++i)
     for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
      {
       P_k [i] [j] +=  M [i] [k] * P_kk [k] [j];
      }    

     } 
     
    /** Μοντέλο προέβλεψης **/ 
    x_pred = x_est + cmd_vel_x * Dt * cos(yaw_est);
    y_pred = y_est + cmd_vel_x * Dt * sin(yaw_est);
    yaw_pred = yaw_est + imuAngVelZ * Dt;

    x_vel_pred = x_vel_est + imuLinAccX * Dt; /** Πρόβλεψη της ταχύτητας εκτός του μοντέλου πρόβλεψης καθώς ταυτόχρονα αποτελεί και βέλτιστη εκτίμηση **/
    y_vel_pred = y_vel_est + imuLinAccY * Dt;    

   /** Μήτρα διακύμανσης για το θόρυβο **/
    Cw [0] [0] = 1000 * 1000 * Dt * Dt; /** Επειδή η ταχύτητα που δίνουμε στους τροχούς δεν μεταφράζεται σωστά θεωρούμε εντελώς αβέβαιη την πρόβλεψη **/
    Cw [0] [1] = 0;
    Cw [0] [2] = 0;
    Cw [1] [0] = 0; 
    Cw [1] [1] = 1000 * 1000 * Dt * Dt; /** Επειδή η ταχύτητα που δίνουμε στους τροχούς δεν μεταφράζεται σωστά θεωρούμε εντελώς αβέβαιη την πρόβλεψη **/
    Cw [1] [2] = 0;
    Cw [2] [0] = 0;
    Cw [2] [1] = 0;
    Cw [2] [2] = sw * sw * Dt * Dt;

    /** Γραμμικοποίηση του μοντέλου πρόβλεψης **/ 
    A [0] [0] = 1;
    A [0] [1] = 0; 
    A [0] [2] = - cmd_vel_x * Dt * sin(yaw_est);
    A [1] [0] = 0; 
    A [1] [1] = 1;
    A [1] [2] = cmd_vel_x * Dt * cos(yaw_est);
    A [2] [0] = 0;
    A [2] [1] = 0;
    A [2] [2] = 1;


  /** Ανάστροφος του Α **/
  for (int i = 0; i < 3; i++) {
   for (int j = 0; j < 3; j++) {
    A_T [j] [i] = A [i] [j];
    }
   }
   
   /** Μετασχηματίζουμε την προλεψη της γωνίας στο διάστημα [-π,π] **/
   if (yaw_pred > pi) {
     yaw_pred = -2 * pi + yaw_pred;
     }

    if (yaw_pred <= -pi) {
     yaw_pred = 2 * pi + yaw_pred;
     } 


    t = t + Dt;      /** Περνάμε στο επόμενο βήμα της δειγματοληψίας **/

    estimYaw = yaw_est; // orientation to be estimated (-pi,pi]
    // position to be estimated
    ekf_estimation_msg.pose.pose.position.x = x_est;
    ekf_estimation_msg.pose.pose.position.y = y_est;
    ekf_estimation_msg.pose.pose.position.z = 1.0; // always zero value in 2D navigation /** Τόσο φαίνεται να είναι το ύψος από το κέντρο μάζας **/
    // RPY to quaternion
    ekf_estimation_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      estimRoll,
      estimPitch,
      estimYaw);  
    // velocities to be estimated
    ekf_estimation_msg.twist.twist.linear.x = cmd_vel_x; // x-linear velocity to be estimated
    ekf_estimation_msg.twist.twist.linear.y = cmd_vel_y; // y-linear velocity to be estimated
    ekf_estimation_msg.twist.twist.linear.z = cmd_vel_z; // always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.x = 0.0; // always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.y = 0.0; // always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.z = imuAngVelZ; // angular velocity to be estimated

    x_vel_est = x_vel_pred; /** βέλτιστη εκτίμηση της ταχύτητας η πρόβλεψη **/
    y_vel_est = y_vel_pred; 

     ekf_estimation_msg.pose.covariance = { {Cov1[0][0], Cov1[0][1],Cov1[0][2],Cov1[0][3],Cov1[0][4],Cov1[0][5],Cov1[1][0],Cov1[1][1],Cov1[1][2],Cov1[1][3],Cov1[1][4],Cov1[1][5],Cov1[2][0],Cov1[2][1],Cov1[2][2],Cov1[2][3],Cov1[2][4],Cov1[2][5],Cov1[3][0],Cov1[3][1],Cov1[3][2],Cov1[3][3],Cov1[3][4],Cov1[3][5],Cov1[4][0],Cov1[4][1],Cov1[4][2],Cov1[4][3],Cov1[4][4],Cov1[4][5],Cov1[5][0],Cov1[5][1],Cov1[5][2],Cov1[5][3],Cov1[5][4],Cov1[5][5] } };
  /**   ekf_estimation_msg.twist.covariance **/
     /**http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
    */

    // publish estimation


    

    ekf_pub.publish(ekf_estimation_msg);

    loop_rate.sleep();
    ros::spinOnce();
  }


  return 0;
}
