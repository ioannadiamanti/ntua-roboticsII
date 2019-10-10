#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <math.h>

#include <sstream>


/************************************************************** ΥΛΟΠΟΙΗΣΗ ΜΕ PD ΕΛΕΓΚΤΗ *******************************************************************************************************/


float sonarF_val, sonarFL_val, sonarFR_val, sonarL_val, sonarR_val;

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

int main(int argc, char **argv)
{
double a, error, prev_error, t, tf, st, v_d, a_d;
  ros::init(argc, argv, "follower");
  ros::NodeHandle n;
  ros::Subscriber sonarFront_sub = n.subscribe("sonarFront_scan", 1, &sonarFrontCallback);
  ros::Subscriber sonarFrontLeft_sub = n.subscribe("sonarFrontLeft_scan", 1, &sonarFrontLeftCallback);
  ros::Subscriber sonarFrontRight_sub = n.subscribe("sonarFrontRight_scan", 1, &sonarFrontRightCallback);
  ros::Subscriber sonarLeft_sub = n.subscribe("sonarLeft_scan", 1, &sonarLeftCallback);
  ros::Subscriber sonarRight_sub = n.subscribe("sonarRight_scan", 1, &sonarRightCallback);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Rate loop_rate(10);

error = 0;                         /* Σφάλμα θέσης το οποίο θα μπει στον PD ελεγκτή */
t = 0;                             /* Αρχικοποίηση του χρόνου */
st = 0 ;                           /* Η κατάσταση στην οποία βρίσκεται το ρομπότ ως προς τα εμπόδια */

  while (ros::ok()){
    geometry_msgs::Twist velocity;
    if (t <= 194){                     /* Συνθήκη για μηδενισμό ταχυτήτων και σταμάτημα του ρομπότ */
	if (t < 18){                      /* Συνθήκη για να έρθει το ρομπότ κοντά στον τοίχο και να ξεκινήσει το wall following */ 
		if (st != 1){                     /* 1ο state ξεκινάει το ρομπότ μέχρι να έρθει κοντά στον τοίχο */ 
			tf = 0.1;                        /* Μια αρχικοποίση του χρόνου (περίοδος δειγματοληψίας 0.1s) */
			v_d = 0;                         /* Αρχικοποίηση της ταχύτητας */
			a_d = 0;                         /* Αρχικοποίηση της γωνιακής ταχύτητας */
		}
		st = 1;
		if (tf <= 0.5){                  /* Φάση σταθερής επιτάχυνσης */ 
			v_d = 2.4 * tf * tf - 3.2 * tf * tf *tf;    
			a_d = 2.4 * tf * tf - 3.2 * tf * tf *tf;
			tf = tf + 0.1; 
		}
		else{                           /* Φάση σταθερής ταχύτητας */
			v_d = 0.2;
			a_d = 0.2;
		}
		velocity.linear.x = v_d;
		velocity.angular.z = a_d; 
	}
	else{
		prev_error = error;           /* Υπολογισμός του προηγούμενου σφάλματος του ελεγκτή (για τον D όρο) */
		a = atan ((sonarFL_val * 0.70710678 - sonarL_val) / (sonarFL_val * 0.70710678));    /* Υπολογισμός της γωνίας α */
		error = - cos(a) * sonarL_val + 0.3;       /* Υπολογισμός του σφάλματος */
		if (sonarF_val <= 0.29 || sonarFR_val <=0.29) { /* Συνθήκη όπου έχουμε έρθει πολύ κοντά στον τοίχο (το 0.29 ορίζουμε 									απόσταση ασφαλείας) και αν συνεχίσουμε θα συγκρουστούμε οπότε πρέπει να 									στρίψει (2ο state) */
                                			/* Χρησιμεύει όταν ο τοίχος μπροστά σχηματίζει γωνία μικρότερη των 90 σε σχέση με 							     τον τοίχο που κινείται το ρομπότ οπότε δεν φτάνουν οι μετρήσεις των FL και L sonars */
			if (st !=2){                                /* Αρχικοποιήσεις */
				tf = 0.1;
				v_d = 0.2;
				a_d = 0;
			}
			st = 2;
			if (tf <= 0.5){                           /* Ομαλή επιβράδυνση της γραμμικής και επιτάχυνση της στροφικής */
				v_d = 0.2 -2.4 * tf * tf + 3.2 * tf * tf *tf;
				a_d = 2.4 * tf * tf - 3.2 * tf * tf *tf;
				tf = tf + 0.1; 
			}
			else{                                    /* Φάση σταθερής ταχύτητας */
				v_d = 0;
				a_d = 0.2;
			}
			velocity.linear.x = v_d;
			velocity.angular.z = a_d;
		}
		else{                                   /* Αλλιώς μπαίνω κανονικά στον ελεγκτή μου */
			if (st != 1){                   /* Κάνω αρχικοποιήσεις και ανάλογα από την αρχική τιμή της ταχύτητας που θα πάρω, 								επιταχύνω αν πήρα αρχική 0 και κρατάω σταθερή αν πήρα αρχική 0.2 */ 
				tf = 0.1;
				v_d = 0;
			}
			st = 1;
			if (tf <= 0.5){                         /* Φάση επτάχυνσης */
				v_d = 2.4 * tf * tf - 3.2 * tf * tf *tf;
				tf = tf + 0.1; 
			}
			else v_d = 0.2;                         /* Φάση σταθερής ταχύτητας */ 
			velocity.angular.z = 12 * (error) + 11.3 * (error - prev_error);           /**** Υλοποίση του PD ελεγκτη που κρατάει 													σταθερή κάθετη απόσταση (από αριστερά) του 													ρομπότ με τον τοίχο 0.3 ****/
			velocity.linear.x = v_d;
		}
     	}
  }
  else {          /* Φάση 3: σταματάει το ρομπότ. Επιβραδύνω ομαλά τη γραμμική μέχρι να σταματήσει (δεν χρειάζεται να κάνω το ίδιο με τη 			γωνιακή γιατί είμαι στη μόνιμη κατάσταση οπότε πρακτικά είναι μηδέν */
	if (st != 3){                      /* Αρχικοποιήσεις */
		tf = 0.1;
		v_d = 0.2;
	}
	st = 3;
	if (tf <= 0.5){                    /* Φάση επιβράδυνσης */ 
		v_d = 0.2 -2.4 * tf * tf + 3.2 * tf * tf *tf;
		tf = tf + 0.1; 
	} 
	else v_d = 0;                      /* Μηδενισμός και μένω εκεί */
	velocity.linear.x = v_d;
	velocity.angular.z = 0;
  }
t = t + 0.1;               /* Αυξάνω το χρόνο (0.1s περίοδος δειγματοληψίας) */
velocity_pub.publish(velocity);
loop_rate.sleep();
ros::spinOnce();
}
  return 0;
}
