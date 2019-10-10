#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <math.h>

#include <sstream>

/************************************************ ΥΛΟΠΟΙΗΣΗ ΧΩΡΙΣ PD ΕΛΕΓΚΤΗ **************************************************************************************************************************/

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
double t, v_d, a_d, aa_d, vel_old, vel_new, a_real, v_f ,a_f, al_d, tf, a_k, aa_k, tf2, prev_extra_st, s, g, h, tf3, time, previous_time, time_now;
int st, extra_st;
  ros::init(argc, argv, "follower");
  ros::NodeHandle n;
  ros::Subscriber sonarFront_sub = n.subscribe("sonarFront_scan", 1, &sonarFrontCallback);
  ros::Subscriber sonarFrontLeft_sub = n.subscribe("sonarFrontLeft_scan", 1, &sonarFrontLeftCallback);
  ros::Subscriber sonarFrontRight_sub = n.subscribe("sonarFrontRight_scan", 1, &sonarFrontRightCallback);
  ros::Subscriber sonarLeft_sub = n.subscribe("sonarLeft_scan", 1, &sonarLeftCallback);
  ros::Subscriber sonarRight_sub = n.subscribe("sonarRight_scan", 1, &sonarRightCallback);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Rate loop_rate(10);

time = 0;
h = 0;                             /* Καθορίζει αν μπαίνω για πρώτη φορά στη φάση ομαλής επιβράδυνσης αρνητικής γωνιακής ταχύτητας */
g = 0;                             /* Δείχνει αν η προηγούμενη φάση που ήμουν ήταν η φάση 2 */
s = 0;                             /* Δείχνει αν άλλαξε η τιμή του extra state από 1 σε 0 */
extra_st = 0;                      /* Δείχνει αν είμαι μέσα στο βρόγχο διόρθωσης */ 
tf2 = 0;                           /* Παράμετρος του χρόνου που χρησιμοποιείται για την κίνηση στο βρόγχο διόρθωσης */ 
v_d = 0;                           /* Γραμμική επιθυμητή ταχύτητα */
al_d = 0;                          /* Γραμμική επιθυμητή επιτάχυνση */
a_d = 0;                           /* Στροφική επιθυμητή ταχύτητα */
aa_d = 0;                          /* Στροφική επιθυμητή επιτάχυνση */
st = 0;                            /* Δείχνει που βρίσκομαι σε σχέση με τα εμπόδια */                             
t = 0;                             /* Χρόνος */
vel_old = 0;                       /* Παλιά μέτρηση της ταχύτητας */
vel_new = 0;                       /* Καινούργια μέτρηση της ταχύτητας */
a_real = 0;                        /* Η πραγματική γραμμική στιγμιαία επιτάχυνση */

while (ros::ok()){
	geometry_msgs::Twist velocity;


	if (t<=255){                /* Συνθήκη τερματισμού (αν δεν μπει δίνεται εντολή στο ρομπότ να σταματήσει) */

		if (sonarF_val>=0.3 && sonarFL_val>=0.3 && sonarL_val >= 0.295){ /* Συνθήκη που μου δείχνει αν φτάνω στο εμπόδιο. Αν δεν 											βλέπω εμπόδιο σε απόσταση 0.3m μπαίνει στο if (φάση 1) */

			prev_extra_st = extra_st;   /* Δείχνει αν την προηγούμενη φορά ήμουν στο βρόγχο ελέγχου */ 
			if (st != 1){               /* Την πρώτη φορά που θα μπω θα γίνει η αρχικοποίηση */
				tf = 0.1;           /* Αρχικοποίηση μιας χρονικής μεταβλητής για να περιγράψω την ομαλή κίνηση */
      				v_f = v_d;          /* Αρχική γραμμική ταχύτητα αυτή που είχα πριν */
      				a_f = al_d;         /* Αρχική γραμμική επιτάχυνση αυτή που είχα πριν */
      				a_k = a_d;          /* Αρχική γωνιακή ταχύτητα αυτή που είχα πριν */
      				aa_k = aa_d;        /* Αρχική γωνιακή επιτάχυνση αυτή που είχα πριν */
			}
   			st = 1;                     /* Αλλάζω το state μου σε 1 δηλαδή ότι δεν βλέπω εμπόδιο */
   			if (v_d < 0.2 && al_d >= 0){ /* Συνθήκη που μου δείχνει πότε φτάνει ομαλά η επιθυμητή ταχύτητα στο 0.2 								οπότε τότε την κρατάω σταθερή όπως και την επιτάχυνση */
 				v_d = v_f + a_f * tf +  2.4 * tf * tf - 3.2* tf * tf *tf; 
     				al_d = a_f +  4.8 * tf - 9.6 * tf * tf;
      				tf = tf + 0.1;       /* Αύξηση του χρόνου κατά την περίοδο δειγματοληψίας */ 
			}
			else{                        /* Φάση σταθερής ταχύτητας */ 
				v_d = 0.2;
				al_d = 0;
			}

			if (g == 0){               /* Αν η προηγούμενη φάση που βρισκόμουν ήταν η φάση 2 (έχω δει τοίχο και στρίβω 						           με θετική ταχύτητα) τότε επιβραδύνω ομαλά τη θετική ταχύτητα */
				if (a_d > 0 && aa_d <= 0){     /* Συνθήκη που μου δείχνει πότε φτάνει ομαλά η επιθυμητή ταχύτητα στο 0 									οπότε τότε την κρατάω σταθερή όπως και την επιτάχυνση */
					a_d = a_k + aa_k * tf -  2.4 * tf * tf + 3.2 * tf * tf *tf; 
					aa_d = aa_k -  4.8 * tf + 9.6 * tf * tf;
					tf = tf + 0.1;
				}
				else{               /* Φάση σταθερής μηδενικής ταχύτητας */ 
					a_d = 0;
					aa_d = 0;
				}
			}
			if ( sonarL_val >=0.305 && sonarL_val<=0.6){                   /* Βρόγχος διόρθωσης */ 
				g = 1;           /* Καθορίζει ότι μπήκα στο βρόγχο οπότε δεν υπάρχει περίπτωση να θέλω να 							επιβραδύνω μια θετική γωνιακή ταχύτητα εκτός και αν πάω πάλι στη φάση 2*/               
				s = 0;          /* Από τη στιγμή που μπαίνω στο βρόγχο διόρθωσης θέλω να επιταχύνω αρνητικά άρα δεν θέλω να 							μπω στη φάση επιβράδυνσης μέχρις ότου να σταματήσω να μπαίνω στο βρόγχο διόρθωσης */
				h = 0;          /* Η επόμενη φορά που θα μπώ στη φάση επιβράδυνσης της αρνητικής γωνιακής ταχύτητας θα 							είναι η πρώτη άρα θα πρέπει να αρχικοποιήσω τα μεγέθη μου */ 
				if (extra_st != 1){ /* Αρχικοποιώ τα μεγέθη μου κάθε πρώτη φορά που μπάινω στο βρόγχο διόρθωσης*/
					tf2 = 0.1;
					a_k = a_d;
					aa_k = aa_d;
				}

				extra_st = 1;  /* Μπήκα στο βρόγχο διόρθωσης */  

				if (a_d > -0.2 && aa_d <= 0){    /* Φάση ομαλής επιτάχυνσης της ταχύτητας μέχρις ότου να γίνει -0.2 όπου 									την κρατάμε σταθερή */ 
					a_d = a_k + aa_k * tf2 -  2.4 * tf2 * tf2 + 3.2 * tf2 * tf2 *tf2; 
					aa_d = aa_k -  4.8 * tf2 + 9.6 * tf2 * tf2;
					tf2 = tf2 + 0.1;
				}
				else{                            /* Φάση σταθερής ταχύτητας */
					a_d = -0.2;
					aa_d = 0;
				}
			}
			else{ 
				extra_st = 0;                   /* Δεν μπήκα στο βρόγχο διόρθωσης */ 
			}
			if (extra_st==0 && prev_extra_st==1) s=1;   /* Αν ήμουν στο βρόγχο διόρθωσης και βγώ θα έχω αρνητική ταχύτητα άρα 									πρέπει να την επιβραδύνω, άρα θα πρέπει να μπω στη φάση επιβράδυνσης της 									αρνητικής ταχύτητας */			
			if (s){                         /* Φάση επιβράδυνσης της αρνητικής γωνιακής ταχύτητας (θα μείνω σε αυτή τη φάση 							μέχρις ότου να μπω ξανά το βρόγχο ελέγχου είτε στη φάση 2 ή 3) */ 

				if (h != 1){            /* Αρχικοποιώ τα μεγέθη μου κάθε πρώτη φορά που μπαίνω στη φάση ομαλής επιβράδυνσης 								της αρνητικής γωνιακής ταχύτητας */
					tf3 = 0.1;
					v_f = v_d; 
					a_f = al_d;
					a_k = a_d;
					aa_k = aa_d;
				}
				h = 1;            /* Είμαι από πριν στη φάση ομαλής επιβράδυνσης της αρνητικής γωνιακής ταχύτητας */
				if (a_d < 0 && aa_d >= 0){      /* Φάση ομαλής επιβράδυνσης της γωνιακής ταχύτητας μέχρις ότου να 									γίνει 0 όπου την κρατάμε σταθερή */ 
					a_d = a_k + aa_k * tf3 +  2.4 * tf3 * tf3 - 3.2* tf3 * tf3 *tf3; 
					aa_d = aa_k +  4.8 * tf3 - 9.6 * tf3 * tf3;
					tf3 = tf3 + 0.1;
				}
				else{                            /* Φάση σταθερής μηδενικής ταχύτητας */ 
					a_d = 0;
					aa_d = 0;
				}
			}
		}

		else{                  /* Μπαίνω στη φάση όπου βλέπω εμπόδιο σε απόσταση 0.3m (φάση 2) */ 
			g = 0;         /* Η τελευταία φάση που ήμουν ήταν η φάση 2 (χρησιμεύει όταν μπαίνω στη φάση 1 και θέλω να ξέρω αν 						πρέπει να επιβραδύνω τη θετική ταχύτητα που είχα από τη φάση 2) */ 
			extra_st = 0;  /* Προφανώς δεν είμαι στο βρόγχο διόρθωης αφού δεν είμαι στη φάση 1 (απλή νέα αρχικοποίηση 						μεταβλητής) */
			if (st != 2){  /* Την πρώτη φορά που μπαίνω στη φάση 2 αρχικοποιώ τα βασικά μου μεγέθη */ 
				tf = 0.1;
				v_f = v_d; 
				a_f = al_d;
				a_k = a_d;
				aa_k = aa_d;
			}
			st = 2;        /* Είμαι στη φάση 2 */
			if (v_d > 0 && al_d <= 0){          /* Φάση ομαλής επιβράδυνσης της γραμμικής ταχύτητας μέχρις ότου να γίνει 0 όπου 								την κρατάμε σταθερή */ 
				v_d = v_f + a_f * tf -  2.4 * tf * tf + 3.2 * tf * tf *tf; 
				al_d = a_f -  4.8 * tf + 9.6 * tf * tf;
				tf = tf + 0.1;
			}
			else{       /* Φάση σταθερής γραμμικής μηδενικής ταχύτητας */ 
				v_d = 0;
				al_d = 0;
			}
			if (a_d < 0.2 && aa_d >= 0){     /* Φάση ομαλής επιτάχυνσης της γωνιακής ταχύτητας μέχρις ότου να γίνει 0.2 όπου 								την κρατάμε σταθερή */
				a_d = a_k + aa_k * tf +  2.4 * tf * tf - 3.2* tf * tf *tf; 
				aa_d = aa_k +  4.8 * tf - 9.6 * tf * tf;
				tf = tf + 0.1;
			}
			else{       /* Φάση σταθερής γωνιακής ταχύτητας */ 
				a_d = 0.2;
				aa_d = 0;
			}
		}
	}          
	else{                      /* Σταμάτημα του ρομπότ με ομαλό μηδενισμό */
		if (st != 3){
			v_f = v_d;                                                               
			a_f = al_d;      
			tf = 0.1;
		}
		st = 3;
		if (v_d > 0 && al_d <= 0){
			v_d = v_f + a_f * tf -  2.4 * tf * tf + 3.2 * tf * tf *tf; 
			al_d = a_f -  4.8 * tf + 9.6 * tf * tf;
			tf = tf + 0.1;
		}
		else{
			v_d = 0;
			al_d = 0;
		}
		a_d = 0;
	}

/***** Εξισώσεις ελέγχου *****/
velocity.angular.z = a_d;
velocity.linear.x = v_d;
t = t + 0.1;           /**** αυξάνουμε το χρόνο κατά την περίοδο δειγματοληψίας ****/

    velocity_pub.publish(velocity);

    loop_rate.sleep();
    ros::spinOnce();
}
return 0;
}
