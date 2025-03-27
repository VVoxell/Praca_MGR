#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <iostream>
#include <fstream>

class TFSubscriber : public rclcpp::Node
{
public:
    TFSubscriber() : Node("anglecalc_subscriber")
    {
        // Subskrypcja topicu /tf
        tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&TFSubscriber::tfCallback, this, std::placeholders::_1));
            
        pos_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose2D>("/pos", 10, std::bind(&TFSubscriber::PoseCallback, this, std::placeholders::_1));
            
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
    }
    /*
    void zerowanie()
    {
        auto zero_twist = geometry_msgs::msg::Twist();
        zero_twist.linear.x = 0.0;
        zero_twist.linear.y = 0.0;
        zero_twist.linear.z = 0.0;
        zero_twist.angular.x = 0.0;
        zero_twist.angular.y = 0.0;
        zero_twist.angular.z = 0.0;
        
        twist_publisher_->publish(zero_twist);
        RCLCPP_INFO(this->get_logger(), "Wyzerowano prędkości");
    }
    */
    
    geometry_msgs::msg::Twist twist_msg;
    rclcpp::Time start = this->get_clock()->now();
    rclcpp::Time peroid; 
    bool znaleziono = false;

private:
     void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
     
     	peroid = this->get_clock()->now();
     	auto delta = peroid - start;
     
        // Przeszukiwanie wszystkich transformacji w wiadomości
        for (const auto& transform : msg->transforms) {
            // Sprawdzanie, czy transformacja jest między "camera_color_optical_frame" a "map"
            
            if (transform.header.frame_id == "camera_color_optical_frame" &&
                transform.child_frame_id == "map") {
                
                znaleziono = true;
                
                // Obliczanie odległości
                double dx = transform.transform.translation.x;
                double dy = transform.transform.translation.y;
                double dz = transform.transform.translation.z;
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                
                // Przekształcanie kwaternionu na kąty Eulera (roll, pitch, yaw)
                tf2::Quaternion q(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                );

                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                double kat = atan2(dx, dz);
                //RCLCPP_INFO(this->get_logger(), "Dx: %.2f, Dy: %.2f, Dz: %.2f, KAT: %.2f", dx, dy, dz, kat);

                // Wyświetlanie odległości
                //RCLCPP_INFO(this->get_logger(), "Odległość między 'camera_color_optical_frame' a 'map': %.2f", distance);

                // Wyświetlanie kątów Eulera
                //RCLCPP_INFO(this->get_logger(), "Kąty Eulera (radiany) - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
                
                double k = -1.0;
                auto vel = kat*k;
                vel = vel * pow(cos(kat), 2);
                auto dist_05 = distance-0.5;
                             
                
                if (abs(vel) < 1.0)
                {
                	twist_msg.angular.z = vel;
                }
                else
                {
                	twist_msg.angular.z = 1.0;
                }
                
                if (-1.0 < dist_05 && dist_05 < 1.0)
                {
                	twist_msg.linear.x = 0.5*sin(M_PI_2*dist_05);
                }
                else if(dist_05 >= 1.0)
                {
                	twist_msg.linear.x = 0.5;
                }
                else
                {
              		twist_msg.linear.x = -0.5;
              	}
                             
                RCLCPP_INFO(this->get_logger(), "Opublikowano Twist - Linear X: %.2f, Angular Z: %.2f", twist_msg.linear.x, twist_msg.angular.z);
                RCLCPP_INFO(this->get_logger(), "ODystans: %.2f", distance); 
            }
            else if(delta.seconds() < 15.0 && znaleziono == false){
            	twist_msg.angular.z = 0.5;
            	twist_msg.linear.x = 0.15;
            	RCLCPP_INFO(this->get_logger(), "Nie widzę taga od: %.2f sekund", delta.seconds()); 
            }
            else if(delta.seconds() > 15.0 && znaleziono == false){
            	twist_msg.angular.z = 0.0;
            	twist_msg.linear.x = 0.0;
            }
	    
	    twist_publisher_->publish(twist_msg);
	    std::ofstream Speed ("Prędkości_robota.csv", std::ios::app);
	    Speed << twist_msg.linear.x << "\t" << twist_msg.angular.z << std::endl;
	    Speed.close();
	    
        }
     }
        
     void PoseCallback (const geometry_msgs::msg::Pose2D pose)
     {
     	time_t rawtime;
  	struct tm * timeinfo;
  	char buffer[80];

  	time (&rawtime);
  	timeinfo = localtime(&rawtime);

  	strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
  	std::string str(buffer);
  	
     	std::ofstream Pozycje ("Pozycje_robota.csv", std::ios::app);
     	
     	auto x = pose.x;
     	auto y = pose.y;
     	auto theta = pose.theta;
     	
     	Pozycje << str << "\t" << x << "\t" << y << "\t" << theta << std::endl;
     	//RCLCPP_INFO(this->get_logger(), "Pozycja robota: x - %.2f, y - %.2f", x, y);
     	
     	Pozycje.close();
     	
     }
     
    // Subskrybent topicu /tf
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
    
    // Subskrybent topicu /pose
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pos_subscriber_;
    
    // Publisher wiadomości Twist
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFSubscriber>());
    rclcpp::shutdown();
    return 0;
}
