#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std;

struct Waypoint {
    double x, y, theta;
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double compute(double error) {
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

    void set_gains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

class PIDNavigator : public rclcpp::Node {
public:
    PIDNavigator() : Node("pid"),
                     current_x_(0), current_y_(0), current_theta_(0), target_index_(0),
                     is_first_target_(true),  // 初期化順を修正
                     linear_pid_(0.0, 0.0, 0.0),  // 初期化時にデフォルト値を設定
                     angular_pid_(0.0, 0.0, 0.0) {  // 初期化時にデフォルト値を設定
        // パラメータの宣言
        this->declare_parameter<double>("linear_kp", 1.0);
        this->declare_parameter<double>("linear_ki", 0.0);
        this->declare_parameter<double>("linear_kd", 0.0);
        this->declare_parameter<double>("angular_kp", 0.05);
        this->declare_parameter<double>("angular_ki", 0.0);
        this->declare_parameter<double>("angular_kd", 0.0);
        this->declare_parameter<double>("max_linear_velocity", 0.5);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<double>("arrival_threshold", 0.1);
        this->declare_parameter<double>("theta_threshold", 0.1);
        this->declare_parameter<int>("wait_time", 1);
        this->declare_parameter("field_color", "red");
        this->declare_parameter("red_csv", "/home/yuki/ros2_ws/src/uni_tes/path/red.csv");
        this->declare_parameter("blue_csv", "/home/yuki/ros2_ws/src/uni_tes/path/blue.csv");

        this->get_parameter("linear_kp", linear_kp);
        this->get_parameter("linear_ki", linear_ki);
        this->get_parameter("linear_kd", linear_kd);
        this->get_parameter("angular_kp", angular_kp);
        this->get_parameter("angular_ki", angular_ki);
        this->get_parameter("angular_kd", angular_kd);
        
        cout <<"angular_kp: " << angular_kp << endl;
        linear_pid_.set_gains(linear_kp, linear_ki, linear_kd);
        angular_pid_.set_gains(angular_kp, angular_ki, angular_kd);

        this->get_parameter("max_linear_velocity", max_linear_vel_);
        this->get_parameter("max_angular_velocity", max_angular_vel_);
        this->get_parameter("arrival_threshold", arrival_threshold_);
        this->get_parameter("theta_threshold", theta_threshold_);
        this->get_parameter("wait_time", wait_time_);
        field_color_ = this->get_parameter("field_color").as_string();
        red_csv = this->get_parameter("red_csv").as_string();
        blue_csv = this->get_parameter("blue_csv").as_string();

        if(field_color_ == "red") load_waypoints(red_csv);
        else if(field_color_ == "blue") load_waypoints(blue_csv);  


        RCLCPP_INFO(this->get_logger(), "field_color: %s", field_color_.c_str());

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&PIDNavigator::pose_callback, this, std::placeholders::_1));
        start_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start", 10, std::bind(&PIDNavigator::start_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pid_cmd_vel", 10);
        state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/state", 10);
        launcher1_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/launcher1", 10);
        kokuban_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/kokuban", 10);
        ball_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/ball", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PIDNavigator::control_loop, this));
        std::this_thread::sleep_for(std::chrono::seconds(wait_time_));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr launcher1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kokuban_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Waypoint> waypoints_;
    double current_x_, current_y_, current_theta_;
    size_t target_index_;
    bool is_first_target_;
    PIDController linear_pid_;
    PIDController angular_pid_;
    // パラメータの読み込み
    double linear_kp, linear_ki, linear_kd;
    double angular_kp, angular_ki, angular_kd;
    double max_linear_vel_, max_angular_vel_;
    double arrival_threshold_, theta_threshold_;
    int wait_time_;
    string field_color_,red_csv,blue_csv;
    bool start_flag = false;

    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
    }

    void start_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        start_flag = msg->data;
    }

    void control_loop() {
        if (target_index_ >= waypoints_.size()) {
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            cout<<"Reached the goal!"<<endl;
            return;
        }

        if(!start_flag) return;

        const auto& target = waypoints_[target_index_];
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance = sqrt(dx * dx + dy * dy);
        double angle_error = target.theta - current_theta_;

        double linear_vel = linear_pid_.compute(distance);
        double angular_vel = angular_pid_.compute(angle_error);

        // 速度制限
        linear_vel = max(min(linear_vel, max_linear_vel_), -max_linear_vel_);
        angular_vel = max(min(angular_vel, max_angular_vel_), -max_angular_vel_);

        double v_x = dx * cos(current_theta_) + dy * sin(current_theta_);
        double v_y = -dx * sin(current_theta_) + dy * cos(current_theta_);

        if (distance < arrival_threshold_ && abs(angle_error) < theta_threshold_) {

            //stateが2,5の終わりのときに射出を送る
            if(target_index_ == 2 || target_index_ == 5){
                auto shoot_cmd = std_msgs::msg::Bool();
                shoot_cmd.data = true;
                launcher1_publisher_->publish(shoot_cmd);
            }
            //stateが0の終わりに黒板回収
            if(target_index_ == 0){
                auto kokuban_cmd = std_msgs::msg::Bool();
                kokuban_cmd.data = true;
                kokuban_publisher_->publish(kokuban_cmd);
            }
            //stateが3の終わりにボール回収
            if(target_index_ == 3){
                auto ball_cmd = std_msgs::msg::Bool();
                ball_cmd.data = true;
                ball_publisher_->publish(ball_cmd);
            }

            target_index_++;
            cout<<"target_index: "<<target_index_<<endl;
            auto state_msg = std_msgs::msg::Int32();
            state_msg.data = target_index_;
            state_publisher_->publish(state_msg);

            linear_pid_.reset();
            angular_pid_.reset();

            auto stop_cmd = geometry_msgs::msg::Twist();
            stop_cmd.linear.x = 0.0;
            stop_cmd.linear.y = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);

            this_thread::sleep_for(chrono::seconds(wait_time_));
        }

        auto cmd = geometry_msgs::msg::Twist();
        double norm_factor = sqrt(v_x * v_x + v_y * v_y);
        if (norm_factor > 0) {
            cmd.linear.x = linear_vel * (v_x / norm_factor);
            cmd.linear.y = linear_vel * (v_y / norm_factor);
        } else {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
        }
        cmd.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd);
    }

    void load_waypoints(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            Waypoint wp;
            char comma;
            if (ss >> wp.x >> comma >> wp.y >> comma >> wp.theta) {
                waypoints_.push_back(wp);
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
