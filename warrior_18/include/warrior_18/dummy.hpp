#ifndef DUMMIE_HPP
#define DUMMIE_HPP

#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/string.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

// CONSTANTES DE NAVEGACIÓN
const float MAX_LINEAR_SPEED = 0.8f;  // Un poco más rápido para compensar
const float MAX_ANGULAR_SPEED = 0.6f;
const float BATTERY_THRESHOLD = 35.0f; 
const float VFF_REPULSIVE_K = 0.45f;   
const float VFF_ATTRACTIVE_K = 1.2f;  
const float WALL_THRESHOLD = 0.9f;
const float OPONENT_THRESHOLD = 2.5f;

enum Estado { 
    GRANJERO,  // Recolectar cajas
    PREPARANDO, // Ir a cargar
    ACECHADOR,  // Perseguir enemigo
    KAMIKAZE    // Ataque final
};

class Warrior : public rclcpp::Node
{
public:
    Warrior();
    ~Warrior();

private:
    // --- CLASE PID INTERNA ---
    class PIDController {
    public:
        PIDController();
        void set_gains(double kp, double ki, double kd);
        double compute(double error, double dt);
    private:
        double kp_, ki_, kd_;
        double prev_error_, integral_error_, deriv_filter_state_;
        bool first_run_;
        const double INTEGRAL_LIMIT = 0.3;
        const double OUTPUT_LIMIT = 2.5;
        const double FILTER_ALPHA = 0.6;
    };

    // Callbacks
    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process_scene_info(const std_msgs::msg::String::SharedPtr msg);

    // Funciones Lógicas
    void update_fsm();          
    void execute_navigation();  
    void find_nearest_target(const std::vector<std::vector<float>>& targets);
    void find_nearest_enemy(); // Versión legal: Solo busca al más cercano visualmente

    // Helpers
    void publish_cmd(float v, float w);
    float normalize_angle(float angle);

    // Comunicaciones
    rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr pub_vel_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_scene_;

    // Variables de Estado
    float battery, pos_x, pos_y, gamma;
    bool hammer_enabled = false, shield_enabled = false;
    bool hammer_prev = false; 

    // Navegación
    double target_x, target_y;
    bool target_set = false;
    float last_target_pos_x, last_target_pos_y;

    // Identificación
    std::string code = "-1";
    std::string warrior_nick = "";

    // Memoria Visual (Solo lo que ve el robot)
    std::vector<std::vector<float>> skills_pos;
    std::vector<std::vector<float>> chargers_pos;
    std::vector<std::vector<float>> players_pos;

    // VFF
    float repulsive_force_x = 0.0f;
    float repulsive_force_y = 0.0f;

    // Control
    Estado current_state;
    PIDController pid_; 
    rclcpp::Time last_pid_time_;

    // Variables Hit & Run (Rebote)
    bool is_bouncing = false;
    rclcpp::Time bounce_start_time;
};

#endif