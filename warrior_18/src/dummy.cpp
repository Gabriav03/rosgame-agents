#include "warrior_18/dummy.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

// =========================================================
// PID CONTROLLER (Para movimiento suave y preciso)
// =========================================================
Warrior::PIDController::PIDController()
    : kp_(0), ki_(0), kd_(0), prev_error_(0), integral_error_(0), deriv_filter_state_(0), first_run_(true) {}

void Warrior::PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

double Warrior::PIDController::compute(double error, double dt) {
    if (dt <= 0.0001) return 0.0;
    if (first_run_) { prev_error_ = error; first_run_ = false; return 0.0; }

    double p = kp_ * error;
    double current_deriv = (error - prev_error_) / dt;
    // Filtro paso bajo para la derivada (evita sacudidas)
    deriv_filter_state_ = (FILTER_ALPHA * current_deriv) + ((1.0 - FILTER_ALPHA) * deriv_filter_state_);
    double d = kd_ * deriv_filter_state_;

    integral_error_ = std::clamp(integral_error_ + (error * dt), -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    double i = ki_ * integral_error_;

    prev_error_ = error;
    return std::clamp(p + i + d, -OUTPUT_LIMIT, OUTPUT_LIMIT);
}

// =========================================================
// SETUP Y REGISTRO
// =========================================================

Warrior::Warrior(): Node("robot_warrior"), last_pid_time_(0, 0, this->get_clock()->get_clock_type())
{
    warrior_nick = "Warrior_16";
    
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request->username = warrior_nick;
    
    while (!client->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio de registro...");
    }

    int retry_count = 0;
    while (code == "-1" && rclcpp::ok()) {
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            code = result.get()->code;
            if (code == "-1") {
                request->username = warrior_nick + std::to_string(++retry_count);
            }
        }
    }

    // Suscriptores y Publicadores estándar
    pub_vel_ = create_publisher<rosgame_msgs::msg::RosgameTwist>("/" + code + "/cmd_vel", 10);
    sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>("/" + code + "/laser_scan", 10, std::bind(&Warrior::process_laser_info, this, _1));
    sub_scene_ = create_subscription<std_msgs::msg::String>("/" + code + "/scene_info", 10, std::bind(&Warrior::process_scene_info, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Robot LEGAL registrado: %s", code.c_str());

    current_state = GRANJERO;
    // PID tuneado para respuesta rápida
    pid_.set_gains(0.75, 0.005, 0.1); 
}

Warrior::~Warrior() {}

// =========================================================
// PERCEPCIÓN
// =========================================================

void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
    float rep_x = 0.0f;
    float rep_y = 0.0f;
    // Solo consideramos obstáculos en un cono frontal de 60 grados para evitar bloqueos traseros
    float vision_cone_rad = 70.0f * (M_PI / 180.0f);

    for (size_t i = 0; i < msg->ranges.size(); i++) 
    {
        float range = msg->ranges[i];
        float angle = msg->angle_min + (i * msg->angle_increment);

        // Ignoramos lecturas fuera de rango o muy lejanas
        if (range < 0.05f || std::isinf(range)) continue;

        if (std::abs(normalize_angle(angle)) > vision_cone_rad) continue; 

        if (range < WALL_THRESHOLD) 
        {
            // Fuerza inversamente proporcional al cuadrado de la distancia
            float magnitude = 1.0f / (range * range); 
            rep_x -= std::cos(angle) * magnitude;
            rep_y -= std::sin(angle) * magnitude;
        }
    }
    repulsive_force_x = rep_x;
    repulsive_force_y = rep_y;
}

void Warrior::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    Json::CharReaderBuilder reader;
    Json::Value data;
    std::istringstream stream(msg->data);
    std::string errs;
    if (!Json::parseFromStream(reader, stream, &data, &errs)) return;

    battery = data["Battery_Level"].asFloat();
    pos_x = data["Robot_Pose"]["x"].asFloat();
    pos_y = data["Robot_Pose"]["y"].asFloat();
    gamma = data["Robot_Pose"]["gamma"].asFloat();
    
    // Detección de flanco (perder martillo = choque)
    hammer_prev = hammer_enabled;
    hammer_enabled = data["Skills"]["Hammer"].asBool();
    shield_enabled = data["Skills"]["Shield"].asBool();

    auto parse_vec = [](const Json::Value& json_arr, std::vector<std::vector<float>>& out_vec) {
        out_vec.clear();
        for (const auto& item : json_arr) {
            out_vec.push_back({item[0].asFloat(), item[1].asFloat()});
        }
    };

    parse_vec(data["FOV"]["Skills_Positions"], skills_pos);
    parse_vec(data["FOV"]["Chargers_Positions"], chargers_pos);
    parse_vec(data["FOV"]["Players_Positions"], players_pos);

    // Ciclo principal de decisión
    update_fsm();
}

// =========================================================
// LÓGICA DE ESTADOS
// =========================================================

void Warrior::update_fsm()
{
    // 1. MANIOBRA DE EVASIÓN (HIT & RUN)
    // Si perdimos el martillo, significa que golpeamos. Huimos para evitar represalias inmediatas.
    if (is_bouncing) {
        execute_navigation(); 
        return;
    }

    if (hammer_prev && !hammer_enabled) {
        RCLCPP_INFO(this->get_logger(), "¡GOLPE! Iniciando maniobra de escape.");
        is_bouncing = true;
        bounce_start_time = this->get_clock()->now();
        execute_navigation();
        return; 
    }

    bool low_battery = battery < BATTERY_THRESHOLD;
    bool full_charged = battery >= 98.0f;

    // 2. MÁQUINA DE ESTADOS
    switch (current_state)
    {
    case GRANJERO:
        if (hammer_enabled) {
            current_state = PREPARANDO; // Ya tengo arma, vamos a prepararnos
            return;
        }
        
        if (low_battery) {
            find_nearest_target(chargers_pos);
        } else {
            // Hysteresis: Si pierdo el target o cambia mi estado de arma, busco nuevo
            if (!target_set || hammer_prev != hammer_enabled) {
                find_nearest_target(skills_pos);
            }
        }
        execute_navigation();
        break;

    case PREPARANDO:
        if (!hammer_enabled) { current_state = GRANJERO; return; } // Perdí arma, vuelvo a buscar
        
        if (!full_charged) {
            find_nearest_target(chargers_pos); // Prioridad: Cargar a tope antes de pelear
            execute_navigation();
        } else {
            current_state = ACECHADOR; // Cargado y armado -> A Cazar
            target_set = false;
        }
        break;

    case ACECHADOR:
        if(low_battery) { current_state = PREPARANDO; target_set = false; break; }
        if (!hammer_enabled) { current_state = GRANJERO; break; }

        find_nearest_enemy(); // Busca visualmente al enemigo más cercano

        if (target_set) {
            float d = std::hypot(target_x - pos_x, target_y - pos_y);
            // Si estamos cerca, activamos modo ataque agresivo
            if (d < OPONENT_THRESHOLD) { 
                current_state = KAMIKAZE;
            }
        }
        execute_navigation();
        break;

    case KAMIKAZE:
        if (!hammer_enabled || low_battery) {
            current_state = GRANJERO;
            target_set = false;
            return;
        }
        // En kamikaze no re-evaluamos objetivo, vamos a muerte al último conocido
        execute_navigation();
        break;
    }  
}

// =========================================================
// NAVEGACIÓN Y CONTROL
// =========================================================

void Warrior::execute_navigation()
{
    // --- Lógica de Rebote ---
    if (is_bouncing) {
        rclcpp::Time now = this->get_clock()->now();
        if ((now - bounce_start_time).seconds() < 0.5) { 
            publish_cmd(-1.0, 2.0); // Marcha atrás y giro fuerte
            return;
        } else {
            is_bouncing = false; 
            current_state = GRANJERO; // Reset a estado seguro
            target_set = false;       
            publish_cmd(0.0, 0.0);
            return;
        }
    }

    if (!target_set) {
        publish_cmd(0.0, 0.8); // Girar buscando objetivos visuales
        return;
    }

    // --- Campos de Potencial (VFF) ---
    float att_x = target_x - pos_x;
    float att_y = target_y - pos_y;
    float dist = std::sqrt(att_x*att_x + att_y*att_y);
    if (dist > 0.001) { att_x /= dist; att_y /= dist; }

    // Transformar repulsión local a global
    float rep_global_x = (repulsive_force_x * std::cos(gamma)) - (repulsive_force_y * std::sin(gamma));
    float rep_global_y = (repulsive_force_x * std::sin(gamma)) + (repulsive_force_y * std::cos(gamma));

    float K_rep = VFF_REPULSIVE_K;
    if (current_state == KAMIKAZE) K_rep = 0.1f; // Ignorar casi todo en ataque final

    float total_x = (VFF_ATTRACTIVE_K * att_x) + (K_rep * rep_global_x);
    float total_y = (VFF_ATTRACTIVE_K * att_y) + (K_rep * rep_global_y);

    float desired_angle = std::atan2(total_y, total_x);
    
    // --- Control PID ---
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_pid_time_).seconds();
    if (dt <= 0.0 || dt > 0.2) dt = 0.1;
    last_pid_time_ = now;

    float error = normalize_angle(desired_angle - gamma);
    double w = pid_.compute(error, dt);
    
    // Control de velocidad lineal
    double v = MAX_LINEAR_SPEED;
    if (std::abs(error) < 1.57f) v *= std::max(0.1f, (float)std::cos(error));
    else v = 0.0; // Si el error es muy grande, gira sobre su eje

    if (current_state == KAMIKAZE) {
        v = MAX_LINEAR_SPEED * 1.5; // Turbo en ataque
    } else {
        if (dist < 0.8) v *= 0.5; // Frenar al llegar
        if (dist < 0.2) {
            v = 0.0; 
            if (current_state != PREPARANDO) target_set = false; // Objetivo alcanzado
        }
    }

    publish_cmd(v, w);
}

// =========================================================
// UTILIDADES DE BÚSQUEDA
// =========================================================

void Warrior::find_nearest_target(const std::vector<std::vector<float>>& targets)
{
    if (targets.empty()) { target_set = false; return; }

    float min_dist = 1e9;
    int best_idx = -1;

    for (size_t i = 0; i < targets.size(); i++) {
        float d = std::hypot(targets[i][0] - pos_x, targets[i][1] - pos_y);
        
        // Hysteresis: Preferimos mantener el objetivo actual si está cerca
        if (target_set) {
             float dist_to_prev = std::hypot(targets[i][0] - last_target_pos_x, targets[i][1] - last_target_pos_y);
             if (dist_to_prev < 0.5) d -= 0.6; // Bonus de adherencia
        }

        if (d < min_dist) {
            min_dist = d;
            best_idx = i;
        }
    }

    if (best_idx != -1) {
        target_x = targets[best_idx][0];
        target_y = targets[best_idx][1];
        last_target_pos_x = target_x; 
        last_target_pos_y = target_y;
        target_set = true;
    }
}

void Warrior::find_nearest_enemy()
{
    // Búsqueda puramente visual (Legal)
    if (players_pos.empty()) { target_set = false; return; }

    float min_dist = 1e9;
    int best_idx = -1;

    for (size_t i = 0; i < players_pos.size(); i++) {
        float d = std::hypot(players_pos[i][0] - pos_x, players_pos[i][1] - pos_y);
        
        if (target_set) {
             // Hysteresis para no cambiar de objetivo erráticamente
             float dist_to_prev = std::hypot(players_pos[i][0] - last_target_pos_x, players_pos[i][1] - last_target_pos_y);
             if (dist_to_prev < 1.0) d -= 1.0; 
        }

        if (d < min_dist) {
            min_dist = d;
            best_idx = i;
        }
    }

    if (best_idx != -1) {
        target_x = players_pos[best_idx][0];
        target_y = players_pos[best_idx][1];
        last_target_pos_x = target_x;
        last_target_pos_y = target_y;
        target_set = true;
    } else {
        target_set = false;
    }
}

void Warrior::publish_cmd(float v, float w)
{
    rosgame_msgs::msg::RosgameTwist msg;
    msg.code = this->code;
    msg.vel.linear.x = v;
    msg.vel.angular.z = w;
    pub_vel_->publish(msg);
}

float Warrior::normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}