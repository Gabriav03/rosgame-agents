#include "warrior_18/warrior.hpp"
#include <chrono>

using std::placeholders::_1;

// PID
PIDController::PIDController()
    : kp_(0), ki_(0), kd_(0), prev_error_(0), integral_error_(0), deriv_filter_state_(0), first_run_(true) {}

void PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void PIDController::reset() { 
    first_run_ = true; integral_error_ = 0.0; prev_error_ = 0.0; 
}

double PIDController::compute(double error, double dt) {
    if (dt <= 0.0001) return 0.0;
    if (first_run_) { prev_error_ = error; first_run_ = false; return 0.0; }

    double p = kp_ * error;
    double current_deriv = (error - prev_error_) / dt;
    deriv_filter_state_ = (FILTER_ALPHA * current_deriv) + ((1.0 - FILTER_ALPHA) * deriv_filter_state_);
    double d = kd_ * deriv_filter_state_;

    integral_error_ = std::clamp(integral_error_ + (error * dt), -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    double i = ki_ * integral_error_;

    prev_error_ = error;
    return std::clamp(p + i + d, -OUTPUT_LIMIT, OUTPUT_LIMIT);
}

// WARRIOR
Warrior::Warrior(): Node("robot_warrior"), last_pid_time_(0, 0, this->get_clock()->get_clock_type())
{
    warrior_nick = "warrior_18";
    
    // REGISTRO AUTOMATICO
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request->username = warrior_nick;
    
    while (!client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio...");
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

    // Inicializacion de Comunicaciones
    pub_vel_ = create_publisher<rosgame_msgs::msg::RosgameTwist>("/" + code + "/cmd_vel", 10);
    sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>("/" + code + "/laser_scan", 10, std::bind(&Warrior::process_laser_info, this, _1));
    sub_scene_ = create_subscription<std_msgs::msg::String>("/" + code + "/scene_info", 10, std::bind(&Warrior::process_scene_info, this, _1));
    pub_state_ = create_publisher<std_msgs::msg::String>(INTERNAL_STATE_TOPIC, 10);
    
    sub_integrity_check_ = create_subscription<rosgame_msgs::msg::RosgameTwist>(
    "/" + code + "/cmd_vel", 
    rclcpp::SensorDataQoS(), 
    std::bind(&Warrior::check_integrity, this, _1)
    );

    timer_scan_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&Warrior::scan_enemies, this)
    );

    RCLCPP_INFO(this->get_logger(), "Robot registrado: %s", code.c_str());

    current_state = GRANJERO;
    pid_.set_gains(0.6, 0.001, 0.0); 
}


Warrior::~Warrior() {

}

void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
    // Solo guardamos el escaneo. La lógica reactiva se calcula
    // en execute_navigation para tener la decisión más fresca posible.
    last_scan_ = msg; 
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

   

    hammer_prev = hammer_enabled;
    hammer_enabled = data["Skills"]["Hammer"].asBool();

    shield_enabled = data["Skills"]["Shield"].asBool();
    health = data["Health"].asFloat();

    auto parse_vec = [](const Json::Value& json_arr, std::vector<std::vector<float>>& out_vec)
    {
        out_vec.clear();
        for (const auto& item : json_arr)
        {
            out_vec.push_back({item[0].asFloat(), item[1].asFloat()});
        }
    };

    parse_vec(data["FOV"]["Skills_Positions"], skills_pos);
    parse_vec(data["FOV"]["Chargers_Positions"], chargers_pos);
    parse_vec(data["FOV"]["Players_Positions"], players_pos);

    for (const auto& pos : players_pos)
    {
        float vis_x = pos[0];
        float vis_y = pos[1];

        std::string best_id = "";
        float min_d = 1000.0f;
        for (auto& [id, info] : enemies_data_)
        {
            float d = (std::abs(info.x) > 0.1) ? std::hypot(info.x - vis_x, info.y - vis_y) : 0.5f;

            if (d < min_d && d < 2.0f)
            {
                min_d = d;
                best_id = id;
            }
        }

        if (!best_id.empty()) {
            enemies_data_[best_id].x = vis_x;
            enemies_data_[best_id].y = vis_y;
        }
    }
    update_fsm();

}

bool Warrior::check_path_clear(float angle, float dist_needed)
{
    if (!last_scan_) return false;

    // Convertir ángulo relativo a índice del array
    float angle_idx = (angle - last_scan_->angle_min) / last_scan_->angle_increment;
    int idx = static_cast<int>(angle_idx);

    // Revisar no solo el punto central, sino un pequeño arco (ancho del robot)
    // para no rozar con el hombro.
    int margin = static_cast<int>(ROBOT_WIDTH_ANGLE / last_scan_->angle_increment);
    
    int start = std::max(0, idx - margin);
    int end = std::min(static_cast<int>(last_scan_->ranges.size()) - 1, idx + margin);

    for (int i = start; i <= end; i++) {
        float r = last_scan_->ranges[i];
        if (std::isinf(r)) continue;
        if (r < dist_needed) return false; // Bloqueado
    }
    return true; // Libre
}

float Warrior::find_safe_angle(float target_angle_local)
{
    if (!last_scan_) return target_angle_local;

    // 1. ¿El camino directo está libre?
    // Usamos una distancia de seguridad dinámica (Kamikaze arriesga más)
    float required_dist = (current_state == KAMIKAZE) ? 0.8f : SAFE_DISTANCE;
    
    if (check_path_clear(target_angle_local, required_dist)) {
        return target_angle_local;
    }

    // 2. Si está bloqueado, BUSCAR HUECO (Gap Finding)
    // Barremos desde el centro hacia los lados (alternando izq/der) 
    // para encontrar el ángulo libre más cercano al objetivo.
    
    const float SEARCH_STEP = 0.087f; // 5 grados aprox
    const float MAX_SEARCH_ANGLE = M_PI / 1.5f; // Buscar hasta +/- 120 grados

    for (float offset = SEARCH_STEP; offset < MAX_SEARCH_ANGLE; offset += SEARCH_STEP)
    {
        // Revisar izquierda
        float left_candidate = normalize_angle(target_angle_local + offset);
        if (check_path_clear(left_candidate, required_dist * 0.9f)) { // 0.9 tolerancia
            return left_candidate;
        }

        // Revisar derecha
        float right_candidate = normalize_angle(target_angle_local - offset);
        if (check_path_clear(right_candidate, required_dist * 0.9f)) {
            return right_candidate;
        }
    }

    // 3. Si todo está bloqueado, estamos acorralados.
    // Devolvemos un valor especial o el ángulo más abierto que encontremos (simplificado aquí)
    return target_angle_local + M_PI; // Media vuelta (comando de pánico)
}

void Warrior::check_integrity(const rosgame_msgs::msg::RosgameTwist::SharedPtr msg)
{
    const float EPSILON = 0.05f;
    float diff_v = std::abs(msg->vel.linear.x - last_sent_v_);
    float diff_w = std::abs(msg->vel.angular.z - last_sent_w_);

    if (diff_v > EPSILON || diff_w > EPSILON) 
    {
        if (!under_cyberattack_) {
            RCLCPP_ERROR(this->get_logger(), "¡ALERTA DE SEGURIDAD! Detectada inyección de comandos externos.");
            under_cyberattack_ = true;
        }

        rosgame_msgs::msg::RosgameTwist correction_msg;
        correction_msg.code = this->code;
        correction_msg.vel.linear.x = last_sent_v_;
        correction_msg.vel.angular.z = last_sent_w_;
        
        pub_vel_->publish(correction_msg);
        pub_vel_->publish(correction_msg);
    } 
    else {
        if (under_cyberattack_) {
            RCLCPP_INFO(this->get_logger(), "ECM: Señal estabilizada.");
            under_cyberattack_ = false;
        }
    }
}

void Warrior::update_fsm()
{
    std_msgs::msg::String state_msg;
    const char* state_strs[] = {"DESCONOCIDO", "GRANJERO", "PREPARANDO", "ACECHADOR", "KAMIKAZE", "HUIDA"};
    state_msg.data = code + "|" + state_strs[current_state];
    pub_state_->publish(state_msg);

    // He quitado la lógica de rebote (is_bouncing) como pediste
    // Ahora confiamos en la Fuerza Tangencial para salir de atascos

    bool low_battery = battery < BATTERY_THRESHOLD;
    bool cargado = battery >= 95.0f;

    switch (current_state)
    {
    case GRANJERO:
        if (hammer_enabled) {
            current_state = PREPARANDO;
            return;
        }
        
        if (low_battery) {
            find_nearest_target(chargers_pos);
        } else {
            if (!target_set || hammer_prev != hammer_enabled) {
                find_nearest_target(skills_pos);
            }
        }
        execute_navigation();
        break;

    case PREPARANDO:
        if (!hammer_enabled) {
            current_state = GRANJERO;
            return;
        }
        if (!cargado) {
            find_nearest_target(chargers_pos);
            execute_navigation();
        } else {
            current_state = ACECHADOR;
            target_set = false;
        }
        break;

    case ACECHADOR:
    {
        if(low_battery) {
            current_state = PREPARANDO;
            target_set = false;
            break;
        }

        find_weakest_player();

        if (target_set) {
            float d = std::hypot(target_x - pos_x, target_y - pos_y);
            if (d < OPONENT_THRESHOLD) { 
                current_state = KAMIKAZE;
            }
        }
        
        execute_navigation();
        break;
    }

    case KAMIKAZE:
        if (!hammer_enabled || low_battery) {
            current_state = GRANJERO;
            target_set = false;
            return;
        }
        execute_navigation();
        break;
        
    default:
        break;
    }  
}

// ... (Includes y PIDController se mantienen igual)
// ... (Warrior::Warrior y demás funciones se mantienen igual)

void Warrior::execute_navigation()
{
    // --- 0. SEGURIDAD ANTE CHOQUE INMINENTE (ANTI-ATASCO MEJORADO) ---
    if (last_scan_) {
        int center_idx = last_scan_->ranges.size() / 2;
        float front_dist = 100.0f;
        int vision_width = 15; 
        
        for(int i = -vision_width; i <= vision_width; i++) {
            if (center_idx + i >= 0 && center_idx + i < (int)last_scan_->ranges.size()) {
                float r = last_scan_->ranges[center_idx + i];
                if (!std::isinf(r) && !std::isnan(r)) {
                    front_dist = std::min(front_dist, r);
                }
            }
        }

        if (front_dist < CRITICAL_DISTANCE) {
            publish_cmd(-0.4f, 0.6f); 
            return;
        }
    }

    if (!target_set) {
        publish_cmd(0.0, 0.8); 
        return;
    }

    // --- 1. CÁLCULO DE VECTORES ---
    float dx = target_x - pos_x;
    float dy = target_y - pos_y;
    float dist_target = std::hypot(dx, dy); // Distancia real al objetivo

    float angle_global = std::atan2(dy, dx);
    float target_angle_local = normalize_angle(angle_global - gamma);

    // --- 2. BUSCADOR DE HUECOS (GAP SELECTOR) ---
    float best_angle = find_safe_angle(target_angle_local);

    // --- 3. PID (Giro) ---
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_pid_time_).seconds();
    if (dt > 0.2) dt = 0.1;
    last_pid_time_ = now;

    double w = pid_.compute(best_angle, dt);

    // --- 4. CONTROL DE VELOCIDAD (MODIFICADO) ---
    float v = MAX_LINEAR_SPEED;

    // A) CARGA / ITEM
    if (current_state == PREPARANDO || current_state == GRANJERO) {
        float approach_speed = dist_target * 0.7f; 
        v = std::clamp(approach_speed, 0.25f, MAX_LINEAR_SPEED); 
    }

    // B) FRENADO EN CURVAS (Estándar para navegación segura)
    if (std::abs(best_angle) > 0.6f) v *= 0.5f;
    if (std::abs(best_angle) > 1.8f) v = 0.0f; 

    // C) TURBO KAMIKAZE DE PRECISIÓN
    if (current_state == KAMIKAZE) {
        // 1. Velocidad Base: Vamos rápido pero controlado para poder girar bien
        v = MAX_LINEAR_SPEED; 

        // 2. Condición de Disparo:
        //    - Estamos cerca (distancia de carga)
        //    - Y estamos alineados (el enemigo está casi en el centro de la mira)
        bool in_range = dist_target < KAMIKAZE_CHARGE_DIST;
        bool aimed = std::abs(best_angle) < KAMIKAZE_AIM_TOLERANCE;

        if (in_range && aimed) {
            // ¡IMPACTO! Sobrescribimos todo. Velocidad máxima absoluta.
            v = 1.6f; 
        } 
        // Si no estamos alineados (aimed == false), v se queda en MAX_LINEAR_SPEED (0.8)
        // Esto permite que el PID tenga autoridad para girar el robot hacia el enemigo
        // antes de lanzar el ataque final.
    }
    
    // D) PARADA FINAL
    if (current_state != KAMIKAZE && dist_target < 0.25f) { 
        v = 0.0f;
        target_set = false;
    }

    publish_cmd(v, w);
}

// ... (Resto del archivo se mantiene igual)

void Warrior::find_nearest_target(const std::vector<std::vector<float>>& targets)
{
    if (targets.empty()) { target_set = false; return; }
    float min_dist = 1e9;
    int best_idx = -1;
    for (size_t i = 0; i < targets.size(); i++) {
        float tx = targets[i][0];
        float ty = targets[i][1];
        float d = std::hypot(tx - pos_x, ty - pos_y);
        if (target_set) {
             float dist_to_prev = std::hypot(tx - last_target_pos_x, ty - last_target_pos_y);
             if (dist_to_prev < 0.5) d -= 0.5; 
        }
        if (last_scan_) {
            float angle_to_obj = std::atan2(ty - pos_y, tx - pos_x);
            float angle_rel = normalize_angle(angle_to_obj - gamma);
            if (angle_rel >= last_scan_->angle_min && angle_rel <= last_scan_->angle_max) {
                int idx = static_cast<int>((angle_rel - last_scan_->angle_min) / last_scan_->angle_increment);
                if (idx >= 0 && idx < static_cast<int>(last_scan_->ranges.size())) {
                    float wall_dist = last_scan_->ranges[idx];
                    if (wall_dist < d - 0.2f) continue;
                    if (std::abs(wall_dist - d) < ITEM_WALL_MARGIN) continue;
                }
            }
        }
        if (d < min_dist) { min_dist = d; best_idx = i; }
    }
    if (best_idx != -1) {
        target_x = targets[best_idx][0];
        target_y = targets[best_idx][1];
        last_target_pos_x = target_x; 
        last_target_pos_y = target_y;
        target_set = true;
    }
}

void Warrior::publish_cmd(float v, float w)
{
    last_sent_v_ = v;
    last_sent_w_ = w;

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

void Warrior::process_enemy_data(const std_msgs::msg::String::SharedPtr msg, std::string enemy_id)
{
    Json::CharReaderBuilder reader;
    Json::Value data;
    std::istringstream stream(msg->data);
    Json::parseFromStream(reader, stream, &data, nullptr);

    if (enemies_data_.find(enemy_id) != enemies_data_.end()) {
        enemies_data_[enemy_id].health = data["Health"].asFloat();
        enemies_data_[enemy_id].battery = data["Battery_Level"].asFloat();
        enemies_data_[enemy_id].has_hammer = data["Skills"]["Hammer"].asBool();
        enemies_data_[enemy_id].has_shield = data["Skills"]["Shield"].asBool();
    }
}

float Warrior::distance_between(float x1, float y1, float x2, float y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

float Warrior::calculate_target_score(const EnemyInfo& target)
{
    float score = 0.0f;
    score += (100.0f - target.health); 
    float dist_to_me = distance_between(pos_x, pos_y, target.x, target.y);
    score -= (dist_to_me * 5.0f);

    for (const auto& pair : enemies_data_) {
        const EnemyInfo& other = pair.second;
        if (other.id == target.id) continue;
        float dist_between_them = distance_between(target.x, target.y, other.x, other.y);
        if (dist_between_them < 2.0f) {
            score -= 200.0f; 
        }
    }

    if (target.has_hammer) score -= 40.0f;
    if (target.has_shield) score -= 20.0f;
    if (hammer_enabled && target.health < 50.0f) score += 500.0f;

    return score;
}

void Warrior::find_weakest_player()
{
    std::string best_enemy_id = "";
    float highest_score = -10000.0f; 

    for (auto const& [id, enemy] : enemies_data_)
    {
        if (std::abs(enemy.x) < 0.1 && std::abs(enemy.y) < 0.1) continue;

        float current_score = calculate_target_score(enemy);

        if (target_set && std::abs(target_x - enemy.x) < 0.5) {
            current_score += 50.0f; 
        }

        if (enemy.health <= 0.0f) {
            continue;
        }

        if (enemy.has_hammer) {
            current_score -= 500.0f; 
        }

        if (enemy.has_shield) {
            current_score -= 300.0f; 
        }

        if (current_score > highest_score) {
            highest_score = current_score;
            best_enemy_id = id;
        }
    }

    if (!best_enemy_id.empty()) 
    {
        target_x = enemies_data_[best_enemy_id].x;
        target_y = enemies_data_[best_enemy_id].y;
        target_set = true;
    }
    else 
    {
        target_set = false;
    }
}

void Warrior::scan_enemies()
{
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    for (const auto& topic : topic_names_and_types) {
        std::string name = topic.first;
        
        if (name.find("/scene_info") != std::string::npos && 
            name.find("/Robot") == std::string::npos && 
            name.find(code) == std::string::npos)       
        {
            size_t first = name.find("/");
            size_t second = name.find("/", first + 1);
            if (second != std::string::npos) {
                std::string enemy_id = name.substr(first + 1, second - first - 1);
                
                if (enemy_subscriptions_.find(enemy_id) == enemy_subscriptions_.end()) {
                    
                    auto callback = [this, enemy_id](const std_msgs::msg::String::SharedPtr msg) {
                        this->process_enemy_data(msg, enemy_id);
                    };

                    enemy_subscriptions_[enemy_id] = this->create_subscription<std_msgs::msg::String>(
                        name, 10, callback
                    );
                    
                    EnemyInfo info;
                    info.id = enemy_id;
                    enemies_data_[enemy_id] = info;
                }
            }
        }
    }
}