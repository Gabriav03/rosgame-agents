#include "warrior_18/racer.hpp"
#include <chrono>
#include <thread>
#include <algorithm> 
#include <cmath>

Racer::Racer(): Node ("robot_warrior")
{
    warrior_nick = "warrior_18";
    int cont = 0;

    // --- Variables de Estado y Navegación ---
    battery = 100.0f;
    pos_x = 0.0f; pos_y = 0.0f; gamma = 0.0f;
    objetivo_x = 0.0f; objetivo_y = 0.0f;
    code = "-1"; 
    
    // -- Parámetros de sensores y percepción --
    front_dist_ = 10.0f; 
    frontal_cone_rad_ = 80.0 * (M_PI / 180.0);

    // -- Flags de control de objetivos --
    target_locked_ = false;
    current_target_pos_ = {0.0f, 0.0f};
    lin_vel_base = 0.9f; 
    umbral_distancia_meta = 0.25f; 
    
    // Controlador PID Angular
    pid_kp = 0.8f;   
    pid_kd = 2.9f;    
    prev_error = 0.0f;

    // Ganancia proporcional para aproximación a estación de carga
    kp_cargador = 0.5f; 

    // Parámetros VFF (Virtual Force Field)
    peso_atraccion = 1.0f;       
    peso_repulsion_max = 0.9f;   

    // Inicialización de Máquina de Estados (FSM)
    current_state_ = States::DECIDIR_OBJETIVO;
    objetivo_actual = TipoObjetivo::NINGUNO;

    fuerza_repulsion = {0.0f, 0.0f};
    f_att = {0.0f, 0.0f};

    reset_stuck_monitor();

    // --- Registro en el Sistema ---
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request->username = warrior_nick;
    
    while(!client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio...");
    }
    
    while (code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            code = response->code;
            if (code == "-1") {
                warrior_nick = "warrior_18_" + std::to_string(++cont);
                request->username = warrior_nick;
            } else {   
                // Inicialización de interfaces de comunicación con ID asignado
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 10 );
                pub2_ = create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + code + "/goal_x_y", 10 );
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 1, std::bind(&Racer::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 1, std::bind(&Racer::process_scene_info, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "RACER ID: %s", code.c_str());           
            }
        }
    }
}

Racer::~Racer() { RCLCPP_ERROR(this->get_logger(), "Destruido"); }

// Reinicia el temporizador y posición de referencia para detección de bloqueos
void Racer::reset_stuck_monitor()
{
    last_stuck_check_time_ = this->now();
    last_stuck_pos_x_ = pos_x;
    last_stuck_pos_y_ = pos_y;
}

// Detecta si el robot no ha avanzado significativamente en X segundos
bool Racer::check_if_stuck()
{
    auto current_time = this->now();
    if ((current_time - last_stuck_check_time_).seconds() > 4.0)
    {
        float dist = sqrt(pow(pos_x - last_stuck_pos_x_, 2) + pow(pos_y - last_stuck_pos_y_, 2));
        reset_stuck_monitor();
        if (dist < 0.15f) return true; // Umbral de movimiento mínimo
    }
    return false;
}

// Cálculo de Vector de Repulsión (VFF)
void Racer::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    last_scan_ = msg;
    fuerza_repulsion = {0.0f, 0.0f};
    
    float rango_seguridad = 0.7f; 
    float rango_tangencial = 0.35f; 
    int rayos_activos = 0;
    float min_dist = 100.0f;
    float min_ang = 0.0f;

    // Iteración sobre lecturas del láser para fuerza repulsiva
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {   
        float d = msg->ranges[i];
        float ang = msg->angle_min + i * msg->angle_increment;
        // Filtrado de ruido
        if (d < 0.05f || std::isinf(d)) continue;

        if (d < min_dist) { min_dist = d; min_ang = ang; }

        if (std::abs(ang) > (100.0 * M_PI / 180.0)) continue; // Ignorar obstáculos de detrás

        // Repulsión
        if (d < rango_seguridad) 
        {
            float fuerza = pow((rango_seguridad - d) / rango_seguridad, 2);
            fuerza_repulsion.f_x += -cos(ang) * fuerza;
            fuerza_repulsion.f_y += -sin(ang) * fuerza;
            rayos_activos++;
        }
    }

    // Normalización del vector resultante
    if (rayos_activos > 0)
    {
        float intensidad = std::min(1.0f, (float)sqrt(pow(fuerza_repulsion.f_x, 2) + pow(fuerza_repulsion.f_y, 2)));
        if (intensidad > 0.001f) {
             float current_mod = sqrt(pow(fuerza_repulsion.f_x, 2) + pow(fuerza_repulsion.f_y, 2));
             fuerza_repulsion.f_x = (fuerza_repulsion.f_x / current_mod) * intensidad * peso_repulsion_max;
             fuerza_repulsion.f_y = (fuerza_repulsion.f_y / current_mod) * intensidad * peso_repulsion_max;
        }
    }

    // Componente tangencial para evitar mínimos locales (deslizamiento en paredes)
    if (min_dist < rango_tangencial) {
        float tan_x, tan_y;
        if (min_ang < 0) { tan_x = -sin(min_ang); tan_y = cos(min_ang); } 
        else { tan_x = sin(min_ang); tan_y = -cos(min_ang); }
        float peso_tan = 7.5f; 
        fuerza_repulsion.f_x += tan_x * peso_tan;
        fuerza_repulsion.f_y += tan_y * peso_tan;
    }
    front_dist_ = GetMinInSector(msg, frontal_cone_rad_);
}

void Racer::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    std::string errs;
    
    if(Json::parseFromStream(reader, jsonStream, &JsonSceneData, &errs)) {
        battery = JsonSceneData["Battery_Level"].asFloat();
        pos_x = JsonSceneData["Robot_Pose"]["x"].asFloat();
        pos_y = JsonSceneData["Robot_Pose"]["y"].asFloat();
        gamma = JsonSceneData["Robot_Pose"]["gamma"].asFloat();

        chargers_pos_array.clear(); 
        coins_pos_array.clear();

        const Json::Value &chargers_pos = JsonSceneData["FOV"]["Chargers_Positions"];
        for (const Json::Value &charger : chargers_pos) {
            std::vector<float> data;
            for (const Json::Value &val : charger) data.push_back(val.asFloat());
            chargers_pos_array.push_back(data);
        }

        // Permite navegar a cargadores fuera del campo de visión
        if (!chargers_pos_array.empty()) {
            known_chargers_ = chargers_pos_array;
        }

        const Json::Value &coins_pos = JsonSceneData["FOV"]["Coins_Positions"];
        for (const Json::Value &coin : coins_pos) {
            std::vector<float> data;
            for (const Json::Value &val : coin) data.push_back(val.asFloat());
            coins_pos_array.push_back(data);
        }
    }
}

// Función de coste: J = distancia + 2 * alineación_angular
float Racer::calcula_coste(const std::vector<float>& pos)
{
    float dx = pos[0] - pos_x;
    float dy = pos[1] - pos_y;
    float dist = sqrt(dx*dx + dy*dy);
    
    float ang_to_target = atan2(dy, dx) - gamma;
    while (ang_to_target > M_PI) ang_to_target -= 2*M_PI;
    while (ang_to_target < -M_PI) ang_to_target += 2*M_PI;
    
    float costo_total = dist + (std::abs(ang_to_target) * 2.0f);
    return costo_total; 
}

// Verifica si hay obstáculos LIDAR en la línea de visión al objetivo
bool Racer::check_obstacle_in_path(const std::vector<float>& target_pos)
{
    if (!last_scan_) return false; 
    float dx = target_pos[0] - pos_x;
    float dy = target_pos[1] - pos_y;
    float dist_target = sqrt(dx*dx + dy*dy);
    float angle_target = atan2(dy, dx) - gamma;
    while (angle_target > M_PI) angle_target -= 2*M_PI;
    while (angle_target < -M_PI) angle_target += 2*M_PI;

    // Verificar si el objetivo está fuera del rango del sensor
    if (angle_target < last_scan_->angle_min || angle_target > last_scan_->angle_max) return false;

    int idx = (angle_target - last_scan_->angle_min) / last_scan_->angle_increment;
    int margen_idx = (25.0 * M_PI / 180.0) / last_scan_->angle_increment; // Margen de seguridad angular

    for (int k = -margen_idx; k <= margen_idx; k++) {
        int check_idx = idx + k;
        if (check_idx >= 0 && check_idx < (int)last_scan_->ranges.size()) {
            float range = last_scan_->ranges[check_idx];
            // Detección de obstáculo si está más cerca que el objetivo
            if (range > 0.05f && range < (dist_target - 0.1f)) return true;
        }
    }
    return false; 
}

// Densidad de monedas alrededor de un cargador
int count_coins_near_charger(const std::vector<float>& charger_pos, const std::vector<std::vector<float>>& coins) {
    int count = 0;
    for (const auto& coin : coins) {
        float d = sqrt(pow(coin[0] - charger_pos[0], 2) + pow(coin[1] - charger_pos[1], 2));
        if (d < 7.5f) count++;
    }
    return count;
}

bool Racer::necesito_cargar()
{
    // Mantener estado de carga hasta completar ciclo (95%)
    if (objetivo_actual == TipoObjetivo::BATERIA) {
        if (battery < 95.0f) return true; 
        return false; 
    }

    // Umbral crítico de seguridad
    if (battery < 15.0f) return true;

    const auto& cargadores_a_usar = known_chargers_.empty() ? chargers_pos_array : known_chargers_;

    if (!cargadores_a_usar.empty()) {
        float best_score = -1000.0f;
        float dist_to_best = 1000.0f;
        bool blocked_best = false;

        // Selección del mejor cargador basado en recompensa (monedas cercanas) vs distancia
        for(auto& c : cargadores_a_usar) {
            float d = sqrt(pow(c[0]-pos_x, 2) + pow(c[1]-pos_y, 2));
            int coins_nearby = count_coins_near_charger(c, coins_pos_array);
            float score = (coins_nearby * 5.0f) - d; 
            
            if (score > best_score) {
                best_score = score;
                dist_to_best = d;
                blocked_best = check_obstacle_in_path(c);
            }
        }

        // Estimación de coste energético del camino + penalización por obstáculos
        float coste_estimado = (dist_to_best * 4.0f) 
                             + (blocked_best ? 37.0f : 0.0f) 
                             + 20.0f; // Margen de seguridad

        if (battery < coste_estimado) return true;

    } else {
        if (battery < 25.0f) return true;
    }
    
    return false; 
}

void Racer::decidir_objetivo()
{
    const auto& cargadores_disponibles = known_chargers_.empty() ? chargers_pos_array : known_chargers_;

    bool urgencia_bateria = necesito_cargar();

    // Batería
    if (urgencia_bateria && !cargadores_disponibles.empty()) {
        objetivo_actual = TipoObjetivo::BATERIA;
        float best_score = -1000.0f;
        
        for (const auto &c : cargadores_disponibles) {
            float dist = sqrt(pow(c[0] - pos_x, 2) + pow(c[1] - pos_y, 2));
            int coins_nearby = count_coins_near_charger(c, coins_pos_array);
            // Valor de monedas cercanas vs coste de desplazamiento
            float score = (coins_nearby * 6.0f) - dist;
            if (score > best_score) {
                best_score = score;
                objetivo_x = c[0]; objetivo_y = c[1];
            }
        }
        objetivo_decidido = true;
        target_locked_ = true;
        current_target_pos_ = {objetivo_x, objetivo_y};
        return; 
    }

    // Mantenimiento de objetivo previo si es válido y no está bloqueado
    if (target_locked_) {

        if (check_obstacle_in_path(current_target_pos_)) {
            target_locked_ = false;
        } else {
            objetivo_x = current_target_pos_[0];
            objetivo_y = current_target_pos_[1];
            objetivo_decidido = true;
            return; 
        }
    }

    // Recolección de monedas
    if (!coins_pos_array.empty())
    {
        objetivo_actual = TipoObjetivo::MONEDA;
        float min_coste = 10000.0f;
        bool found = false;

        for (const auto &coin : coins_pos_array)
        {
            if (check_obstacle_in_path(coin)) continue; 
            float coste = calcula_coste(coin);
            
            if (coste < min_coste) {
                min_coste = coste;
                objetivo_x = coin[0]; objetivo_y = coin[1];
                found = true;
            }
        }
        
        if (found) {
            objetivo_decidido = true;
            target_locked_ = true;
            current_target_pos_ = {objetivo_x, objetivo_y};
            return;
        } 
        
        // Estrategia de salida: Si estoy en cargador y veo monedas, intentar salir
        bool estoy_en_cargador = false;
        if (!cargadores_disponibles.empty()) {
            for(auto& c : cargadores_disponibles) {
                if (sqrt(pow(c[0]-pos_x, 2) + pow(c[1]-pos_y, 2)) < 1.0f) {
                    estoy_en_cargador = true; break;
                }
            }
        }

        if (estoy_en_cargador) {
             min_coste = 10000.0f;
             for (const auto &coin : coins_pos_array) {
                 float coste = calcula_coste(coin); 
                 if (coste < min_coste) {
                     min_coste = coste;
                     objetivo_x = coin[0]; objetivo_y = coin[1];
                     found = true;
                 }
             }
             if (found) {
                 objetivo_decidido = true;
                 target_locked_ = true;
                 current_target_pos_ = {objetivo_x, objetivo_y};
                 return;
             }
        }
    } 

    // Ir a zona segura/cargador si no hay nada más
    if (!cargadores_disponibles.empty()) {
        objetivo_actual = TipoObjetivo::BATERIA; 
        float best_score = -1000.0f;
        for (const auto &c : cargadores_disponibles) {
            float dist = sqrt(pow(c[0] - pos_x, 2) + pow(c[1] - pos_y, 2));
            float score = -dist; // Elegir el más cercano
            if (score > best_score) {
                best_score = score;
                objetivo_x = c[0]; objetivo_y = c[1];
            }
        }
        objetivo_decidido = true;
        target_locked_ = true;
        current_target_pos_ = {objetivo_x, objetivo_y};
    } else {
        objetivo_decidido = false; 
        target_locked_ = false;
        objetivo_actual = TipoObjetivo::NINGUNO;
    }
}

// Cálculo del vector de atracción hacia el objetivo en coordenadas locales
Force Racer::calc_fuerza_atraccion()
{
    Force f = {0.0f, 0.0f};
    
    float target_x_local = objetivo_x;
    float target_y_local = objetivo_y;

    if (!objetivo_decidido) return f;

    float dx = target_x_local - pos_x;
    float dy = target_y_local - pos_y;
    // Transformación al marco de referencia del robot
    float dx_local = dx * cos(gamma) + dy * sin(gamma);
    float dy_local = -dx * sin(gamma) + dy * cos(gamma);
    float mod = sqrt(dx_local*dx_local + dy_local*dy_local);
    
    if (mod > 0.05f) {
        f.f_x = (dx_local / mod) * peso_atraccion;
        f.f_y = (dy_local / mod) * peso_atraccion;
    }
    return f;
}

void Racer::control_navegacion()
{
    f_att = calc_fuerza_atraccion();

    if (objetivo_actual == TipoObjetivo::BATERIA && !coins_pos_array.empty()) {
        for (const auto& coin : coins_pos_array) {
            float dx = coin[0] - pos_x;
            float dy = coin[1] - pos_y;
            float dist = sqrt(dx*dx + dy*dy);

            // Moneda muy cercana (< 2m)
            if (dist < 2.0f) {
                float angle_to_coin = atan2(dy, dx) - gamma;
                while (angle_to_coin > M_PI) angle_to_coin -= 2*M_PI;
                while (angle_to_coin < -M_PI) angle_to_coin += 2*M_PI;

                // Filtrado angular: Dentro del cono frontal (+/- 20 deg)
                if (std::abs(angle_to_coin) < (10.0 * M_PI / 180.0)) {
                    if (!check_obstacle_in_path(coin)) {                        
                        float dx_l = dx * cos(gamma) + dy * sin(gamma);
                        float dy_l = -dx * sin(gamma) + dy * cos(gamma);
                        float mod_l = sqrt(dx_l*dx_l + dy_l*dy_l);
                        
                        if (mod_l > 0.01f) {
                            f_att.f_x = (dx_l / mod_l) * 1.5f;
                            f_att.f_y = (dy_l / mod_l) * 1.5f;
                        }
                        break; // Atacar solo la primera oportunidad válida
                    }
                }
            }
        }
    }

    // Atracción + Repulsión
    float fx_total = f_att.f_x + fuerza_repulsion.f_x;
    float fy_total = f_att.f_y + fuerza_repulsion.f_y;
    
    // Control PID Angular
    float target_angle = atan2(fy_total, fx_total);
    while (target_angle > M_PI) target_angle -= 2 * M_PI;
    while (target_angle < -M_PI) target_angle += 2 * M_PI;
    error = target_angle; 
    float derivative = error - prev_error;
    float angular_cmd = (pid_kp * error) + (pid_kd * derivative); 
    prev_error = error;
    
    // Saturación de actuadores
    angular_cmd = std::max(-2.0f, std::min(2.0f, angular_cmd));
    
    // Control de velocidad lineal adaptativa
    float linear_cmd = lin_vel_base;
    float proximity_factor = std::clamp(front_dist_ / 1.3f, 0.1f, 1.0f);
    linear_cmd *= proximity_factor; // Frenado ante obstáculos
    if (std::abs(error) > 0.6f) linear_cmd *= 0.4f; // Frenado en giros cerrados

    // Aproximación a cargadores
    if (objetivo_actual == TipoObjetivo::BATERIA) {
        float dist = sqrt(pow(objetivo_x - pos_x, 2) + pow(objetivo_y - pos_y, 2));
        if (battery > 5.0f) {
            if (dist < 3.5f) {
                linear_cmd = kp_cargador * dist;
                linear_cmd = std::max(0.15f, std::min(linear_cmd, lin_vel_base)); 
            }
        } else {
            // Aproximación de emergencia
            linear_cmd = std::max(linear_cmd, 0.4f); 
        }
    }
    PublishVelocity(linear_cmd, angular_cmd);
}

// Bucle principal de la FSM
void Racer::FSM_Control_Loop()
{
    if (necesito_cargar() && objetivo_actual != TipoObjetivo::BATERIA) {
         target_locked_ = false;
         current_state_ = States::DECIDIR_OBJETIVO;
         reset_stuck_monitor();
    }

    switch (current_state_)
    {
    case States::DECIDIR_OBJETIVO:
        decidir_objetivo(); 
        if (objetivo_decidido) {
            prev_error = 0.0f; 
            current_state_ = States::NAVEGANDO;
            reset_stuck_monitor();
        } else { PublishVelocity(0.0, 0.5); }
        break;

    case States::NAVEGANDO:
        control_navegacion();
        if (he_llegado_a_objetivo()) {
            PublishVelocity(0.0, 0.0);
            target_locked_ = false; 
            if (objetivo_actual == TipoObjetivo::BATERIA) current_state_ = States::ESTOY_CARGANDO;
            else current_state_ = States::DECIDIR_OBJETIVO; 
        } else if (check_if_stuck()) {
            RCLCPP_ERROR(this->get_logger(), "ATASCADO");
            recovery_start_time_ = this->now();
            current_state_ = States::RECOVERY;
        }
        break;

    case States::RECOVERY:
        // Retroceso + Giro
        if (necesito_cargar() && objetivo_actual != TipoObjetivo::BATERIA) {
            current_state_ = States::DECIDIR_OBJETIVO;
            target_locked_ = false; 
            reset_stuck_monitor();
            break;
        }
        // Secuencia de recuperación
        if ((this->now() - recovery_start_time_).seconds() < 1.5) PublishVelocity(-0.23f, 0.0f);
        else if ((this->now() - recovery_start_time_).seconds() < 3.0) PublishVelocity(0.0f, 1.6f); 
        else {
            PublishVelocity(0.0, 0.0);
            current_state_ = States::DECIDIR_OBJETIVO;
            target_locked_ = false; 
            reset_stuck_monitor();
        }
        break;

    case States::ESTOY_CARGANDO:
        // Orientarse hacia monedas visibles
        if (!coins_pos_array.empty()) {
            float min_c = 10000.0f;
            float next_x = 0.0f, next_y = 0.0f;
            for (const auto &coin : coins_pos_array) {
                float d = sqrt(pow(coin[0]-pos_x, 2) + pow(coin[1]-pos_y, 2));
                if (d < min_c) { min_c = d; next_x = coin[0]; next_y = coin[1]; }
            }
            float dx = next_x - pos_x;
            float dy = next_y - pos_y;
            float target_ang = atan2(dy, dx) - gamma;
            while (target_ang > M_PI) target_ang -= 2*M_PI;
            while (target_ang < -M_PI) target_ang += 2*M_PI;
            float w_load = 1.0f * target_ang; // Control P simple
            w_load = std::max(-0.5f, std::min(0.5f, w_load)); 
            PublishVelocity(0.0, w_load); 
        } else { PublishVelocity(0.0, 0.0); }

        if (!necesito_cargar()) {
            current_state_ = States::DECIDIR_OBJETIVO;
            target_locked_ = false; 
            objetivo_actual = TipoObjetivo::NINGUNO; 
        }
        break;
    }
}

bool Racer::he_llegado_a_objetivo()
{
    float dist = sqrt(pow(objetivo_x - pos_x, 2) + pow(objetivo_y - pos_y, 2));
    
    if (dist < umbral_distancia_meta) {
        // Evitar falsos positivos por monedas de paso
        if (objetivo_actual == TipoObjetivo::BATERIA) {
             return true;
        }

        if (objetivo_actual == TipoObjetivo::MONEDA) RCLCPP_INFO(this->get_logger(), "Moneda conseguida!");
        target_locked_ = false; 
        return true;
    }
    return false;
}

// Distancia mínima en un sector angular específico del LIDAR
float Racer::GetMinInSector(const sensor_msgs::msg::LaserScan::SharedPtr msg, float angle_width_rad)
{
    float min_val = 10.0f;
    int center_idx = (msg->angle_max - msg->angle_min) / msg->angle_increment / 2;
    int range_width = (angle_width_rad / msg->angle_increment) / 2;
    for (int i = center_idx - range_width; i < center_idx + range_width; i++) {
        if (i >= 0 && i < (int)msg->ranges.size()) {
             float d = msg->ranges[i];
             if (d > 0.05 && d < min_val) min_val = d;
        }
    }
    return min_val;
}

void Racer::PublishVelocity(float vel_lineal, float vel_angular)
{
    rosgame_msgs::msg::RosgameTwist cmd;
    cmd.code = this->code;
    cmd.vel.linear.x = vel_lineal;
    cmd.vel.angular.z = vel_angular;
    pub1_->publish(cmd);
}

// ros2 launch warrior_18 racer_launch.xml
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Racer>();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    rclcpp::Rate rate(10); 
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->FSM_Control_Loop();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}