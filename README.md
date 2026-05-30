# 🤖 Rosgame Agents: Autonomous Navigation & Strategy

Este repositorio contiene el código fuente de dos agentes robóticos autónomos desarrollados para el entorno de simulación competitivo *Rosgame*. Los paquetes están programados en **C++** sobre **ROS 2** y demuestran la implementación de máquinas de estados (FSM), navegación híbrida, algoritmos reactivos (VFF y Gap Finding), control de bajo nivel (PID industrial) y toma de decisiones en tiempo real.

---

## ⚠️ Nota sobre Dependencias y Entorno
Este repositorio alberga de forma exclusiva la lógica de control de los agentes (`racer_18` y `warrior_18`). 

El entorno de simulación base (paquetes `rosgame_bridge`, `rosgame_msgs`, `rosgame_scenes`) es propiedad intelectual y académica de la Universidad de Málaga (UMA). Por respeto a dicha autoría y normativa, esos paquetes no se incluyen en este repositorio público. El código aquí presente está estructurado para permitir la auditoría técnica de los algoritmos de navegación y control por parte de terceros.

---

## 📦 Paquetes del Monorepo

### 1. `racer_18`: Navegación Híbrida y Recolección
Agente enfocado en la recolección eficiente de recursos (monedas) y gestión autónoma de energía.
* **Controlador Reactivo (VFF):** Implementa un algoritmo de Campos de Potencial Virtuales procesando los datos del `/laser_scan` (cono frontal de 200º). Genera vectores de repulsión cuadráticos y tangenciales para esquivar muros con un margen de seguridad de 0.7m.
* **Memoria Global (JSON Parsing):** Procesa la telemetría global de la escena para mantener una memoria de la posición de las estaciones de carga, permitiendo al robot tomar decisiones a largo plazo aunque los objetivos salgan de su campo de visión.
* **FSM y Watchdog:** Máquina de estados con 4 comportamientos (Decisión, Navegación, Carga, Recuperación). Incluye un monitor odómetrico que detecta atascos cinemáticos e inyecta maniobras de evasión automáticas.

### 2. `warrior_18`: Combate y Estrategia
Agente enfocado en la gestión estratégica de recursos (batería, martillos y escudos), supervivencia y sabotaje.
* **Navegación Determinista (Gap Finding):** Evoluciona de los campos de potencial a un algoritmo de búsqueda de huecos libres, garantizando trayectorias agresivas y precisas en combate.
* **Controlador PID Industrial:** Incorpora un lazo de control con Anti-Windup y un Filtro Pasa-Baja en la acción derivativa para suavizar el ruido del sensor LiDAR y asegurar giros letales sin vibraciones.
* **Trabajo en equipo y Guerra Electrónica:** La arquitectura emplea un `MultiThreadedExecutor` para ejecutar dos nodos simultáneos. Mientras el `Warrior` gestiona el combate, el nodo `Estratega` actúa como un sistema AWACS, espiando la red y lanzando ciberataques a los actuadores de los rivales.

---

## 🧠 Estrategias de Agentes Autónomos: Deep Dive

### 🏎️ Agente Racer: Navegación Óptima y Tolerancia a Fallos
Su núcleo de control se basa en un enfoque híbrido que fusiona el conocimiento global de los recursos con una navegación puramente reactiva:
* **Inmunidad a Mínimos Locales:** Para evitar que el robot se quede bloqueado en esquinas (el clásico problema de los VFF), se inyecta un vector tangencial artificial cuando un obstáculo se encuentra a menos de 0.35m. Esto fuerza al robot a "deslizarse" por la pared en lugar de detenerse.
* **Gestión Dinámica de Batería:** El robot no carga ciegamente. La función `necesito_cargar()` evalúa si la energía restante cubre el "coste de viaje" hacia el cargador más cercano, añadiendo penalizaciones numéricas si detecta obstáculos en la ruta mediante trazado de rayos.
* **Función de Coste de Rutas:** Para elegir la moneda óptima, el robot minimiza la ecuación $J = \text{distancia} + 2 \cdot |\text{alineación angular}|$. Esto prioriza objetivos frontales, ahorrando energía en giros cerrados.

### ⚔️ Agente Warrior: Máquina de Estados y Ciber-Guerra
Diseñado para el combate total, su supervivencia se basa en su FSM y en la explotación de vulnerabilidades en la red de ROS 2 del entorno:

**1. Lógica de Combate y Control Físico**
* **Selección Inteligente de Objetivos (`ACECHADOR`):** Evalúa a los rivales mediante un sistema de puntuación dinámico, penalizando a enemigos con escudo o agrupados para asegurar duelos favorables.
* **Modo `KAMIKAZE`:** Ignora la velocidad de crucero segura y sobrescribe el actuador lineal (1.6 m/s) solo cuando está perfectamente alineado con su objetivo.
* **Seguridad Activa (ECM):** Incluye una función `check_integrity` que vigila su propio tópico de velocidad (`/cmd_vel`). Si detecta comandos externos inyectados por un rival, publica ráfagas correctivas inmediatas para recuperar el control.

**2. El Nodo "Estratega" (Explotación del Entorno)**
La genialidad del equipo radica en saltarse las limitaciones del nodo árbitro (`rosgame_bridge`). Mientras que el entorno supone que los robots actúan en base a su propia percepción, el nodo `ESTRATEGA` intercepta las telemetrías y lanza ciberataques directos a los tópicos de control de los rivales (`/enemy_id/cmd_vel`).

* **Defensa Activa:** Si el Warrior es vulnerable y un enemigo se acerca a menos de 6.5m, inyecta comandos forzando al agresor a girar y alejarse (`lin: -2.5, ang: 1.4`).
* **Asistencia de Asesinato:** "Paraliza" a la presa enviándole velocidades casi nulas (`0.03`) justo antes del impacto Kamikaze.
* **Drenaje Oportunista:** Si un rival tiene batería crítica (<30%), le envía comandos de movimiento continuo para evitar que recargue y provocar su "muerte" energética.

---

## 🚀 Compilación y Ejecución

Al ser paquetes estándar de `ament_cmake`, se integran en cualquier *workspace* de ROS 2:

```bash
# 1. Navegar al workspace
cd ~/rosgame_ws

# 2. Compilar los agentes del monorepo
colcon build --packages-select racer_18 warrior_18

# 3. Cargar el entorno
source install/setup.bash

# 4. Ejecución
# (El lanzamiento completo requiere los paquetes de simulación de la UMA)
ros2 launch warrior_18 racer_launch.xml
# o
ros2 run warrior_18 warrior