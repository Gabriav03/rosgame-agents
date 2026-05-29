# 🤖 Rosgame Agents: Autonomous Navigation & Strategy

Este repositorio contiene el código fuente de dos agentes robóticos autónomos desarrollados para el entorno de simulación competitivo *Rosgame*. Los paquetes están programados en **C++** sobre **ROS 2** y demuestran la implementación de máquinas de estados (FSM), navegación híbrida, algoritmos reactivos (VFF) y toma de decisiones en tiempo real.

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
Agente enfocado en la gestion estratégica de recursos (batería, martillos y escudos) y supervivencia.
* **Racer adaptado al combate:** Implementa las herramientas programadas en el racer y adaptadas a un entorno más hostil.
* **FSM y Watchdog:** Máquina de estados con 4 comportamientos (Granjero, Preparando, Acechador, Kamikaze). Además, detecta las interferencias causadas por los rivales que hayan esquivado al árbitro.
* **Trabajo en equipo:** En el combate no gana sólo el que gestiona mejor sus recursos, sino el que más información tiene para plantear una estrategia implacable. Mientras un nodo se encarga del combate, se está comunicando constantemente con otro nodo más informado y poderoso. 

---

# 🤖 Estrategias de Agentes Autónomos: Rosgame

Este documento detalla la arquitectura y las estrategias de decisión implementadas en los dos agentes robóticos desarrollados para el entorno de competición. Cada agente fue diseñado con un propósito distinto, utilizando desde algoritmos de navegación de bajo nivel hasta tácticas de "guerra electrónica" para manipular el entorno a su favor.

## 🏎️ Agente Racer: Navegación Óptima y Tolerancia a Fallos

El objetivo del **Racer** es la supervivencia a largo plazo mediante la recolección eficiente de monedas y la gestión milimétrica de su energía. Su núcleo de control se basa en un enfoque híbrido que fusiona el conocimiento global de los recursos con una navegación puramente reactiva.

* **Campos de Potencial Virtuales (VFF):** La evasión de obstáculos se calcula dinámicamente procesando los datos del láser frontal. Genera un vector de repulsión que aleja al robot de los muros.
* **Inmunidad a Mínimos Locales:** Para evitar que el robot se quede bloqueado en esquinas (el clásico problema de los VFF), se inyecta un vector tangencial artificial cuando un obstáculo se encuentra a menos de 0.35m. Esto fuerza al robot a "deslizarse" por la pared en lugar de detenerse.
* **Gestión Dinámica de Batería:** El robot no va a cargar ciegamente. La función `necesito_cargar()` evalúa si la energía restante es suficiente para cubrir el "coste de viaje" hacia el cargador más cercano, añadiendo penalizaciones numéricas si detecta obstáculos en la ruta mediante un trazado de rayos.
* **Función de Coste de Rutas:** Para elegir la moneda óptima, el robot minimiza la ecuación $J = \text{distancia} + 2 \cdot |\text{alineación angular}|$. Esto prioriza los objetivos que están frente al robot, evitando giros cerrados que consumen más energía y tiempo.
* **Watchdog Anti-Atascos:** Un monitor en segundo plano vigila la odometría; si el robot se desplaza menos de 0.15m en 4 segundos, asume un atasco físico (fricción o colisión no detectada) y activa el estado de emergencia `RECOVERY`, ejecutando una maniobra evasiva programada en bucle abierto.

---

## ⚔️ Agente Warrior: Máquina de Estados y Ciber-Guerra

El **Warrior** está diseñado para el combate total. Su supervivencia no solo depende de su Máquina de Estados Finitos (FSM), sino de un nodo auxiliar (`ESTRATEGA`) que interviene directamente en la red de ROS 2 de los rivales.

### 🧠 1. Lógica de Combate y FSM
El comportamiento del Warrior muta según su estado, sus habilidades adquiridas (cajas misteriosas) y su energía:
* **Selección Inteligente de Objetivos (`ACECHADOR`):** No ataca al robot más cercano, sino al más vulnerable. Evalúa a todos los rivales aplicando un sistema de puntuación que resta puntos si el enemigo tiene escudo, martillo, o si está agrupado con otros (evitando refriegas múltiples).
* **Modo `KAMIKAZE`:** Cuando el robot fija un objetivo y entra en distancia de impacto, ignora temporalmente la velocidad de crucero segura y sobrescribe el actuador lineal con una velocidad máxima absoluta de 1.6 m/s, garantizando el impacto.
* **Seguridad Activa (ECM):** Como contramedida, el propio Warrior incluye una función `check_integrity` que vigila su propio tópico de velocidad (`/cmd_vel`). Si detecta que recibe comandos que él mismo no ha generado, inyecta ráfagas correctivas para sobreescribir el ciberataque enemigo.

### 🕵️ 2. El Nodo "Estratega" (Explotación del Entorno)
La genialidad del equipo radica en saltarse las limitaciones del nodo árbitro (`rosgame_bridge`). Mientras que el entorno supone que los robots actúan en base a su propia percepción, el nodo `ESTRATEGA` intercepta las telemetrías y lanza ciberataques directos a los tópicos de control de los rivales (`/enemy_id/cmd_vel`).

Esta "guerra electrónica" opera en tres frentes:
1.  **Defensa Activa:** Si el Warrior está vulnerable (`GRANJERO` o `PREPARANDO`) y un enemigo se acerca demasiado, el Estratega le inyecta comandos para obligarlo a girar y alejarse (`lin: -2.5, ang: 1.4`).
2.  **Asistencia de Asesinato:** Cuando el Warrior entra en modo `KAMIKAZE`, el Estratega "paraliza" a la presa enviándole velocidades lineales casi nulas (`0.03`), asegurando que no pueda esquivar el impacto.
3.  **Drenaje Oportunista:** Si un rival tiene menos del 30% de batería, el Estratega le envía ráfagas de movimiento continuo (`lin: 0.5, ang: 0.2`) para evitar que pueda recargar y forzar su "muerte" por agotamiento energético.

---

## 🚀 Compilación

Al ser paquetes estándar de `ament_cmake`, se integran en cualquier *workspace* de ROS 2:

```bash
# Navegar al workspace
cd ~/rosgame_ws

# Compilar los agentes
colcon build --packages-select racer_18 warrior_18

# Cargar el entorno
source install/setup.bash