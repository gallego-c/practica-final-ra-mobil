#ifndef CTF_NAVIGATION_PLANNER_5D_H
#define CTF_NAVIGATION_PLANNER_5D_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <vector>

namespace ctf_navigation
{

// Estado del robot en el espacio 5D
struct State5D
{
    double x;      // posición x en el mapa
    double y;      // posición y en el mapa
    double theta;  // orientación
    double v;      // velocidad lineal
    double omega;  // velocidad angular
};

// Nodo del árbol de búsqueda A*
struct SearchNode
{
    State5D state;
    double  cost;      // coste acumulado desde el inicio (g)
    double  heuristic; // estimación al goal (h)
    int     parent;    // índice del nodo padre (-1 si es raíz)

    double total() const { return cost + heuristic; }

    bool operator>(const SearchNode& o) const
    {
        return total() > o.total();
    }
};

class Planner5D : public nav_core::BaseLocalPlanner
{
public:
    Planner5D();
    ~Planner5D() {}

    // ── Interfaz obligatoria de nav_core::BaseLocalPlanner ──────────────────

    void initialize(std::string name,
                    tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros) override;

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

    bool isGoalReached() override;

private:

    // Simula un paso de integración cinemática (modelo diferencial)
    State5D simulate(const State5D& s,
                     double v, double omega, double dt) const;

    // Heurística: distancia euclídea al goal + penalización de orientación
    double heuristic(const State5D& s) const;

    // Comprueba si el estado colisiona con el costmap
    bool isCollision(const State5D& s) const;

    // Extrae la pose actual del robot desde el costmap
    State5D getCurrentState() const;

    // ── Parámetros cargados desde ROS params ────────────────────────────────
    double max_vel_x_;
    double min_vel_x_;
    double max_vel_theta_;
    double acc_lim_x_;
    double acc_lim_theta_;
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
    double sim_time_;       // horizonte de simulación en segundos
    double sim_step_;       // paso de integración en segundos
    int    v_samples_;      // número de muestras de velocidad lineal
    int    omega_samples_;  // número de muestras de velocidad angular
    int    max_iterations_; // límite máximo de nodos A* expandidos
    double path_follow_weight_;
    double proximity_weight_;

    // ── Estado interno ───────────────────────────────────────────────────────
    bool initialized_;
    bool goal_reached_;

    // Escape sequence when no rollout is valid: backup, then rotate.
    int             escape_mode_;
    int             rotation_sign_;
    ros::Time       last_escape_switch_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped              goal_pose_;

    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer*          tf_;
};

} // namespace ctf_navigation

#endif // CTF_NAVIGATION_PLANNER_5D_H
