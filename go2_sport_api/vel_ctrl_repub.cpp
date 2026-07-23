#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cmath>

#include "common/ros2_sport_client.h"
#include "unitree_api/msg/request.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>   // Nav2 (Humble) publishes a PLAIN Twist on /cmd_vel

rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_suber;  // NEW: Nav2 commands

unitree_api::msg::Request req;
SportClient sport_req;

// --- Joystick state (HUMAN control), filled by joystickHandler ---
float joySpeed = 0;         // forward, normalized [-1,1] treated as m/s
float joySpeedYaw = 0;      // turn, rad/s (already scaled)
float joySpeedLateral = 0;  // strafe, m/s (already scaled)
bool  checkObstacle = true;

// --- Nav2 command state (AUTONOMOUS control), filled by vel_cmd_callback ---
float  cmdVx = 0, cmdVy = 0, cmdVyaw = 0;  // straight from /cmd_vel (m/s, m/s, rad/s)
double lastCmdTime = 0;                     // seconds; when the last /cmd_vel arrived

// --- Final body velocity actually sent to the legs ---
float vx = 0, vy = 0, vyaw = 0;

rclcpp::Node::SharedPtr nh;

float PI = 3.141592653589397;
float maxSpeedYaw = 1.4;
float maxSpeedLateral = 0.5;

// SAFETY 1: if Nav2 goes silent (crash / network drop) longer than this, STOP the robot.
const double CMD_TIMEOUT = 0.5;   // seconds
// SAFETY 2: ignore joystick noise around center, else autonomy would never engage.
const float  JOY_DEADZONE = 0.05f;

// Nav2 -> here. Only CACHE the latest command + its arrival time; decision is made in the loop.
void vel_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    cmdVx   = msg->linear.x;    // forward m/s (RPP cruises at desired_linear_vel)
    cmdVy   = msg->linear.y;    // strafe m/s (0 from RPP for now; passed through)
    cmdVyaw = msg->angular.z;   // turn rad/s
    lastCmdTime = nh->now().seconds();
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
    joySpeed        = joy->axes[4];
    joySpeedLateral = joy->axes[3] * maxSpeedLateral;
    joySpeedYaw     = joy->axes[0] * maxSpeedYaw;

    if (joySpeed >  1.0) joySpeed =  1.0;
    if (joySpeed < -1.0) joySpeed = -1.0;
    if (joySpeedLateral >  maxSpeedLateral) joySpeedLateral =  maxSpeedLateral;
    if (joySpeedLateral < -maxSpeedLateral) joySpeedLateral = -maxSpeedLateral;
    if (joy->axes[4] == 0) joySpeed = 0;

    // right trigger (axes[5]) keeps the CMU obstacle-check flag (unchanged behavior)
    checkObstacle = (joy->axes[5] > -0.1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("vel_cmd_repub");

    joy_suber = nh->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, joystickHandler);
    // NEW: subscribe to Nav2's velocity commands.
    vel_cmd_suber = nh->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, vel_cmd_callback);
    req_puber = nh->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);

    rclcpp::Rate rate(100);   // push a command to the legs at 100 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(nh);   // process any joy / cmd_vel callbacks that arrived

        double now = nh->now().seconds();
        bool joyActive = (std::fabs(joySpeed)        > JOY_DEADZONE ||
                          std::fabs(joySpeedYaw)     > JOY_DEADZONE ||
                          std::fabs(joySpeedLateral) > JOY_DEADZONE);
        bool cmdFresh  = (now - lastCmdTime) < CMD_TIMEOUT;

        // ARBITRATION (priority, safest first):
        //   1) joystick moving  -> HUMAN OVERRIDE (grab a stick to take control any time)
        //   2) else fresh Nav2  -> AUTONOMOUS (follow /cmd_vel)
        //   3) else             -> STOP (Nav2 silent AND sticks centered)
        const char *mode;
        if (joyActive) {
            vx = joySpeed;  vy = joySpeedLateral;  vyaw = joySpeedYaw;  mode = "JOY";
        } else if (cmdFresh) {
            vx = cmdVx;     vy = cmdVy;            vyaw = cmdVyaw;      mode = "NAV2";
        } else {
            vx = 0;         vy = 0;                vyaw = 0;            mode = "STOP";
        }

        // ⭐ ПОМОЩЬ ПРИ РАЗВОРОТЕ НА МЕСТЕ (особенность этой собаки, 2026-07-23).
        //
        // Nav2 (RPP) на развороте шлёт СТРОГО vx=0 + angular.z — у него в
        // rotateToHeading() линейная скорость захардкожена в 0.0, параметра нет.
        // Наша собака на такую команду НЕ РАЗВОРАЧИВАЕТСЯ: лапы застревают,
        // цель зависает, а /cmd_vel при этом выглядит совершенно живым.
        //
        // Лечим здесь, а не в конфиге Nav2: это свойство конкретного робота,
        // а не логики планирования. Добавляем маленький «вперёд» ТОЛЬКО когда
        // команда — чистое вращение (есть поворот, нет хода). Прямое движение
        // и остановка не затрагиваются.
        //
        // ⚠️ Если собака при развороте слишком уезжает вперёд — уменьшить.
        //    Если всё ещё застревает — увеличить (но помни: слишком МАЛЕНЬКИЕ
        //    скорости на этой собаке хуже, чем нормальные, см. 0.15 м/с).
        const float ROTATE_ASSIST_VX = 0.10f;
        if (vx == 0 && vy == 0 && vyaw != 0) {
            vx = ROTATE_ASSIST_VX;
        }

        // ⭐ ПРИ НУЛЕВОЙ КОМАНДЕ ШЛЁМ StopMove, А НЕ Move(0,0,0).
        //
        // Так делает pathFollower.cpp (CMU, строки 431-437) — единственный код в этом
        // репозитории, который реально ездил на живой Go2. Наш мост раньше слал
        // Move(0,0,0) непрерывно, и это единственное содержательное отличие от рабочего
        // образца (частота у обоих сопоставима: у CMU 50 Гц, у нас 100).
        //
        // Почему это важно, а не косметика (найдено 2026-07-23 на живом роботе):
        // мануал Unitree прямо предупреждает, что одновременная отправка
        // высокоуровневых API-команд и команд с пульта ведёт к нестабильности.
        // Непрерывный Move(0,0,0) — это именно постоянная команда ДВИЖЕНИЯ в стоящего
        // робота, которая конфликтует с пультом. За сессию собака дважды сложилась
        // (моторы отключались) при запущенном мосте и включённом пульте.
        // StopMove — команда ОСТАНОВКИ, она не держит робота в режиме локомоции.
        if (vx == 0 && vy == 0 && vyaw == 0) {
            sport_req.StopMove(req);
        } else {
            sport_req.Move(req, vx, vy, vyaw);   // body velocity -> gait controller -> legs
        }
        req_puber->publish(req);

        static int pc = 0;                    // print ~5 Hz, not 100 Hz (avoid console spam)
        if (++pc % 20 == 0)
            std::cout << "[" << mode << "] vx:" << vx << " vy:" << vy << " vyaw:" << vyaw << std::endl;

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
