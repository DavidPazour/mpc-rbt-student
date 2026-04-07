#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

    // 1. Subscriber pro odometrii (pozice robota)
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

   // 2. Subscriber pro laserový skener (detekce kolizí)
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/tiago_base/Hokuyo_URG_04LX_UG01", rclcpp::SensorDataQoS(), 
    std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

    // 3. Publisher pro ovládání robota (rychlosti)
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 4. Client pro plánování cesty
    plan_client_ = create_client<nav_msgs::srv::GetPlan>("/plan_path");

    // 5. Inicializace Action Serveru (pro navigaci na bod)
    using namespace std::placeholders;
    go_to_goal = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this,
        "go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, _1, _2),
        std::bind(&MotionControlNode::navHandleCancel, this, _1),
        std::bind(&MotionControlNode::navHandleAccepted, this, _1));

    RCLCPP_INFO(get_logger(), "Motion control node started.");

    // 6. Čekání na dostupnost plánovací služby
    while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for service /plan_path to be available...");
    }
}

bool MotionControlNode::checkCollision() {
    double thresh = 0.3; // Bezpečnostní vzdálenost 

    if (laser_scan_.ranges.empty()) return false;

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        float r = laser_scan_.ranges[i];
        float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;

        // Kontrola pouze předního kužele
        if (angle > -0.78 && angle < 0.78) {        //pouze cone -45 + 45
            if (std::isinf(r) || std::isnan(r)) continue;

            if (r <= thresh) { 
                return true; // je pred nami zed
            }
        }
    }
    return false; // Cesta vpřed je volná
}

void MotionControlNode::updateTwist() {
    if (!goal_handle_ || !goal_handle_->is_active() || path_.poses.empty()) {
        return;
    }

    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;

    // 1. Získání aktuálního úhlu robota (Yaw)
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, robot_yaw;
    m.getRPY(roll, pitch, robot_yaw);

    // --- PARAMETRY PURE PURSUIT ---
    double Ld = 0.4;                // Look-ahead distance (40 cm před robota)
    double max_linear_vel = 0.4;    // Max rychlost vpřed
    double max_angular_vel = 0.8;   // Max rotace

    // 2. NALEZENÍ LOOK-AHEAD BODU
    // Jako default vezmeme úplně poslední bod cesty (kdyby náhodou byly všechny moc blízko)
    double target_x = path_.poses.back().pose.position.x;
    double target_y = path_.poses.back().pose.position.y;

    // Projdeme celou cestu a najdeme PRVNÍ bod, který je od nás dál než naše kružnice Ld
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double px = path_.poses[i].pose.position.x;
        double py = path_.poses[i].pose.position.y;
        double dist = std::hypot(px - robot_x, py - robot_y);

        if (dist >= Ld) {
            target_x = px;
            target_y = py;
            break; // Našli jsme náš bod na kružnici, končíme hledání
        }
    }

// 3. PŘEVOD DO LOKÁLNÍCH SOUŘADNIC ROBOTA (Získání x_G a y_G)
    // Nejdřív spočítáme rozdíly v globálních souřadnicích
    double dx_global = target_x - robot_x;
    double dy_global = target_y - robot_y;

    // Aplikujeme 2D rotační matici podle aktuálního natočení robota (yaw).
    // x_G je vzdálenost cíle PŘED/ZA robotem (kladná = před, záporná = za)
    // y_G je vzdálenost cíle VLEVO/VPRAVO (kladná = vlevo, záporná = vpravo)
    double x_G = std::cos(robot_yaw) * dx_global + std::sin(robot_yaw) * dy_global;
    double y_G = -std::sin(robot_yaw) * dx_global + std::cos(robot_yaw) * dy_global;

    double l_sq = Ld * Ld; 

    geometry_msgs::msg::Twist twist;
    bool obstacle_ahead = checkCollision();

    // 4. CHYTRÁ LOGIKA A KINETIKA (PURE PURSUIT PODLE SLAJDŮ)
    
    // Spočítáme si úhel k look-ahead bodu v lokálních souřadnicích.
    // atan2(Y, X) nám rovnou řekne, o kolik radiánů se musíme otočit.
    double alpha = std::atan2(y_G, x_G);

    // Pokud je bod o více než 0.5 radiánu (~30 stupňů) mimo naši osu,
    // NEBO pokud je cíl úplně za námi (alpha se blíží PI), točíme se jako tank!
    if (std::abs(alpha) > 0.5) { 
        twist.linear.x = 0.0;
        // Pokud je alpha kladná (cíl vlevo), točíme se doleva, jinak doprava
        twist.angular.z = (alpha > 0) ? max_angular_vel : -max_angular_vel;
    } 
    else {
        // Cesta je víceméně před námi (jsme natočeni k cíli)
        if (obstacle_ahead) {
            geometry_msgs::msg::Twist stop_msg;
            twist_publisher_->publish(stop_msg);
            if (goal_handle_ && goal_handle_->is_active()) {
                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
                RCLCPP_WARN(get_logger(), "!!! KOLIZE VPŘED! Nelze pokračovat k cíli. ZASTAVUJI !!!");
            }
            return;
        } 
        else {
            // Zpomalení v prudších zatáčkách (čím větší y_G, tím pomaleji jede)
            twist.linear.x = max_linear_vel * (1.0 - (std::abs(y_G) / Ld));
            if (twist.linear.x < 0.05) twist.linear.x = 0.05; // Aby nezastavil úplně

            // TOTO JE TVŮJ VZOREC ZE SLAJDŮ: omega = (2 * v * y_G) / l^2
            twist.angular.z = (2.0 * twist.linear.x * y_G) / l_sq;

            // Zastropování maximální rotace
            if (twist.angular.z > max_angular_vel) twist.angular.z = max_angular_vel;
            if (twist.angular.z < -max_angular_vel) twist.angular.z = -max_angular_vel;
        }
    }

    // 8. Odeslání vypočítaných rychlostí
    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Přijat nový požadavek na cíl: [X: %.2f, Y: %.2f]",
        goal->pose.pose.position.x, goal->pose.pose.position.y);

    (void)uuid; // Tímto kompilátoru řekneme, že víme, že proměnnou nepoužíváme (aby neházel varování)

    // Pro náš úkol přijmeme každý cíl.
    // Kdybys chtěl robota omezit (např. cíl je mimo mapu), vrátil bys REJECT.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Přijat požadavek na zrušení aktuálního cíle.");
    (void)goal_handle;

    // Umožníme bezproblémové zrušení akce
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // Uložíme si handle cíle, abychom s ním mohli pracovat v execute()
    goal_handle_ = goal_handle;

    // Vytvoříme požadavek pro  /plan_path
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();

    // Nastavíme start (aktuální poloha z odometrie) a cíl (z akce)
    request->start.pose = current_pose_.pose;; // Předpokládá se, že v odomCallback ukládáš do odom_
    request->goal = goal_handle->get_goal()->pose;

    // ASYNCHRONNÍ VOLÁNÍ:
    // Tento řádek "vystřelí" dotaz na tvůj PlanningNode.
    // Jakmile PlanningNode odpoví, automaticky se zavolá funkce pathCallback.
    auto future = plan_client_->async_send_request(
        request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Sending request to planning service...");
}

void MotionControlNode::execute() {
    RCLCPP_INFO(get_logger(), "Pure Pursuit execute() spuštěn!");
    rclcpp::Rate loop_rate(10.0);

    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok()) {
        if (!goal_handle_->is_active()) return;
        
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(result);
            RCLCPP_INFO(get_logger(), "Navigace zrušena.");
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            return;
        }

        if (!path_.poses.empty()) {
            double robot_x = current_pose_.pose.position.x;
            double robot_y = current_pose_.pose.position.y;

            // HLEDÁME KONEČNÝ CÍL (poslední prvek v poli)
            double final_x = path_.poses.back().pose.position.x;
            double final_y = path_.poses.back().pose.position.y;
            double distance_to_goal = std::hypot(final_x - robot_x, final_y - robot_y);

            feedback->distance_remaining = distance_to_goal;
            goal_handle_->publish_feedback(feedback);

            // JSME V CÍLI? (Tolerance 0.35 metru od finálního bodu)
            if (distance_to_goal < 0.35) {
                geometry_msgs::msg::Twist stop;
                twist_publisher_->publish(stop);
                goal_handle_->succeed(result);
                RCLCPP_INFO(get_logger(), "Cíl úspěšně dosažen Pure Pursuit algoritmem!");
                return;
            }

            // ODMAZÁVÁNÍ "PROJETÝCH" BODŮ (Údržba pole)
            // Najdeme bod na cestě, který je k nám úplně nejblíž, a vše před ním smažeme
            double min_dist = 1e9; // Velké číslo pro začátek
            size_t closest_idx = 0;
            for (size_t i = 0; i < path_.poses.size(); ++i) {
                double d = std::hypot(path_.poses[i].pose.position.x - robot_x, 
                                      path_.poses[i].pose.position.y - robot_y);
                if (d < min_dist) {
                    min_dist = d;
                    closest_idx = i;
                }
            }
            
            // Smažeme všechny body, které jsme už bezpečně minuli
            if (closest_idx > 0) {
                path_.poses.erase(path_.poses.begin(), path_.poses.begin() + closest_idx);
            }

        } else {
            goal_handle_->succeed(result);
            return;
        }

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get(); // Získáme odpověď ze služby

    RCLCPP_INFO(get_logger(), "pathCallback zavolán, poses: %zu", 
        response ? response->plan.poses.size() : 0);

    // Pokud máme cestu a není prázdná
    if (response && response->plan.poses.size() > 0) {

        // 1. Uložíme si cestu do naší globální proměnné, aby k ní mohl updateTwist()
        path_ = response->plan;

        RCLCPP_INFO(get_logger(), "Cesta nalezena. Začínám sledovat trajektorii.");

        // 2. Spustíme smyčku execute() v novém nezávislém vlákně
        std::thread(&MotionControlNode::execute, this).detach();

    }
    else {
        // Pokud plánovač cestu nenašel, úkol přerušíme
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
        RCLCPP_ERROR(get_logger(), "Nepodařilo se najít cestu k cíli!");
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry& msg) {
    current_pose_.pose = msg.pose.pose;
    
    // Žádné ify. Necháme rozhodnutí čistě na mozku.
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "LIDAR FUNGUJE! Vidim %zu paprsku.", msg.ranges.size());
    // Vždy, když přijde nová zpráva z laseru, uložíme si ji
    laser_scan_ = msg;
}
