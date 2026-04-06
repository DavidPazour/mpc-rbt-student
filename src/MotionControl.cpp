#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

    // 1. Subscriber pro odometrii (pozice robota)
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

    // 2. Subscriber pro laserový skener (detekce kolizí)
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

    // 3. Publisher pro ovládání robota (rychlosti)
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 4. Client pro plánování cesty
    plan_client_ = create_client<nav_msgs::srv::GetPlan>("/planPath");

    // 5. Inicializace Action Serveru (pro navigaci na bod)
    using namespace std::placeholders;
    nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose",
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

void MotionControlNode::checkCollision() {
    // Bezpečnostní vzdálenost (např. 0.3 metru = 30 cm)
    double thresh = 0.3;

    // Pokud ještě nepřišla žádná data z laseru, nemá smysl nic kontrolovat
    if (laser_scan_.ranges.empty()) {
        return;
    }

    // Projdeme všechny naměřené vzdálenosti (paprsky laseru)
    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {

        // Zkontrolujeme, zda je hodnota platná (často laser vrací 0 nebo nekonečno u chyb měření)
        // a zda je menší než náš bezpečnostní limit.
        if (laser_scan_.ranges[i] > laser_scan_.range_min && laser_scan_.ranges[i] < thresh) {

            // PŘEKÁŽKA DETEKOVÁNA!
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;  // Zastav lineární pohyb
            stop_msg.angular.z = 0.0; // Zastav rotaci

            // Pošleme příkaz k zastavení motorům
            twist_publisher_->publish(stop_msg);

            // Vypíšeme varování do terminálu (throttle zajistí, že to nevypíše 100x za sekundu)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "KOLIZE DETEKOVÁNA! Zastavuji robota.");

            // Jakmile najdeme jednu překážku a zastavíme, nemusíme kontrolovat další paprsky
            return;
        }
    }
}

void MotionControlNode::updateTwist() {
    // 1. Je navigační akce aktivní a máme naplánovanou cestu?
    // goal_handle_->is_active() zajistí, že neposíláme rychlosti, pokud akce neprobíhá.
    if (!goal_handle_ || !goal_handle_->is_active() || path_.poses.empty()) {
        return;
    }

    // 2. Extrakce souřadnic cílového bodu (Look-ahead point)
    double target_x = path_.poses.front().pose.position.x;
    double target_y = path_.poses.front().pose.position.y;

    // 3. Extrakce aktuální pozice robota z odometrie
    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;

    // 4. Výpočet aktuálního natočení robota (Yaw) z Quaternionu
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, robot_yaw;
    m.getRPY(roll, pitch, robot_yaw);

    // 5. Výpočet požadovaného úhlu k cíli (atan2 nám dá absolutní úhel vektoru k cíli)
    double target_yaw = std::atan2(target_y - robot_y, target_x - robot_x);

    // Výpočet úhlové odchylky (Error)
    double angle_error = target_yaw - robot_yaw;

    // Normalizace odchylky na interval <-PI, PI> pro nejkratší možnou rotaci
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // --- 6. BEZPEČNÉ PARAMETRY A LIMITY ---
    double P_angular = 1.5;         // Proporcionální konstanta pro rotaci (P regulátor)
    double max_linear_vel = 0.2;    // Omezená dopředná rychlost [m/s] (max je teoreticky 1.0)
    double max_angular_vel = 0.8;   // Omezená rotační rychlost [rad/s]

    // --- 7. VÝPOČET RYCHLOSTÍ (KINETIKA) ---
    geometry_msgs::msg::Twist twist;

    // Aplikace P-regulátoru pro úhlovou rychlost s oříznutím na maximální povolenou hodnotu
    double cmd_angular_z = P_angular * angle_error;
    if (cmd_angular_z > max_angular_vel) cmd_angular_z = max_angular_vel;
    if (cmd_angular_z < -max_angular_vel) cmd_angular_z = -max_angular_vel;

    twist.angular.z = cmd_angular_z;

    // Logika dopředné rychlosti:
    // Pokud je robot hodně vytočený mimo směr k cíli, zastavíme ho, ať se nejdřív otočí na místě.
    // Tím se vyhneme širokým a nepřesným obloukům.
    if (std::abs(angle_error) > 0.5) {
        twist.linear.x = 0.0;
    }
    else {
        // Pokud směřujeme víceméně k cíli, zrychlujeme úměrně tomu, jak přesně míříme
        twist.linear.x = max_linear_vel * (1.0 - (std::abs(angle_error) / 0.5));
        if (twist.linear.x < 0.0) twist.linear.x = 0.0; // Pojistka proti couvání
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
    // V zadání máš 1 Hz, ale pro plynulé odmazávání bodů z cesty 
    // doporučuji alespoň 10 Hz (rychlejší reakce). 
    rclcpp::Rate loop_rate(10.0);

    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok()) {

        // --- 1. KONTROLA ZRUŠENÍ (CANCEL) ---
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(result);
            RCLCPP_INFO(get_logger(), "Navigace byla zrušena uživatelem.");

            // Bezpečnostní zastavení robota
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            return;
        }

        // --- 2. LOGIKA SLEDOVÁNÍ CESTY ---
        if (!path_.poses.empty()) {
            // Kde je aktuální cíl (tzv. Look-ahead bod)
            double target_x = path_.poses.front().pose.position.x;
            double target_y = path_.poses.front().pose.position.y;

            // Kde je robot
            double robot_x = current_pose_.pose.position.x;
            double robot_y = current_pose_.pose.position.y;

            // Výpočet vzdálenosti k tomuto bodu (Pythagorova věta)
            double distance_to_target = std::hypot(target_x - robot_x, target_y - robot_y);

            // Pokud jsme k bodu blíž než např. 0.2 metru, "sežereme" ho a jedeme na další
            if (distance_to_target < 0.2) {
                path_.poses.erase(path_.poses.begin()); // Smaže první prvek
            }

            // --- 3. PUBLIKOVÁNÍ FEEDBACKU ---
            // Aby RViz nebo uživatel věděl, že se něco děje
            feedback->distance_remaining = distance_to_target; // Můžeš doplnit sofistikovanější výpočet
            goal_handle_->publish_feedback(feedback);

        }
        else {
            // --- 4. JSME V CÍLI ---
            // Cesta je prázdná, dojeli jsme na konec

            // Zastavíme robota
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);

            // Oznámíme úspěch
            goal_handle_->succeed(result);
            RCLCPP_INFO(get_logger(), "Cíl úspěšně dosažen!");
            return; // Konec smyčky
        }

        // Uspíme vlákno na zbývající zlomek vteřiny
        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get(); // Získáme odpověď ze služby

    // Pokud máme cestu a není prázdná
    if (response && response->plan.poses.size() > 0) {

        // 1. Uložíme si cestu do naší globální proměnné, aby k ní mohl updateTwist()
        path_ = response->plan;

        // 2. Řekneme Action Serveru, že začínáme reálně vykonávat úkol
        goal_handle_->execute();

        RCLCPP_INFO(get_logger(), "Cesta nalezena. Začínám sledovat trajektorii.");

        // 3. Spustíme smyčku execute() v novém nezávislém vlákně
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
    // 1. NEJDŮLEŽITĚJŠÍ: Uložíme si aktuální polohu robota z odometrie do členské proměnné.
    // Bez tohohle by funkce updateTwist() a execute() nevěděly, kde robot zrovna je!
    current_pose_.pose = msg.pose.pose;

    // 2. Provedeme kontrolu okolí z laseru (bezpečnost)
    checkCollision();

    // 3. Vypočítáme odchylku a pošleme rychlosti do motorů
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // Vždy, když přijde nová zpráva z laseru, uložíme si ji
    laser_scan_ = msg;
}
