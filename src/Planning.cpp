#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map           - ukol 2
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path         - ukol 3
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));
        
        // Publisher for path      - ukol 4
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the map service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Map service not available, waiting again...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

        RCLCPP_INFO(get_logger(), "Trying to fetch map...");

        // Asynchronní odeslání s nabindovaným callbackem
        auto future = map_client_->async_send_request(
            request,
            std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response) {
            map_ = response->map;
            RCLCPP_INFO(get_logger(), "Map received successfully. Resolution: %.2f m/px", map_.info.resolution);

            // Po obdržení mapy ji rovnou upravíme (nafoukneme překážky)
            dilateMap();
        }
        else {
            RCLCPP_ERROR(get_logger(), "Service call failed: map response is empty.");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    RCLCPP_INFO(get_logger(), "Received path planning request.");

    // Hlavní logika: volání A* a následné vyhlazení
    aStar(request->start, request->goal);
    smoothPath();

    // Naplnění odpovědi služby a publikování na topic pro RViz
    response->plan = path_;
    path_pub_->publish(path_);

    RCLCPP_INFO(get_logger(), "Path planned and published.");
}

void PlanningNode::dilateMap() {
    if (map_.data.empty()) return;

    RCLCPP_INFO(get_logger(), "Starting map dilation...");
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    // Nastavení poloměru nafouknutí (počet buněk)
    int dilation_cells = 8;

    for (int y = 0; y < (int)map_.info.height; y++) {
        for (int x = 0; x < (int)map_.info.width; x++) {
            int idx = y * map_.info.width + x;

            // Pokud je v původní mapě překážka (100) nebo neznámo (-1)
            if (map_.data[idx] > 50 || map_.data[idx] == -1) {

                // Projdi okolí buňky
                for (int dy = -dilation_cells; dy <= dilation_cells; dy++) {
                    for (int dx = -dilation_cells; dx <= dilation_cells; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;

                        // Kontrola mezí mapy
                        if (nx >= 0 && nx < (int)map_.info.width && ny >= 0 && ny < (int)map_.info.height) {
                            int n_idx = ny * map_.info.width + nx;
                            dilatedMap.data[n_idx] = 100; // Označ jako překážku
                        }
                    }
                }
            }
        }
    }
    // Přepíšeme původní mapu nafouknutou verzí
    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Dilation finished.");
}


void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) {
    // 1. Inicializace cesty a kontrola mapy
    path_.poses.clear();
    path_.header.stamp = this->now();
    path_.header.frame_id = map_.header.frame_id;

    if (map_.data.empty()) {
        RCLCPP_ERROR(get_logger(), "Map is empty, cannot plan path!");
        return;
    }

    // 2. Převod metrických souřadnic [m] na indexy v mřížce [px]
    auto poseToGrid = [this](double world_x, double world_y, int& grid_x, int& grid_y) {
        grid_x = std::floor((world_x - map_.info.origin.position.x) / map_.info.resolution);
        grid_y = std::floor((world_y - map_.info.origin.position.y) / map_.info.resolution);
        };

    int start_x, start_y, goal_x, goal_y;
    poseToGrid(start.pose.position.x, start.pose.position.y, start_x, start_y);
    poseToGrid(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

    int start_idx = start_y * map_.info.width + start_x;
    int goal_idx = goal_y * map_.info.width + goal_x;
    
    RCLCPP_INFO(get_logger(), "Plánuji: StartGrid[%d,%d], GoalGrid[%d,%d]", start_x, start_y, goal_x, goal_y);
    RCLCPP_INFO(get_logger(), "Mapa info: OriginY: %.2f, Height: %d", map_.info.origin.position.y, map_.info.height);

    // Základní kontrola mezí
    if (start_x < 0 || start_x >= (int)map_.info.width || start_y < 0 || start_y >= (int)map_.info.height ||
        goal_x < 0 || goal_x >= (int)map_.info.width || goal_y < 0 || goal_y >= (int)map_.info.height) {
        RCLCPP_ERROR(get_logger(), "Start or goal out of map bounds!");
        return;
    }

    // 3. Příprava A*
    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.width * map_.info.height, false);

    auto startNode = std::make_shared<Cell>(start_x, start_y);
    startNode->g = 0;
    startNode->h = std::hypot(start_x - goal_x, start_y - goal_y);
    startNode->f = startNode->g + startNode->h;

    openList.push_back(startNode);

    // 4. Hlavní smyčka
    while (!openList.empty() && rclcpp::ok()) {
        // Najdi uzel s nejnižším f
        auto current_it = std::min_element(openList.begin(), openList.end(),
            [](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) { return a->f < b->f; });

        std::shared_ptr<Cell> current = *current_it;
        int cur_idx = current->y * map_.info.width + current->x;

        // Dosažení cíle
        if (current->x == goal_x && current->y == goal_y) {
            RCLCPP_INFO(get_logger(), "Goal reached, reconstructing path...");
            auto temp = current;
            while (temp != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_.header;
                pose.pose.position.x = temp->x * map_.info.resolution + map_.info.origin.position.x;
                pose.pose.position.y = temp->y * map_.info.resolution + map_.info.origin.position.y;
                path_.poses.push_back(pose);
                temp = temp->parent;
            }
            std::reverse(path_.poses.begin(), path_.poses.end());
            return;
        }

        openList.erase(current_it);
        closedList[cur_idx] = true;

        // Prozkoumání 8 sousedů
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                int nx = current->x + dx;
                int ny = current->y + dy;
                int n_idx = ny * map_.info.width + nx;

                // Kontrola mezí, překážek a uzavřeného seznamu
                if (nx < 0 || nx >= (int)map_.info.width || ny < 0 || ny >= (int)map_.info.height) continue;
                if (closedList[n_idx] || map_.data[n_idx] > 50 || map_.data[n_idx] == -1) continue;

                float step_cost = (dx != 0 && dy != 0) ? 1.414f : 1.0f;
                float new_g = current->g + step_cost;

                auto it = std::find_if(openList.begin(), openList.end(),
                    [nx, ny](const std::shared_ptr<Cell>& c) { return c->x == nx && c->y == ny; });

                if (it == openList.end()) {
                    auto neighbor = std::make_shared<Cell>(nx, ny);
                    neighbor->g = new_g;
                    neighbor->h = std::hypot(nx - goal_x, ny - goal_y);
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                    openList.push_back(neighbor);
                }
                else if (new_g < (*it)->g) {
                    (*it)->g = new_g;
                    (*it)->f = (*it)->g + (*it)->h;
                    (*it)->parent = current;
                }
            }
        }
    }
    RCLCPP_ERROR(get_logger(), "Unable to plan path: Open list is empty.");
}

void PlanningNode::smoothPath() {
    // Vyhlazování dává smysl pouze pro trasu s alespoň 3 body
    if (path_.poses.size() < 3) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Smoothing path...");

    // Vytvoříme pracovní kopii bodů trasy
    std::vector<geometry_msgs::msg::PoseStamped> smoothedPath = path_.poses;

    // Parametry algoritmu (vhodné pro mobilní roboty v mřížce)
    float weight_data = 0.5f;   // Jak moc držet bod u původní pozice
    float weight_smooth = 0.5f; // Jak moc vyhladit (přitáhnout k sousedům)
    float tolerance = 0.001f;   // Minimální změna pro ukončení iterací

    float change = tolerance;
    int max_iterations = 1000;  // Pojistka proti nekonečné smyčce
    int iter = 0;

    while (change >= tolerance && iter < max_iterations) {
        change = 0.0f;

        // Iterujeme přes všechny body kromě prvního (start) a posledního (cíl)
        for (size_t i = 1; i < path_.poses.size() - 1; i++) {
            float x_old = smoothedPath[i].pose.position.x;
            float y_old = smoothedPath[i].pose.position.y;

            // Gradientní posun pro X
            smoothedPath[i].pose.position.x += weight_data * (path_.poses[i].pose.position.x - smoothedPath[i].pose.position.x) +
                weight_smooth * (smoothedPath[i - 1].pose.position.x + smoothedPath[i + 1].pose.position.x - 2.0f * smoothedPath[i].pose.position.x);

            // Gradientní posun pro Y
            smoothedPath[i].pose.position.y += weight_data * (path_.poses[i].pose.position.y - smoothedPath[i].pose.position.y) +
                weight_smooth * (smoothedPath[i - 1].pose.position.y + smoothedPath[i + 1].pose.position.y - 2.0f * smoothedPath[i].pose.position.y);

            // Výpočet celkové změny v této iteraci
            change += std::abs(x_old - smoothedPath[i].pose.position.x) + std::abs(y_old - smoothedPath[i].pose.position.y);
        }
        iter++;
    }

    // Aktualizace cesty vyhlazenými body
    path_.poses = smoothedPath;
    RCLCPP_INFO(get_logger(), "Smoothing finished after %d iterations.", iter);
}

Cell::Cell(int c, int r) : x(c), y(r), f(0.0f), g(1e9f), h(0.0f), parent(nullptr) {}
