#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include <map>
#include <string>
#include <stdexcept>

struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
    LookupPose(const std::string& name, const BT::NodeConfig& config)
        : SyncActionNode(name, config)
    {
        // TODO: Naplňte tabulku souřadnic pose_table_.
        // Manipulátory: ID "1", "2", "3" (viz tabulka v zadání).
        // Sklady: ID "A1", "A2", "B1", "B2", "C1", "C2", "D1", "D2" (viz tabulka v zadání).
        // Příklad: pose_table_["1"] = { 4.5, 1.5 };
        // --- Manipulátory ---
        pose_table_["1"] = { 4.5,  1.5 };
        pose_table_["2"] = { 4.5, -0.5 };
        pose_table_["3"] = { 4.5, -2.5 };

        // --- Skladové pozice ---
        pose_table_["A1"] = { 1.5,  0.5 };
        pose_table_["A2"] = { 1.5, -1.5 };

        pose_table_["B1"] = { -0.5,  0.5 };
        pose_table_["B2"] = { -0.5, -1.5 };

        pose_table_["C1"] = { -2.5,  0.5 };
        pose_table_["C2"] = { -2.5, -1.5 };

        pose_table_["D1"] = { -4.5,  0.5 };
        pose_table_["D2"] = { -4.5, -1.5 };
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Location ID to look up (e.g. '1', 'A1')"),
            BT::OutputPort<double>("x", "Looked up X coordinate"),
            BT::OutputPort<double>("y", "Looked up Y coordinate")
        };
    }

    BT::NodeStatus tick() override
    {
        // TODO: Načtěte location_id z input portu pomocí getInput<std::string>.
        // Vyhledejte ID v pose_table_. Pokud neexistuje, vraťte FAILURE.
        // Zapište souřadnice do output portů "x" a "y" pomocí setOutput().
        // Vraťte SUCCESS.
        auto id = getInput<std::string>("location_id");
        if (!id) return BT::NodeStatus::FAILURE;
        auto it = pose_table_.find(id.value());
        if (it == pose_table_.end()) return BT::NodeStatus::FAILURE;
        setOutput("x", it->second.x);
        setOutput("y", it->second.y);
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::map<std::string, Pose2D> pose_table_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LookupPose>("LookupPose");
}
