#ifndef SPRING_JOINT_PLUGIN__SPRING_PLUGIN_HPP_
#define SPRING_JOINT_PLUGIN__SPRING_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <sdf/sdf.hh>
#include <memory>
#include <string>
#include <vector>

namespace spring_joint {

class SpringPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    SpringPlugin() = default;
    ~SpringPlugin() override = default;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override;

private:
    struct Spring {
        std::string parent_name;
        std::string child_name;
        
        gz::sim::Entity parent_entity{gz::sim::kNullEntity};
        gz::sim::Entity child_entity{gz::sim::kNullEntity};

        gz::math::Vector3d parent_point{gz::math::Vector3d::Zero};
        gz::math::Vector3d child_point{gz::math::Vector3d::Zero};

        double k_linear{0.0};
    };

    bool initialized_{false};
    std::vector<Spring> springs_;
};

}  // namespace spring_joint

#endif  // SPRING_JOINT_PLUGIN__SPRING_PLUGIN_HPP_