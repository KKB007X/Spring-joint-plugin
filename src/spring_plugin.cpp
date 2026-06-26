#include "spring_joint_plugin/spring_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <iostream>

namespace spring_joint {

void SpringPlugin::Configure(const gz::sim::Entity &,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
    auto sdf = const_cast<sdf::Element*>(_sdf.get());
    if (!sdf->HasElement("spring")) {
        std::cerr << "[SpringPlugin] Error: No <spring> elements defined inside configuration!" << std::endl;
        return;
    }

    for (auto spring_element = sdf->GetElement("spring"); spring_element; spring_element = spring_element->GetNextElement("spring")) {
        Spring spring;
        
        spring.parent_name = spring_element->Get<std::string>("parent_link");
        spring.child_name = spring_element->Get<std::string>("child_link");
        spring.k_linear = spring_element->Get<double>("k_linear", 0.0).first;

        if (spring_element->HasElement("parent_point")) {
            spring.parent_point = spring_element->Get<gz::math::Vector3d>("parent_point");
        }
        if (spring_element->HasElement("child_point")) {
            spring.child_point = spring_element->Get<gz::math::Vector3d>("child_point");
        }

        _ecm.Each<gz::sim::components::Name>([&](const gz::sim::Entity &entity, const gz::sim::components::Name *name) {
            if (name->Data() == spring.parent_name) {
                spring.parent_entity = entity;
            } else if (name->Data() == spring.child_name) {
                spring.child_entity = entity;
            }
            return true;
        });

        if (spring.parent_entity == gz::sim::kNullEntity || spring.child_entity == gz::sim::kNullEntity) {
            std::cerr << "[SpringPlugin] Error: Failure mapping entities for links: " 
                      << spring.parent_name << " <-> " << spring.child_name << std::endl;
            continue;
        }

        springs_.push_back(spring);
        std::cout << "[SpringPlugin] Instantiated pure linear spring between "
                  << spring.parent_name << " and " << spring.child_name << std::endl;
    }

    initialized_ = !springs_.empty();
}

void SpringPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
    if (!initialized_ || _info.paused) return;

    for (auto &spring : springs_) {
        gz::sim::Link p_link(spring.parent_entity);
        gz::sim::Link c_link(spring.child_entity);

        auto p_pose = p_link.WorldPose(_ecm);
        auto c_pose = c_link.WorldPose(_ecm);
        if (!p_pose || !c_pose) continue;

        // Map local anchor attachments to global world coordinates
        gz::math::Vector3d p_anchor_world = p_pose->Pos() + p_pose->Rot().RotateVector(spring.parent_point);
        gz::math::Vector3d c_anchor_world = c_pose->Pos() + c_pose->Rot().RotateVector(spring.child_point);

        // Calculate world space linear displacement error
        gz::math::Vector3d linear_displacement = p_anchor_world - c_anchor_world;
        
        // F = -k * x (No rotational components, zero torque vectors)
        gz::math::Vector3d spring_force = -spring.k_linear * linear_displacement;

        // Apply forces at their respective attachment point offsets
        p_link.AddWorldWrench(_ecm, spring_force, gz::math::Vector3d::Zero, spring.parent_point);
        c_link.AddWorldWrench(_ecm, -spring_force, gz::math::Vector3d::Zero, spring.child_point);
    }
}

}  // namespace spring_joint

GZ_ADD_PLUGIN(
    spring_joint::SpringPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)