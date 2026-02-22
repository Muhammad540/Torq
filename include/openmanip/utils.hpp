#ifndef OPENMANIP_UTILS_HPP
#define OPENMANIP_UTILS_HPP

#include <utility>
#include <pinocchio/multibody/model.hpp>

namespace openmanip{

    enum class ErrorCode {
        None = 0,
        FrameNotFound,
        NotWithinConfigurationLimits
    };
  
    inline std::pair<int,int> get_root_joint_dim(const pinocchio::Model& model){
      if (model.existJointName("root_joint")) {
            auto root_joint_id = model.getJointId("root_joint");
            const auto& root_joint = model.joints[root_joint_id];
        return {root_joint.nq(), root_joint.nv()};
      }
      return {0,0};
    }

    /* */
    // inline pinocchio::GeometryData process_collision_pairs(pinocchio::Model& model, pinocchio::GeometryModel& collision_model, std::string srdf_path){
    //   /*
    //     Process collision pairs.
    //   */

    //   collision_model.addAllCollisionPairs();
    //   if (srdf_path.empty()){
    //     pinocchio.removeCollisionPairs(model, collision_model, srdf_path);
    //   }
        
    //   // Collision models have been modified => re-generate corresponding data.
    //   auto collision_data = pinocchio::GeometryData(collision_model)
  
    //   // Enable contact detection for avoiding Nans at collisions
    //   collision_data.enable_contact = True
  
    //   return collision_data
    // }
} // namespace openmanip

#endif // OPENMANIP_UTILS_HPP