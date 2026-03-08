#ifndef TORQ_UTILS_HPP
#define TORQ_UTILS_HPP

#include <utility>
#include <pinocchio/multibody/model.hpp>

namespace torq{

    /**
     * @brief Error codes for non-exception error reporting.
     */
    enum class ErrorCode {
        None = 0,                    ///< No error.
        FrameNotFound,               ///< Requested frame does not exist in the model.
        NotWithinConfigurationLimits  ///< Current \f$q\f$ violates joint position limits.
    };
  
    /**
     * @brief Get the (nq, nv) dimensions of the root joint, or (0, 0) if there is no root joint.
     * @param model  Pinocchio model.
     * @return Pair (nq, nv) of the root joint.
     */
    inline std::pair<int,int> get_root_joint_dim(const pinocchio::Model& model){
      if (model.existJointName("root_joint")) {
            auto root_joint_id = model.getJointId("root_joint");
            const auto& root_joint = model.joints[root_joint_id];
        return {root_joint.nq(), root_joint.nv()};
      }
      return {0,0};
    }

    // NOTE: process_collision_pairs is commented out — will be implemented with Barriers.

} // namespace torq

#endif // TORQ_UTILS_HPP
