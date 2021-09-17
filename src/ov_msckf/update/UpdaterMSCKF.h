/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "feat/FeatureInitializerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"
#include <Eigen/Eigen>

#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <chrono>

namespace ov_msckf {

/**
 * @brief Will compute the system for our sparse features and update the filter.
 *
 * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
 * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
 * After this we compress all the measurements to have an efficient update and update the state.
 */
class UpdaterMSCKF {

public:
  /**
   * @brief Default constructor for our MSCKF updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterMSCKF(UpdaterOptions &options, FeatureInitializerOptions &feat_init_options) : _options(options) {

    // Save our raw pixel noise squared
    _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);

    // Save our feature initializer
    initializer_feat = std::unique_ptr<FeatureInitializer>(new FeatureInitializer(feat_init_options));
  }

  /**
   * @brief Given tracked features, this will try to use them to update the state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec);

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Feature initializer class object
  std::unique_ptr<FeatureInitializer> initializer_feat;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_H
