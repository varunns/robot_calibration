/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <robot_calibration/ceres/optimizer.h>

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/camera3d_to_arm_error.h>
#include <robot_calibration/ceres/data_functions.h>
#include <robot_calibration/ceres/outrageous_error.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

namespace robot_calibration
{

Optimizer::Optimizer(const std::string& robot_description)
{
  if (!model_.initString(robot_description))
    std::cerr << "Failed to parse URDF." << std::endl;
}

Optimizer::~Optimizer()
{
  if (free_params_)
    delete[] free_params_;
  if (problem_)
    delete problem_;
  if (offsets_)
    delete offsets_;
}

int Optimizer::optimize(OptimizationParams& params,
                        std::vector<robot_calibration_msgs::CalibrationData> data,
                        bool progress_to_stdout)
{
  // Load KDL from URDF
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  // Create models
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain")
    {
      ROS_INFO_STREAM("Creating chain '" << params.models[i].name << "' from " <<
                                            params.base_link << " to " <<
                                            params.models[i].params["frame"]);
      ChainModel* model = new ChainModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"]);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera3d")
    {
      ROS_INFO_STREAM("Creating camera3d '" << params.models[i].name << "' in frame " <<
                                               params.models[i].params["frame"]);
      Camera3dModel* model = new Camera3dModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"]);
      models_[params.models[i].name] = model;
    }
    else
    {
      // ERROR unknown
    }
  }

  // Setup  parameters to calibrate
  offsets_ = new CalibrationOffsetParser();
  for (size_t i = 0; i < params.free_params.size(); ++i)
  {
    offsets_->add(params.free_params[i]);
  }
  for (size_t i = 0; i < params.free_frames.size(); ++i)
  {
    offsets_->addFrame(params.free_frames[i].name,
                       params.free_frames[i].x,
                       params.free_frames[i].y,
                       params.free_frames[i].z,
                       params.free_frames[i].roll,
                       params.free_frames[i].pitch,
                       params.free_frames[i].yaw);
  }

  // Allocate space
  free_params_ = new double[offsets_->size()];
  for (int i = 0; i < offsets_->size(); ++i)
    free_params_[i] = 0.0;

  // Houston, we have a problem...
  problem_ = new ceres::Problem();

  // For each observation:
  for (size_t i = 0; i < data.size(); ++i)
  {
    for (size_t j = 0; j < params.error_blocks.size(); ++j)
    {
      if (params.error_blocks[j].type == "camera3d_to_arm")
      {
        ceres::CostFunction * cost = Camera3dToArmError::Create(
          dynamic_cast<Camera3dModel*>(models_[static_cast<std::string>(params.error_blocks[j].params["camera"])]),
          models_[static_cast<std::string>(params.error_blocks[j].params["arm"])],
          offsets_, data[i]);

        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params_;
          double * residuals = new double[data[i].observations[0].features.size() * 3];  // TODO: should check that all features are same length?

          cost->Evaluate(params, residuals, NULL);
          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
          for (size_t k = 0; k < data[i].observations[0].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 0)];
          std::cout << std::endl << "  y: ";
          for (size_t k = 0; k < data[i].observations[0].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 1)];
          std::cout << std::endl << "  z: ";
          for (size_t k = 0; k < data[i].observations[0].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
          std::cout << std::endl << std::endl;
        }

        problem_->AddResidualBlock(cost,
                                   NULL /* squared loss */,
                                   free_params_);
      }
      else if (params.error_blocks[j].type == "outrageous")
      {
        problem_->AddResidualBlock(
          OutrageousError::Create(offsets_,
                                  params.error_blocks[j].name,
                                  static_cast<double>(params.error_blocks[j].params["joint_scale"]),
                                  static_cast<double>(params.error_blocks[j].params["position_scale"]),
                                  static_cast<double>(params.error_blocks[j].params["rotation_scale"])),
          NULL, // squared loss
          free_params_);
      }
      else
      {
        // ERROR unknown
      }
    }
  }

  // Setup the actual optimization
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-10; 
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = progress_to_stdout;
  //options.use_nonmonotonic_steps = true;

  if (progress_to_stdout)
    std::cout << "\nSolver output:" << std::endl;
  summary_ = new ceres::Solver::Summary();
  ceres::Solve(options, problem_, summary_);
  if (progress_to_stdout)
    std::cout << "\n" << summary_->BriefReport() << std::endl;

  // TODO output stats
  /*if (progress_to_stdout)
  {
    CalibrationOffsetParser no_offsets;
    offsets_->update(free_params_);
    for (size_t i = 0; i < data.size(); ++i)
    {
      std::cout << "Sample " << i << std::endl;
      printSimpleDistanceError(arm_model_, camera_model_, &no_offsets, offsets_, data[i]);
      printComparePoints(arm_model_, camera_model_, &no_offsets, offsets_, data[i]);
    }
  }*/

  // Note: the error blocks will be managed by scoped_ptr in cost functor
  //       which takes ownership, and so we do not need to delete them here

  return 0;
}

}  // namespace robot_calibration
