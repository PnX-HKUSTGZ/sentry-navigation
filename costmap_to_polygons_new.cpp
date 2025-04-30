/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann, Otniel Rinaldo
 *********************************************************************/

 #include <costmap_converter/costmap_to_polygons.h>
 #include <costmap_converter/misc.h>
 #include <pluginlib/class_list_macros.hpp>
 
 PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons)
 
 namespace costmap_converter {
 
 // =============================  Constructor / Destructor  ==================
 CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH() : BaseCostmapToPolygons()
 {
   costmap_            = nullptr;
   neighbor_size_x_    = neighbor_size_y_ = -1;
   offset_x_           = offset_y_ = 0.0;
   slope_threshold_    = 0.35;                 // default ramp gradient limit
 }
 
 CostmapToPolygonsDBSMCCH::~CostmapToPolygonsDBSMCCH() {}
 
 // ================================  Initialise  =============================
 void CostmapToPolygonsDBSMCCH::initialize(rclcpp::Node::SharedPtr nh)
 {
   BaseCostmapToPolygons::initialize(nh);
 
   costmap_ = nullptr;
 
   nh->get_parameter_or<double>("cluster_max_distance",          parameter_.max_distance_,            0.4  );
   nh->get_parameter_or<int   >("cluster_min_pts",               parameter_.min_pts_,                 2    );
   nh->get_parameter_or<int   >("cluster_max_pts",               parameter_.max_pts_,                 30   );
   nh->get_parameter_or<double>("convex_hull_min_pt_separation", parameter_.min_keypoint_separation_, 0.1  );
 
   // NEW: slope threshold parameter -----------------------------------------
   nh->declare_parameter<double>("slope_threshold", slope_threshold_);
   nh->get_parameter("slope_threshold", slope_threshold_);
   if (slope_threshold_ <= 0.0)
   {
     RCLCPP_WARN(getLogger(), "slope_threshold <= 0 supplied.  Reset to 0.35");
     slope_threshold_ = 0.35;
   }
 
   parameter_buffered_ = parameter_;
 }
 
 // ==========================  Main compute() entry  =========================
 void CostmapToPolygonsDBSMCCH::compute()
 {
   std::vector<std::vector<KeyPoint>> clusters;
   dbScan(clusters);
 
   PolygonContainerPtr polygons(new std::vector<geometry_msgs::msg::Polygon>());
 
   // convex hulls for each real cluster (cluster 0 is noise) ---------------
   for (std::size_t i = 1; i < clusters.size(); ++i)
   {
     polygons->emplace_back();
     convexHull2(clusters[i], polygons->back());
   }
 
   // add noise points as individual obstacles ------------------------------
   if (!clusters.empty())
   {
     for (const auto &kp : clusters.front())
     {
       polygons->emplace_back();
       convertPointToPolygon(kp, polygons->back());
     }
   }
 
   updatePolygonContainer(polygons);
 }
 
 // ==============================  Costmap hook  ============================
 void CostmapToPolygonsDBSMCCH::setCostmap2D(nav2_costmap_2d::Costmap2D *costmap)
 {
   if (!costmap) return;
   costmap_ = costmap;
   updateCostmap2D();
 }
 
 // =====================  Extract occupied cells with slope filter  =========
 void CostmapToPolygonsDBSMCCH::updateCostmap2D()
 {
   occupied_cells_.clear();
   slope_cells_.clear();
 
   if (!costmap_ || !costmap_->getMutex())
   {
     RCLCPP_ERROR(getLogger(), "Cannot update costmap: null pointer or mutex");
     return;
   }
 
   {
     std::lock_guard<std::mutex> lock(parameter_mutex_);
     parameter_ = parameter_buffered_;
   }
 
   std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
 
   // allocate neighbour lookup grid ----------------------------------------
   int cells_x = int(costmap_->getSizeInMetersX() / parameter_.max_distance_) + 1;
   int cells_y = int(costmap_->getSizeInMetersY() / parameter_.max_distance_) + 1;
   if (cells_x != neighbor_size_x_ || cells_y != neighbor_size_y_)
   {
     neighbor_size_x_ = cells_x;
     neighbor_size_y_ = cells_y;
     neighbor_lookup_.resize(neighbor_size_x_ * neighbor_size_y_);
   }
   offset_x_ = costmap_->getOriginX();
   offset_y_ = costmap_->getOriginY();
   for (auto &n : neighbor_lookup_) n.clear();
 
   // iterate over map cells -------------------------------------------------
   const unsigned size_x = costmap_->getSizeInCellsX();
   const unsigned size_y = costmap_->getSizeInCellsY();
   const double   res    = costmap_->getResolution();
 
   for (unsigned i = 1; i < size_x - 1; ++i)              // avoid border cells for gradient
   {
     for (unsigned j = 1; j < size_y - 1; ++j)
     {
       unsigned char value = costmap_->getCost(i, j);
       if (value < nav2_costmap_2d::LETHAL_OBSTACLE) continue;
 
       //---------------- gradient magnitude ---------------------------------
       double dzdx = (static_cast<double>(costmap_->getCost(i + 1, j)) -
                      static_cast<double>(costmap_->getCost(i - 1, j))) /
                     (2.0 * res);
       double dzdy = (static_cast<double>(costmap_->getCost(i, j + 1)) -
                      static_cast<double>(costmap_->getCost(i, j - 1))) /
                     (2.0 * res);
       double slope_mag = std::sqrt(dzdx * dzdx + dzdy * dzdy);
 
       double wx, wy;
       costmap_->mapToWorld(i, j, wx, wy);
 
       if (slope_mag < slope_threshold_)
       {
         // Treat as ramp: not an obstacle, but store for debug visualisation
         geometry_msgs::msg::Point p; p.x = wx; p.y = wy; p.z = 0.0;
         slope_cells_.push_back(p);
         continue;
       }
 
       addPoint(wx, wy);  // genuine obstacle cell
     }
   }
 }
 
 // =======================  Slope cell accessor ============================
 const std::vector<geometry_msgs::msg::Point>& CostmapToPolygonsDBSMCCH::getSlopeCells() const
 {
   return slope_cells_;
 }
 
 // ========================  Remaining functions ============================
 // (dbScan, regionQuery, convexHull, simplifyPolygon etc. stay UNCHANGED)
 //  ...  (content omitted here for brevity – they are identical to the
 //        original implementation above).
 
 } // namespace costmap_converter
 