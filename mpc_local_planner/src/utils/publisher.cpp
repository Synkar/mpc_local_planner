/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#include <mpc_local_planner/utils/publisher.h>

#include <mpc_local_planner/utils/conversion.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>
#include <iomanip>
#include <memory>

namespace mpc_local_planner {

Publisher::Publisher(ros::NodeHandle& nh, RobotDynamicsInterface::Ptr system, const std::string& map_frame) { initialize(nh, system, map_frame); }

void Publisher::initialize(ros::NodeHandle& nh, RobotDynamicsInterface::Ptr system, const std::string& map_frame)
{
    if (_initialized) ROS_WARN("mpc_local_planner: Publisher already initialized. Reinitalizing...");

    _system    = system;
    _map_frame = map_frame;

    // register topics
    _global_plan_pub = nh.advertise<nav_msgs::Path>("global_plan", 1);
    _local_plan_pub  = nh.advertise<nav_msgs::Path>("local_plan", 1);
    _mpc_marker_pub  = nh.advertise<visualization_msgs::Marker>("mpc_markers", 1000);
    _mpc_marker_velocity_pub = nh.advertise<visualization_msgs::MarkerArray>("mpc_markers_velocity", 1);

    _initialized = true;
}

void Publisher::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const
{
    if (!_initialized) return;
    base_local_planner::publishPlan(local_plan, _local_plan_pub);
}

void Publisher::publishLocalPlan(const corbo::TimeSeries& ts) const
{
    if (!_initialized) return;
    if (!_system)
    {
        ROS_ERROR("Publisher::publishLocalPlan(): cannot publish since the system class is not provided.");
        return;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;
    convert(ts, *_system, local_plan, _map_frame);
    base_local_planner::publishPlan(local_plan, _local_plan_pub);
}

void Publisher::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const
{
    if (!_initialized) return;
    base_local_planner::publishPlan(global_plan, _global_plan_pub);
}

void Publisher::publishRobotFootprintModel(const teb_local_planner::PoseSE2& current_pose,
                                           const teb_local_planner::BaseRobotFootprintModel& robot_model, const std::string& ns,
                                           const std_msgs::ColorRGBA& color)
{
    if (!_initialized) return;

    std::vector<visualization_msgs::Marker> markers;
    robot_model.visualizeRobot(current_pose, markers, color);
    if (markers.empty()) return;

    int idx = 1000000;  // avoid overshadowing by obstacles
    for (visualization_msgs::Marker& marker : markers)
    {
        marker.header.frame_id = _map_frame;
        marker.header.stamp    = ros::Time::now();
        marker.action          = visualization_msgs::Marker::ADD;
        marker.ns              = ns;
        marker.id              = idx++;
        marker.lifetime        = ros::Duration(2.0);
        _mpc_marker_pub.publish(marker);
    }
}

void Publisher::publishObstacles(const teb_local_planner::ObstContainer& obstacles) const
{
    if (obstacles.empty() || !_initialized) return;

    visualization_msgs::MarkerArray velocity_marker_array;


    // Visualize point obstacles
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = _map_frame;
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "PointObstacles";
        marker.id              = 0;
        marker.type            = visualization_msgs::Marker::POINTS;
        marker.action          = visualization_msgs::Marker::ADD;
        marker.lifetime        = ros::Duration(2.0);

        for (const ObstaclePtr& obst : obstacles)
        {
            // std::shared_ptr<PointObstacle> pobst = std::dynamic_pointer_cast<PointObstacle>(obst); // TODO(roesmann): change teb_local_planner
            // types to std lib
            boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst);
            if (!pobst) continue;

            geometry_msgs::Point point;
            point.x = pobst->x();
            point.y = pobst->y();
            point.z = 0;
            marker.points.push_back(point);
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        _mpc_marker_pub.publish(marker);
    }

    // Visualize line obstacles
    {
        int idx = 0;
        for (const ObstaclePtr& obst : obstacles)
        {
            // LineObstacle::Ptr pobst = std::dynamic_pointer_cast<LineObstacle>(obst);
            boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(obst);
            if (!pobst) continue;

            visualization_msgs::Marker marker;
            marker.header.frame_id = _map_frame;
            marker.header.stamp    = ros::Time::now();
            marker.ns              = "LineObstacles";
            marker.id              = idx++;
            marker.type            = visualization_msgs::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.lifetime        = ros::Duration(2.0);
            geometry_msgs::Point start;
            start.x = pobst->start().x();
            start.y = pobst->start().y();
            start.z = 0;
            marker.points.push_back(start);
            geometry_msgs::Point end;
            end.x = pobst->end().x();
            end.y = pobst->end().y();
            end.z = 0;
            marker.points.push_back(end);

            if(obst->isDynamic()){
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                visualization_msgs::Marker velocity_marker;
                velocity_marker.header.frame_id = _map_frame;
                velocity_marker.header.stamp = ros::Time::now();
                velocity_marker.type = visualization_msgs::Marker::ARROW;
                velocity_marker.action = visualization_msgs::Marker::ADD;
                velocity_marker.id = marker.id;
                geometry_msgs::Point start, end;
                Eigen::Vector2d centroid = obst->getCentroid();
                Eigen::Vector2d centroid_velocity = obst->getCentroidVelocity();
                start.x = centroid.x();
                start.y = centroid.y();
                start.z = 0;
                velocity_marker.points.push_back(start);
                
                end.x = centroid.x() + centroid_velocity.x();
                end.y = centroid.y() + centroid_velocity.y();
                end.z = 0;
                velocity_marker.points.push_back(end);

                velocity_marker.scale.x = 0.05;
                velocity_marker.scale.y = 0.1;
                velocity_marker.scale.z = 0;
                velocity_marker.color.a = 1.0; 
                velocity_marker.color.r = 1.0;
                velocity_marker.color.g = 0.0;
                velocity_marker.color.b = 0.0;
                velocity_marker_array.markers.push_back(velocity_marker);

            }else{
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }

            _mpc_marker_pub.publish(marker);
        }
    }

    // Visualize polygon obstacles
    {

        int idx = 0;
        for (const ObstaclePtr& obst : obstacles)
        {   


            // PolygonObstacle::Ptr pobst = std::dynamic_pointer_cast<PolygonObstacle>(obst);
            boost::shared_ptr<PolygonObstacle> pobst = boost::dynamic_pointer_cast<PolygonObstacle>(obst);
            if (!pobst) continue;

            visualization_msgs::Marker marker;
            marker.header.frame_id = _map_frame;
            marker.header.stamp    = ros::Time::now();
            marker.ns              = "PolyObstacles";
            marker.id              = idx++;
            marker.type            = visualization_msgs::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.lifetime        = ros::Duration(2.0);

            for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
            {
                geometry_msgs::Point point;
                point.x = vertex->x();
                point.y = vertex->y();
                point.z = 0;
                marker.points.push_back(point);
            }

            // Also add last point to close the polygon
            // but only if polygon has more than 2 points (it is not a line)
            if (pobst->vertices().size() > 2)
            {
                geometry_msgs::Point point;
                point.x = pobst->vertices().front().x();
                point.y = pobst->vertices().front().y();
                point.z = 0;
                marker.points.push_back(point);
            }
            if(obst->isDynamic()){
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                visualization_msgs::Marker velocity_marker;
                velocity_marker.header.frame_id = _map_frame;
                velocity_marker.header.stamp = ros::Time::now();
                velocity_marker.type = visualization_msgs::Marker::ARROW;
                velocity_marker.action = visualization_msgs::Marker::ADD;
                velocity_marker.id = marker.id;
                geometry_msgs::Point start, end;
                Eigen::Vector2d centroid = obst->getCentroid();
                Eigen::Vector2d centroid_velocity = obst->getCentroidVelocity();
                start.x = centroid.x();
                start.y = centroid.y();
                start.z = 0;
                velocity_marker.points.push_back(start);
                
                end.x = centroid.x() + centroid_velocity.x();
                end.y = centroid.y() + centroid_velocity.y();
                end.z = 0;
                velocity_marker.points.push_back(end);

                velocity_marker.scale.x = 0.05;
                velocity_marker.scale.y = 0.1;
                velocity_marker.scale.z = 0;
                velocity_marker.color.a = 1.0; 
                velocity_marker.color.r = 1.0;
                velocity_marker.color.g = 0.0;
                velocity_marker.color.b = 0.0;
                velocity_marker_array.markers.push_back(velocity_marker);

            }else{
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }

            _mpc_marker_pub.publish(marker);
        }

    }


    // Visualize Circular obstacles
    {
        int idx = 0;
        for (const ObstaclePtr& obst : obstacles)
        {   


            // CircularObstacle::Ptr pobst = std::dynamic_pointer_cast<CircularObstacle>(obst);
            boost::shared_ptr<CircularObstacle> pobst = boost::dynamic_pointer_cast<CircularObstacle>(obst);
            if (!pobst) continue;

            visualization_msgs::Marker marker;
            marker.header.frame_id = _map_frame;
            marker.header.stamp    = ros::Time::now();
            marker.ns              = "CircularObstacles";
            marker.id              = idx++;
            marker.type            = visualization_msgs::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.lifetime        = ros::Duration(2.0);

            int num_points = 36;
            for (int i=0; i<=num_points; i++)
            {

                float angle = float(i)*M_PI/18.0;
                geometry_msgs::Point point;
                point.x = pobst->x() + pobst->radius()*cos(angle);
                point.y = pobst->y() + pobst->radius()*sin(angle);
                point.z = 0;
                marker.points.push_back(point);
            }

            if(obst->isDynamic()){
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                visualization_msgs::Marker velocity_marker;
                velocity_marker.header.frame_id = _map_frame;
                velocity_marker.header.stamp = ros::Time::now();
                velocity_marker.type = visualization_msgs::Marker::ARROW;
                velocity_marker.action = visualization_msgs::Marker::ADD;
                velocity_marker.id = marker.id;
                geometry_msgs::Point start, end;
                Eigen::Vector2d centroid = obst->getCentroid();
                Eigen::Vector2d centroid_velocity = obst->getCentroidVelocity();
                start.x = centroid.x();
                start.y = centroid.y();
                start.z = 0;
                velocity_marker.points.push_back(start);
                
                end.x = centroid.x() + centroid_velocity.x();
                end.y = centroid.y() + centroid_velocity.y();
                end.z = 0;
                velocity_marker.points.push_back(end);

                velocity_marker.scale.x = 0.05;
                velocity_marker.scale.y = 0.1;
                velocity_marker.scale.z = 0;
                velocity_marker.color.a = 1.0; 
                velocity_marker.color.r = 1.0;
                velocity_marker.color.g = 0.0;
                velocity_marker.color.b = 0.0;
                velocity_marker_array.markers.push_back(velocity_marker);

            }else{
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }

            _mpc_marker_pub.publish(marker);
        }

    }

    if(!velocity_marker_array.markers.empty()){
        _mpc_marker_velocity_pub.publish(velocity_marker_array);
    }
}

void Publisher::publishViaPoints(const std::vector<teb_local_planner::PoseSE2>& via_points, const std::string& ns) const
{
    if (via_points.empty() || !_initialized) return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = _map_frame;
    marker.header.stamp    = ros::Time::now();
    marker.ns              = ns;
    marker.id              = 55555;
    marker.type            = visualization_msgs::Marker::POINTS;
    marker.action          = visualization_msgs::Marker::ADD;
    marker.lifetime        = ros::Duration(2.0);

    for (const teb_local_planner::PoseSE2& via_point : via_points)
    {
        geometry_msgs::Point point;
        point.x = via_point.x();
        point.y = via_point.y();
        point.z = 0;
        marker.points.push_back(point);
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    _mpc_marker_pub.publish(marker);
}

std_msgs::ColorRGBA Publisher::toColorMsg(float a, float r, float g, float b)
{
    std_msgs::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
}

}  // namespace mpc_local_planner
