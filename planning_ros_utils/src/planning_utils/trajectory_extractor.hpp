#include <mpl_basis/trajectory.h>
#include <planning_ros_msgs/TrajectoryCommand.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include "quad_pos_ctrl/ctrl_ref.h"
#include <deque>

class TrajectoryExtractor {
 public:
  TrajectoryExtractor(const planning_ros_msgs::Trajectory& msg, double dt) 
  {
    const auto traj = toTrajectory3D(msg);
    int N = std::ceil(traj.getTotalTime() / dt);
    const auto ws = traj.sample(N);
    cmd_array.resize(ws.size());
    const auto t0 = ros::Time::now();    
    for (unsigned int i = 0; i < ws.size(); i++) 
    {
      ctrl_ref.header.stamp = t0 + ros::Duration(ws[i].t);
      for (unsigned char j = 0; j< 20; j++)
      {             
        Eigen::Vector3d ref_ned_p, ref_ned_v, ref_ned_a;
        if ((i+j) >= ws.size())
        {
          ref_ned_p << ws[ws.size()-1].pos(0), -ws[ws.size()-1].pos(1), -ws[ws.size()-1].pos(2);
          ref_ned_v << ws[ws.size()-1].vel(0), -ws[ws.size()-1].vel(1), -ws[ws.size()-1].vel(2);
          ref_ned_a << ws[ws.size()-1].acc(0), -ws[ws.size()-1].acc(1), -ws[ws.size()-1].acc(2);
        }
        else
        {
          ref_ned_p << ws[i+j].pos(0), -ws[i+j].pos(1), -ws[i+j].pos(2);
          ref_ned_v << ws[i+j].vel(0), -ws[i+j].vel(1), -ws[i+j].vel(2);
          ref_ned_a << ws[i+j].acc(0), -ws[i+j].acc(1), -ws[i+j].acc(2);
        }
        ctrl_ref.posx_ref[j] = ref_ned_p(0);
        ctrl_ref.posy_ref[j] = ref_ned_p(1);
        ctrl_ref.posz_ref[j] = ref_ned_p(2);
        ctrl_ref.velx_ref[j] = ref_ned_v(0);
        ctrl_ref.vely_ref[j] = ref_ned_v(1);
        ctrl_ref.velz_ref[j] = ref_ned_v(2);
        ctrl_ref.accx_ref[j] = ref_ned_a(0);
        ctrl_ref.accy_ref[j] = ref_ned_a(1);
        ctrl_ref.accz_ref[j] = ref_ned_a(2);
        ctrl_ref.yaw_ref[j] = ws[i+j].yaw;
        if (ref_ned_v.norm()<0.2f && ref_ned_a.norm() < 0.8f)
            ctrl_ref.ref_mask[j] = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
        else  ctrl_ref.ref_mask[j] = 7;
      }
      cmd_array[i] = ctrl_ref;
    }
    
  }

  std::vector<quad_pos_ctrl::ctrl_ref> getCommands() {
    return cmd_array;
  }

 private:
  std::vector<quad_pos_ctrl::ctrl_ref> cmd_array;
  // std::deque<quad_pos_ctrl::ctrl_ref> cmd_array;
  quad_pos_ctrl::ctrl_ref ctrl_ref;
};
