#include <boost/asio/io_service.hpp>
#include <iostream>
#include "abb_libegm/egm_trajectory_interface.h"
#include "abb_libegm/egm_common.h"

int main(int argc, char* argv[])
{
  std::cout << "Hello, EGM!" << std::endl;
  boost::asio::io_service io_service;
  unsigned short port_number = 6510;
  abb::egm::BaseConfiguration configuration;
  abb::egm::EGMTrajectoryInterface egm_interface(io_service, port_number, configuration);

  bool initialized = egm_interface.isInitialized();

  boost::thread_group thread_group;
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  bool wait = true;
  std::cout << "1: Wait for an EGM communication session to start..." << std::endl;
  while(wait)
  {
    bool connected = egm_interface.isConnected();

    if(egm_interface.isConnected())
    {
      abb::egm::wrapper::Status_RAPIDExecutionState rapid_execution_state =
        egm_interface.getStatus().rapid_execution_state();
      if(rapid_execution_state == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        std::cout << "RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program."
          << std::endl;
      }
      else
      {
        wait = rapid_execution_state != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }
    sleep(1);
  }

  abb::egm::wrapper::trajectory::TrajectoryGoal trajectory_1;
  abb::egm::wrapper::trajectory::PointGoal* p_point;

  p_point = trajectory_1.add_points();
  p_point->set_reach(true);
  p_point->set_duration(2.5);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

  p_point = trajectory_1.add_points();
  p_point->set_reach(true);
  p_point->set_duration(2.5);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

  p_point = trajectory_1.add_points();
  p_point->set_reach(true);
  p_point->set_duration(2.5);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(30.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

  p_point = trajectory_1.add_points();
  p_point->set_reach(true);
  p_point->set_duration(2.3);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);
  p_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(0.0);

  std::cout << "2: add joint trajectories to the execution queue" << std::endl;
  egm_interface.addTrajectory(trajectory_1);

  std::cout << "3: Wait for the trajectory execution to finish..." << std::endl;
  abb::egm::wrapper::trajectory::ExecutionProgress execution_progress;
  wait = true;
  while(wait)
  {
    sleep(1);

    if(egm_interface.retrieveExecutionProgress(&execution_progress))
    {
      wait = execution_progress.goal_active();
    }
  }

  std::cout << "4: all done!" << std::endl;

  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  std::cout << "Bye, EGM!" << std::endl;
  return 0;
}