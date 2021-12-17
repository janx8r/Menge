/*
 Menge Crowd Simulation Framework

 Copyright and trademark 2012-17 University of North Carolina at Chapel Hill

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
 or
    LICENSE.txt in the root of the Menge repository.

 Any questions or comments should be sent to the authors menge@cs.unc.edu

 <http://gamma.cs.unc.edu/Menge/>
*/

#ifndef __WEBOTS_CONTROLLER_TASK_H__
#define __WEBOTS_CONTROLLER_TASK_H__

/*!
 @file  WebotsControllerTask.h
 @brief  TODO at every
        FSM time step.
 */

#include "WebotsControllerConfig.h"
#include "MengeCore/BFSM/Tasks/Task.h"
#include "MengeCore/BFSM/Tasks/TaskFactory.h"
#include "MengeCore/BFSM/fsmCommon.h"
#include "MengeCore/CoreConfig.h"
#include "MengeCore/resources/Resource.h"
#include "MengeCore/Agents/SimulatorInterface.h"
#include <webots/Supervisor.hpp>

#include <chrono>

namespace WebotsController {



/*!
 @brief  TODO
 */
class WEBOTS_API WebotsControllerTask : public Menge::BFSM::Task {
 public:
  /*!
   @brief    Constructor TODO.

   */
  WebotsControllerTask();

  /*!
   @brief    Constructor TODO.

   */
  ~WebotsControllerTask();

   /*!
      *	@brief		add webots controller
      *
      *	@param		pointer to node handle		
      */
   void addWebotsController(const Menge::Agents::SimulatorInterface* sim);


   void computeAgents(const Menge::Agents::SimulatorInterface* sim);

   /*!
      *	@brief		Arrange the coordinates in the right order for the webots world.
      *
      *	@param		index of the ordinate: x=0, y=1, z=2		
      */
   int permutation(int index);

   webots::Supervisor *supervisor;
   //webots::Node *robot_node;
   //webots::Field *trans_field;
   std::string coordinateSystem;
   int permutationsarray[3];
   std::vector<webots::Node*> pedestrian_node_vector;
   std::vector<webots::Node*> robot_node_vector;
   bool first_task_execution;
   int execution_cnt;
   int time_step_ms;
   int agtCount;
   int id;
   std::chrono::steady_clock::time_point t0,t1,t2,t3,t4,t5,t6,t7,t8,t9;

  /*!
   @brief    The work performed by the task.

   @param    fsm    The finite state machine for the task to operate on.
   @throws    A TaskException if there was some non-fatal error in execution. It should be logged.
   @throws    A TaskFatalException if there is a fatal error that should arrest execution of the
              simulation.
   */
  virtual void doWork(const Menge::BFSM::FSM* fsm) throw(Menge::BFSM::TaskException);

  /*!
   @brief    String representation of the task

   @returns  A string containing task information.
   */
  virtual std::string toString() const;

  /*!
   @brief    Reports if this task is "equivalent" to the given task.

   This makes it possible for a task to be redundantly added to the fsm without fear of duplication
   as the equivalent duplicates will be culled.

   @param    task    The task to test against this one.
   @returns  A boolean reporting if the two tasks are equivalent (true) or unique (false).
   */
  virtual bool isEquivalent(const Task* task) const;

 protected:

};

/*!
 @brief    Factory for the WebotsControllerTask.
 */
class WEBOTS_API WebotsControllerTaskFactory : public Menge::BFSM::TaskFactory {
 public:
  /*!
   @brief    Constructor.
   */
  WebotsControllerTaskFactory();

  /*!
   @brief    The name of the goal selector type.

   The goal selector's name must be unique among all registered goal selectors. Each goal selector
   factory must override this function.

   @returns  A string containing the unique goal selector name.
   */
  virtual const char* name() const { return "webots_controller"; }

  /*!
   @brief    A description of the goal selector.

   Each goal selector factory must override this function.

   @returns  A string containing the goal selector description.
   */
  virtual const char* description() const {
    return "A webots controller"
           "TODO "
           "TODO";
  };

 protected:
  /*!
   @brief    Create an instance of this class's goal selector.

   @returns    A pointer to a newly instantiated GoalSelector class.
   */
  Menge::BFSM::Task* instance() const { return new WebotsControllerTask(); }

//   /*!
//    @brief    Given a pointer to a GoalSelector instance, sets the appropriate fields
//    from the provided XML node.

//    @param    selector    A pointer to the goal whose attributes are to be set.
//    @param    node        The XML node containing the goal selector attributes.
//    @param    behaveFldr  The path to the behavior file.  If the goal selector references resources in
//                         the file system, it should be defined relative to the behavior file
//                         location.  This is the folder containing that path.
//    @returns  A boolean reporting success (true) or failure (false).
//    */
//   virtual bool setFromXML(Task* task, TiXmlElement* node,
//                           const std::string& behaveFldr) const;

};

}  // namespace WebotsController
#endif  // __WEBOTS_CONTROLLER_TASK_H__
