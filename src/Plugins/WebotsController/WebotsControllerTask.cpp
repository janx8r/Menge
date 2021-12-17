/*

License

Menge
Copyright � and trademark � 2012-14 University of North Carolina at Chapel Hill.
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation
for educational, research, and non-profit purposes, without fee, and without a
written agreement is hereby granted, provided that the above copyright notice,
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North
Carolina at Chapel Hill. The software program and documentation are supplied "as is,"
without any accompanying services from the University of North Carolina at Chapel
Hill or the authors. The University of North Carolina at Chapel Hill and the
authors do not warrant that the operation of the program will be uninterrupted
or error-free. The end-user understands that the program was developed for research
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the authors {menge,geom}@cs.unc.edu

*/

#include "WebotsControllerTask.h"

#include "MengeCore/Agents/BaseAgent.h"
#include "MengeCore/Agents/SimulatorInterface.h"
#include "MengeCore/BFSM/FSM.h"


#include <exception>

#include <webots/Supervisor.hpp>
#include <mutex>

#include <chrono>

namespace WebotsController {

using Menge::Logger;
using Menge::logger;





void WebotsControllerTask::addWebotsController(const Menge::Agents::SimulatorInterface* sim){
  // webots supervisor
  this->supervisor = new webots::Supervisor();


  // create webots pedestrian_node for each agent in Menge 
  this->agtCount = (int)sim->getNumAgents();
  for (int a = 0; a < this->agtCount; ++a) {
    webots::Node *pedestrian_node = supervisor->getFromDef("PEDESTRIAN_"+std::to_string(a));
    if(pedestrian_node){
      //füge pedestrian_node zu Liste hinzu
      pedestrian_node_vector.push_back(pedestrian_node);
      std::cout << "pedestrian_node added: " << a << std::endl;
    }
  }

  // create a webots robot_node for each webots robot named "ROBOT_*"
  for (int a = 0; a < this->agtCount; ++a) {
    webots::Node *robot_node = supervisor->getFromDef("ROBOT_"+std::to_string(a));
    if(robot_node){
      //füge robot_node zu Liste hinzu
      robot_node_vector.push_back(robot_node);
      std::cout << "robot_node added: " << a << std::endl;
    }
  }

  
  // get coordinate system used in webots
  this->coordinateSystem = this->supervisor->getRoot()->getField("children")->getMFNode(0)->getField("coordinateSystem")->getSFString();
  std::cout << "coordinateSystem: " << this->coordinateSystem << std::endl;

  permutationsarray[0] = coordinateSystem.find("E");
  permutationsarray[1] = coordinateSystem.find("N");
  permutationsarray[2] = coordinateSystem.find("U");

  this->time_step_ms = (int)(sim->getTimeStep()*1000);
  // agtCount = (int)sim->getNumAgents();
  std::cout << "agtCount: " << agtCount << std::endl;
  
}

int WebotsControllerTask::permutation(int index){
  // std::size_t ret;
  // if(index == 0) ret = coordinateSystem.find("E"); //east, x-axis
  // else
  // if(index == 1) ret = coordinateSystem.find("N"); //north, y-axis
  // else
  // if(index == 2) ret = coordinateSystem.find("U"); //up, z-axis
  
  // if(ret==std::string::npos){
  //   std::cout << "unknown coordinateSystem: \"" << coordinateSystem << "\"" << std::endl;
  //   return -1;
  // }
  // else
  //   return (int) ret;
  return permutationsarray[index];
}


/////////////////////////////////////////////////////////////////////
//                   Implementation of WebotsControllerTask
/////////////////////////////////////////////////////////////////////

WebotsControllerTask::WebotsControllerTask()
    : Menge::BFSM::Task() {
  first_task_execution = true;
  execution_cnt = 0;
  id = rand();
}

WebotsControllerTask::~WebotsControllerTask(){
  delete supervisor;
}

/////////////////////////////////////////////////////////////////////




void WebotsControllerTask::doWork(const Menge::BFSM::FSM* fsm) throw(Menge::BFSM::TaskException) {
  //t1 = std::chrono::steady_clock::now();
  //std::cout<<"Menge:"<<std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "[µs]"<<std::endl;
  
  // t4 = std::chrono::steady_clock::now();
  const Menge::Agents::SimulatorInterface* sim = fsm->getSimulator();

  // erstelle beim ersten Aufruf den Webots Controller
  if(first_task_execution){
    first_task_execution = false;
    addWebotsController(sim);
  }
  else{
    

    size_t exceptionCount = 0;

    int ped_idx = 0;
    int rob_idx = 0;
    int ped_vec_size = pedestrian_node_vector.size();
    int rob_vec_size = robot_node_vector.size();
    
    // #pragma omp parallel for reduction(+ : exceptionCount)
    // #pragma omp parallel for ordered
    for (int a = 0; a < agtCount; ++a) {
      try {
        
        // t0 = std::chrono::steady_clock::now();
        Menge::Agents::BaseAgent* agt = sim->getAgent(a);
        // fsm->advance(agt);
        // Tue etwas mit den Agenten
        
        if(agt->_external){ 
          
          //Agent wird extern gesteuert, also ein von Webots gesteuerter Roboter
          if(rob_idx < rob_vec_size){
            webots::Node *robot_node = robot_node_vector[rob_idx];
            if(robot_node){
              webots::Field *trans_field = robot_node->getField("translation");
              webots::Field *rot_field = robot_node->getField("rotation");
              webots::Field *vel_field = robot_node->getProtoField("linearVelocity");

              if(trans_field && rot_field && vel_field){

                // // get actual position
                
                double *position = trans_field->getSFVec3f();
                double *rotation = rot_field->getSFRotation();
                double *velocity = vel_field->getSFVec3f();

                // // calculate orientation from old position to new position
                double x = position[permutation(0)];
                double y = position[permutation(1)];
                double theta = rotation[3];
                double vx = velocity[permutation(0)];
                double vy = velocity[permutation(1)];

                agt->_pos._x = x;
                agt->_pos._y = y;
                agt->_orient._x = cos(theta);
                agt->_orient._y = sin(theta);
                agt->_vel._x = vx;
                agt->_vel._y = vy;
              }
            }
          }
          rob_idx++;
          
        }
        else{ 
          //Agent wird nicht extern gesteuert, also ein von Menge gesteuerter Mensch
          //wenn es den Agenten in dem pedestrian_node_vector gibt
          if(ped_idx < ped_vec_size){
            // t0 = std::chrono::steady_clock::now();
            //hole den pedestrian_node
            webots::Node *pedestrian_node = pedestrian_node_vector[ped_idx];
            //t1 = std::chrono::steady_clock::now();
            //wenn pedestrian_node gültig ist (nicht NULL)
            if(pedestrian_node){
              //hole Zugriffsfelder für die Translation und Rotation des Pedestrian
              
              //t0 = std::chrono::steady_clock::now();
              webots::Field *trans_field = pedestrian_node->getField("translation");
              //t1 = std::chrono::steady_clock::now();
              //std::cout<<"get field:"<<std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "[µs]"<<std::endl;
              
              webots::Field *rot_field = pedestrian_node->getField("rotation");
              // t4 = std::chrono::steady_clock::now();
              if(trans_field && rot_field){
                // get actual position
                //t0 = std::chrono::steady_clock::now();
                double *position = trans_field->getSFVec3f();  
                //t1 = std::chrono::steady_clock::now(); 
                //std::cout<<"getSFVec3f:"<<std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "[µs]"<<std::endl;        
                // t5 = std::chrono::steady_clock::now();

                // calculate orientation from old position to new position
                double dx = agt->_pos._x - position[permutation(0)];
                double dy = agt->_pos._y - position[permutation(1)];

                // To avoid wobbling when standing still, the pedestrian's orientation should only be changed if he or she moves more than 3mm
                if(sqrt(dx*dx+dy*dy) > 0.003){
                  double alpha = atan2(dy,dx);
                  double orientation[4] = {0,0,0,0};
                  orientation[permutation(2)] = 1.0; //rotation around z-axis
                  orientation[3] = alpha;
                  rot_field->setSFRotation(orientation);
                }
                
                // change x and y
                position[permutation(0)] = agt->_pos._x;
                position[permutation(1)] = agt->_pos._y;

                //if position is not NAN
                if(position[0] == position[0] && position[1] == position[1]){
                  // write modified position
                  // t6 = std::chrono::steady_clock::now();
                  //t0 = std::chrono::steady_clock::now();
                  trans_field->setSFVec3f(position);
                  //t1 = std::chrono::steady_clock::now(); 
                  //std::cout<<"setSFVec3f:"<<std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "[µs]"<<std::endl;   
                  // t7 = std::chrono::steady_clock::now();
                }
                else{
                  std::cout<<"agt pos nan "<<ped_idx<<std::endl;
                }
                
              }
            }
          }
          ped_idx++;
        }
      
      // std::cout<<"Zeit Agent"<<a<<": "<<std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "[µs]"<<std::endl;

      } catch (Menge::MengeException& e) {
        logger << Logger::ERR_MSG << e.what() << "\n";
        ++exceptionCount;
      } catch (std::exception& e) {
        logger << Logger::ERR_MSG << "Unanticipated system exception: ";
        logger << e.what() << ".";
        ++exceptionCount;
      }
    }

    if (exceptionCount > 0) {
      throw Menge::BFSM::TaskFatalException();
    }

    
  }
  // int dt = supervisor->step(time_step_ms);
  int dt = supervisor->step(time_step_ms);
  
  // std::cout << dt << std::endl;
  //t0 = std::chrono::steady_clock::now();
}
/////////////////////////////////////////////////////////////////////

std::string WebotsControllerTask::toString() const {
  return "Webots Controller Task";
}

/////////////////////////////////////////////////////////////////////

bool WebotsControllerTask::isEquivalent(const Task* task) const {
  const WebotsControllerTask* other = dynamic_cast<const WebotsControllerTask*>(task);
  if (other == 0x0) {
    return false;
  } else {
    return id == other->id;
  }
}


WebotsControllerTaskFactory::WebotsControllerTaskFactory() : TaskFactory() {
}

// bool WebotsControllerTaskFactory::setFromXML(Task* task, TiXmlElement* node,
//                                                const std::string& behaveFldr) const {

//   return true;
// }

}  // namespace WebotsController
