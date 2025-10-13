from gs_ros_interfaces.srv import (
    SetEntityPose,
    PathPlanTarget,
    IKTarget,
    FKTarget,
    SuctionSwitch,
    JoinEntities,
    StartRecording,
    PauseRecording,
    StopRecording,
    SetPhysicsAttributes,
    )
import genesis as gs

from std_srvs.srv import SetBool
import numpy as np
from .gs_ros_utils import (
    ros_quat_to_gs_quat,
    gs_quat_to_ros_quat,
    quat_angle_difference,
    get_entity,
    get_links_idx,
    get_dofs_idx)

class GsRosServices:
    def __init__(self,scene,ros_node,robots,objects,all_sensors):
        gs.logger.info("Starting all ros2 services")
        self.scene=scene
        self.ros_node=ros_node
        self.robots=robots
        self.objects=objects
        self.all_sensors=all_sensors
        # self.ns=""
        
        self.setup_fk()
        self.setup_ik()
        self.setup_path_plan()
        
        self.setup_join_entities_service()
        self.setup_reset_simulator_service()
        self.setup_suction_cup_switch_service()
        self.setup_set_entity_pose_service()
        self.setup_get_entity_pose_service()
        self.setup_set_dofs_physics_attr_service()
        self.setup_set_links_physics_attr_service()
        
    def setup_set_entity_pose_service(self):
        gs.logger.info("Setting up SetEntityPose service")
        def set_entity_pose_service_callback(request,response):
            if request.entity_name is None:
                response.success=False
                response.message="Entity name must be given"
                return response
            else:
                entity=get_entity(self.objects,request.entity_name)
                if entity is None:
                    entity=get_entity(self.robots,request.entity_name)
                if entity is None:
                    response.success=False
                    response.message="Robot or object with the given name not found in the scene"
                    return response
            try:
                entity.set_pos(request.pose.position)
                entity.set_quat(ros_quat_to_gs_quat(request.pose.orientation))
                response.success=True
                response.message="Entity Moved to the requested position"
            except:
                response.success=False
                response.message="Set Entity operation unsucessfull"
            return response
            
        self.set_entity_pose_service=self.ros_node.create_service(SetEntityPose,"/set_entity_pose",set_entity_pose_service_callback)
    
    def setup_get_entity_pose_service(self):
        gs.logger.info("Setting up GetEntityPose service")
        def get_entity_pose_service_callback(request,response):
            if request.entity_name is None:
                response.success=False
                response.message="Entity name must be given"
                return response
            else:
                entity=get_entity(self.objects,request.entity_name)
                if entity is None:
                    entity=get_entity(self.robots,request.entity_name)
                if entity is None:
                    response.success=False
                    response.message="Robot or object with the given name not found in the scene"
                    return response
            try:
                response.pose.position=entity.get_pos()
                response.pose.orientation=gs_quat_to_ros_quat(entity.get_quat())
                response.success=True
                response.message="Entity pose obtained"
            except:
                response.success=False
                response.message="Get Entity pose operation unsucessfull"
            return response
            
        self.get_entity_pose_service=self.ros_node.create_service(SetEntityPose,"/get_entity_pose",get_entity_pose_service_callback)

    def setup_ik(self):
        gs.logger.info("Setting up Ik service")
        def ik_service_callback(self, request, response):
            response.success=False
            if request.robot_name is None:
                response.success=False
                response.message="Robot name must be given"
                return response
            else:
                robot=get_entity(self.robots,request.robot_name)
                if robot is None:
                    response.success=False
                    response.message="Robot with the given name not found in the scene"
                    return response
            if request.target_link_name is None:
                response.success=False
                response.message="Link name must be given"
                return response
            else:
                target_link=robot.get_link(request.target_link_name)
                if target_link is None:
                    response.success=False
                    response.message="Link not found in robot"
                    return response
            try:
                target_qpos,error=robot.inverse_kinematics(
                    link=target_link,
                    pos=request.position,
                    quat=ros_quat_to_gs_quat(request.orientation),
                    init_qpos=request.init_robot_pos,
                    respect_joint_limit=request.request_joint_limit,
                    max_samples=request.max_samples,
                    max_solver_iters=request.max_solver_iterartions,
                    damping=request.damping,
                    pos_tol=request.pos_tol,
                    rot_tol=request.rot_tol,
                    pos_mask=request.pos_mask,
                    rot_mask=request.rot_mask,
                    max_step_size=request.max_step_size,
                    dofs_idx_local=request.dofs_idx_local,
                    return_error=True,
                )
            except Exception as e:
                response.success=False
                response.message=str(e)
                return 
            if target_qpos is None or error is None:
                response.solution_found=False
                response.message="IK solution not found found"
            else:
                response.success=True
                response.joint_angles=target_qpos 
                response.target_error=error
                response.message="IK solution found"
                if request.visualize:
                    pose_debug = self.scene.draw_debug_frame(request.pos,request.quat)
                if request.execute==True:
                    try:
                        robot.control_dofs_position(target_qpos)
                    except Exception as e:
                        response.sucess=False
                        response.message=str(e)
                        return response
                    eef_pos=robot.get_links_pos(target_link.idx_local)
                    eef_quat=robot.get_links_quat(target_link.idx_local)
                    pos_error_norm=np.linalg.norm(eef_pos-request.position) 
                    quat_error_norm=quat_angle_difference(eef_quat-ros_quat_to_gs_quat(request.orientation))
                    if  pos_error_norm <request.pos_tol and \
                        quat_error_norm <request.rot_tol:
                        response.sucess=True
                        response.message="IK Solution found, and Robot moved to target_position"
                    else:
                        response.sucess=False
                        response.message="IK Solution found, but Robot not moved to target_position"
            if pose_debug is not None:
                self.scene.clear_debug_object(pose_debug)                           
            return response
        self.ik_service=self.ros_node.create_service(IKTarget,"/inverse_kinematics_target",ik_service_callback)
        
    def setup_fk(self):
        gs.logger.info("Setting up FK service")
        def fk_service_callback(self,request,response):
            if request.robot_name is None:
                response.success=False
                response.message="Robot name must be given"
                return response
            else:
                robot=get_entity(self.robots,request.robot_name)
                if robot is None:
                    response.success=False
                    response.message="Robot with the given name not found in the scene"
                    return response
            if request.qpos is None:
                response.sucess=False
                response.message="The joint pos and dof names cant be none"
                return response
            else:
                if request.dof_names is not None:
                    if len(request.qpos)!=len(request.dof_names):
                        response.sucess=False
                        response.message="The joint pos and the dof names must be of the same length"
                        return response
                elif len(request.qpos)!=robot.n_dofs:
                        response.sucess=False
                        response.message="The joint pos and the number of dofs of the robot must be qual id dof_names id not provided"
                        return response
                    
            if request.link_names is not None:
                links_idx_local=get_links_idx(self.robot,request.link_names)
            else:
                links_idx_local=None
            if request.dof_names is not None:
                dofs_idx_local=get_dofs_idx(self.robot,request.dof_names)
            else:
                dofs_idx_local=None
            
            try:
                link_pos,link_quat=robot.forward_kinematics(request.qpos,
                                                            dofs_idx_local,
                                                            links_idx_local)
            except Exception as e:
                response.plan_found=False
                response.message=False
                response.success = str(e)
                return response
            if link_pos is not None and link_quat is not None:
                response.link_pos=link_pos
                response.link_quat=link_quat
                response.success=True
                response.message="FK Solution found"
            return response
        self.fk_service=self.ros_node.create_service(FKTarget,"/forward_kinematics_target",fk_service_callback)
        
    def setup_path_plan(self):
        gs.logger.info("Setting up PathPlanTarget service")
        def plan_path_callback(self,request, response):
            if request.robot_name is None:
                response.success=False
                response.message="Robot name must be given"
                return response
            else:
                robot=get_entity(self.robots,request.robot_name)
                if robot is None:
                    response.success=False
                    response.message="Robot with the given name not found in the scene"
                    return response
                
            attached_entity=None
            if request.attached_entity is not None:
                attached_entity=get_entity(self.objects,request.attached_entity)
                if attached_entity is None:
                    response.success=False
                    response.message="Robot with the given name not found in the scene"
                    return response

            try:
                path=robot.plan_path(
                        qpos_goal=request.goal_state,
                        qpos_start=request.start_state, 
                        max_nodes=request.max_nodes,
                        resolution=request.resolution,
                        timeout=request.timeout, 
                        max_retry=request.max_retries,
                        smooth_path=request.smooth_path, 
                        num_waypoints=request.num_waypoints, 
                        ignore_collision=request.ignore_collision, 
                        ignore_joint_limit=request.ignore_collision,
                        planner=request.planner,
                        ee_link_name=request.ee_link_name,
                        with_entity=attached_entity,
                )
            except Exception as e:
                response.plan_found=False
                response.message=False
                response.success = str(e)
                return response
            if path is None:
                response.plan_found=False
                response.success=False
                response.message="Planning unsucessfull"
            else:
                response.success=True
                response.path=path 
                response.message="Path planned"
                if request.visualize:
                    path_debug = self.scene.draw_debug_path(path, self.robot)
                if request.execute==True:
                    for waypoint in path:
                        robot.control_dofs_position(waypoint)
                    current_robot_state=robot.get_dofs_position()
                    joint_state_error=np.linalg.norm(current_robot_state-request.goal_state)
                    if joint_state_error <request.tolerace:
                        response.sucess=True
                        response.message="Path Plan sucessful and robot moved to the goal state "
                    else:
                        response.sucess=False
                        response.message="Path Plan sucessful but the robot didn't move to the goal state "      
            if path_debug is not None:
                self.scene.clear_object(path_debug)            
            return response
        self.path_plan_service=self.ros_node.create_service(PathPlanTarget,"/path_plan_target",plan_path_callback)
            
    def setup_suction_cup_switch_service(self):
        gs.logger.info("Setting up SuctionSwitch service")
        def suction_cup_switch_callback(self,request,response):
            if len(request.entity_one)<=0 or len(request.entity_one)<=0 \
                or len(request.link_one)<=0 or len(request.link_one)<=0:
                    response.success=False
                    response.message="Both of the entity names and the link names must be provided"
            entity_one=get_entity(self.robots,request.entity_one)
            entity_two=get_entity(self.robots,request.entity_two)
            
            if entity_one is None or entity_two is None:
                response.success=False
                response.message="Couldnt access the entities"
                return response
            
            link_one=entity_one.get_link(request.link_one).idx
            link_two=entity_two.get_link(request.link_two).idx
            
            if link_one is None:
                response.success=False
                response.message=f"Entity two has no link {request.link_one}"
                return response
                
            if link_two is None:
                response.success=False
                response.message=f"Entity two has no link {request.link_two}"
                return response
            
            rigid=self.scene.sim.rigid_solver
            if request.switch:
                rigid.add_weld_constraint(link_one,link_two)
                response.sucess=True
                response.message="Turned on suction"
                return response
            elif not request.switch:
                rigid.add_weld_constraint(link_one,link_two)
                response.sucess=True
                response.message="Turned off suction"
                return response
        self.suction_cup_switch=self.ros_node.create_service(SuctionSwitch,"/set_suction_state",suction_cup_switch_callback)

    def setup_join_entities_service(self):
        gs.logger.info("Setting up JoinEntities service")
        def join_entities_callback(self,request,response):
            if len(request.entity_one)<=0 or len(request.entity_one)<=0 \
                or len(request.link_one)<=0 or len(request.link_one)<=0:
                    response.success=False
                    response.message="Both of the entity names and the link names must be given"
            
            if len(request.entity_one_type)>=0:
                if request.entity_one_type=='object':
                    entity_one=get_entity(self.objects,request.entity_one)
                else:
                    entity_one=get_entity(self.robots,request.entity_one)
            else:
                entity_one=get_entity(self.robots,request.entity_one)
            
            if len(request.entity_two_type)>=0:
                if request.entity_two_type=='object':
                    entity_two=get_entity(self.objects,request.entity_two)
                else:
                    entity_two=get_entity(self.robots,request.entity_two)
            else:
                entity_one=get_entity(self.robots,request.entity_one)
            
            if entity_one is None:
                response.success=False
                response.message="Entity one not found in scene"
                return response
            if entity_two is None:
                response.success=False
                response.message="Entity two not found in scene"
                return response
            
            try:
                self.scene.link_entities(entity_one,entity_two,request.link_one,request.link_two)
            except Exception as e:
                response.success=False
                response.success=str(e)
            
            response.success=True
            response.message="Entities joined sucessfully"
            return  response
        self.join_entities_service=self.ros_node.create_service(JoinEntities,"/join_entities",join_entities_callback)
        
    def setup_start_recording_service(self):
        gs.logger.info("Setting up StartRecording service")
        def start_recording_callback(self,request,response):
            if request.camera_name is None or len(request.camera_name)==0 or \
                request.robot_name is None or len(request.robot_name)==0:
                response.success=False
                response.message="Camera name cant be empty"
                return response
            cams_per_entity=self.cameras.get(request.entity_name,None)
            if cams_per_entity is not None:
                cam=cams_per_entity.get(request.camera_name,None)
                if cam is None:
                    response.success=False
                    response.message="Camera not found"
                    return response
            else:
                response.success=False
                response.message=f"No cameras found for the entity names {request.robot_name}" 
                return response
            if cam._inrecording:
                response.success=False
                response.message="Camera already recording"
                return response    
            cam.start_recording()
            response.sucess=True
            response.message=f"Camera named {request.camera_name} started recording"
            return response
        self.start_recording_service=self.ros_node.create_service(StartRecording,"/start_recording",start_recording_callback)
        
    def setup_pause_recording_service(self):
        gs.logger.info("Setting up PauseRecording service")
        def pause_recording_callback(self,request,response):
            if request.camera_name is None or len(request.camera_name)==0 or \
                request.robot_name is None or len(request.robot_name)==0:
                response.success=False
                response.message="Camera name cant be empty"
            cams_per_entity=self.cameras.get(request.entity_name,None)
            if cams_per_entity is not None:
                cam=cams_per_entity.get(request.camera_name,None)
                if cam is None:
                    response.success=False
                    response.message="Camera not found"
                    return response 
            else:
                response.success=False
                response.message=f"No cameras found for the entity names {request.robot_name}" 
            if not cam._inrecording:
                response.success=False
                response.message="Camera hasnt started recording"
                return response    
            cam.pause_recording()
            response.sucess=True
            response.message=f"Camera named {request.camera_name} paused recording"
            return response
        self.pause_recording_service=self.ros_node.create_service(PauseRecording,"/pause_recording",pause_recording_callback)
    
    def setup_stop_recording_service(self):
        gs.logger.info("Setting up StopRecording service")
        def stop_recording_callback(self,request,response):
            if request.camera_name is None or len(request.camera_name)==0 or \
                request.robot_name is None or len(request.robot_name)==0:
                response.success=False
                response.message="Camera name cant be empty"
            cams_per_entity=self.cameras.get(request.entity_name,None)
            if cams_per_entity is not None:
                cam=cams_per_entity.get(request.camera_name,None)
                if cam is None:
                    response.success=False
                    response.message="Camera not found"
                    return response
            else:
                response.success=False
                response.message=f"No cameras found for the entity names {request.robot_name}"  
            if not cam._inrecording:
                response.success=False
                response.message="Camera hasnt started recording"
                return response    
            cam.stop_recording(save_to_filename=request.file_name,fps=request.fps)
            response.sucess=True
            response.message=f"Camera named {request.camera_name} stopped recording Video saved sucessfully"
            return response
        self.stop_recording_service=self.ros_node.create_service(StopRecording,"/start_recording",stop_recording_callback)
        
    # def joint_visualisation_callback(self,request,response):
    #     if request.axis_length<=0 or request.origin_size<=0 or request.axis_radius<=0:
    #         response.success=False
    #         response.message="The size and the radius of the frame cant be nagative"
    #     self.scene.draw_debug_frame(
    #         request.T,axis_length=request.axis_length,
    #         origin_size=request.origin_size,axis_radius=request.axis_radius)
    #     response.sucess=True
    #     response.message="Joint Frames visualised sucessfully"
        
    # def setup_joint_visualisation_service(self):
    #     self.joint_visualisation_service=self.ros_node.create_service(VisualiseJointStates,"/visualise_joint_states",self.joint_visualisation_callback)
                    
    def setup_set_dofs_physics_attr_service(self):
        gs.logger.info("Setting up SetDofsPhysicsAttributes service")
        def set_dofs_physics_attr_callback(self,request,response):
            if request.robot_name is None:
                response.success=False
                response.message="Robot name must be given"
                return response
            else:
                robot=get_entity(self.robots,request.robot_name)
                if robot is None:
                    response.success=False
                    response.message="Robot with the given name not found in the scene"
                    return response
            if request.attribute is None or len(request.attribute)==0:
                response.success=False
                response.message="Physics attribute to modify was not provided"
            if request.dof_names is None or len(request.dof_names)==0:
                response.success=False
                response.message="No dof names provided"
            if request.value is None or len(request.value)==0:
                response.success=False
                response.message="No values provided"
            if len(request.value)!=len(request.dof_names) and request.attribute.lower()!="force_range":
                response.success=False
                response.message="Length of values and motor_dofs dont match"
            elif len(request.value)!=len(request.dof_names)*2 and request.attribute.lower()=="force_range":
                response.success=False
                response.message="Length of values and motor_dofs dont match"
                
            motors_dof_idx=get_dofs_idx(robot,request.dof_names)
            if request.attribute.lower()=="kp":
                robot.set_dofs_kp(response.value,motors_dof_idx)
            elif request.attribute.lower()=="kv":
                robot.set_dofs_kv(response.value,motors_dof_idx)
            elif request.attribute.lower()=="force_range":
                robot.set_dofs_force_range(
                    response.value[:len(response.value)/2],response.value[len(response.value)/2+1:],motors_dof_idx)
            elif request.attribute.lower()=="armature":
                robot.set_dofs_armature(response.value,motors_dof_idx)
            elif request.attribute.lower()=="stiffness":
                robot.set_dofs_stiffness(response.value,motors_dof_idx)
            elif request.attribute.lower()=="invweight":
                robot.set_dofs_invweight(response.value,motors_dof_idx)
            elif request.attribute.lower()=="damping":
                robot.set_dofs_damping(response.value,motors_dof_idx)
            else:
                response.sucess=False
                response.message="Unknown attribute provided"
                return response
            response.success=True
            response.message="dofs Physics attribute set succesfully"
            
            return response
        self.set_dofs_physics_attr_service=self.ros_node.create_service(SetPhysicsAttributes,"/set_dofs_physics_attribute",set_dofs_physics_attr_callback)
                    
    def setup_set_links_physics_attr_service(self):
        gs.logger.info("Setting up SetLinksPhysicsAttributes service")
        def set_links_physics_attr_callback(self,request,response):
            if request.robot_name is None:
                response.success=False
                response.message="Robot name must be given"
                return response
            else:
                robot=get_entity(self.robots,request.robot_name)
                if robot is None:
                    response.success=False
                    response.message="Robot with the given name not found in the scene"
                    return response
            if request.attribute is None or len(request.attribute)==0:
                response.success=False
                response.message="Physics attribute to modify was not provided"
            if request.names is None or len(request.names)==0:
                response.success=False
                response.message="No names provided"
            if request.value is None or len(request.value)==0:
                response.success=False
                response.message="No values provided"
            if len(request.value)!=len(request.dof_names):
                response.success=False
                response.message="Length of values and links dont match"

            links_idx=get_links_idx(request.names)
            if request.attribute.lower()=="inertial_mass":
                robot.set_links_inertial_mass(response.value,links_idx)
            elif request.attribute.lower()=="invweight":
                robot.set_links_invweight(response.value,links_idx)
            else:
                response.sucess=False
                response.message="Unknown attribute provided"
                return response
            response.success=True
            response.message="Link Physics attribute set succesfully"
            return response
        self.set_links_physics_attr_service=self.ros_node.create_service(SetPhysicsAttributes,"/set_links_physics_attribute",set_links_physics_attr_callback)
               
    def setup_reset_simulator_service(self):
        gs.logger.info("Setting up ResetSimulator service")
        def reset_simulator_callback(self,request,response):
            if request.data:
                self.scene.reset()
                response.success=True
                response.message="reset sucessfull"
                
            return response
        self.reset_simulator_service=self.ros_node.create_service(SetBool,"/RESET_SIMULATOR",reset_simulator_callback)
        