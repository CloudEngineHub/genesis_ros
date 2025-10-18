import genesis as gs
import numpy as np
from builtin_interfaces.msg import Time

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

def create_qos_profile(
    history: str = "keep_last",
    depth: int = 10,
    reliability: str = "reliable",
    durability: str = "volatile"
) -> QoSProfile:
    """
    Create a QoSProfile from string parameters.
    Allowed values (case-insensitive):
    - history: "keep_last", "keep_all"
    - reliability: "reliable", "best_effort"
    - durability: "transient_local", "volatile"
    """
    qos = QoSProfile(depth=depth)

    hist = history.lower()
    if hist == "keep_all":
        qos.history = QoSHistoryPolicy.KEEP_ALL
    elif hist == "keep_last":
        qos.history = QoSHistoryPolicy.KEEP_LAST
    else:
        raise ValueError(f"Invalid history policy: '{history}'")

    rel = reliability.lower()
    if rel == "reliable":
        qos.reliability = QoSReliabilityPolicy.RELIABLE
    elif rel == "best_effort":
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
    else:
        raise ValueError(f"Invalid reliability policy: '{reliability}'")

    dur = durability.lower()
    if dur == "transient_local":
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    elif dur == "volatile":
        qos.durability = QoSDurabilityPolicy.VOLATILE
    else:
        raise ValueError(f"Invalid durability policy: '{durability}'")

    return qos

def quat_angle_difference(q1, q2):
    """
    Compute the angle difference (in radians) between two unit quaternions.
    q1 and q2 must be 4D arrays: [x, y, z, w]
    """
    q1 = np.array(q1, dtype=np.float64)
    q2 = np.array(q2, dtype=np.float64)

    # Normalize to ensure unit quaternions
    q1 /= np.linalg.norm(q1)
    q2 /= np.linalg.norm(q2)

    # Compute the dot product
    dot = np.dot(q1, q2)

    # Clamp the dot product to [-1, 1] to avoid numerical issues
    dot = np.clip(dot, -1.0, 1.0)

    # Angle = 2 * arccos(|dot|) in radians
    angle = 2 * np.arccos(np.abs(dot))
    return angle

def get_entity(entities,idx=None,name=None):
    if idx is None and name is None:
        return None
    elif idx is not None:
        for _,entry_id,entry in entities:
            if entry_id==idx:
                return entry
    elif name is not None:
        for entry_name,_,entry in entities:
            if entry_name==name:
                return entry
            
def get_dofs_idx(robot,joint_names):
    motor_dofs=[]
    if joint_names is not None:
        for joint_name in joint_names:
            if joint_name=="root_joint":
                continue
            for joint in robot.joints:
                if joint.name ==joint_name:
                    motor_dofs.append(joint.dofs_idx_local[0])
        return motor_dofs
    
def get_links_idx(robot,link_names):
    links_idx_local=[]
    if link_names is not None:
        for link_name in link_names:
            for link in robot.links:
                if link.name ==link_name:
                    links_idx_local.append(link.idx_local)
        return links_idx_local
            
def get_current_timestamp(scene):
    timestamp=Time()
    timestamp.sec=int(scene.cur_t)
    timestamp.nanosec=int((scene.cur_t-timestamp.sec)*10e8)
    return timestamp

def get_joint_names(robot):
    joint_names=[]
    dofs_idx_local=[]
    for joint in robot.joints:
        if "root_joint" not in joint.name:
            joint_names.append(joint.name)
            dofs_idx_local.append(joint.dofs_idx_local[0])
    return joint_names,dofs_idx_local

def ros_quat_to_gs_quat(input_quat):
    input_quat[0],input_quat[1:3]=input_quat[3],input_quat[:2]
    return input_quat

def gs_quat_to_ros_quat(input_quat):
    input_quat[3],input_quat[:2]=input_quat[0],input_quat[1:3]
    return input_quat

def make_gs_scene(scene_config):
    sim_config=scene_config["sim"]
    scene=gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=sim_config.get("dt", 1e-2),
            substeps=sim_config.get("substeps", 1),
            substeps_local=sim_config.get("substeps_local", None),
            gravity=sim_config.get("gravity", (0.0, 0.0, -9.81)),
            floor_height=sim_config.get("floor_height", 0.0),
            requires_grad=sim_config.get("requires_grad", False)
            ),
        coupler_options = make_coupler_options(scene_config.get("coupler_config", None)),
        tool_options = gs.options.ToolOptions(
            dt=scene_config.get("tool_config", {}).get("dt", 1e-2),
            floor_height=scene_config.get("tool_config", {}).get("floor_height", 0)
        ),
        rigid_options = make_rigid_options(scene_config.get("rigid_config", None)),
        avatar_options = make_avatar_options(scene_config.get("avatar_config", None)),
        mpm_options = make_mpm_options(scene_config.get("mpm_config", None)),
        sph_options = make_sph_options(scene_config.get("sph_config", None)),
        fem_options = make_fem_options(scene_config.get("fem_config", None)),
        sf_options = make_sf_options(scene_config.get("sf_config", None)),
        pbd_options = make_pbd_options(scene_config.get("pbd_config", None)),
        vis_options = make_vis_options(scene_config.get("vis_config", None)),
        viewer_options = make_viewer_options(scene_config.get("viewer_config", None)),
        profiling_options = gs.options.ProfilingOptions(
            show_FPS=scene_config.get("profiling_options", {}).get("show_FPS", None),
            FPS_tracker_alpha=scene_config.get("profiling_options", {}).get("FPS_tracker_alpha", None)
        ),
        renderer = make_renderer_options(scene_config.get("renderer_config", None)),
        show_viewer = scene_config.get("show_Viewer", None)


    )    
    return scene

def make_rigid_options(rigid_config):
    if rigid_config is None:
        return None
    rigid_options=gs.options.RigidOptions(
        dt=rigid_config.get("dt", None),
        gravity=rigid_config.get("gravity", None),
        enable_collision=rigid_config.get("enable_collision", True),
        enable_joint_limit=rigid_config.get("enable_joint_limit", True),
        enable_self_collision=rigid_config.get("enable_self_collision", True),
        enable_adjacent_collision=rigid_config.get("enable_adjacent_collision", False),
        disable_constraint=rigid_config.get("disable_constraint", False),
        max_collision_pairs=rigid_config.get("max_collision_pairs", 300),
        integrator=rigid_config.get("integrator", gs.integrator.approximate_implicitfast),
        IK_max_targets=rigid_config.get("IK_max_targets", 6),

        # batching info
        batch_links_info=rigid_config.get("batch_links_info", False),
        batch_joints_info=rigid_config.get("batch_joints_info", False),
        batch_dofs_info=rigid_config.get("batch_dofs_info", False),

        # constraint solver
        constraint_solver=rigid_config.get("constraint_solver", gs.constraint_solver.Newton),
        iterations=rigid_config.get("iterations", 50),
        tolerance=rigid_config.get("tolerance", 1e-8),
        ls_iterations=rigid_config.get("ls_iterations", 50),
        ls_tolerance=rigid_config.get("ls_tolerance", 1e-2),
        sparse_solve=rigid_config.get("sparse_solve", False),
        contact_resolve_time=rigid_config.get("contact_resolve_time", None),
        constraint_timeconst=rigid_config.get("constraint_timeconst", 0.01),
        use_contact_island=rigid_config.get("use_contact_island", False),
        box_box_detection=rigid_config.get("box_box_detection", False),

        # hibernation threshold
        use_hibernation=rigid_config.get("use_hibernation", False),
        hibernation_thresh_vel=rigid_config.get("hibernation_thresh_vel", 1e-3),
        hibernation_thresh_acc=rigid_config.get("hibernation_thresh_acc", 1e-2),

        # dynamic properties
        max_dynamic_constraints=rigid_config.get("max_dynamic_constraints", 8),

        # experimental/debug options
        enable_multi_contact=rigid_config.get("enable_multi_contact", True),
        enable_mujoco_compatibility=rigid_config.get("enable_mujoco_compatibility", False),

        # GJK collision detection
        use_gjk_collision=rigid_config.get("use_gjk_collision", False)
    )
    return rigid_options

def make_avatar_options(avatar_config):
    if avatar_config is None:
        return None
    avatar_options=gs.options.AvatarOptions(
        dt=avatar_config.get("dt", None),
        enable_collision=avatar_config.get("enable_collision", False),
        enable_self_collision=avatar_config.get("enable_self_collision", False),
        enable_adjacent_collision=avatar_config.get("enable_adjacent_collision", False),
        max_collision_pairs=avatar_config.get("max_collision_pairs", 300),
        IK_max_targets=avatar_config.get("IK_max_targets", 6),
        max_dynamic_constraints=avatar_config.get("max_dynamic_constraints", 8),
    )
    return avatar_options

def make_mpm_options(mpm_config):
    if mpm_config is None:
        return None
    mpm_options=gs.options.MPMOptions(
        dt=mpm_config.get("dt", None),
        gravity=mpm_config.get("gravity", None),
        particle_size=mpm_config.get("particle_size", None),
        grid_density=mpm_config.get("grid_density", 64),
        enable_CPIC=mpm_config.get("enable_CPIC", False),
        lower_bound=mpm_config.get("lower_bound", (-1.0, -1.0, 0.0)),
        upper_bound=mpm_config.get("upper_bound", (1.0, 1.0, 1.0)),
        use_sparse_grid=mpm_config.get("use_sparse_grid", False),
        leaf_block_size=mpm_config.get("leaf_block_size", 8),
    )
    return mpm_options

def make_sph_options(sph_config):
    if sph_config is None:
        return None
    sph_options=gs.options.SPHOptions(
        dt=sph_config.get("dt", None),
        gravity=sph_config.get("gravity", None),
        particle_size=sph_config.get("particle_size", 0.02),
        pressure_solver=sph_config.get("pressure_solver", "WCSPH"),
        lower_bound=sph_config.get("lower_bound", (-100.0, -100.0, 0.0)),
        upper_bound=sph_config.get("upper_bound", (100.0, 100.0, 100.0)),
        hash_grid_res=sph_config.get("hash_grid_res", None),
        hash_grid_cell_size=sph_config.get("hash_grid_cell_size", None),
        max_divergence_error=sph_config.get("max_divergence_error", 0.1),
        max_density_error_percent=sph_config.get("max_density_error_percent", 0.05),
        max_divergence_solver_iterations=sph_config.get("max_divergence_solver_iterations", 100),
        max_density_solver_iterations=sph_config.get("max_density_solver_iterations", 100),
    )
    return sph_options

def make_pbd_options(pbd_config):
    if pbd_config is None:
        return None
    pbd_options=gs.options.PBDOptions(
        dt=pbd_config.get("dt", None),
        gravity=pbd_config.get("gravity", None),
        max_stretch_solver_iterations=pbd_config.get("max_stretch_solver_iterations", 4),
        max_bending_solver_iterations=pbd_config.get("max_bending_solver_iterations", 1),
        max_volume_solver_iterations=pbd_config.get("max_volume_solver_iterations", 1),
        max_density_solver_iterations=pbd_config.get("max_density_solver_iterations", 1),
        max_viscosity_solver_iterations=pbd_config.get("max_viscosity_solver_iterations", 1),
        particle_size=pbd_config.get("particle_size", 1e-2),
        hash_grid_res=pbd_config.get("hash_grid_res", None),
        hash_grid_cell_size=pbd_config.get("hash_grid_cell_size", None),
        lower_bound=pbd_config.get("lower_bound", (-100.0, -100.0, 0.0)),
        upper_bound=pbd_config.get("upper_bound", (100.0, 100.0, 100.0)),
    )
    return pbd_options

def make_fem_options(fem_config):
    if fem_config is None:
        return None
    fem_options=gs.options.FEMOptions(
        dt=fem_config.get("dt", None),
        gravity=fem_config.get("gravity", None),
        damping=fem_config.get("damping", 0.0),
        floor_height=fem_config.get("floor_height", None),
        use_implicit_solver=fem_config.get("use_implicit_solver", False),
        n_newton_iterations=fem_config.get("n_newton_iterations", 1),
        n_pcg_iterations=fem_config.get("n_pcg_iterations", 500),
        n_linesearch_iterations=fem_config.get("n_linesearch_iterations", 0),
        newton_dx_threshold=fem_config.get("newton_dx_threshold", 1e-6),
        pcg_threshold=fem_config.get("pcg_threshold", 1e-6),
        linesearch_c=fem_config.get("linesearch_c", 1e-4),
        linesearch_tau=fem_config.get("linesearch_tau", 0.5),
        damping_alpha=fem_config.get("damping_alpha", 0.5),
        damping_beta=fem_config.get("damping_beta", 5e-4),
    )
    return fem_options

def make_sf_options(sf_config):
    if sf_config is None:
        return None
    sf_options=gs.options.SFOptions(
        dt=sf_config.get("dt", None),
        res=sf_config.get("res", 128),
        solver_iters=sf_config.get("solver_iters", 500),
        decay=sf_config.get("decay", 0.99),
        T_low=sf_config.get("T_low", 1.0),
        T_high=sf_config.get("T_high", 0.0),
        inlet_pos=sf_config.get("inlet_pos", (0.6, 0.0, 0.1)),
        inlet_vel=sf_config.get("inlet_vel", (0, 0, 1)),
        inlet_quat=sf_config.get("inlet_quat", (1, 0, 0, 0)),
        inlet_s=sf_config.get("inlet_s", 400.0),
    )
    return sf_options

def make_viewer_options(viewer_config):
    if viewer_config is None:
        return None
    viewer_options=gs.options.ViewerOptions(
        res=viewer_config.get("res", None),
        run_in_thread=viewer_config.get("run_in_thread", None),
        refresh_rate=viewer_config.get("refresh_rate", 60),
        max_FPS=viewer_config.get("max_FPS", 60),
        camera_pos=viewer_config.get("camera_pos", (3.5, 0.5, 2.5)),
        camera_lookat=viewer_config.get("camera_lookat", (0.0, 0.0, 0.5)),
        camera_up=viewer_config.get("camera_up", (0.0, 0.0, 1.0)),
        camera_fov=viewer_config.get("camera_fov", 40),
        enable_interaction=viewer_config.get("enable_interaction", False)
    )
    return viewer_options

def make_vis_options(vis_config):
    if vis_config is None:
        return None
    vis_options=gs.options.VisOptions(
        show_world_frame=vis_config.get("show_world_frame", True),
        world_frame_size=vis_config.get("world_frame_size", 1.0),
        show_link_frame=vis_config.get("show_link_frame", False),
        link_frame_size=vis_config.get("link_frame_size", 0.2),
        show_cameras=vis_config.get("show_cameras", False),
        shadow=vis_config.get("shadow", True),
        plane_reflection=vis_config.get("plane_reflection", False),
        env_separate_rigid=vis_config.get("env_separate_rigid", False),
        background_color=vis_config.get("background_color", (0.04, 0.08, 0.12)),
        ambient_light=vis_config.get("ambient_light", (0.1, 0.1, 0.1)),
        visualize_mpm_boundary=vis_config.get("visualize_mpm_boundary", False),
        visualize_sph_boundary=vis_config.get("visualize_sph_boundary", False),
        visualize_pbd_boundary=vis_config.get("visualize_pbd_boundary", False),
        segmentation_level=vis_config.get("segmentation_level", "link"),
        render_particle_as=vis_config.get("render_particle_as", "sphere"),
        particle_size_scale=vis_config.get("particle_size_scale", 1.0),
        contact_force_scale=vis_config.get("contact_force_scale", 0.01),
        n_support_neighbors=vis_config.get("n_support_neighbors", 12),
        n_rendered_envs=vis_config.get("n_rendered_envs", None),
        rendered_envs_idx=vis_config.get("rendered_envs_idx", None),
        lights=vis_config.get("lights", [
            {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 5.0}
        ])
    )
    return vis_options
    
    
def make_coupler_options(coupler_config):
    if coupler_config is None:
        return None
    if coupler_config["type"]=="legacy":
        legacy_coupler_config=coupler_config["legacy"]
        coupler_options=gs.options.LegacyCouplerOptions(
            rigid_mpm=legacy_coupler_config.get("rigid_mpm", True),
            rigid_sph=legacy_coupler_config.get("rigid_sph", True),
            rigid_pbd=legacy_coupler_config.get("rigid_pbd", True),
            rigid_fem=legacy_coupler_config.get("rigid_fem", True),
            mpm_sph=legacy_coupler_config.get("mpm_sph", True),
            mpm_pbd=legacy_coupler_config.get("mpm_pbd", True),
            fem_mpm=legacy_coupler_config.get("fem_mpm", True),
            fem_sph=legacy_coupler_config.get("fem_sph", True)
            )
        return coupler_options
    elif coupler_config["type"]=="sap":
        sap_coupler_config=coupler_config["sap"]
        coupler_options=gs.options.SAPCouplerOptions(
            n_sap_iterations=sap_coupler_config.get("n_sap_iterations", 5),
            n_pcg_iterations=sap_coupler_config.get("n_pcg_iterations", 100),
            n_linesearch_iterations=sap_coupler_config.get("n_linesearch_iterations", 10),
            sap_convergence_atol=sap_coupler_config.get("sap_convergence_atol", 1e-6),
            sap_convergence_rtol=sap_coupler_config.get("sap_convergence_rtol", 1e-5),
            sap_taud=sap_coupler_config.get("sap_taud", 0.1),
            sap_beta=sap_coupler_config.get("sap_beta", 1.0),
            sap_sigma=sap_coupler_config.get("sap_sigma", 1e-3),
            pcg_threshold=sap_coupler_config.get("pcg_threshold", 1e-6),
            linesearch_ftol=sap_coupler_config.get("linesearch_ftol", 1e-6),
            linesearch_max_step_size=sap_coupler_config.get("linesearch_max_step_size", 1.5),
            hydroelastic_stiffness=sap_coupler_config.get("hydroelastic_stiffness", 1e8),
            point_contact_stiffness=sap_coupler_config.get("point_contact_stiffness", 1e8),
            fem_floor_type=sap_coupler_config.get("fem_floor_type", "tet"),
            fem_self_tet=sap_coupler_config.get("fem_self_tet", True)
            )
        return coupler_options
    else:
        return None
    
def make_renderer_options(renderer_config):
    if renderer_config is None:
        return gs.renderers.Rasterizer()
    else:
        raytracer_options = gs.renderers.RayTracer(
            device_index=renderer_config.get("device_index", 0),
            logging_level=renderer_config.get("logging_level", "warning"),
            state_limit=renderer_config.get("state_limit", 2**25),
            tracing_depth=renderer_config.get("tracing_depth", 32),
            rr_depth=renderer_config.get("rr_depth", 0),
            rr_threshold=renderer_config.get("rr_threshold", 0.95),
            env_surface=renderer_config.get("env_surface", None),
            env_radius=renderer_config.get("env_radius", 1000.0),
            env_pos=renderer_config.get("env_pos", (0.0, 0.0, 0.0)),
            env_euler=renderer_config.get("env_euler", (0.0, 0.0, 0.0)),
            env_quat=renderer_config.get("env_quat", None),
            lights=renderer_config.get(
                "lights",
                [{"pos": (0.0, 0.0, 10.0), "color": (1.0, 1.0, 1.0), "intensity": 10.0, "radius": 4.0}]
            ),
            normal_diff_clamp=renderer_config.get("normal_diff_clamp", 180),
        )
        return raytracer_options
    
def make_morph(morph_config):
    if morph_config is None:
        return None
    entity_path=morph_config["path"]
    morph_type=morph_config.get("type", None)
    if morph_type=="drone":
        morph=gs.mprphs.Drone(
            file = morph_config.get("file"),
            scale = morph_config.get("scale", 1.0),
            pos = morph_config.get("pos", (0.0, 0.0, 0.0)),
            euler = morph_config.get("euler", (0.0, 0.0, 0.0)),
            quat = morph_config.get("quat"),
            decimate = morph_config.get("decimate", True),
            decimate_face_num = morph_config.get("decimate_face_num", 500),
            decimate_aggressiveness = morph_config.get("decimate_aggressiveness", 5),
            convexify = morph_config.get("convexify", True),
            decompose_nonconvex = morph_config.get("decompose_nonconvex", False),
            decompose_object_error_threshold = morph_config.get("decompose_object_error_threshold", 0.15),
            decompose_robot_error_threshold = morph_config.get("decompose_robot_error_threshold", float("inf")),
            coacd_options = morph_config.get("coacd_options"),
            visualization = morph_config.get("visualization", True),
            collision = morph_config.get("collision", True),
            prioritize_urdf_material = morph_config.get("prioritize_urdf_material", False),
            model = morph_config.get("model", "CF2X"),
            COM_link_name = morph_config.get("COM_link_name"),
            propellers_link_names = morph_config.get("propellers_link_names"),
            propellers_link_name = morph_config.get("propellers_link_name", ("prop0_link", "prop1_link", "prop2_link", "prop3_link")),
            propellers_spin = morph_config.get("propellers_spin", (-1, 1, -1, 1)),
            merge_fixed_links = morph_config.get("merge_fixed_links", True),
            links_to_keep = morph_config.get("links_to_keep", []),
            default_armature = morph_config.get("default_armature", 0.1),
            default_base_ang_damping_scale = morph_config.get("default_base_ang_damping_scale", 1e-5)
        )
    elif morph_type=="URDF" or entity_path.lower().endswith(".urdf"):
        morph=gs.morphs.URDF(file=entity_path,
                        pos=morph_config.get("pos", (0.0, 0.0, 0.0)),
                        euler=morph_config.get("euler", (0.0, 0.0, 0.0)),
                        quat=morph_config.get("quat", None),
                        visualization=morph_config.get("visualization", True),
                        collision=morph_config.get("collision", True),
                        requires_jac_and_IK=morph_config.get("requires_jac_and_IK", True),
                        scale=morph_config.get("scale", 1.0),
                        convexify=morph_config.get("convexify", None),
                        recompute_inertia=morph_config.get("recompute_inertia", False),
                        fixed=morph_config.get("fixed", False),
                        prioritize_urdf_material=morph_config.get("prioritize_urdf_material", False),
                        merge_fixed_links=morph_config.get("merge_fixed_links", True),
                        links_to_keep=morph_config.get("links_to_keep", [])
                        )
    elif morph_type=="MJCF" or entity_path.lower().endswith(".xml"):
        morph=gs.morphs.MJCF(file=entity_path,
                        pos = morph_config.get("pos", None),
                        euler = morph_config.get("euler", None),
                        quat = morph_config.get("quat", None),
                        visualization = morph_config.get("visualization", True),
                        collision = morph_config.get("collision", True),
                        requires_jac_and_IK = morph_config.get("requires_jac_and_IK", True),
                        scale = morph_config.get("scale", 1.0),
                        convexify = morph_config.get("convexify", None),
                        recompute_inertia = morph_config.get("recompute_inertia", False)
                        )
    else:
        morph=gs.morphs.Mesh(file=entity_path,
                        pos = morph_config.get("pos", (0.0, 0.0, 0.0)),
                        euler = morph_config.get("euler", (0.0, 0.0, 0.0)),
                        quat = morph_config.get("quat", None),
                        visualization = morph_config.get("visualization", True),
                        collision = morph_config.get("collision", True),
                        requires_jac_and_IK = morph_config.get("requires_jac_and_IK", False),
                        scale = morph_config.get("scale", 1.0),
                        convexify = morph_config.get("convexify", None),
                        recompute_inertia = morph_config.get("recompute_inertia", False),
                        parse_glb_with_trimesh = morph_config.get("parse_glb_with_trimesh", False),
                        fixed = morph_config.get("fixed", False),
                        group_by_material = morph_config.get("group_by_material", True),
                        merge_submeshes_for_collision = morph_config.get("merge_submeshes_for_collision", True),
                        decimate = morph_config.get("decimate", True),
                        decimate_face_num = morph_config.get("decimate_face_num", 500),
                        decompose_nonconvex = morph_config.get("decompose_nonconvex", None),
                        coacd_options = morph_config.get("coacd_options", None),
                        order = morph_config.get("order", 1),
                        mindihedral = morph_config.get("mindihedral", 10),
                        minratio = morph_config.get("minratio", 1.1),
                        nobisect = morph_config.get("nobisect", True),
                        quality = morph_config.get("quality", True),
                        verbose = morph_config.get("verbose", 0),
                        force_retet = morph_config.get("force_retet", False)
                        )
    return morph

def make_material(material_config):
    #TODO: add more materials
    if material_config is None:
        return None
    if material_config["type"].lower() =="rigid":
        material=gs.materials.Rigid(
            rho = material_config.get("rho", 200.0),
            friction = material_config.get("friction", None),
            needs_coup = material_config.get("needs_coup", True),
            coup_friction = material_config.get("coup_friction", 0.1),
            coup_softness = material_config.get("coup_softness", 0.002),
            coup_restitution = material_config.get("coup_restitution", 0.0),
            sdf_cell_size = material_config.get("sdf_cell_size", 0.005),
            sdf_min_res = material_config.get("sdf_min_res", 32),
            sdf_max_res = material_config.get("sdf_max_res", 128),
            gravity_compensation = material_config.get("gravity_compensation", 0)
            )
    elif material_config["type"].lower() =="fem.muscle":
        material=gs.materials.FEM.Muscle(  # to allow setting group
            E=float(material_config.get("E", 1e6)),
            nu=material_config.get("nu", 0.2),
            rho=material_config.get("rho", 1000.0),
            model=material_config.get("model", "neohooken"),
        ),
    elif material_config["type"].lower() =="hybrid":
        material_rigid=make_material(material_config.get("material_rigid", None))
        material_soft=make_material(material_config.get("material_soft", None))
        material=gs.materials.Hybrid(
            material_rigid=material_rigid,
            material_soft=material_soft,
            fixed=material_config.get('fixed', False),
            use_default_coupling=material_config.get('use_default_coupling', False),
            damping=material_config.get('damping', 0.0),
            thickness=material_config.get('thickness', 0.05),
            soft_dv_coef=material_config.get('soft_dv_coef', 0.01),
            func_instantiate_rigid_from_soft=None,
            func_instantiate_soft_from_rigid=None,
            func_instantiate_rigid_soft_association=None,
        )
    return material
        
def make_surface(surface_config):
    if surface_config is None:
        return None
    surface=gs.surfaces.Default(
                    color = surface_config.get("color", None),
                    opacity = surface_config.get("opacity", None),
                    roughness = surface_config.get("roughness", None),
                    metallic = surface_config.get("metallic", None),
                    emissive = surface_config.get("emissive", None),
                    ior = surface_config.get("ior", None),
                    opacity_texture = surface_config.get("opacity_texture", None),
                    roughness_texture = surface_config.get("roughness_texture", None),
                    metallic_texture = surface_config.get("metallic_texture", None),
                    normal_texture = surface_config.get("normal_texture", None),
                    emissive_texture = surface_config.get("emissive_texture", None),
                    default_roughness = surface_config.get("default_roughness", 1.0),
                    vis_mode = surface_config.get("vis_mode", None),
                    smooth = surface_config.get("smooth", True),
                    double_sided = surface_config.get("double_sided", None),
                    normal_diff_clamp = surface_config.get("normal_diff_clamp", 180),
                    recon_backend = surface_config.get("recon_backend", 'splashsurf'),
                    generate_foam = surface_config.get("generate_foam", False),
                    foam_options = surface_config.get("foam_options", None),
                    diffuse_texture = surface_config.get("diffuse_texture", None),
                    specular_trans = surface_config.get("specular_trans", 0.0),
                    diffuse_trans = surface_config.get("diffuse_trans", 0.0)
                    )
    return surface
