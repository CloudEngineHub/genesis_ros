"""Handler for entity lifecycle services"""

from simulation_interfaces.msg import Result, EntityInfo, EntityCategory
from simulation_interfaces.srv import SpawnEntity
from gs_simulation_interfaces.scene_manager import SceneManager
import genesis as gs
import re

ROS_NAME_PATTERN = r"^[A-Za-z/~][A-Za-z0-9_/]*$"


class EntityLifecycleHandler:
    """Handles SpawnEntity and DeleteEntity services"""

    # Class-level constants
    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    RESULT_INCORRECT_STATE = 3
    RESULT_OPERATION_FAILED = 4

    NAME_NOT_UNIQUE = 101
    NAME_INVALID = 102
    UNSUPPORTED_FORMAT = 103
    NO_RESOURCE = 104
    NAMESPACE_INVALID = 105
    RESOURCE_PARSE_ERROR = 106
    MISSING_ASSETS = 107
    UNSUPPORTED_ASSETS = 108
    INVALID_POSE = 109

    def __init__(self, node, scene_manager):
        """Initialize the entity lifecycle handler."""
        self.node = node
        self.scene_manager = scene_manager

    def spawn_entity_callback(self, request, response):
        """Add a new entity to the scene and register its metadata."""
        """Spawn an entity (robot, object) from URDF/SDF/USD/MJCF"""
        gs.logger.info(f"SpawnEntity service called: {request.name}")

        response.result = Result()

        # Validate scene state
        if self.scene_manager.scene is None:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Scene is not initialized"
            gs.logger.critical("Scene is not initialized")
            return response

        if self.scene_manager.scene.is_built:
            response.result.result = self.RESULT_INCORRECT_STATE
            response.result.error_message = "Scene is already built, all entities must be added before the scene is built"
            gs.logger.critical(
                "Scene is already built, all entities must be added before the scene is built"
            )
            return response

        # Validate resource
        if not request.entity_resource.uri:
            response.result.result = self.NO_RESOURCE
            response.result.error_message = "Presource uri path is empty"
            return response

        try:
            # Determine the entity resource type (URDF, MJCF, or SDF)
            if request.entity_resource.uri.endswith(".urdf"):
                entity_morph = gs.morphs.URDF(
                    request.entity_resource.uri,
                    pos=request.initial_pose.position,
                    rot=SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
                )
            elif request.entity_resource.uri.endswith(".xml"):
                entity_morph = gs.morphs.MJCF(
                    request.entity_resource.uri,
                    pos=request.initial_pose.position,
                    rot=SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
                )
            else:
                entity_morph = gs.morphs.SDF(
                    request.entity_resource.uri,
                    pos=request.initial_pose.position,
                    rot=SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
                )

            entity = self.scene_manager.scene.add_Entity(entity_morph)

        except Exception as e:
            response.result.result = self.UNSUPPORTED_FORMAT
            response.result.error_message = str(e)
            return response

        # Determine entity name
        entity_name = request.name
        if entity_name == "":
            if not request.allow_renaming:
                response.result.result = self.NAME_INVALID
                response.result.error_message = (
                    "Name is empty and allow_renaming is false"
                )
                return response
            else:
                request.entity_name = entity.uid[-5:]
        elif not re.fullmatch(ROS_NAME_PATTERN, entity_name):
            if not request.allow_renaming:
                response.result.result = self.NAME_INVALID
                response.result.error_message = (
                    f"Entity name '{entity_name}' is invalid"
                )
                return response

        # Check namespace validity
        if request.entity_namespace != "":
            if not re.match(ROS_NAME_PATTERN, request.entity_namespace):
                if not request.allow_renaming:
                    response.result.result = self.NAMESPACE_INVALID
                    response.result.error_message = (
                        f"Entity namespace '{request.entity_namespace}' is invalid"
                    )
                    return response
                else:
                    request.entity_namespace = entity_name
                    response.result.error_message = f"Entity namespace '{request.entity_namespace}' is invalid, using the entity_name for the namesppace"
            elif request.entity_namespace in self.scene_manager.entity_namespaces:
                request.entity_namespace = (
                    response.entity_namespace + "_" + entity.uid[-5:]
                )

        # Check for name conflicts
        if entity_name in self.scene_manager.entities_info.keys():
            if not request.allow_renaming:
                response.result.result = self.NAME_NOT_UNIQUE
                response.result.error_message = (
                    f"Entity name '{entity_name}' already exists"
                )
                return response
            # Generate unique name
            request.entity_name = f"{entity_name}_{entity.uid[-5:]}"

        # Track the entity
        self.scene_manager.entities_info[entity_name] = {
            "entity_attr": entity,
            "entity_morph": entity_morph,
            "resource": request.entity_resource.uri,
            "namespace": request.entity_namespace,
            "initial_pos": request.initial_pose.position,
            "initial_quat": SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
            "initialisation_pending": True,
            "category": EntityCategory(),
            "description": f"Spawned entity: {entity_name}",
            "tags": [],
            "sensors": {
                "cameras": [],
                "lidars": [],
                "imus": [],
                "contacts": [],
                "contact_forces": [],
            },
        }

        response.result.result = self.RESULT_OK
        response.entity_name = entity_name

        gs.logger.info(f"Entity spawned successfully: {entity_name}")
        return response
