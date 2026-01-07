"""Handler for world management services"""

from simulation_interfaces.msg import Result, WorldResource, Resource
import genesis as gs
from gs_simulation_interfaces.scene_manager import SceneManager


class WorldManagementHandler:
    """Handles LoadWorld, UnloadWorld, GetCurrentWorld, GetAvailableWorlds services"""

    RESULT_OK = 1
    RESULT_NOT_FOUND = 2
    RESULT_INCORRECT_STATE = 3
    NO_RESOURCE = 4
    UNSUPPORTED_FORMAT = 5

    def __init__(self, node, scene_manager):
        """Initialize the world management handler."""
        self.node = node
        self.scene_manager = scene_manager

    def load_world_callback(self, request, response):
        """Load a static world model or environment into the scene."""
        """Load a world/scene file"""
        gs.logger.info("LoadWorld service called")

        response.result = Result()
        response.world = WorldResource()

        # Validate scene state
        if self.scene_manager.scene is None:
            response.result.result = self.RESULT_NOT_FOUND
            response.result.error_message = "Scene is not initialized"
            return response

        if self.scene_manager.scene.is_built:
            response.result.result = self.RESULT_INCORRECT_STATE
            response.result.error_message = "Scene is already built, the world must be added before the scene is built"
            return response

        # Validate resource
        if not request.entity_resource.uri:
            response.result.result = self.NO_RESOURCE
            response.result.error_message = "uri is empty"
            return response

        try:
            # Determine the entity resource type (URDF, MJCF, or SDF)
            if request.entity_resource.uri.endswith(".urdf"):
                world_morph = gs.morphs.URDF(
                    request.entity_resource.uri,
                    pos=request.initial_pose.position,
                    rot=SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
                )
            elif request.entity_resource.uri.endswith(".xml"):
                world_morph = gs.morphs.MJCF(
                    request.entity_resource.uri,
                    pos=request.initial_pose.position,
                    rot=SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
                )
            else:
                world_morph = gs.morphs.File(
                    request.entity_resource.uri,
                    pos=request.initial_pose.position,
                    rot=SceneManager.xyzw_to_wxyz(request.initial_pose.orientation),
                )

            world = self.scene_manager.scene.add_entity(world_morph)

        except Exception as e:
            response.result.result = self.UNSUPPORTED_FORMAT
            response.result.error_message = str(e)
            return response

        # Initialize entity info
        self.scene_manager.world_info["world_name"] = world.uid["-5:"]
        self.scene_manager.world_info["world_resource"] = request.entity_resource.uri
        self.scene_manager.world_info["description"] = ""
        self.scene_manager.world_info["tags"] = []

        response.result.result = self.RESULT_OK
        response.world = WorldResource()
        response.world.name = self.scene_manager.world_info.get("world_name", "")
        response.world.resource = Resource()
        response.world.resource.uri = self.scene_manager.world_info.get(
            "world_resource", ""
        )
        response.world.description = self.scene_manager.world_info.get(
            "description", ""
        )
        response.world.tags = self.scene_manager.world_info.get("tags", [])

        gs.logger.info(
            f"World spawned successfully: {self.scene_manager.world_info['world_name']}"
        )
        return response

    def get_current_world_callback(self, request, response):
        """Retrieve information about the currently loaded world."""
        """Get currently loaded world info"""
        gs.logger.info("GetCurrentWorld service called")

        response.result = Result()

        if self.scene_manager.world_info is None:
            response.result.result = self.NO_WORLD_LOADED
            response.result.error_message = "No world currently loaded"
            return response

        response.world = WorldResource()
        response.world.name = self.scene_manager.world_info.get("world_name", "")
        response.world.world_resource = Resource()
        response.world.world_resource.uri = self.scene_manager.world_info.get(
            "world_resource", ""
        )
        response.world.description = self.scene_manager.world_info.get(
            "description", ""
        )
        response.world.tags = self.scene_manager.world_info.get("tags", [])
        response.result.result = self.RESULT_OK
        return response
