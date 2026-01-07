"""State manager for OSI simulation interface"""

from simulation_interfaces.msg import SimulationState


class SceneManager:
    """Manages the Genesis simulation scene, tracking entities, sensors, and execution state."""

    PLAYING = 1
    STOPPED = 0
    PAUSED = 2

    def __init__(self, scene):
        """Initialize the scene manager with a Genesis scene object."""
        # Simulation state
        self.current_state_code = self.PLAYING
        self.scene = scene
        self.time_offset = 0

        # Entity tracking
        self.entities_info = {}

        # World tracking
        self.world_info = {}  # Currently loaded WorldResource
        self.latest_timestamp = None
        self.PENDING_REST = False

    @staticmethod
    def wxyz_to_xyzw(quat):
        return [quat[1], quat[2], quat[3], quat[0]]

    @staticmethod
    def xyzw_to_wxyz(quat):
        return [quat[3], quat[0], quat[1], quat[2]]

    def get_time(self):
        return self.scene.cur_t
