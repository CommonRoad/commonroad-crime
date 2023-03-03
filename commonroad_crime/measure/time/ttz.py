__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import math
from shapely.geometry import Point
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np

from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.state import InitialState
from commonroad.scenario.lanelet import LaneletType

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor
from commonroad_crime.measure.time.ttc import TTC

logger = logging.getLogger(__name__)


class TTZ(CriMeBase):
    """
    The distance to the zebra/crosswalk divided by the speed at any given moment in time.
    """
    measure_name = TypeTime.TTZ

    def __init__(self, config: CriMeConfiguration):
        super(TTZ, self).__init__(config)
        self._zebra_list = []

    def compute(self, time_step: int = 0, other_id = None):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.time_step = time_step
        zebra_list = []
        for ll in self.sce.lanelet_network.lanelets:
            if LaneletType.CROSSWALK in ll.lanelet_type:
                zebra_list.append(ll)
        if zebra_list:
            ttz_list = []
            for zebra in zebra_list:
                init_state = InitialState(**{"position": zebra.polygon.center,
                                             "orientation": np.arctan(
                                                 (zebra.center_vertices[1][1] - zebra.center_vertices[0][1]) /
                                                 (zebra.center_vertices[1][0] - zebra.center_vertices[0][0])),
                                             "velocity": 0.})
                obstacle_center_shape = zebra.polygon.translate_rotate(translation=-zebra.polygon.center, angle=0.)
                zebra_obs = \
                    StaticObstacle(self.sce.generate_object_id(), ObstacleType.CONSTRUCTION_ZONE,
                                   obstacle_center_shape, init_state,)

                self._zebra_list.append(zebra_obs)
                if self.configuration.scenario:
                    self.configuration.scenario.add_objects(zebra_obs)
                else:
                    self.configuration.scene.add_objects(zebra_obs)
                ttc_object = TTC(self.configuration)
                ttz_list.append(
                    ttc_object.compute(zebra_obs.obstacle_id, self.time_step)
                )
            if min(ttz_list) is not math.inf:
                self.value = utils_gen.int_round(min(ttz_list), 2)
            else:
                self.value = min(ttz_list)
        else:
            utils_log.print_and_log_info(logger, f"*\t\t there exists no zebra")
            self.value = math.inf
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=10))
        self.rnd.render()
        # ------------- Plot zebra crossing ------------------

        def imshow_affine(ax, z, *args, **kwargs):
            im = ax.imshow(z, *args, **kwargs)
            x1, x2, y1, y2 = im.get_extent()
            im._image_skew_coordinate = (x2, y1)
            return im

        pictogram = plt.imread(self.configuration.general.path_icons + 'crosswalk.png')
        for zebra in self._zebra_list:
            obs_zebra = zebra.obstacle_shape.shapely_object
            # get minimum bounding box around polygon
            box = obs_zebra.minimum_rotated_rectangle

            # get coordinates of polygon vertices
            x, y = box.exterior.coords.xy

            # get length of bounding box edges
            edge_length = (Point(x[0], y[0]).distance(Point(x[1], y[1])), Point(x[1], y[1]).distance(Point(x[2], y[2])))

            # get length of polygon as the longest edge of the bounding box
            width = max(edge_length)

            # get width of polygon as the shortest edge of the bounding box
            height = min(edge_length)

            xi, yi, deg = zebra.initial_state.position[0], zebra.initial_state.position[1], \
                          math.degrees(zebra.initial_state.orientation)

            im = imshow_affine(self.rnd.ax, pictogram, interpolation='none',
                               extent=[0, width, 0, height], clip_on=True,
                               alpha=1.0, zorder=15)
            center_x, center_y = width / 2, height / 2
            im_trans = (mtransforms.Affine2D()
                        .rotate_deg_around(center_x, center_y, deg)
                        .translate(xi - center_x, yi - center_y)
                        + self.rnd.ax.transData)
            im.set_transform(im_trans)
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        plt.title(f"{self.measure_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output,
                               self.time_step)
        else:
            plt.show()
