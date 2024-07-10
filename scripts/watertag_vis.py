#!/usr/bin/env python3
"""
Information visualization Watertag app.
Displays real-time position of the AUV LoLo
measured by a USBL top-side unit mounted on
a service boat. Information is transferred via
ROS.
"""
import time
import utm
import collections
import datetime
import json

import rospy
import rospkg
from dmac.msg import mUSBLFix
from dmac.msg import DMACPayload
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgEkfQuat
from sensor_msgs.msg import NavSatFix

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import tilemapbase

import numpy as np
from scipy.spatial.transform import Rotation

# USBL measurement types.
RANGE_ONLY = 0
AZIMUTH_ONLY = 1
FULL_FIX = 2
CARTESIAN = 3

class WatertagVisualizer:

    def __init__(self, grid_size, use_ahrs_rot, degree_padding, zoom_level):

        self.fig, self.ax = plt.subplots(figsize=(9,9), dpi=100)
        self.grid_size = grid_size
        self.is_plot_init = False
        self.extent = None
        self.mission = None
        self.has_mission_plan = False
        self.degree_range = degree_padding
        self.zoom_level = zoom_level

        # Plot instructions and legend.
        self.legend = self.ax.text(0.125, 0.16, self._get_legend(),
                                   transform=self.fig.transFigure,
                                   fontsize=9, verticalalignment='top')
        # Uncomment if you want instructions on the window, not very useful.
        # self.instructions = self.ax.text(0.7, 0.13, self._get_instructions(),
                                         # transform=self.fig.transFigure,
                                         # fontsize=8, verticalalignment='top')
        self.fix_display = self.ax.text(0.015, 0.1, self._get_fixes(),
                                        transform=self.fig.transFigure,
                                        fontsize=8, verticalalignment='top')
        self.pos_display = self.ax.text(0.5, 0.1, self._get_pos_msgs(),
                                        transform=self.fig.transFigure,
                                        fontsize=8, verticalalignment='top')

        # Mission plot.
        self.mission_plot = None
        # Annotation box for the waypoints.
        self.wp_label = self.ax.annotate("", xy=(0,0), xytext=(20,20),
                                         textcoords="offset points",
                                         bbox=dict(boxstyle="round", fc="w"),
                                         arrowprops=dict(arrowstyle="->"))
        self.wp_label.set_visible(False)

        # User related variables.
        self.is_user_init = False
        self.use_ahrs_rot = use_ahrs_rot
        self.user_scatt = plt.scatter([],[])
        self.user_aux_label = self.ax.annotate("", xy=(0,0))
        self.user_label = self.ax.annotate("", xy=(0,0))
        self.user_x_data = []
        self.user_y_data = []
        self.user_vel = 0.0
        self.user_heading_enu = 0.0
        self.user_heading_ned = 0.0
        self.user_rot_ned = None
        self.user_rot_enu = None
        self.new_gps_fix = False
        self.last_user_update = 0.0

        # AUV related variables (as measured by the USBL).
        self.auv_scatt = plt.scatter([],[])
        self.auv_aux_label = self.ax.annotate("", xy=(0,0))
        self.auv_label = self.ax.annotate("", xy=(0,0))
        self.auv_range = 0.0
        self.auv_x_data = []
        self.auv_y_data = []
        self.auv_z_data = 0.0
        self.auv_buffer = collections.deque(maxlen=5)
        self.last_auv_update = 0.0
        # AUV-related variables (as relayed by the AUV) i.e. AUV's INS.
        self.ins_scatt = plt.scatter([],[])
        self.ins_x_data = collections.deque(maxlen=5000)
        self.ins_y_data = collections.deque(maxlen=5000)
        self.ins_z_data = 0.0
        self.ins_buffer = collections.deque(maxlen=5)
        self.last_ins_update = 0.0
        self.new_pos_msg = False

        # USBL full fix plotting
        self.usbl_scatt = plt.scatter([],[])
        self.raw_usbl_x = collections.deque(maxlen=5000)
        self.raw_usbl_y = collections.deque(maxlen=5000)
        self.new_usbl_fix = False
        # USBL azimuth only plotting
        self.bearing_x_data = 0.0
        self.bearing_y_data = 0.0
        self.bearing_buffer = collections.deque(maxlen=10)
        self.new_bearing_fix = False

        # Useful for paths.
        self.rospack = rospkg.RosPack()

        #TODO: for debugging.
        self.real_scatt = plt.scatter([],[])
        self.real_aux_label = self.ax.annotate("", xy=(0,0))
        self.real_label = self.ax.annotate("", xy=(0,0))
        self.real_x_data = []
        self.real_y_data = []
        self.real_buffer = collections.deque(maxlen=2)


    def _get_instructions(self):
        text =  "Instructions\n"
        text += "Use the buttons on the lower left corner to,\n"
        text += "in order from left to right:\n"
        text += " - go back to the original zoomed-out view,\n"
        text += " - go to the previous view (if available),\n"
        text += " - go to the next view (if available),\n"
        text += " - pan plot (click and drag),\n"
        text += " - zoom to region (click and drag),\n"
        text += " - adjust plot in window (not available),\n"
        text += " - save the current view as image.\n"
        return text

    def _get_legend(self):
        text = "h: heading in NED [$\degree$]\t"
        text += "t: time since last update [s]\t"
        text += "r: relative range [m]\t"
        text += "grid size: {} [m]\t".format(self.grid_size)
        return text

    def _get_fixes(self):
        """
        Add a small text display to the bottom of the GUI and write down
        a small history of USBL fixes.
        """
        text = "USBL history, depth in [m], time in UTC:\n"
        if not self.is_plot_init:
            text += "1:\n"
            text += "2:\n"
            text += "3:\n"
            text += "4:\n"
            text += "5:\n"
        else:
            n = len(self.auv_buffer)
            for i in range(n):
                fix = self.auv_buffer[n-i-1]
                fix_str = self.fix_to_string(fix)
                time_str = datetime.datetime.fromtimestamp(fix[-1]).replace(microsecond=0)
                text += "{0}. {1} @ {2}\n".format(str(i), fix_str, time_str)
        return text

    def _get_pos_msgs(self):
        """
        Add a small text display to the bottom of the GUI and write down
        a small history of POS msgs.
        """
        text = "POS history, depth in [m], time in UTC:\n"
        if not self.is_plot_init:
            text += "1:\n"
            text += "2:\n"
            text += "3:\n"
            text += "4:\n"
            text += "5:\n"
        else:
            n = len(self.ins_buffer)
            for i in range(n):
                fix = self.ins_buffer[n-i-1]
                fix_str = self.fix_to_string(fix)
                time_str = datetime.datetime.fromtimestamp(fix[-1]).replace(microsecond=0)
                text += "{0}. {1} @ {2}\n".format(str(i), fix_str, time_str)
        return text

    def fix_to_string(self, fix):
        """
        It is what it is.
        """
        x = str(np.round(fix[0],7))
        y = str(np.round(fix[1],7))
        z = str(np.round(fix[2],7))
        text = "lat:{0}  lon:{1}   depth: {2}".format(y, x, z)
        return text

    def init_plot(self, tiles, mission_file):
        # Center the map at the user's coords by default.
        center_at_user = True
        # Check if we have a valid mission to center the map on.
        if mission_file != "None":
            mid_lat, mid_lon = self.load_mission(mission_file)
            center_at_user = False
            self.has_mission_plan = True
            if mid_lat == None or mid_lon == None:
                # This means that loading failed.
                center_at_user = True
                self.has_mission_plan = False
        # Otherwise use the user's GPS position to center the map.
        else:
            if not self.is_user_init:
                return

        if center_at_user:
            mid_lon = self.user_x_data
            mid_lat = self.user_y_data

        rospy.loginfo("(WaterTag) Initializing visualization...")

        # Setup the window and plot.
        self.fig.canvas.set_window_title("WaterTag")
        plt.subplots_adjust(top=0.995)
        # self.ax.set_title("tracking mission plan: " + self.mission["mission_name"])
        self.ax.xaxis.set_visible(False)
        self.ax.yaxis.set_visible(False)

        self.extent = tilemapbase.Extent.from_lonlat(mid_lon - self.degree_range, mid_lon + self.degree_range,
                                                     mid_lat - self.degree_range, mid_lat + self.degree_range)
        self.extent = self.extent.to_aspect(1.0)
        # plotter = tilemapbase.Plotter(self.extent, tiles, width=800)
        plotter = tilemapbase.Plotter(self.extent, tiles, zoom=self.zoom_level)
        plotter.plot(self.ax)

        # Add north rose.
        rospy.loginfo("(WaterTag) Inserting rose...")
        path = self.rospack.get_path('usbl_tools')
        arrow = plt.imread(path + "/imgs/north_rose.png")
        imagebox = OffsetImage(arrow, zoom=0.05)
        x = self.ax.get_xlim()[1]-5e-6
        y = self.ax.get_ylim()[1]+5e-6
        self.north_box = AnnotationBbox(imagebox, xy=(x,y), xycoords="data",
                                        frameon=False)
        self.ax.add_artist(self.north_box)

        # Plot mission plan.
        if self.has_mission_plan:
            self.plot_mission_plan()

        # Finally, add grid.
        self.add_grid()

        # Connect the hover event if we have a mission plan.
        if self.has_mission_plan:
            self.fig.canvas.mpl_connect("motion_notify_event", self.hover)

        self.is_plot_init = True

    def load_mission(self, mission_file):
        try:
            with open(mission_file) as f:
                self.mission = json.load(f)
            # If loading the json worked, initiate plot around the mission coordinates.
            self.waypoints = self.mission['waypoints']
            rospy.loginfo("(WaterTag) Loading mission {} with {} waypoints".format(self.mission['mission_name'],
                                                                                   len(self.waypoints)))
            lats = []
            lons = []
            for w in self.waypoints:
                lats.append(w['lat'])
                lons.append(w['lon'])
            mid_lat = (min(lats) + max(lats)) / 2.0
            mid_lon = (min(lons) + max(lons)) / 2.0
        except:
            rospy.logwarn("(WaterTag) Could not read {} as a json dictionary".format(mission_file))
            rospy.logwarn("(WaterTag) Initializing plot without mission plan.")
            return None, None

        return mid_lat, mid_lon

    def plot_mission_plan(self):
        rospy.loginfo("(WaterTag) Overlaying mission plan.")
        x_points = [tilemapbase.project(w['lon'], w['lat'])[0] for w in self.waypoints]
        y_points = [tilemapbase.project(w['lon'], w['lat'])[1] for w in self.waypoints]
        names = [w['name'] for w in self.waypoints]

        self.mission_plot, = self.ax.plot(x_points, y_points, color='k',
                                         marker='o', markersize=3.0, markerfacecolor='r',
                                         linestyle=':', linewidth=0.5, alpha=0.3)

        for i, n in enumerate(names):
            self.ax.annotate(n, (x_points[i], y_points[i]), fontsize='x-small')

    def _waypoint_to_text(self, i):
        text =  str(self.waypoints[i]["name"]) + "\n"
        text += "lat: " + str(round(self.waypoints[i]["lat"],2)) + "\n"
        text += "lon: " + str(round(self.waypoints[i]["lon"],2)) + "\n"
        text += "alt: " + str(self.waypoints[i]["altitude"]) + "\n"
        text += "dep: " + str(self.waypoints[i]["depth"]) + "\n"
        return text

    def update_waypoint_label(self, ind):
        x, y = self.mission_plot.get_data()
        self.wp_label.xy = (x[ind["ind"][0]], y[ind["ind"][0]])
        try:
            text = self._waypoint_to_text(int(ind["ind"]))
        except:
            return

        self.wp_label.set_text(text)
        self.wp_label.get_bbox_patch().set_alpha(0.6)

    def hover(self, event):
        vis = self.wp_label.get_visible()
        if event.inaxes == self.ax:
            cont, ind = self.mission_plot.contains(event)
            if cont:
                self.update_waypoint_label(ind)
                self.wp_label.set_visible(True)
                self.fig.canvas.draw_idle()
            else:
                if vis:
                    self.wp_label.set_visible(False)
                    self.fig.canvas.draw_idle()

    def add_grid(self):

        # We'll use UTM back and forth to have the right scaling.
        min_lon_lat = tilemapbase.to_lonlat(self.extent.xmin, self.extent.ymin)
        max_lon_lat = tilemapbase.to_lonlat(self.extent.xmax, self.extent.ymax)
        lon_limits = [min_lon_lat[0], max_lon_lat[0]]
        lat_limits = [min_lon_lat[1], max_lon_lat[1]]

        x_min, y_min, zone, band = utm.from_latlon(min(lat_limits),
                                                   min(lon_limits))
        x_max, y_max, _, _ = utm.from_latlon(max(lat_limits),
                                             max(lon_limits))

        x_ranges = np.arange(x_min, x_max, self.grid_size)
        y_ranges = np.arange(y_min, y_max, self.grid_size)

        x_points = []
        y_points = []
        for i in range(min([len(x_ranges), len(y_ranges)])):
            # First for the vertical lines, i.e. x axis points.
            lat, lon = utm.to_latlon(x_ranges[i], min(y_ranges), zone, band)
            x_points.append(tilemapbase.project(lon,lat)[0])
            # Then for the horizontal lines, i.e. y axis points.
            lat, lon = utm.to_latlon(min(x_ranges), y_ranges[i], zone, band)
            y_points.append(tilemapbase.project(lon,lat)[1])

        # Project max limits.
        max_limits = tilemapbase.project(max(lon_limits), max(lat_limits))
        min_limits = tilemapbase.project(min(lon_limits), min(lat_limits))

        # Plot h lines.
        self.ax.hlines(y=y_points, xmin=min_limits[0], xmax=max_limits[0],
                       alpha=0.5, linewidth=1, linestyle='--')
        # Plot v line.
        self.ax.vlines(x=x_points, ymin=min_limits[1], ymax=max_limits[1],
                       alpha=0.5, linewidth=1, linestyle='--')

    def usbl_callback(self, msg):
        """
        Recieves the measured USBL position fix and sends it
        to the right function to process it.
        I'm gonna do a bunch of processing in the callback,
        don't think it should matter since USBL is so slow.
        """
        meas_type = msg.type

        if meas_type == RANGE_ONLY:
            rospy.logwarn("(WaterTag) RANGE_ONLY processing not yet implemented.")
            self._process_range_only(msg)
        elif meas_type == AZIMUTH_ONLY:
            rospy.logwarn("(WaterTag) AZIMUTH_ONLY received.")
            self._process_azimuth_only(msg)
        elif meas_type == FULL_FIX:
            self._process_full_fix(msg)
        elif meas_type == CARTESIAN:
            rospy.logwarn("(WasterTag) Got a Cartesian fix, no idea what it means.")
            return

    def payload_callback(self, msg):
        """
        Process the whatever payload the AUV sent to the topside unit.
        ATM we only process the POS command.
        """

        try:
            payload = [m for m in msg.payload.split(" ")]
        except:
            rospy.logwarn("(WaterTag) Unable to split message, prolly corrupted.")
            return

        # First string is supposed to be the command ID.
        command = payload[0]

        # Trigger whatever function necessary wrt the command we got.
        if command.lower() == 'pos':
            self._process_pos_cmd(msg, payload)
        # elif: whateverothercommmand.
        else:
            rospy.logwarn("(WaterTag) Couldn't understand AUV, message was:")
            rospy.loginfo(msg.payload)
        return

    def _process_pos_cmd(self, msg, payload):
        """
        Print, store, and raise flag for GUI to display the POS that
        the AUV sent2

        payload is already spliced: [cmd, lat, lon, depth]
        """
        ins_y_data = float(payload[1])
        ins_x_data = float(payload[2])
        # FIXME: We get the carriage return and newline attached to the depth...
        try:
            ins_z_data = float(payload[3][:-2])
        except:
            rospy.logwarn("(WaterTag) Corrupted depth message, setting equal to last.")
            ins_z_data = self.ins_buffer[-1][2]
        self.ins_buffer.append([ins_x_data, ins_y_data,
                                ins_z_data, msg.header.stamp.to_sec()])
        self.new_pos_msg = True

        rospy.loginfo("(WaterTag) Obtained POS from AUV:")
        rospy.loginfo(self.fix_to_string(self.ins_buffer[-1]))

    def _process_azimuth_only(self, msg):
        """
        Process the azimuth only usbl measurement and raises a flag for the
        GUI to plot it.
        """
        bearing = msg.bearing_raw
        elevation = msg.elevation_raw
        # Gonna make up a large range to only to get the general direction.
        meas_range = 10000 #[m]

        # If we don't have AHRS heading, we can't use the raw bearing and elevation.
        if not self.use_ahrs_rot or not self.is_user_init:
            rospy.logwarn("(WaterTag) Got AZIMUTH_ONLY fix, need AHRS heading to compute.")
            return

        # If we have AHRS heading, we're golden.
        pos = np.array([[meas_range * np.cos(bearing) * np.cos(elevation)],
                        [meas_range * np.sin(bearing) * np.cos(elevation)],
                        [meas_range * np.sin(elevation)]])
        # Rotate to world frame using user's orientation.
        pos = self.user_rot_enu.as_matrix() @ pos
        relative_pos = {'x': pos[0,0],
                        'y': pos[1,0],
                        'z': pos[2,0]}

        # Save it and let the GUI know it can be plotted.
        self.bearing_y_data, self.bearing_x_data = self.relative_to_latlon(relative_pos)
        self.new_bearing_fix = True

    def _process_full_fix(self, msg):
        """
        Processes the full fix and raises a flag for the GUI to plot it.
        """
        meas_range = msg.range
        # Round and store for display.
        self.auv_range = round(msg.range,1)
        # If flag is toggled, use AHRS orientation and raw USBL fix.
        if self.use_ahrs_rot and self.is_user_init:
            bearing = msg.bearing_raw
            elevation = msg.elevation_raw
            pos = np.array([[meas_range * np.cos(bearing) * np.cos(elevation)],
                            [meas_range * np.sin(bearing) * np.cos(elevation)],
                            [meas_range * np.sin(elevation)]])
            # Rotate to world frame using user's orientation.
            pos = self.user_rot_enu.as_matrix() @ pos
            relative_pos = {'x': pos[0,0],
                            'y': pos[1,0],
                            'z': pos[2,0]}
        else:
            relative_pos = {'x': msg.relative_position.x,
                            'y': msg.relative_position.y,
                            'z': msg.relative_position.z}

        self.auv_y_data, self.auv_x_data = self.relative_to_latlon(relative_pos)
        self.auv_z_data = np.round(abs(relative_pos['z']), decimals=1)
        self.auv_buffer.append([self.auv_x_data, self.auv_y_data,
                                self.auv_z_data, msg.header.stamp.to_sec()])
        self.new_usbl_fix = True

        rospy.loginfo("(WaterTag) Received USBL FULL_FIX:")
        rospy.loginfo(self.fix_to_string(self.auv_buffer[-1]))

    def ahrs_callback(self, msg):
        """
        Recieves the measured position and attitude of the
        service boat and send it to the GUI for manipulation
        and plotting.
        """
        self.user_vel = np.round(np.linalg.norm([msg.velocity.x,
                                                 msg.velocity.y,
                                                 msg.velocity.z]),
                                 decimals=1)

        self.user_x_data = msg.longitude
        self.user_y_data = msg.latitude

        if not self.is_user_init:
            rospy.loginfo("(WaterTag) User position initialized!")
            self.is_user_init = True

        self.new_gps_fix = True

    def ahrs_quat_callback(self, msg):
        """
        Receives the filtered heading of the user's AHRS in NED.
        """
        self.user_rot_ned = Rotation.from_quat([msg.quaternion.x,
                                                msg.quaternion.y,
                                                msg.quaternion.z,
                                                msg.quaternion.w])
        roll, pitch, yaw = self.user_rot_ned.as_euler("xyz", degrees=True)
        self.user_rot_enu = Rotation.from_euler("xyz", [roll, -pitch, 90 - yaw], degrees=True)

        #TODO: hardcoded 135 deg offset of IMU wrt bow of the boat.
        # self.user_heading = 90 - (yaw + 135)
        self.user_heading_ned = yaw % 360 # Wrap to 0-360 [deg].
        self.user_heading_enu = 90 - yaw

    def auv_gps_callback(self, msg):
        self.real_y_data = msg.latitude
        self.real_x_data = msg.longitude

    def relative_to_latlon(self, relative_pos):
        x, y, self.utm_zone, self.utm_band = utm.from_latlon(self.user_y_data,
                                                             self.user_x_data)
        lat, lon = utm.to_latlon(x + relative_pos["x"], y + relative_pos["y"],
                                 self.utm_zone, self.utm_band)

        return (lat, lon)

    def get_auv_heading(self):
        """
        Simple calculation of the AUV's heading using consecutive USBL fixes.
        """
        if len(self.auv_buffer) < 2:
            rospy.logwarn("(WaterTag) buffer incomplete, setting heading to 0.")
            return 0.0
        prior = self.auv_buffer[0]
        current = self.auv_buffer[-1]
        return np.degrees(np.arctan2(current[1] - prior[1], current[0] - prior[0]))

    def update_real_plot(self):
        """
        Used to debug if AUV's GNSS data available.
        """
        # For debugging.
        self.real_scatt.remove()
        x, y = tilemapbase.project(self.real_x_data, self.real_y_data)
        self.real_scatt = plt.scatter(x, y, color='g')

    def update_usbl_plot(self, x, y):
        """
        Update the raw USBL plot w/ a max no. of usbl fixes.
        """
        self.usbl_scatt.remove()
        self.raw_usbl_x.append(x)
        self.raw_usbl_y.append(y)

        self.usbl_scatt = self.ax.scatter(self.raw_usbl_x, self.raw_usbl_y,
                                     marker='x', alpha=0.5, color='k',
                                     s=5**2)

    def update_user_plot(self):
        """
        This is the main function that deals with any plotting that has
        anything to do with the user: user's pose and labels.
        """
        if not self.new_gps_fix:
            self._draw_user_label(update_time=True)
            return
        self.new_gps_fix = False

        self.user_scatt.remove()
        x, y = tilemapbase.project(self.user_x_data, self.user_y_data)

        marker, scale = gen_marker(self.user_heading_enu)
        self.user_scatt = plt.scatter(x, y, marker=marker, s=(25*scale)**2,
                                      color='b')
        self._draw_user_label(x, y)

    def update_auv_plot(self):
        """
        This is the main function that deals with any plotting that has
        to do with the AUV: AUV's position, USBL fix, POS payload response,
        labels, etc.
        """
        # If we only have the bearing to the AUV.
        if self.new_bearing_fix:
            self.new_bearing_fix = False
            self._plot_azimuth_only()

        # If we got a POS response from the AUV.
        if self.new_pos_msg:
            self.new_pos_msg = False
            self._update_pos_plot()
            self._display_pos_msgs()

        if not self.new_usbl_fix:
            self._draw_auv_label(update_time=True)
            return

        self.new_usbl_fix = False
        # Display the latest fix as text.
        self._display_fixes()

        self.auv_scatt.remove()
        x, y = tilemapbase.project(self.auv_x_data,
                                   self.auv_y_data)
        self.update_usbl_plot(x, y)

        heading = self.get_auv_heading()
        marker, scale = gen_marker(heading)
        self.auv_scatt = plt.scatter(x, y, marker=marker, s=(25*scale)**2,
                                     color='r')

        self._draw_auv_label(x,y)

    def _update_pos_plot(self):
        """
        Plot the recieved POS payload as a 'true' position
        of the AUV.
        """
        pos = self.ins_buffer[-1]
        x, y = tilemapbase.project(pos[0], pos[1])

        self.ins_scatt.remove()
        self.ins_x_data.append(x)
        self.ins_y_data.append(y)

        self.ins_scatt = self.ax.scatter(self.ins_x_data, self.ins_y_data,
                                         marker='o', alpha=0.5, color='g',
                                         s=5**2)

    def _display_pos_msgs(self):
        """
        Print the POS message in the GUI.
        """
        self.pos_display.set_text(self._get_pos_msgs())

    def _draw_user_label(self, x=0, y=0, update_time=False):
        last_update = str(np.round(time.time() - self.last_user_update,
                                   decimals=1))
        label_string = "user\n" + "h:" + str(round(self.user_heading_ned)) + "$\degree$" + "\nt:" + last_update + "s"

        if update_time:
            self.user_label.set_text(label_string)
            return

        self.user_aux_label.remove()
        self.user_label.remove()

        self.user_aux_label = self.ax.annotate("", xy=(x, y), xycoords="data")
        self.user_label = self.ax.annotate(label_string,
                                           xy=(1,1),
                                           xycoords=self.user_aux_label.get_window_extent,
                                           xytext=(15,15),
                                           textcoords="offset points",
                                           va="center",
                                           bbox=dict(boxstyle="round", fc="w", alpha=0.3),
                                           arrowprops=dict(arrowstyle="-"))
        self.last_user_update = time.time()

    def _draw_auv_label(self, x=0, y=0, update_time=False):
        last_update = str(np.round(time.time() - self.last_auv_update, decimals=1))
        label_string = "lolo\n" + "r:" + str(self.auv_range)+ "m" + "\nt:" + last_update + "s"

        if update_time:
            self.auv_label.set_text(label_string)
            return

        self.auv_aux_label.remove()
        self.auv_label.remove()

        self.auv_aux_label = self.ax.annotate("", xy=(x, y), xycoords="data")
        self.auv_label = self.ax.annotate(label_string,
                                          xy=(-1,-1),
                                          xycoords=self.auv_aux_label.get_window_extent,
                                          xytext=(15,-15),
                                          textcoords="offset points",
                                          va="center",
                                          bbox=dict(boxstyle="round", fc="w", alpha=0.3),
                                          arrowprops=dict(arrowstyle="-"))
        self.last_auv_update = time.time()

    def _plot_azimuth_only(self):
        """
        Plot an infinite-ish line from the user's position with the bearing
        measured by the USBL.
        """
        user_x, user_y = tilemapbase.project(self.user_x_data,
                                             self.user_y_data)
        bearing_x, bearing_y = tilemapbase.project(self.bearing_x_data,
                                                   self.bearing_y_data)
        line = self.ax.plot([user_x, bearing_x], [user_y, bearing_y], 'r-',
                            linewidth=1, alpha=0.5)

        # Check if the buffer is going to overflow.
        if len(self.bearing_buffer) >= self.bearing_buffer.maxlen:
            # If it is, delete the line that will leave the buffer.
            self.ax.lines.remove(self.bearing_buffer[0])
        # Append the new bearing plot.
        self.bearing_buffer.append(line[0])

    def _display_fixes(self):
        """
        Display the USBL fixes in the buffer as text.
        """
        self.fix_display.set_text(self._get_fixes())

def gen_marker(rot=0.0):
    """
    Borrowed from stackedoverflow, edited to make the arrows
    look like unicorns.
    """
    arr = np.array([[.1, .3], [.1, -.3], [1, 0], [3, 0], [1, 0], [.1, .3]])  # arrow shape
    angle = rot / 180 * np.pi
    rot_mat = np.array([
        [np.cos(angle), np.sin(angle)],
        [-np.sin(angle), np.cos(angle)]
        ])
    arr = np.matmul(arr, rot_mat)  # rotates the arrow

    # scale
    x0 = np.amin(arr[:, 0])
    x1 = np.amax(arr[:, 0])
    y0 = np.amin(arr[:, 1])
    y1 = np.amax(arr[:, 1])
    scale = np.amax(np.abs([x0, x1, y0, y1]))
    codes = [mpl.path.Path.MOVETO, mpl.path.Path.LINETO,
             mpl.path.Path.LINETO, mpl.path.Path.LINETO,
             mpl.path.Path.LINETO, mpl.path.Path.CLOSEPOLY]
    arrow_head_marker = mpl.path.Path(arr, codes)
    return arrow_head_marker, scale

if __name__ == '__main__':
    # Init node.
    rospy.init_node("watertag")
    rospy.loginfo("(WaterTag) Executing ROS interface node...")

    # Get relevant parameters from the server.
    try:
        grid_size = rospy.get_param("/watertag_tracking_gui/grid_size")
        degree_padding = rospy.get_param("/watertag_tracking_gui/degree_padding")
        zoom_level = rospy.get_param("/watertag_tracking_gui/zoom_level")
        mission_file = rospy.get_param("/watertag_tracking_gui/mission_file")
        offline_mode = rospy.get_param("/watertag_tracking_gui/offline_mode")
        use_ahrs_rot = rospy.get_param("/watertag_tracking_gui/use_ahrs_rot")
        debug = rospy.get_param("/watertag_tracking_gui/debug")
    except:
        rospy.logerr("Could not get a parameter from the server. Check launchfile.")
        exit()

    # Instantiate visualization.
    vis = WatertagVisualizer(grid_size, use_ahrs_rot, degree_padding, zoom_level)
    plt.ion()

    # Subcribe to everything.
    rospy.loginfo("(WaterTag) App initialized, starting listeners...")
    rospy.Subscriber("/evologics_modem/measurement/usbl_fix", mUSBLFix,
                        vis.usbl_callback)
    rospy.Subscriber("/evologics_modem/recv", DMACPayload,
                        vis.payload_callback)
    rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav,
                        vis.ahrs_callback)
    rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat,
                     vis.ahrs_quat_callback)

    # Subscribe to the AUV's gps topic for debugging.
    if debug:
        rospy.Subscriber("/lolo/core/gps", NavSatFix,
                        vis.auv_gps_callback)

    rospy.loginfo("(WaterTag) Starting tilemapbase...")
    tilemapbase.start_logging()
    tilemapbase.init(create=True)
    tiles = tilemapbase.tiles.build_OSM()

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        # Init plot.
        if not vis.is_plot_init:
            vis.init_plot(tiles, mission_file)
            plt.pause(0.1)
            rate.sleep()
            continue
        # Update if needed.
        vis.update_user_plot()
        vis.update_auv_plot()
        if debug:
            vis.update_real_plot()
        # Pause for plotting.
        vis.fig.canvas.draw_idle()
        plt.pause(0.01)
        rate.sleep()
