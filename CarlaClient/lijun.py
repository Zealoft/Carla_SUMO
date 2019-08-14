#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot

    TAB          : change sensor position
    `            : next sensor
    [1-9]        : change to sensor [1-9]
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
sys.path.append('/usr/local/lib/python3.7/site-packages/')
import lcm 
import math
import coordinate
import pandas as pd
#import thread
sys.path.append('./lcmMsg/')
from lidar import pointcloud 
from lidar import lidarframe
from gps import structNAVINFO    
from lasermap import structLASERMAP
from carcontrol import structCARCONTROL

import _thread
lc = lcm.LCM()
rlc = lcm.LCM()

lcm_acc = 0;
lcm_brake = 0;
lcm_steer = 0;

def carcontrol_handler(channel, data) :
    msg = structCARCONTROL.decode(data)
    global lcm_acc, lcm_brake, lcm_steer
    lcm_acc = msg.acc;
    lcm_brake = msg.brake
    lcm_steer = msg.steer
    print('received carcontrol', lcm_acc, lcm_brake, lcm_steer)

subscription_rlc = rlc.subscribe("CARCONTROL", carcontrol_handler)

def receive_rlc() :
    try:
        while True:
            rlc.handle()
    except KeyboardInterrupt:
        pass 

_thread.start_new_thread(receive_rlc, ())

try:
    sys.path.append('/home/autolab/0.9.4/PythonAPI/carla-0.9.4-py3.5-linux-x86_64.egg')
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

#def my_handler(channel, data):
#    msg = Xxxx.decode(data)
#    print("receive msg on channel %s " % channel)
#    print("receive data %s", %  msg.xxx)

mylcm = lcm.LCM()
#subcription = mylcm.subscribe("EXAMPLE", my_handler)

#try:
#    while True:
#        mylcm.handle()
#except KeyboardInterrupt:
#    pass



def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name


class World(object):
    def __init__(self, carla_world, hud):
        self.world = carla_world
        self.hud = hud
        self.world.on_tick(hud.on_world_tick)
        blueprint = self.world.get_blueprint_library().filter('vehicle')[19]
        spawn_points = self.world.get_map().get_spawn_points()
        #Start_Point
        #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        spawn_point = spawn_points[0]
        self.vehicle = self.world.spawn_actor(blueprint, spawn_point)
        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle, self.hud)
        self.camera_manager = CameraManager(self.vehicle, self.hud)
        self.camera_manager.set_sensor(0, notify=False)
        self.controller = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

    def restart(self):
        cam_index = self.camera_manager._index
        cam_pos_index = self.camera_manager._transform_index
        start_pose = self.vehicle.get_transform()
        start_pose.location.z += 2.0
        start_pose.rotation.roll = 0.0
        start_pose.rotation.pitch = 0.0
        blueprint = self._get_random_blueprint()
        self.destroy()
        self.vehicle = self.world.spawn_actor(blueprint, start_pose)
        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle, self.hud)
        self.camera_manager = CameraManager(self.vehicle, self.hud)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.vehicle)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.vehicle.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        actors = [

            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.vehicle]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def _get_random_blueprint(self):
        bp = random.choice(self.world.get_blueprint_library().filter('vehicle'))
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        return bp


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        world.vehicle.set_autopilot(self._autopilot_enabled)
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == K_q:
                    self._control.reverse = not self._control.reverse
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.vehicle.set_autopilot(self._autopilot_enabled)
                    world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

    def _parse_keys(self, keys, milliseconds):
        global lcm_acc, lcm_brake, lcm_steer
        self._control.steer = lcm_steer
        self._control.brake = lcm_brake
        self._control.throttle = lcm_acc
        self._control.hand_brake = 0
        world.vehicle.apply_control(self._control) 

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================

#global carTrans
#carTrans = np.eye(4, dtype = float)#add by lijun

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame_number = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.carPos = np.eye(4, dtype = float)#add by lijun
        self.originPos = coordinate.UTMCoor(354664.1253431414, 3456156.075134088)

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame_number = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        if not self._show_info:
            return
        t = world.vehicle.get_transform()
        v = world.vehicle.get_velocity()
        c = world.vehicle.get_vehicle_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame_number - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        #print('Server FPS = %s' % self.server_fps)
        self._info_text = [
            'Server:  % 16d FPS' % self.server_fps,
            'Client:  % 16d FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.vehicle, truncate=20),
            'Map:     % 20s' % world.world.map_name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '',
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)
        ]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.vehicle.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        t = world.vehicle.get_transform()
        
        translation = (t.location.x, t.location.y, t.location.z)
        yaw = t.rotation.yaw
        roll = t.rotation.roll
        pitch = t.rotation.pitch
        yawMatrix = np.matrix([ \
                [math.cos(yaw), -math.sin(yaw), 0], \
                [math.sin(yaw), math.cos(yaw), 0], \
                [0, 0, 1] \
        ])
        pitchMatrix = np.matrix([ \
                [math.cos(pitch), 0, math.sin(pitch)], \
                [0, 1, 0], \
                [-math.sin(pitch), 0, math.cos(pitch)] \
        ])
        rollMatrix = np.matrix([ \
                [1, 0, 0], \
                [0, math.cos(roll), -math.sin(roll)], \
                [0, math.sin(roll), math.cos(roll)] \
        ])
        R = yawMatrix.dot(pitchMatrix).dot(rollMatrix)
        self.carPos[:3,:3] = R
        self.carPos[:3,3] = translation
       
        yawtemp = yaw - 90.0;
        if(yawtemp < -179.999):
            yawtemp += 360.0
        
        gpsmsg = structNAVINFO()
        nowUtm = coordinate.UTMCoor(self.originPos.x + translation[1], self.originPos.y + translation[0])
        WGS84coor = coordinate.UTMXYToLatLon(nowUtm)
        gpsmsg.mLat = WGS84coor.lat.getValue()
        gpsmsg.mLon = WGS84coor.lon.getValue()
        gpsmsg.utmX = nowUtm.x#translation[1] 
        gpsmsg.utmY = nowUtm.y#translation[0]
        gpsmsg.height = translation[2]
        gpsmsg.mHeading = math.radians(-yawtemp)
        gpsmsg.mRoll = roll
        print(-yawtemp)
        gpsmsg.mPitch = pitch
        gpsmsg.mSpeed3d = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        gpsmsg.mSpeedX = v.x
        gpsmsg.mSpeedY = v.y
        lc.publish("NAVINFO",gpsmsg.encode())
        #print(world.vehicle)
        self._notifications.tick(world, clock)

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item: # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._history = []
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self._history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self._hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self._history.append((event.frame_number, intensity))
        if len(self._history) > 4000:
            self._history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_detector')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        text = ['%r' % str(x).split()[-1] for x in set(event.crossed_lane_markings)]
        self._hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.lidarData = np.zeros([1,3], dtype = np.float) #add by lijun

        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            if item[0].startswith('sensor.lidar'):
                bp.set_attribute('rotation_frequency', str(20))#20
                bp.set_attribute('channels', str(64))
                bp.set_attribute('points_per_second', str(450000))#450000
                bp.set_attribute('upper_fov', str(10))
                bp.set_attribute('lower_fov', str(-30))
                #print(bp.get_attribute('channels'))
                #print(bp.get_attribute('range'))
                #print(bp.get_attribute('points_per_second'))
                #print(bp.get_attribute('rotation_frequency'))
                #print(bp.get_attribute('upper_fov'))
                #print(bp.get_attribute('lower_fov'))
            item.append(bp)
        self._index = None

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
    
    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            #print("frame id: %d"%(image.frame_number))
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))
            dx = image.transform.location.x
            dy = image.transform.location.y
            dz = image.transform.location.z
            roll = image.transform.rotation.roll
            pitch = image.transform.rotation.pitch
            yaw = image.transform.rotation.yaw
            yawMatrix = np.matrix([ \
                    [math.cos(yaw), -math.sin(yaw), 0], \
                    [math.sin(yaw), math.cos(yaw), 0], \
                    [0, 0, 1] \
            ])
            pitchMatrix = np.matrix([ \
                    [math.cos(pitch), 0, math.sin(pitch)], \
                    [0, 1, 0], \
                    [-math.sin(pitch), 0, math.cos(pitch)] \
            ])
            rollMatrix = np.matrix([ \
                    [1, 0, 0], \
                    [0, math.cos(roll), -math.sin(roll)], \
                    [0, math.sin(roll), math.cos(roll)] \
            ])
            Rtmp = yawMatrix.dot(pitchMatrix).dot(rollMatrix) #zyx
            lidarT = np.eye(4, dtype = float)
            lidarT[:3,:3] = Rtmp
            lidarT[:3,3] = (dx, dy, dz)
            #print("lidarT")
            #print(lidarT.astype(np.dtype('f4')))
            
            carT = self._hud.carPos
            #print("carT")
            #print(carT.astype(np.dtype('f4')))
            
            car2lidar = np.linalg.inv(carT).dot(lidarT)
            
            #print("car2lidarT")
            #print(car2lidar.astype(np.dtype('f4')))
            #points = ((yawMatrix.dot(pointstest.T)).T).astype(np.dtype('f4'))
            
            self.lidarData = np.vstack((self.lidarData,points))
            print("receiver num: %s" %points.shape[0])
            if(self.lidarData.shape[0] > 135000):
                frameData = self.lidarData#[:135000,:]
                #print("send frame num: %s" %frameData.shape[0])
                self.lidarData = self.lidarData[135000:,:]
                self.lidarData = np.zeros([1,3], dtype = np.int) #add by lijun
            current_data = np.array(points[:, :2])
            tempData = current_data.copy()
            tempData *= 5
            tempData += (0.5 * 151, 300)
            tempData = tempData.astype(np.int32)
            tempData = np.reshape(tempData, (-1, 2))
            temp_img_size = (401 , 151)
            temp_img = np.zeros(temp_img_size, dtype = np.int32)
            heightdiff = 0.1
            indices = ((tempData[:,0] >= 0) & (tempData[:,0] < 151) & (tempData[:,1] >= 0) & (tempData[:,1] < 401))
            tempData = tempData[np.where(indices == True)]
            pointstmp = points[np.where(indices == True)]
            pointstmp[:,:2] = tempData[:,:2]
            df = pd.DataFrame({'row' : pointstmp[:,1].astype(np.int32), 'col' : pointstmp[:,0].astype(np.int32), 'z' : pointstmp[:,2]})
            groupmax = df.groupby(['row', 'col']).max().reset_index().values
            groupmin = df.groupby(['row', 'col']).min().reset_index().values
            groupdata = groupmin.copy()
            groupdata[:,2] = groupmax[:,2] - groupmin[:,2]
            indexz = groupdata[:,2] > heightdiff
            groupdata = groupdata[np.where(indexz == True)][:,:2].astype(np.int32)
            temp_img[tuple(groupdata.T)] = (1)
            
            lasermsg = structLASERMAP()
            lasermsg.resolution = 0.2
            lasermsg.cols = 151
            lasermsg.rows = 401
            lasermsg.center_col = 75
            lasermsg.center_row = 300
            lasermsg.cells = temp_img.tolist()
            lc.publish("LASERMAP",lasermsg.encode())
                
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame_number)
    
        

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud)
        controller = KeyboardControl(world, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(world, clock):
                return

            global lcm_acc, lcm_brake, lcm_steer
            print(lcm_acc, lcm_brake, lcm_steer)
            controller._control.steer = lcm_steer
            controller._control.brake = lcm_brake
            controller._control.throttle = lcm_acc
            world.vehicle.apply_control(controller._control) 

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)


if __name__ == '__main__':

    main()
