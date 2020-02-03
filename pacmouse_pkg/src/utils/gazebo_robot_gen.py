#!/usr/bin/env python3


import argparse
import os
import math

import pacmouse_pkg.src.params as p


class GazeboRobotGen:

    def __init__(self, _args):
        self.args = _args

    def generate(self):
        header = self.make_header()
        body = self.make_body()
        footer = self.make_footer()

        full_model = header + body + footer
        output_file = os.path.join(self.args.output_directory, 'model.sdf')
        with open(output_file, 'w+') as fo:
            fo.write(full_model)

    @staticmethod
    def make_header():
        header = '<?xml version="1.0" ?>\n' \
                 '<sdf version="1.5">\n' \
                 '<model name="pacmouse">\n'
        return header

    @staticmethod
    def make_body():
        vert_offset = 0.05  # TODO: Do we want a vertical offset here?
        # vert_offset = 0
        body = GazeboRobotGen.indent(f'<pose>0 0 {vert_offset} 0 0 '
                                     f'0</pose>\n', 1)
        body += GazeboRobotGen.indent('<static>false</static>\n', 1)
        body += GazeboRobotGen.make_chassis_link()
        body += GazeboRobotGen.make_wheel_links()
        body += GazeboRobotGen.make_wheel_joints()
        return body

    @staticmethod
    def make_chassis_link():
        chassis = GazeboRobotGen.indent('<link name="chassis">\n', 2)
        # vert_offset = 0.05  # TODO: Do we want a vertical offset here?
        vert_offset = 0
        chassis += GazeboRobotGen.indent(f'<pose>0 0 {vert_offset} 0 0 0'
                                         f'</pose>\n', 3)
        chassis += GazeboRobotGen.indent('<inertial>\n', 3)
        chassis += GazeboRobotGen.indent('<mass>{}</mass>\n'.format(
            p.chassis_mass_kg), 4)
        chassis += GazeboRobotGen.make_chassis_inertia()
        chassis += GazeboRobotGen.indent('</inertial>\n', 3)

        chassis += GazeboRobotGen.make_chassis_collision()

        chassis += GazeboRobotGen.make_chassis_visual()

        chassis += GazeboRobotGen.indent('</link>\n', 2)
        return chassis

    @staticmethod
    def make_chassis_inertia():
        return GazeboRobotGen.make_inertia(p.chassis_ixx, p.chassis_ixy,
                                           p.chassis_ixz, p.chassis_iyy,
                                           p.chassis_iyz, p.chassis_izz)

    @staticmethod
    def make_chassis_collision():
        collision = GazeboRobotGen.indent('<collision '
                                          'name="chassis_collision">\n', 3)
        collision += GazeboRobotGen.make_chassis_geometry()
        collision += GazeboRobotGen.indent('</collision>\n', 3)
        return collision

    @staticmethod
    def make_chassis_visual():
        visual = GazeboRobotGen.indent('<visual '
                                       'name="chassis_visual">\n', 3)
        visual += GazeboRobotGen.make_chassis_geometry()

        # Material colors:
        visual += GazeboRobotGen.indent('<material>\n', 4)
        # Numbers represent RGBA color code
        visual += GazeboRobotGen.indent('<ambient>0.1 1.0 0.1 1</ambient>\n', 5)
        visual += GazeboRobotGen.indent('<diffuse>0.1 0.1 0.1 1</diffuse>\n', 5)
        visual += GazeboRobotGen.indent('</material>\n', 4)

        visual += GazeboRobotGen.indent('</visual>\n', 3)
        return visual

    @staticmethod
    def make_chassis_geometry():
        geometry = GazeboRobotGen.indent('<geometry>\n', 4)
        geometry += GazeboRobotGen.indent('<box>\n', 5)
        geometry += GazeboRobotGen.indent('<size>{} {} {}</size>\n'.format(
            p.chassis_length,
            p.chassis_width,
            p.chassis_height), 6)
        geometry += GazeboRobotGen.indent('</box>\n', 5)
        geometry += GazeboRobotGen.indent('</geometry>\n', 4)
        return geometry

    @staticmethod
    def make_wheel_links():
        wheels = GazeboRobotGen.make_wheel_link('right', 'front',
                                                p.wheel_front_offset,
                                                -p.wheel_lateral_offset, 1)
        wheels += GazeboRobotGen.make_wheel_link('right', 'rear',
                                                 p.wheel_rear_offset,
                                                 -p.wheel_lateral_offset, 1)
        wheels += GazeboRobotGen.make_wheel_link('left', 'front',
                                                 p.wheel_front_offset,
                                                 p.wheel_lateral_offset, -1)
        wheels += GazeboRobotGen.make_wheel_link('left', 'rear',
                                                 p.wheel_rear_offset,
                                                 p.wheel_lateral_offset, -1)
        return wheels

    @staticmethod
    def make_wheel_link(prefix, suffix, x_offset, y_offset, rotation_sign):
        wheel = GazeboRobotGen.indent(f'<link name="{prefix:s}'
                                      f'_{suffix:s}_wheel">\n', 2)
        wheel += GazeboRobotGen.indent(f'<pose>{x_offset} {y_offset} '
                                       f'{p.wheel_vertical_offset} '
                                       f'{rotation_sign*math.pi/2} 0 0'
                                       f'</pose>\n', 3)

        wheel += GazeboRobotGen.indent(f'<visual name="{prefix:s}'
                                       f'_{suffix:s}_wheel_visual">\n', 3)
        wheel += GazeboRobotGen.make_wheel_geometry()

        # Material colors:
        wheel += GazeboRobotGen.indent('<material>\n', 4)
        # Numbers represent RGBA color code
        wheel += GazeboRobotGen.indent('<ambient>0.0 0.0 0.0 1</ambient>\n', 5)
        wheel += GazeboRobotGen.indent('<diffuse>0.1 0.1 0.1 1</diffuse>\n', 5)
        wheel += GazeboRobotGen.indent('</material>\n', 4)

        wheel += GazeboRobotGen.indent('</visual>\n', 3)
        wheel += GazeboRobotGen.indent(f'<collision name="{prefix:s}'
                                       f'_{suffix:s}_wheel_collision">\n', 3)
        wheel += GazeboRobotGen.make_wheel_geometry()
        wheel += GazeboRobotGen.make_wheel_surface()
        wheel += GazeboRobotGen.indent('</collision>\n', 3)

        wheel += GazeboRobotGen.indent('<inertial>\n', 3)
        wheel += GazeboRobotGen.indent('<mass>{}</mass>\n'.format(
            p.wheel_mass_kg), 4)
        wheel += GazeboRobotGen.make_wheel_inertia()
        wheel += GazeboRobotGen.indent('</inertial>\n', 3)

        wheel += GazeboRobotGen.indent('</link>\n', 2)
        return wheel

    @staticmethod
    def make_wheel_joints():
        joints = GazeboRobotGen.make_wheel_joint('right', 'front')
        joints += GazeboRobotGen.make_wheel_joint('right', 'rear')
        joints += GazeboRobotGen.make_wheel_joint('left', 'front')
        joints += GazeboRobotGen.make_wheel_joint('left', 'rear')
        return joints

    @staticmethod
    def make_wheel_joint(prefix, suffix):
        joint = GazeboRobotGen.indent(f'<joint name="{prefix:s}'
                                      f'_{suffix:s}_wheel_joint" '
                                      f'type="revolute">\n', 2)
        joint += GazeboRobotGen.indent('<pose>0 0 0 0 0 0</pose>\n', 3)
        joint += GazeboRobotGen.indent('<parent>chassis</parent>\n', 3)
        joint += GazeboRobotGen.indent(f'<child>{prefix:s}_'
                                       f'{suffix:s}_wheel</child>\n', 3)
        # Rotate about the wheel frame's z-axis
        joint += GazeboRobotGen.indent('<axis><xyz>0 0 1</xyz></axis>\n', 3)
        joint += GazeboRobotGen.indent('</joint>\n', 2)
        return joint

    @staticmethod
    def make_wheel_inertia():
        return GazeboRobotGen.make_inertia(p.wheel_ixx, p.wheel_ixy,
                                           p.wheel_ixz, p.wheel_iyy,
                                           p.wheel_iyz, p.wheel_izz)

    @staticmethod
    def make_inertia(ixx, ixy, ixz, iyy, iyz, izz):
        inertia = GazeboRobotGen.indent('<inertia>\n', 4)
        inertia += GazeboRobotGen.indent(f'<ixx>{ixx:.20f}</ixx>\n', 5)
        inertia += GazeboRobotGen.indent(f'<ixy>{ixy:.20f}</ixy>\n', 5)
        inertia += GazeboRobotGen.indent(f'<ixz>{ixz:.20f}</ixz>\n', 5)
        inertia += GazeboRobotGen.indent(f'<iyy>{iyy:.20f}</iyy>\n', 5)
        inertia += GazeboRobotGen.indent(f'<iyz>{iyz:.20f}</iyz>\n', 5)
        inertia += GazeboRobotGen.indent(f'<izz>{izz:.20f}</izz>\n', 5)
        inertia += GazeboRobotGen.indent('</inertia>\n', 4)
        return inertia

    @staticmethod
    def make_wheel_geometry():
        geometry = GazeboRobotGen.indent('<geometry>\n', 4)
        geometry += GazeboRobotGen.indent('<cylinder>\n', 5)
        geometry += GazeboRobotGen.indent(f'<radius>'
                                          f'{p.wheel_radius}</radius>\n', 6)
        geometry += GazeboRobotGen.indent(f'<length>'
                                          f'{p.wheel_width}</length>\n', 6)
        geometry += GazeboRobotGen.indent('</cylinder>\n', 5)
        geometry += GazeboRobotGen.indent('</geometry>\n', 4)
        return geometry

    @staticmethod
    def make_wheel_surface():
        geometry = GazeboRobotGen.indent('<surface>\n', 4)
        geometry += GazeboRobotGen.indent('<friction>\n', 5)
        geometry += GazeboRobotGen.indent('<ode>\n', 6)
        geometry += GazeboRobotGen.indent(f'<mu>'
                                          f'{p.wheel_surface_friction}</mu>\n',
                                          7)
        geometry += GazeboRobotGen.indent(f'<mu2>'
                                          f'{p.wheel_surface_friction}</mu2>\n',
                                          7)
        geometry += GazeboRobotGen.indent('</ode>\n', 6)
        geometry += GazeboRobotGen.indent('</friction>\n', 5)
        geometry += GazeboRobotGen.indent('</surface>\n', 4)
        return geometry

    @staticmethod
    def make_footer():
        footer = '</model>\n' \
                 '</sdf>\n'
        return footer

    @staticmethod
    def indent(line, indent_level):
        # Insert 2 spaces per indent level
        return ' '*(2*indent_level) + line


if __name__ == '__main__':
    default_model_dir = os.path.realpath(os.path.join(os.path.dirname(
        os.path.realpath(__file__)), '..', '..', 'gazebo_models', 'pacmouse'))
    parser = argparse.ArgumentParser(
        'Utility to turn a robot description into an .sdf model that can be '
        'used in Gazebo. This allows us to compute inertias, for example, '
        'and insert those values into the SDF file. It also allows us to use '
        'named parameter values from the params.py file.')
    parser.add_argument('--output_directory', '-o', type=str,
                        help='Absolute path of directory for the model.sdf '
                             'file to be output (default: %(default)s).',
                        default=default_model_dir)

    args = parser.parse_args()

    gazebo_robot_gen = GazeboRobotGen(args)
    gazebo_robot_gen.generate()
