class DeviceScanner:
    import serial
    import serial.tools.list_ports

    device = None

    def __init__(self):
        self.device = {'system': None, 'cruise': None, 'steering': None}

    def update(self):
        available_serial = [i.device for i in list(self.serial.tools.list_ports.comports())]
        for i in available_serial:
            s = self.serial.Serial(i, 57600, timeout=1)
            s.reset_input_buffer()
            s.write(b'GI\x00\x00\x00\x00\x00\x00\x90')
            recv = s.read(9)
            if recv == b'GRSWCTR\x02,':
                self.device['system'] = i
            elif recv == b'GRCCCTR\x02\x08':
                self.device['cruise'] = i
            elif recv == b'GRSCCTR\x02\x18':
                self.device['steering'] = i
            s.close()


class SystemController:
    import serial

    assign = ((b'GA\x00\x00\x00\x00\x00\x00\x88', b'GA\x00\x01\x00\x00\x00\x00\x89'), (b'GA\x01\x00\x00\x00\x00\x00\x89', b'GA\x01\x01\x00\x00\x00\x00\x8a', b'GA\x01\x02\x00\x00\x00\x00\x8b'), (b'GA\x02\x00\x00\x00\x00\x00\x8a', b'GA\x02\x01\x00\x00\x00\x00\x8b', b'GA\x02\x02\x00\x00\x00\x00\x8c', b'GA\x02\x03\x00\x00\x00\x00\x8d'), (b'GA\x03\x00\x00\x00\x00\x00\x8b', b'GA\x03\x01\x00\x00\x00\x00\x8c'), (b'GA\x04\x00\x00\x00\x00\x00\x8c', b'GA\x04\x01\x00\x00\x00\x00\x8d', b'GA\x04\x02\x00\x00\x00\x00\x8e'), (b'GA\x05\x00\x00\x00\x00\x00\x8d', b'GA\x05\x01\x00\x00\x00\x00\x8e'), (b'GA\x06\x00\x00\x00\x00\x00\x8e', b'GA\x06\x01\x00\x00\x00\x00\x8f'), (b'GA\x07\x00\x00\x00\x00\x00\x8f', b'GA\x07\x01\x00\x00\x00\x00\x90'), (b'GA\x08\x00\x00\x00\x00\x00\x90', b'GA\x08\x01\x00\x00\x00\x00\x91', b'GA\x08\x02\x00\x00\x00\x00\x92', b'GA\x08\x03\x00\x00\x00\x00\x93'), (b'GA\t\x00\x00\x00\x00\x00\x91', b'GA\t\x01\x00\x00\x00\x00\x92'))
    request = (b'GR\x00\x00\x00\x00\x00\x00\x99', b'GR\x01\x00\x00\x00\x00\x00\x9a', b'GR\x02\x00\x00\x00\x00\x00\x9b', b'GR\x03\x00\x00\x00\x00\x00\x9c', b'GR\x04\x00\x00\x00\x00\x00\x9d', b'GR\x05\x00\x00\x00\x00\x00\x9e', b'GR\x06\x00\x00\x00\x00\x00\x9f', b'GR\x07\x00\x00\x00\x00\x00\xa0', b'GR\x08\x00\x00\x00\x00\x00\xa1', b'GR\t\x00\x00\x00\x00\x00\xa2', b'GR\n\x00\x00\x00\x00\x00\xa3', b'GI\x00\x00\x00\x00\x00\x00\x90')

    device = None

    def __init__(self, comport, baudrate):
        self.device = self.serial.Serial(comport, baudrate)

    def set(self, subsystem, state):
        self.device.write(self.assign[subsystem][state])

    def poll(self, subsystem):
        self.device.reset_input_buffer()
        self.device.write(subsystem)
        if subsystem == 10:
            recv = self.device.read(9)
            if sum(recv[:-2]) == (recv[-2] << 8)| recv[-1]:
                val = recv[2:-2]
                val = (val[0] << 8) | (val[1] << 8) | (val[2] << 8) | val[3]
                return val
            else:
                return -1
        else:
            recv = self.device.read(14)
            if sum(recv[:-2]) == (recv[-2] << 8) | recv[-1]:
                return list(recv[2:-2])
            else:
                return -1


class CruiseController:
    import serial
    import numpy

    request = (b'GR\x00\x00\x00\x00\x00\x00\x99', b'GR\x01\x00\x00\x00\x00\x00\x9a', b'GR\x02\x00\x00\x00\x00\x00\x9b', b'GR\x03\x00\x00\x00\x00\x00\x9c', b'GR\x04\x00\x00\x00\x00\x00\x9d', b'GR\x05\x00\x00\x00\x00\x00\x9e', b'GR\x06\x00\x00\x00\x00\x00\x9f', b'GR\x07\x00\x00\x00\x00\x00\xa0', b'GR\x08\x00\x00\x00\x00\x00\xa1', b'GR\t\x00\x00\x00\x00\x00\xa2', b'GR\n\x00\x00\x00\x00\x00\xa3', b'GR\x0b\x00\x00\x00\x00\x00\xa4', b'GR\x0c\x00\x00\x00\x00\x00\xa5', b'GR\r\x00\x00\x00\x00\x00\xa6', b'GR\x0e\x00\x00\x00\x00\x00\xa7', b'GR\x0f\x00\x00\x00\x00\x00\xa8', b'GR\x10\x00\x00\x00\x00\x00\xa9', b'GR\x11\x00\x00\x00\x00\x00\xaa', b'GI\x00\x00\x00\x00\x00\x00\x90')

    device = None

    def __init__(self, comport, baudrate):
        self.device = self.serial.Serial(comport, baudrate)

    def set(self, subsystem, state):
        state = self.numpy.int32(state)
        output_buffer = b'GA' + bytes([subsystem, (state >> 24) & 255, (state >> 16) & 255, (state >> 8) & 255, state & 255])
        checksum = sum(output_buffer)
        output_buffer = output_buffer + bytes([(checksum >> 8) & 255, checksum & 255])
        self.device.write(output_buffer)

    def poll(self, subsystem):
        self.device.reset_input_buffer()
        self.device.write(self.request[subsystem])
        if subsystem == 17:
            recv = self.device.read(72)
            if sum(recv[:-2]) == (recv[-2] << 8) | recv[-1]:
                val = self.numpy.array(recv[2:-2]).reshape((17, 4))
                val = val[:, 0] | (val[:, 1] << 8) | (val[:, 2] << 16) | (val[:, 3] << 24)
                val = val - (val >> 15) * (1 << 16)
                val = val.tolist()
                return val
            else:
                return -1
        else:
            recv = self.device.read(9)
            if sum(recv[:-2]) == (recv[-2] << 8) | recv[-1]:
                recv = recv[3:-2]
                val = (recv[0] << 24) | (recv[1] << 16) | (recv[2] << 8) | recv[3]
                val = val - (val >> 31) * (1 << 32)
                return val
            else:
                return -1


class SteeringController:
    import serial
    import numpy

    request = (b'GR\x00\x00\x00\x00\x00\x00\x99', b'GR\x01\x00\x00\x00\x00\x00\x9a', b'GR\x02\x00\x00\x00\x00\x00\x9b', b'GR\x03\x00\x00\x00\x00\x00\x9c', b'GR\x04\x00\x00\x00\x00\x00\x9d', b'GR\x05\x00\x00\x00\x00\x00\x9e', b'GR\x06\x00\x00\x00\x00\x00\x9f', b'GR\x07\x00\x00\x00\x00\x00\xa0', b'GR\x08\x00\x00\x00\x00\x00\xa1', b'GR\t\x00\x00\x00\x00\x00\xa2', b'GR\n\x00\x00\x00\x00\x00\xa3', b'GR\x0b\x00\x00\x00\x00\x00\xa4', b'GR\x0c\x00\x00\x00\x00\x00\xa5', b'GR\r\x00\x00\x00\x00\x00\xa6', b'GR\x0e\x00\x00\x00\x00\x00\xa7', b'GR\x0f\x00\x00\x00\x00\x00\xa8', b'GR\x10\x00\x00\x00\x00\x00\xa9', b'GI\x00\x00\x00\x00\x00\x00\x90')

    device = None

    def __init__(self, comport, baudrate):
        self.device = self.serial.Serial(comport, baudrate)

    def set(self, subsystem, state):
        state = self.numpy.int32(state)
        output_buffer = b'GA' + bytes([subsystem, (state >> 24) & 255, (state >> 16) & 255, (state >> 8) & 255, state & 255])
        checksum = sum(output_buffer)
        output_buffer = output_buffer + bytes([(checksum >> 8) & 255, checksum & 255])
        self.device.write(output_buffer)

    def poll(self, subsystem):
        self.device.reset_input_buffer()
        self.device.write(self.request[subsystem])
        if subsystem == 16:
            recv = self.device.read(68)
            if sum(recv[:-2]) == (recv[-2] << 8) | recv[-1]:
                val = self.numpy.array(recv[2:-2]).reshape((16, 4))
                val = val[:, 0] | (val[:, 1] << 8) | (val[:, 2] << 16) | (val[:, 3] << 24)
                val = val - (val >> 31) * (1 << 32)
                val = val.tolist()
                return val
            else:
                return -1
        else:
            recv = self.device.read(9)
            if sum(recv[:-2]) == (recv[-2] << 8) | recv[-1]:
                recv = recv[3:-2]
                val = (recv[0] << 24) | (recv[1] << 16) | (recv[2] << 8) | recv[3]
                val = val - (val >> 31) * (1 << 32)
                return val
            else:
                return -1


class Velodyne:
    import socket
    import numpy
    import threading

    UDPport = None
    vertical_angle = None
    cos_vertical_angle = None
    sin_vertical_angle = None
    rotational_speed = None
    rotational_period = None
    loop_thread = None
    thread_alive = False
    scans = None
    intense = None
    scans_ready = False
    lock_scans = False

    def __init__(self, port, rpm):
        self.UDPport = self.socket.socket(self.socket.AF_INET, self.socket.SOCK_DGRAM)
        self.UDPport.bind((self.socket.gethostbyname(self.socket.gethostname()), port))

        self.rotational_speed = rpm
        self.rotational_period = 60000000 / self.rotational_speed

        self.vertical_angle = self.numpy.array([-15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15])
        self.vertical_angle = self.numpy.deg2rad(self.vertical_angle)
        self.vertical_angle = self.numpy.tile(self.vertical_angle, (12, 1))
        self.vertical_angle = self.vertical_angle.transpose()
        self.cos_vertical_angle = self.numpy.cos(self.vertical_angle)
        self.sin_vertical_angle = self.numpy.sin(self.vertical_angle)

        self.loop_thread = self.threading.Thread(target=self.loop)

    def loop(self):
        while self.thread_alive:
            points = self.numpy.array([0, 0, 0])
            ranges = []
            intensity = []
            first_timestamp = None

            while True:
                try:
                    data = self.UDPport.recv(2048)
                    tail = data[-6:]
                    timestamp = tail[0] | (tail[1] << 8) | (tail[2] << 16) | (tail[3] << 24)
                    data = data[:-6]
                    data = [list(i) for i in data.split(b'\xff\xee')[1:]]
                    data = self.numpy.array(data).transpose()
                    azimuth = self.numpy.deg2rad(((data[1] << 8) | data[0]) / 100)
                    azimuth = self.numpy.tile(azimuth, (16, 1))
                    cos_azimuth = self.numpy.cos(azimuth)
                    sin_azimuth = self.numpy.sin(azimuth)
                    rl = data[[2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47]]
                    rh = data[[3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48]]
                    it = data[[4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49]]
                    r = (rh << 8) | rl
                    l = r * self.cos_vertical_angle
                    z = r * self.sin_vertical_angle
                    y = l * cos_azimuth
                    x = l * sin_azimuth
                    x = x.reshape(192)
                    y = y.reshape(192)
                    z = z.reshape(192)
                    xyz = self.numpy.column_stack((x, y, z))

                    if data.shape == (98, 12):
                        points = self.numpy.row_stack((points, xyz))
                        ranges = ranges + r.reshape(192).tolist()
                        intensity = intensity + it.reshape(192).tolist()

                    if first_timestamp is None:
                        first_timestamp = timestamp
                    elif timestamp - first_timestamp > self.rotational_period:
                        break
                except TypeError:
                    pass

            self.scans_ready = False
            self.scans = points[1:, :]
            self.intense = intensity
            self.scans_ready = True

    def getscans(self):
        while not self.scans_ready:
            pass
        return self.scans, self.intense

    def start(self):
        self.thread_alive = True
        self.loop_thread.start()

    def kill(self):
        self.thread_alive = False
        del self.loop_thread


class Sunnav:
    import socket
    import threading

    TCPport = None
    thread_alive = False
    loop_thread = None
    received_message = None

    GPHDTheading = None
    GPGGAlatitude = None
    GPGGAlongitude = None
    GPGGAquality = None
    GPGGAsatellitesinuse = None
    GPRMClatitude = None
    GPRMClongitude = None
    GPRMCspeedoverground = None
    GPRMCmode = None
    GPVTGtruetrack = None
    GPVTGtrackmagnetic = None
    GPVTGspeed = None
    GPVTGmode = None

    def __init__(self, ip, port):
        self.TCPport = self.socket.socket(self.socket.AF_INET, self.socket.SOCK_STREAM)
        self.TCPport.connect((ip, port))

        self.loop_thread = self.threading.Thread(target=self.loop)

    def loop(self):
        while self.thread_alive:
            self.received_message = self.TCPport.recv(1024)
            data = self.received_message.decode().split('\r\n')[:-1]
            print(data)
            data = [i.split(',') for i in data]
            if data[1][1] == '':
                self.GPHDTheading = None
            else:
                self.GPHDTheading = float(data[1][1])
            if data[0][2] == '':
                self.GPGGAlatitude = None
            else:
                self.GPGGAlatitude = float(data[0][2][:2]) + float(data[0][2][2:])/60
            if data[0][4] == '':
                self.GPGGAlongitude = None
            else:
                self.GPGGAlongitude= float(data[0][4][:3]) + float(data[0][4][3:])/60
            if data[0][6] == '':
                self.GPGGAquality = None
            else:
                self.GPGGAquality = float(data[0][6])
            if data[0][7] == '':
                self.GPGGAsatellitesinuse = None
            else:
                self.GPGGAsatellitesinuse = float(data[0][7])
            if data[3][3] == '':
                self.GPRMClatitude = None
            else:
                self.GPRMClatitude = float(data[3][3][:2]) + float(data[3][3][2:])/60
            if data[3][5] == '':
                self.GPRMClongitude = None
            else:
                self.GPRMClongitude = float(data[3][5][:3]) + float(data[3][5][3:])/60
            if data[3][7] == '':
                self.GPRMCspeedoverground = None
            else:
                self.GPRMCspeedoverground = float(data[3][7])
            self.GPRMCmode = data[3][12][:1]
            if data[2][1] == '':
                self.GPVTGtruetrack = None
            else:
                self.GPVTGtruetrack = float(data[2][1])
            if data[2][3] == '':
                self.GPVTGtrackmagnetic = None
            else:
                self.GPVTGtrackmagnetic = float(data[2][3])
            if data[2][7] == '':
                self.GPVTGspeed = None
            else:
                self.GPVTGspeed = float(data[2][7])
            self.GPVTGmode = data[2][9][:1]

    def start(self):
        self.thread_alive = True
        self.loop_thread.start()

    def kill(self):
        self.thread_alive = False
        del self.loop_thread


class PosLVX:
    import socket
    import threading

    TCPport = None
    thread_alive = False
    loop_thread = None
    received_message = None

    GPHDTheading = None
    GPGGAlatitude = None
    GPGGAlongitude = None
    GPGGAquality = None
    GPGGAsatellitesinuse = None
    GPRMClatitude = None
    GPRMClongitude = None
    GPRMCspeedoverground = None
    GPRMCmode = None
    GPVTGtruetrack = None
    GPVTGtrackmagnetic = None
    GPVTGspeed = None
    GPVTGmode = None

    def __init__(self, ip, port):
        self.TCPport = self.socket.socket(self.socket.AF_INET, self.socket.SOCK_STREAM)
        self.TCPport.connect((ip, port))

        self.loop_thread = self.threading.Thread(target=self.loop)

    def loop(self):
        while self.thread_alive:
            self.received_message = self.TCPport.recv(1024)
            data = self.received_message.decode().split('\r\n')[:-1]
            data = [i.split(',') for i in data]
            try:
                if data[0][1] == '':
                    self.GPHDTheading = None
                else:
                    self.GPHDTheading = float(data[0][1])
                if data[2][2] == '':
                    self.GPGGAlatitude = None
                else:
                    self.GPGGAlatitude = float(data[2][2][:2]) + float(data[2][2][2:])/60
                if data[2][4] == '':
                    self.GPGGAlongitude = None
                else:
                    self.GPGGAlongitude= float(data[2][4][:3]) + float(data[2][4][3:])/60
                if data[2][6] == '':
                    self.GPGGAquality = None
                else:
                    self.GPGGAquality = float(data[2][6])
                if data[2][7] == '':
                    self.GPGGAsatellitesinuse = None
                else:
                    self.GPGGAsatellitesinuse = float(data[2][7])
                if data[1][3] == '':
                    self.GPRMClatitude = None
                else:
                    self.GPRMClatitude = float(data[1][3][:2]) + float(data[1][3][2:])/60
                if data[1][5] == '':
                    self.GPRMClongitude = None
                else:
                    self.GPRMClongitude = float(data[1][5][:3]) + float(data[1][5][3:])/60
                if data[1][7] == '':
                    self.GPRMCspeedoverground = None
                else:
                    self.GPRMCspeedoverground = float(data[1][7])
                self.GPRMCmode = data[1][12][:1]
                if data[3][1] == '':
                    self.GPVTGtruetrack = None
                else:
                    self.GPVTGtruetrack = float(data[3][1])
                if data[3][3] == '':
                    self.GPVTGtrackmagnetic = None
                else:
                    self.GPVTGtrackmagnetic = float(data[3][3])
                if data[3][7] == '':
                    self.GPVTGspeed = None
                else:
                    self.GPVTGspeed = float(data[3][7])
                self.GPVTGmode = data[3][9][:1]
            except:
                print('POSLVX ERROR')

    def start(self):
        self.thread_alive = True
        self.loop_thread.start()

    def kill(self):
        self.thread_alive = False
        del self.loop_thread


class Visualizer:
    import open3d

    def __init__(self, name, width, height, initial_points, point_size=None, background=None):
        self.points = self.open3d.geometry.PointCloud()
        self.display = self.open3d.visualization.Visualizer()
        self.display.create_window(window_name=name, width=width, height=height)
        self.option = self.display.get_render_option()
        self.points.points = self.open3d.utility.Vector3dVector(initial_points)
        self.display.add_geometry(self.points)
        if point_size is not None:
            self.option.point_size = point_size
        if background is not None:
            self.option.background_color = background

    def update(self, points, colors=None):
        self.points.points = self.open3d.utility.Vector3dVector(points)
        if colors is not None:
            self.points.colors = self.open3d.utility.Vector3dVector(colors)
        self.display.update_geometry()
        self.display.poll_events()
        self.display.update_renderer()
        
    def close(self):
        self.display.destroy_window()


class WaypointMap:
    import numpy
    from scipy.spatial import cKDTree

    def __init__(self, filepath, longitude_constant, latitude_constant, elevation, color=numpy.array([1, 0.5, 1])):
        self.geographic_constant = self.numpy.array([longitude_constant, latitude_constant])

        with open(filepath, 'r') as f:
            data = f.readlines()
        data = [i[:-1].split('\t') for i in data]
        data = self.numpy.array(data).astype(float)
        self.raw_waypoint = data[:, :2]
        self.raw_heading = data[:, 2]
        self.raw_speed = data[:, 3]
        if data.shape[1] > 4:
            self.raw_mode = data[:, 4]
        else:
            self.raw_speed = False
        self.raw_waypoint_manager = self.cKDTree(self.raw_waypoint)
        self.speed = self.raw_speed / 3.6
        self.origin = (self.numpy.ptp(self.raw_waypoint, axis=0) / 2) + self.numpy.min(self.raw_waypoint, axis=0)
        self.waypoint = (self.raw_waypoint - self.origin) * self.geographic_constant
        self.waypoint = self.numpy.column_stack((self.waypoint, elevation * self.numpy.ones(self.waypoint.shape[0])))
        self.color = self.numpy.tile(color, (self.waypoint.shape[0], 1))
        self.heading = self.numpy.deg2rad(180 - ((self.raw_heading + 90) % 360))

    def to_meter(self, longitude, latitude, heading):
        position = (self.numpy.array([longitude, latitude]) - self.origin) * self.geographic_constant
        course = self.numpy.deg2rad(180 - ((heading + 90) % 360))
        return position, course

    def to_geographic(self, x, y, heading):
        position = (self.numpy.array([x, y]) / self.geographic_constant) + self.origin
        course = 360 - ((self.numpy.rad2deg(heading) - 90) % 360)
        return position, course

    def get_mode_speed(self, longitude, latitude):
        pos = self.numpy.array([longitude, latitude])
        dist, indx = self.raw_waypoint_manager.query(pos, k=1)
        if self.raw_mode is not False:
            ret = (self.raw_mode[indx], self.raw_speed[indx])
        else:
            ret = (False, self.raw_speed[indx])
        return ret


class SteeringProfile:
    import numpy

    def __init__(self, filepath, from_steering, to_steering, devision_steering, from_time, to_time, devision_time):
        with open(filepath, 'r') as f:
            data = f.read()
        data = data.split('\n')[:-1]
        self.axis = self.numpy.linspace(from_steering, to_steering, devision_steering)
        self.profile = self.numpy.array(data).astype(float).reshape((devision_steering, devision_steering, devision_time))
        self.timespace = self.numpy.tile(self.numpy.linspace(from_time, to_time, devision_time), (devision_steering, 1))
        self.interval = (to_steering - from_steering) / (devision_steering - 1)

    def get_adjusted_profile(self, steering):
        closest_index = self.numpy.argmin(self.numpy.abs(self.axis - steering))
        if steering >= self.axis[-1]:
            adjusted_profile = self.profile[-1, :, :]
        elif steering <= self.axis[0]:
            adjusted_profile = self.profile[0, :, :]
        elif self.axis[closest_index] - steering < 0:
            lower_map_index = closest_index
            upper_map_index = closest_index + 1
            lower_factor = (self.axis[upper_map_index] - steering) / self.interval
            upper_factor = (steering - self.axis[lower_map_index]) / self.interval
            adjusted_profile = (self.profile[lower_map_index, :, :] * lower_factor) + (self.profile[upper_map_index, :, :] * upper_factor)
        elif self.axis[closest_index] - steering > 0:
            lower_map_index = closest_index - 1
            upper_map_index = closest_index
            lower_factor = (self.axis[upper_map_index] - steering) / self.interval
            upper_factor = (steering - self.axis[lower_map_index]) / self.interval
            adjusted_profile = (self.profile[lower_map_index, :, :] * lower_factor) + (self.profile[upper_map_index, :, :] * upper_factor)
        else:
            adjusted_profile = self.profile[closest_index, :, :]
        return adjusted_profile


class ScoredKinematicPath:
    import scipy.integrate as integrate
    from scipy.spatial import cKDTree
    import numpy

    def __init__(self, waypoint, heading, elevation, ravel_shape, split_shape, collision_threshold, position_score, heading_score, obstacle_score):
        self.waypoint = waypoint
        self.waypoint_manager = self.cKDTree(waypoint)
        self.elevation = elevation
        self.heading_chart = heading
        self.ravel_shape = ravel_shape
        self.split_shape = split_shape
        self.ravel_z_predicted_path = elevation * self.numpy.ones(ravel_shape)
        self.collision_threshold = collision_threshold
        self.position_score = position_score
        self.heading_score = heading_score
        self.obstacle_score = obstacle_score
        self.total_cash = self.numpy.ones(split_shape[0]) * self.numpy.inf * -1

    def update(self, position, heading, speed, steering_profile, timespace, obstacle=None):
        path_heading = (speed * steering_profile) + heading
        x_component_speed_factor = self.numpy.cos(path_heading)
        y_component_speed_factor = self.numpy.sin(path_heading)
        x_predicted_path = speed * self.integrate.cumtrapz(x_component_speed_factor, timespace, axis=1, initial=0) + position[0]
        y_predicted_path = speed * self.integrate.cumtrapz(y_component_speed_factor, timespace, axis=1, initial=0) + position[1]
        ravel_x_predicted_path = x_predicted_path.reshape(self.ravel_shape)
        ravel_y_predicted_path = y_predicted_path.reshape(self.ravel_shape)
        ravel_paths_heading_array = path_heading.reshape(self.ravel_shape)
        ravel_xyz_predicted_path = self.numpy.column_stack((ravel_x_predicted_path, ravel_y_predicted_path, self.ravel_z_predicted_path))

        waypoint_distance, waypoint_index = self.waypoint_manager.query(ravel_xyz_predicted_path, k=1)
        heading_deviation = self.numpy.abs(ravel_paths_heading_array - self.heading_chart[waypoint_index])
        waypoint_sum = self.numpy.sum(waypoint_distance.reshape(self.split_shape), axis=1)
        heading_sum = self.numpy.sum(heading_deviation.reshape(self.split_shape), axis=1)
        waypoint_sum = waypoint_sum / self.numpy.max(waypoint_sum)
        heading_sum = heading_sum / self.numpy.max(heading_sum)

        if obstacle is not None:
            obstacle_manager = self.cKDTree(obstacle)
            obstacle_distance, _ = obstacle_manager.query(ravel_xyz_predicted_path, k=1)
            obstacle_sum = self.numpy.sum(obstacle_distance.reshape(self.split_shape), axis=1)
            obstacle_min = self.numpy.min(obstacle_distance.reshape(self.split_shape), axis=1)
            self.numpy.place(obstacle_min, obstacle_min < self.collision_threshold, -self.numpy.inf)
            obstacle_sum = obstacle_sum / self.numpy.max(obstacle_sum)
            if not self.numpy.all(obstacle_min == self.total_cash):
                obstacle_min = obstacle_min / self.numpy.max(obstacle_min[obstacle_min != -self.numpy.inf])
        else:
            obstacle_sum = self.numpy.zeros(self.split_shape[0])
            obstacle_min = self.numpy.zeros(self.split_shape[0])

        if self.numpy.all(obstacle_min == self.total_cash):
            score = False
        else:
            score = (self.position_score * waypoint_sum) + (self.heading_score * heading_sum) - (self.obstacle_score * obstacle_sum) - obstacle_min
        return ravel_xyz_predicted_path, score


class SteeringProfileCorrection:
    import numpy
    from scipy.spatial import cKDTree

    def __init__(self, center, size, gain=1):
        self.center = center
        self.size = size
        self.gain = gain
        self.actual = self.numpy.zeros(size)
        self.command = self.numpy.zeros(size)

    def update(self, ravel_xy_path, time_division, axis, position, steering):
        manager = self.cKDTree(ravel_xy_path)
        d, i = manager.query(position, k=1)
        i = i // time_division
        closest = axis[i]
        self.actual = self.numpy.concatenate((self.numpy.array([closest - self.center]), self.actual[:-1]))
        self.command = self.numpy.concatenate((self.numpy.array([steering - self.center]), self.command[:-1]))
        self.gain = self.numpy.sum(self.actual * self.command) / self.numpy.sum(self.command * self.command)
        return self.gain
