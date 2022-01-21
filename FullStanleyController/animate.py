import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial as serial
import time as time

from stanley_controller import StanleyController
from matplotlib.animation import FuncAnimation
from libs.kinematic_model import KinematicBicycleModel
from libs.car_description import Description
from libs.cubic_spline_interpolator import generate_cubic_spline


# At initialisation
# :param control_gain:                (float) time constant [1/s]
# :param softening_gain:              (float) softening gain [m/s]
# :param yaw_rate_gain:               (float) yaw rate gain [rad]
# :param steering_damp_gain:          (float) steering damp gain
# :param max_steer:                   (float) vehicle's steering limits [rad]
# :param wheelbase:                   (float) vehicle's wheelbase [m]
# :param path_x:                      (numpy.ndarray) list of x-coordinates along the path
# :param path_y:                      (numpy.ndarray) list of y-coordinates along the path
# :param path_yaw:                    (numpy.ndarray) list of discrete yaw values along the path
# :param dt:                          (float) discrete time period [s]

# At every time step
# :param x:                           (float) vehicle's x-coordinate [m]
# :param y:                           (float) vehicle's y-coordinate [m]
# :param yaw:                         (float) vehicle's heading [rad]
# :param target_velocity:             (float) vehicle's velocity [m/s]
# :param steering_angle:              (float) vehicle's steering angle [rad]

# :return limited_steering_angle:     (float) steering angle after imposing steering limits [rad]
# :return target_index:               (int) closest path index
# :return crosstrack_error:           (float) distance from closest path index [m]
class Simulation:

    def __init__(self):

        fps = 30.0

        self.dt = 1/fps
        self.map_size = 100
        self.frames = 1300
        self.loop = False

class Path:

    def __init__(self):

        # Get path to waypoints.csv -> where to attach file with the waypoints
        dir_path = 'data/waypoints.csv'
        df = pd.read_csv(dir_path)

        x = df['X-axis'].values
        y = df['Y-axis'].values
        ds = 0.05

        self.px, self.py, self.pyaw, _ = generate_cubic_spline(x, y, ds)

class Car:

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.v = 10.0
        self.delta = 0.0
        self.wheelbase = 0.26
        self.max_steer = np.deg2rad(10) #numpy -> converts angle from deg to rad *pi/180
        self.dt = sim_params.dt
        self.c_r = 0.01 #??
        self.c_a = 2.0 #??

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 8.0
        self.ksoft = 1.0
        self.kyaw = 0.0
        self.ksteer = 0.0
        self.crosstrack_error = None
        self.target_id = None

        # Description parameters
        self.overall_length = 0.36
        self.overall_width = 0.20
        self.tyre_diameter = 0.06
        self.tyre_width = 0.06
        self.axle_track = 0.18
        self.rear_overhang = (self.overall_length - self.wheelbase) / 2

        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, self.max_steer, self.wheelbase, self.px, self.py, self.pyaw)
        self.kbm = KinematicBicycleModel(self.wheelbase, self.dt)

    def drive(self):
        
        self.delta, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.v, self.delta)
        self.x, self.y, self.yaw = self.kbm.kinematic_model(self.x, self.y, self.yaw, self.v, self.delta)
# OK
def main():
    
    sim = Simulation()
    path = Path()
    car = Car(path.px[0], path.py[0], path.pyaw[0], sim, path)
    desc = Description(car.overall_length, car.overall_width, car.rear_overhang, car.tyre_diameter, car.tyre_width, car.axle_track, car.wheelbase)

    interval = sim.dt * 10**3

    fig, ax = plt.subplots(3, 1)

    ax[0].set_aspect('equal')
    ax[0].plot(path.px, path.py, '--', color='gold')

    annotation = ax[0].annotate(f"Crosstrack error: {float('inf')}", xy=(car.x - 10, car.y + 5), color='black', annotation_clip=False)
    target, = ax[0].plot([], [], '+r')

    outline, = ax[0].plot([], [], color='black')
    fr, = ax[0].plot([], [], color='black')
    rr, = ax[0].plot([], [], color='black')
    fl, = ax[0].plot([], [], color='black')
    rl, = ax[0].plot([], [], color='black')
    rear_axle, = ax[0].plot(car.x, car.y, '+', color='black', markersize=2)

    yaw_arr = []
    yaw_data, = ax[1].plot([], [])
    ax[1].set_xlim(0, sim.frames)
    ax[1].set_ylabel("Yaw")
    ax[1].grid()

    crosstrack_arr = []
    crosstrack_data, = ax[2].plot([], [])
    ax[2].set_xlim(0, sim.frames)
    ax[2].set_ylabel("Crosstrack error")
    ax[2].grid()
    nucleo = serial.Serial(port='/dev/ttyACM0', baudrate=256000, timeout=.1)
    nucleo.close()
    nucleo.open()

    nucleo.write('#4:1.00;;\r\n'.encode('ascii'))
    frames = []
 #OK    
    def animate(frame):

        # Camera tracks car
        ax[0].set_xlim(car.x - sim.map_size, car.x + sim.map_size)
        ax[0].set_ylim(car.y - 13, car.y + 17)

        # Drive and draw car
        car.drive()
        outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = desc.plot_car(car.x, car.y, car.yaw, car.delta)
        outline.set_data(outline_plot[0], outline_plot[1])
        fr.set_data(*fr_plot)
        rr.set_data(*rr_plot)
        fl.set_data(*fl_plot)
        rl.set_data(*rl_plot)
        rear_axle.set_data(car.x, car.y)

        # Show car's target
        target.set_data(path.px[car.target_id], path.py[car.target_id])

        # Annotate car's coordinate above car
        annotation.set_text(f"Crosstrack error: {car.crosstrack_error:.5f}")
        annotation.set_position((car.x - 10, car.y + 5))

        frames.append(frame)

        # Animate yaw
        yaw_arr.append(car.yaw)
        yaw_data.set_data(frames, yaw_arr)
        ax[1].set_ylim(yaw_arr[-1] - 5, yaw_arr[-1] + 5)

        # Animate crosstrack error
        crosstrack_arr.append(car.crosstrack_error)
        crosstrack_data.set_data(frames, crosstrack_arr)
        ax[2].set_ylim(crosstrack_arr[-1] - 1, crosstrack_arr[-1] + 1)

        ax[0].set_title(f'{sim.dt*frame:.2f}s', loc='right')
        l=car.delta*180/3.14 #sterzata
        l= round(l,2)
        l=str(l)  
        print('sterzata:'+ l)
        per=car.yaw*180/3.14 #angolo rispetto al percorso
        per= round(per,2)
        per=str(per)
        print('angolo rispetto al percorso:' + per)

        lul = nucleo.read(20).decode('ascii')
        nucleo.write('#1:0.10;;\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n'.encode('ascii'))
        # #nucleo.write('#2:' + l +';;\r\n').encode('ascii')
        lel=nucleo.read(20).decode('ascii')
        lel=str(lel)
        # #lul = nucleo.read(20).decode('ascii')
        print('la nucleo legge: ' + lel)
        nucleo.write('#2:'.encode('ascii'))
        nucleo.write(l.encode('ascii'))
        nucleo.write(';;\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n'.encode('ascii'))
        # #nucleo.write('#2:' + l )
        # #nucleo.write(l.encode('ascii'))
        # #boh=int(boh)
        # lul = nucleo.read(20).decode('ascii')
        lul=str(lul)
        # #lul = nucleo.read(20).decode('ascii')
        print('la nucleo legge: ' + lul)

        return outline, fr, rr, fl, rl, rear_axle, target, yaw_data, crosstrack_data,

    _ = FuncAnimation(fig, animate, frames=sim.frames, interval=interval, repeat=sim.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=50)
    plt.show()

    print(f"Mean yaw: {np.mean(yaw_arr)}")
    print(f"Mean crosstrack error: {np.mean(crosstrack_arr)}")

if __name__ == '__main__':
    main()
