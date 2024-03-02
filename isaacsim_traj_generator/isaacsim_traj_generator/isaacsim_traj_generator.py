import rclpy
from rclpy.node import Node

import numpy as np

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState


def quintic_traj(th_start, th_end, T, dt):
    if th_start.size != th_end.size:
        raise Exception('[Error] Cannot generate trajectory! DOF of start point and end point are different!')

    if (th_start.size + th_end.size) == 0:
        raise Exception('[Error] Cannot generate trajectory! Both start point and end point are empty!')

    if T < 0:
        raise Exception('[Error] Cannot generate trajectory! Time value(T) of the trajectory must be positive!')

    dof = th_start.size

    traj = np.array([])

    for i in range(0, dof):
        th = np.array([])
        for t in np.append(np.arange(0, T, dt), T):
            s = 10 * ((t / T) ** 3) - 15 * ((t / T) ** 4) + 6 * ((t / T) ** 5)
            th = np.append(th, th_start[i] + s * (th_end[i] - th_start[i]))

        if i == 0:
            traj = th
        else:
            traj = np.vstack((traj, th))

    return traj


class JointTrajSubscriber(Node):

    def __init__(self):
        super().__init__('joint_trajectory_subscriber')
        # create subscriber for '/joint_trajectory'
        self.joint_traj_subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.joint_traj_listener_callback,
            10)
        self.joint_traj_subscription  # prevent unused variable warning

        # create subscriber for '/joint_states'
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_listener_callback,
            10)
        self.joint_states_subscription # prevent unused variable warning

        # create publisher for '/joint_command'
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.timer_period = 0.01 # seconds

    def joint_traj_listener_callback(self, msg):
        self.get_logger().info('Received a request for trajectory generation')

        self.get_joint_traj_points = msg.points

        jtp = msg.points

        joint_names_from_input = np.array(msg.joint_names)
        joint_names_from_joint_state = np.array(self.current_joint_states.name)

        ids = [np.where(joint_names_from_joint_state == name)[0][0] for name in joint_names_from_input]

        self.joint_name_for_th_start = joint_names_from_joint_state[ids]

        for i in range(0, len(jtp)):
            if i == 0:
                th_s = np.array(self.current_joint_states.position)
                th_s = th_s[ids]
                th_e = (np.array(jtp[i].positions))

                T = jtp[i].time_from_start.sec
            else:
                th_s = (np.array(jtp[i-1].positions))
                th_e = (np.array(jtp[i].positions))

                T = jtp[i].time_from_start.sec - jtp[i-1].time_from_start.sec

            traj = quintic_traj(th_s, th_e, T, self.timer_period)

            if i > 0:
                if len(ids) == 1:
                    traj = traj[1:]
                    traj = np.concatenate((traj_prev, traj))
                else:
                    traj = traj[:, 1:]
                    traj = np.concatenate((traj_prev, traj), axis=1)

            traj_prev = traj

        self.traj = traj

        self.get_logger().info('Trajectory generation finished:')
        self.get_logger().info(' Num of JointTrajPoints: %d | Time Length: %.2f sec | Period of JointCmd: %.2f sec ' % (len(jtp), jtp[len(jtp)-1].time_from_start.sec, self.timer_period))

        self.i = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def joint_states_listener_callback(self, msg):
        self.current_joint_states = msg


    # This timer_callback repeats the following when the node subscribes '/joint_trajectory'
    def timer_callback(self):
        msg = JointState()

        for i in range(0,self.joint_name_for_th_start.size):
            msg.name.append(self.joint_name_for_th_start[i])

        if self.joint_name_for_th_start.size == 1:
             msg.position.append(self.traj[self.i])
        else:
            for joint_num in range(0, self.joint_name_for_th_start.size):
                msg.position.append(self.traj[joint_num][self.i])

        self.publisher_.publish(msg)

        self.i += 1

        if self.joint_name_for_th_start.size == 1:
            if self.i >= self.traj.size:
                self.timer.destroy()
                self.get_logger().info('Finished to send JointCmd for generated trajectory')
        else:
            if self.i >= self.traj[0].size:
                self.timer.destroy()
                self.get_logger().info('Finished to send JointCmd for generated trajectory')


def main(args=None):
    rclpy.init(args=args)

    joint_subscriber = JointTrajSubscriber()

    rclpy.spin(joint_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
