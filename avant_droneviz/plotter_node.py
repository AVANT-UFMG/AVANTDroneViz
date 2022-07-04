import rclpy
from rclpy.node import Node

import plotter

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')

        plotter.plot_odom('data/odom.csv', 2, real_time=True)
        plotter.plot_motion('~/droneviz_ws/velocidade.csv','velocidade_linear', 'acceleração_linear')

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()