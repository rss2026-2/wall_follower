from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class VisualizationTools:

    @staticmethod
    def plot_line(x, y, publisher, color=(1.0, 0.0, 0.0), scale=(0.1, 0.1), frame="/base_link"):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.header.frame_id = frame

        # Set the size and color
        line_strip.scale.x = scale[0]
        line_strip.scale.y = scale[1]
        line_strip.color.a = 1.0
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.b = color[2]

        # Fill the line with the desired values
        for xi, yi in zip(x, y):
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        publisher.publish(line_strip)

    @staticmethod
    def draw_sphere(x, y, publisher, frame, scale=(0.2, 0.2, 0.2), color=(1.0, 1.0, 0.0)):
        """
        Publish a marker to represent a sphere in rviz.
        """
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        publisher.publish(marker)
