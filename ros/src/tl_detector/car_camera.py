import rospy
import math
from tf.transformations import euler_from_quaternion

def fit_box(total, pos, box):
    margin = int(box / 2)
    if pos < margin:
        return 0, box
    if pos > (total - margin):
        return total - box, total

    return pos - margin, pos + margin

class CarCamera(object):
    def __init__(self, ci):
        self.image_width = ci['image_width']
        self.image_height = ci['image_height']
        self.fx = ci['focal_length_x'] if 'focal_length_x' in ci else 1.0
        self.fy = ci['focal_length_y'] if 'focal_length_y' in ci else 1.0
        rospy.loginfo("Camera image dimentions: %sx%s", self.image_width, self.image_height)

    def extract_roi(self, cv_image, pos_x, pos_y):
        pos_x_min, pos_x_max = fit_box(self.image_width, pos_x, int(self.image_width / 2))
        pos_y_min, pos_y_max = fit_box(self.image_height, pos_y, int(self.image_height / 2))

        top = pos_y_min
        bottom = pos_y_max

        left = pos_x_min
        right = pos_x_max

        # select part of the image
        return cv_image[top:bottom,left:right]

    @staticmethod
    def to_car_coordinates(car_position, car_orientation, point_in_world):
        l = point_in_world

        pos = car_position
        ori = car_orientation

        theta = math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y), 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
        dx = l.x - pos.x
        dy = l.y - pos.y

        X = dy * math.sin(theta) + dx * math.cos(theta)
        Y = dy * math.cos(theta) - dx * math.sin(theta)

        return (X, Y)

    def to_image_coordinates(self, car_position, car_orientation, point_in_world):
        camera_position = car_position
        camera_orientation = car_orientation

        X, Y = CarCamera.to_car_coordinates(car_position, car_orientation, point_in_world)
        if X < 3:
            rospy.logdebug("light is to close or back from the camera: %s", X)
            return None

        roll, pitch, camera_yaw = euler_from_quaternion([camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w])

        # calculation of X and Y already uses camera_yaw
        pos_x = self.image_width / 2 - int(self.fx * Y / X)

        camera_z = camera_position.z
        adjustment_for_roll = math.tan(roll) * X

        light_z = point_in_world.z
        pos_y = self.image_height /2 + int(self.fy * (light_z - camera_z - adjustment_for_roll) / X)
        rospy.logdebug("light coordinates at image: %s, %s", pos_x, pos_y)
        return (pos_x, pos_y)

    def does_it_fit(self, coords):
        (pos_x, pos_y) = coords            
        if pos_x < 0 or pos_x > self.image_width:
            rospy.logdebug("light x is out of view: %s", pos_x)
            return False
        if pos_y < 0 or pos_y > self.image_height:
            rospy.logdebug("light y is out of view: %s", pos_y)
            return False
        return True


if __name__ == '__main__':
    try:
        CarCamera()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start car camera node.')