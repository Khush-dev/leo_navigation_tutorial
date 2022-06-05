#!/usr/bin/env python

import rospy
import numpy as np
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, Pose
from sensor_msgs.msg import Image
import open3d as o3d
ROOT_LINK = "base_footprint"
DICTIONARY = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

class tag_detector:
      def __init__(self):
            rospy.init_node('tag_detector_node')
            rospy.loginfo('tag_detector_node init')
            

            self.listener = tf.TransformListener()
            self.listener.waitForTransform("map", ROOT_LINK, rospy.Time(0), rospy.Duration(10.0))
            self.mapData = OccupancyGrid()
            # self.olat, self.olon = 19.13079235, 72.91834925#49.8999997, 8.90000001

            rospy.Subscriber('/map', OccupancyGrid, self.mapCallBack)
            rospy.wait_for_message('/mrt/camera1/image_raw',Image, timeout=5)
            rospy.Subscriber('/mrt/camera1/image_raw', Image, self.cam_callback)
            rospy.Subscriber('/mrt/camera1/image_raw', Image, self.cam_callback)
            rospy.Subscriber("/mrt/laser/scan", LaserScan, self.lidar_callback)
            self.frame = None
            self.lidar_data = None
            self.marker_array_pub = rospy.Publisher('/tag_detections',AprilTagDetectionArray,queue_size=10)
            self.completed_list = []
            rospy.Rate(5).sleep()#
            #rospy.spin()

      def cam_callback(self, data):
            # Used to convert between ROS and OpenCV images
            br = CvBridge()
           
            # Output debugging information to the terminal
            #rospy.loginfo("receiving video frame")
             
            # Convert ROS Image message to OpenCV image
            current_frame = br.imgmsg_to_cv2(data)
            self.frame = current_frame

            found, ids, ps, q = self.ar_detect()
            marker_array = AprilTagDetectionArray()
            completed = np.zeros_like(ps)
            for i in range(found):
                  if not completed[i]:
                        completed[i] = True
                        poses = [Pose(ps[i], recast_quaternion(q[i]))]
                        false_ids = [ids[i]*4 + self.get_dir(q[i])] #get_dir returns if near NESW = 0-3
                        for j,id2 in enumerate(ids[i+1:]):
                              if id2 == ids[i]:
                                    poses.append(Pose(ps[j], recast_quaternion(q[j])))
                                    false_ids.append(id2*4 + self.get_dir(q[j]))
                                    completed[j] == True
                        marker_array.detections.append(make_april_marker(timestamp, false_ids, poses, ))
            marker_array.header.frame_id = ROOT_LINK
            marker_array.header.stamp = data.header.stamp
            self.marker_array_pub.publish(marker_array)

      def bot_to_map(self, pos, q, timestamp=rospy.Time.now()):
            ps = PoseStamped()

            #set up the frame parameters
            ps.header.frame_id = ROOT_LINK
            ps.header.stamp = timestamp

            ps.pose.position = pos
            ps.pose.orientation = recast_quaternion(q)

            success = False
            while not rospy.is_shutdown() and success == False:
                  try:
                        new_ps = self.listener.transformPose("map", ps)
                        success = True
                  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        success = False
            new_ps.pose.orientation.x = 0#TODO Check if advisable to do so
            new_ps.pose.orientation.y = 0
            return new_ps.pose.position.x, new_ps.pose.position.y, new_ps.pose.orientation

      def ar_detect(self):
            if self.frame is None:
                  return False, None,None,None

            image = self.frame.copy()
            h,  w = image.shape[:2]
            # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(CALIB_MTX, DIST, (w,h), 1, (w,h))
            # dst = cv2.undistort(image, CALIB_MTX, DIST, None, newcameramtx) ## TODO use camera info
            img_copy = copy.deepcopy(dst)
            #convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # thresh = 100
            #converting image to black and white to make the process robust
            # im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
            # cv2.imshow("in", im_bw)
            im_bw = gray
            #Parameters for the detectors
            parameters =  cv2.aruco.DetectorParameters_create()
            parameters.minMarkerPerimeterRate=0.2#default: 0.05
            #return values: corners, Tag ID array (nonetype), rejected candidates for tags 
            corners, ids, rejects = cv2.aruco.detectMarkers(im_bw, DICTIONARY, parameters=parameters)
            # TODO(Ashwin,Harsh): Use Camera Calibration
            #corners, ids, rejects = cv2.aruco.detectMarkers(im_bw, DICTIONARY, parameters=parameters,cameraMatrix=cameraMatrix) 
            #drawing markers
            img = cv2.aruco.drawDetectedMarkers(img_copy, corners, ids)
            if len(corners) > 0:
                  #print the coordinates (Can use the returned values)
                  corners = np.array(corners).reshape(-1,2)
                  # theta = -(np.average(corners, axis=1)[0]/(np.shape(img)[0]) - 0.5)*45*2
                  found = len(corners)/4
                  theta = []
                  for ar in range(found):
                        centroid_x = np.mean([i[1] for i in corners])
                        pcd = cropped_pcd(self.pcd, corners[ar*4:ar*4+4])
                        z = np.median(np.asarray(pcd.points), axis=0)[2] # get median z TODO
                        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
                        # o3d.visualization.draw_geometries([pcd],
                        #                                   zoom=0.3412,
                        #                                   front=[0.4257, -0.2125, -0.8795],
                        #                                   lookat=[2.6172, 2.0475, 1.532],
                        #                                   up=[-0.0694, -0.9768, 0.2024])
                        # theta.append(-40 + (centroid_x*80)/width)
                        # downpcd.estimate_normals(
                        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                        # o3d.visualization.draw_geometries([downpcd],
                        #                                   zoom=0.3412,
                        #                                   front=[0.4257, -0.2125, -0.8795],
                        #                                   lookat=[2.6172, 2.0475, 1.532],
                        #                                   up=[-0.0694, -0.9768, 0.2024],
                        #                                   point_show_normal=True)
                        
                        plane_model, _ = downpcd.segment_plane(distance_threshold=0.01,
                                                           ransac_n=3,
                                                           num_iterations=500)
                        [a,b,c,d] = plane_model
                        q = q_from_vector3D((a,b,c))
                        print(q)

                  # print("Detected: ", theta)

                  return found, ids, ps, q
            return False, None,None,None

def  q_from_vector3D(point):
      #http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
      q = Quaternion()
      #calculating the half-way vector.
      u = [1,0,0]
      norm = linalg.norm(point)
      v = asarray(point)/norm 
      if (np.all(u == v)):
          q.w = 1
          q.x = 0
          q.y = 0
          q.z = 0
      elif (np.all(-u == v)):
          q.w = 0
          q.x = 0
          q.y = 0
          q.z = 1
      else:
          half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
          q.w = dot(u, half)
          temp = cross(u, half)
          q.x = temp[0]
          q.y = temp[1]
          q.z = temp[2]
      norm = math.sqrt(q.x*q.x + q.y*q.y + 
          q.z*q.z + q.w*q.w)
      if norm == 0:
          norm = 1
      q.x /= norm
      q.y /= norm
      q.z /= norm
      q.w /= norm
      return q

def crop_pcd(pcd, corners):
      corners = np.array(corners)

      # Convert the corners array to have type float64
      bounding_polygon = corners.astype("float64")

      # Create a SelectionPolygonVolume
      vol = o3d.visualization.SelectionPolygonVolume()

      # You need to specify what axis to orient the polygon to.
      # I choose the "Y" axis. I made the max value the maximum Y of
      # the polygon vertices and the min value the minimum Y of the
      # polygon vertices.
      vol.orthogonal_axis = "X"
      vol.axis_max = np.max(bounding_polygon[:, 1])+0.05
      vol.axis_min = np.min(bounding_polygon[:, 1])-0.05

      # Set all the Y values to 0 (they aren't needed since we specified what they
      # should be using just vol.axis_max and vol.axis_min).
      bounding_polygon[:, 1] = 0

      # Convert the np.array to a Vector3dVector
      vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)

      # Crop the point cloud using the Vector3dVector
      cropped_pcd = vol.crop_point_cloud(pcd)

      # Get a nice looking bounding box to display around the newly cropped point cloud
      # (This part is optional and just for display purposes)
      # bounding_box = cropped_pcd.get_axis_aligned_bounding_box()
      # bounding_box.color = (1, 0, 0)

      # Draw the newly cropped PCD and bounding box
      # o3d.visualization.draw_geometries([cropped_pcd, bounding_box],
      #                                   zoom=2,
      #                                   front=[5, -2, 0.5],
      #                                   lookat=[7.67473496, -3.24231903,  0.3062945],
      #                                   up=[1.0, 0.0, 0.0])
      return cropped_pcd

def make_april_marker(timestamp, id, ps, cov=[], size=[0.15]):
    # make april tag detection marker
    m = AprilTagDetection()
    m.id = id
    pos_cov = PoseWithCovarianceStamped()
    pos_cov.header.frame_id = ROOT_LINK
    pos_cov.header.stamp = timestamp
    pos_cov.pose.pose = ps
    # pos_cov.pose.covariance = cov
    m.pose = pos_cov #pose with cov
    m.size = size

    return m

def recast_quaternion(quaternion):
      if quaternion is None:
            q = Quaternion(0,0,0,1)
      elif isinstance(quaternion, list) or isinstance(quaternion, tuple):
            q = Quaternion(*quaternion)
      elif isinstance(quaternion,Quaternion):
            q = quaternion
      else:
            print('Quaternion in incorrect format')
            q = Quaternion(0,0,0,1)
      return q

def uncast_quaternion(quaternion):
      if quaternion is None:
            q = (0,0,0,1)
      elif isinstance(quaternion, list) or isinstance(quaternion, tuple):
            q = quaternion
      elif isinstance(quaternion,Quaternion):
            q = quaternion
            q = (q.x,q.y,q.z,q.w)#lazy+readable code
      else:
            print('Quaternion in incorrect format')
            q = (0,0,0,1)
      return q


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data 

