#!/usr/bin/env python3
import pyproj
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform
from typing import Tuple
from bigbot_interfaces.msg import GeographicPosition
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, PoseStamped
import enum

class Projections:

    def gnss2ecef(lat, lon, alt):
        transformer = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
        x, y, z = transformer.transform( lon,lat,  alt,radians=False)
        return x,y,z

    def geodetic2enu(lat, lon, alt, lat_org, lon_org, alt_org):
        transformer = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
        x, y, z = transformer.transform( lon,lat,  alt,radians=False)
        x_org, y_org, z_org = transformer.transform( lon_org,lat_org,  alt_org,radians=False)
        vec=np.array([[ x-x_org, y-y_org, z-z_org]]).T

        rot1 =  R.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
        rot3 =  R.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

        rotMatrix = rot1.dot(rot3)    
    
        enu = rotMatrix.dot(vec).T.ravel()
        return enu.T
    
    def ecef2gnss(x,y,z):
        transformer2 = pyproj.Transformer.from_crs(
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            )
        lon, lat, alt = transformer2.transform( x,y,z,radians=False)
        return [lat,lon,alt]
    # giving local x y z in meter wrt refpoint (FP_ENU0) in lat and lon  
    def enu2geodetic(x,y,z, lat_org, lon_org, alt_org):
        transformer1 = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
        transformer2 = pyproj.Transformer.from_crs(
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            )
    
        x_org, y_org, z_org = transformer1.transform( lon_org,lat_org,  alt_org,radians=False)
        ecef_org=np.array([[x_org,y_org,z_org]]).T
        
        rot1 =  R.from_euler('x', -(90-lat_org), degrees=True).as_matrix()
        rot3 =  R.from_euler('z', -(90+lon_org), degrees=True).as_matrix()

        rotMatrix = rot1.dot(rot3)

        ecefDelta = rotMatrix.T.dot( np.array([[x,y,z]]).T )
        ecef = ecefDelta+ecef_org
        lon, lat, alt = transformer2.transform( ecef[0,0],ecef[1,0],ecef[2,0],radians=False)

        return [lat,lon,alt]
    
    
    def ENUasTransform(lat, lon, alt) -> Transform:
        transformer1 = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
    
        x_org, y_org, z_org = transformer1.transform( lon,lat, alt,radians=False)
        
        rot1 =  R.from_euler('x', -(90-lat), degrees=True).as_matrix()
        rot3 =  R.from_euler('z', -(90+lon), degrees=True).as_matrix()
        rotMatrix = rot1.dot(rot3)
        r = R.from_matrix(rotMatrix)
        q = r.as_quat()
        q[3] = -q[3] # Have to inverse the w-component, not know why exactly

        tf = Transform()
        tf.translation.x = x_org
        tf.translation.y = y_org
        tf.translation.z = z_org
        tf.rotation.x = q[0]
        tf.rotation.y = q[1]
        tf.rotation.z = q[2]
        tf.rotation.w = q[3]
        return tf
    
    def ENUasPose(lat,lon,alt) -> Pose:
        tr = Projections.ENUasTransform(lat, lon, alt)
        trstamped = TransformStamped()
        trstamped.transform = tr
        poseECEF = do_transform_pose(Pose(), trstamped)
        return poseECEF
    
    def get_GeographicPosition_and_Pose(tf_ecef : str) -> Tuple[GeographicPosition, Pose]:
        ''' Given the TF of the robot in ECEF, return the geographic position and the pose in the local ENU frame 
            (a frame in the robot, this is not the ENU for path reference)

            The geographic position is the latitude, longitude and altitude of the robot in the WGS84 coordinate system.
            The pose is the position and orientation of the robot in the local ENU frame.
            These can be used to construct a PathSections message.
        '''
        lat, lon, height = Projections.ecef2gnss(tf_ecef.transform.translation.x, tf_ecef.transform.translation.y, tf_ecef.transform.translation.z)
        current_geographic_position = GeographicPosition()
        current_geographic_position.latitude = lat
        current_geographic_position.longitude = lon
        current_geographic_position.altitude = height
        tf_ENU = Projections.ENUasTransform(lat, lon, height) # Dit is homemade ENU ter plekke robot
        startpose = Pose()
        startpose.position = tf_ecef.transform.translation
        startpose.orientation = tf_ecef.transform.rotation
        tffinv = Projections.inverse_transform(TransformStamped(transform = tf_ENU))
        transformed_pose = do_transform_pose(startpose, tffinv)
        return current_geographic_position, transformed_pose
    
    def transform_geopose_to_localframe(pose_in_geo : Pose, georeference : GeographicPosition, tf_ENU: TransformStamped) -> Pose:
        ''' given info from PathSections and ENU ref frame (wrt ECEF), give a transformation with which 
            the PathSections can be expressed in the ENU ref frame

            pose_in_geo is the pose of the robot in the WGS84 coordinate system 
            georeference is the latitude, longitude and altitude of the ENU ref frame
            tf_ENU is the transformation of the ENU ref frame wrt the ECEF frame
        '''
        tr = Projections.ENUasTransform(georeference.latitude, georeference.longitude, georeference.altitude)
        trstamped = TransformStamped()
        trstamped.transform = tr
        
        pose_in_ECEF = do_transform_pose(pose_in_geo, trstamped)
        pose_in_ENU = do_transform_pose(pose_in_ECEF, Projections.inverse_transform(tf_ENU))
        return pose_in_ENU 
    
    def inverse_transform(transform : TransformStamped) -> TransformStamped:
        """Inverts a given TransformStamped."""
        # Extract translation
        translation = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])

        # Extract and invert rotation
        quaternion = [transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w]
        rotation = R.from_quat(quaternion)
        inverse_rotation = rotation.inv()

        # Invert translation using the inverse rotation
        inverse_translation = -inverse_rotation.apply(translation)

        # Create a new TransformStamped for the inverse
        inverse_transform = TransformStamped()
        inverse_transform.header.frame_id = transform.child_frame_id
        inverse_transform.child_frame_id = transform.header.frame_id
        inverse_transform.transform.translation.x = inverse_translation[0]
        inverse_transform.transform.translation.y = inverse_translation[1]
        inverse_transform.transform.translation.z = inverse_translation[2]
        inverse_transform.transform.rotation.x = inverse_rotation.as_quat()[0]
        inverse_transform.transform.rotation.y = inverse_rotation.as_quat()[1]
        inverse_transform.transform.rotation.z = inverse_rotation.as_quat()[2]
        inverse_transform.transform.rotation.w = inverse_rotation.as_quat()[3]

        return inverse_transform
    
    def distance(lat1, lon1, alt1, lat2, lon2,alt2):
        transformer = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
        x1, y1, z1 = transformer.transform( lon1,lat1, alt1,radians=False)
        x2, y2, z2 = transformer.transform( lon2,lat2, alt2,radians=False)
        distance = np.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
        return distance
    
    def transform_geopose_to_ENUframe(tr_ecef_to_enu : Transform, latitude, longitude, altitude):
        pose = Projections.ENUasPose(latitude, longitude, altitude)
        return Projections.transform_geopose_to_ENUframePOSE(tr_ecef_to_enu, pose)
    
    def transform_geopose_to_ENUframePOSE(tr_ecef_to_enu : Transform, pose: Pose):
        trstamped = TransformStamped()
        trstamped.transform = tr_ecef_to_enu
        trinv = Projections.inverse_transform(trstamped)    
        pose_in_ENU = do_transform_pose(pose, trinv)
        return pose_in_ENU
    

if __name__ == '__main__':
    
    # The point of interest
    lat = 51.2634922056012  # deg
    lon = 5.571864872572938   # deg
    alt = 83.09958202273997      # meters

    lat2,lon2,alt2 = Projections.enu2geodetic(1.5,2.2,0,lat, lon, alt)

    def distance(lat1, lon1, alt1, lat2, lon2,alt2):
        transformer = pyproj.Transformer.from_crs(
            {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
            {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
            )
        x1, y1, z1 = transformer.transform( lon1,lat1, alt1,radians=False)
        x2, y2, z2 = transformer.transform( lon2,lat2, alt2,radians=False)
        distance = np.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
        return distance

    mydistance = distance(lat,lon,alt,lat2,lon2,alt2)

    tr = Projections.ENUasTransform(lat, lon, alt)
    trstamped = TransformStamped()
    trstamped.transform = tr
    pose = Pose()
    pose.position.x=1.0
    pose.position.y=3.2
    poseECEF = do_transform_pose(pose, trstamped)



   
    #lat1, lon1, height1 = Projections.ecef2gnss(poseECEF.position.x, poseECEF.position.y, poseECEF.position.z)
# Given quaternion describing FP_ENU0 w.r.t. FP_ECEF waarvan x in EAST
    quaternion = [0.222829, 0.245625, 0.698724, 0.633877] # EIGENLIJK ANTWOORD WANT DEZE ORIENTATIE IS GOED !!

    lat1, lon1, height1 = Projections.ecef2gnss(3980366.35324, 388301.46014, 4951997.17765) # from rosbag

    tr = Projections.ENUasTransform(lat1, lon1, height1)
    trstamped = TransformStamped()
    trstamped.transform = tr
    mypose = do_transform_pose(Pose(), trstamped) # Pose is 0,0,0 #mypose.orientation is goed!!! maar tr.rotation is identiek!!!

    mypose2 = Projections.ENUasPose(lat1, lon1, height1)
   

    #mypose is same input ecef2gnss
    ENU0_from_pose = [3980366.35324, 388301.46014, 4951997.17765,	0.22282913895311826, 0.24562485948830065, 0.6987238876229493, 0.6338773793880301]
    # Geef een 
    print("klaar")
    
    #geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.9280872184013855, y=1.126239618955836, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.6765969393657774, w=0.7363535710790453))
    #geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=-1.2005793201942843, y=0.8296811798441841, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.6765969393657774, w=0.7363535710790453))

    Invector = np.array([0.9280872184013855, 1.126239618955836, 0.0])
    Inrot = R.from_quat([0.0, 0.0, 0.6765969393657774, 0.7363535710790453])
    _,_,angle = Inrot.as_euler('xyz')

    mm = np.matrix([[-np.cos(angle), np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    M=Inrot.as_matrix()
    inverse_pos = M.dot(Invector)
    print("klaar")

    def determine_orientation(next_pose, pose):
        ''' get quaternion from pose to next_pose
            next_pose and pose are lists of [x,y,z] 
        '''
        v = np.array(next_pose) - np.array(pose)
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-8:
            # No direction, return identity quaternion
            return [0.0, 0.0, 0.0, 1.0]
        # Full 3D orientation: align x-axis with direction vector
        # Use align_vectors for a more concise solution
        v_unit = v / v_norm
        x_axis = np.array([1.0, 0.0, 0.0])
        rotation = R.align_vectors([v_unit], [x_axis])[0]
        return rotation.as_quat().tolist()
    
   
    def make_pose(x, y, z, qx, qy, qz, qw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose
    def make_tf(x, y, z, qx, qy, qz, qw):
        tf = Transform()
        tf.translation.x = float(x)
        tf.translation.y = float(y)
        tf.translation.z = float(z)
        tf.rotation.x = float(qx)
        tf.rotation.y = float(qy)
        tf.rotation.z = float(qz)
        tf.rotation.w = float(qw)
        return tf
    myenu0 = make_tf(3980358.46898, 388311.4009, 4952002.71428, 0.222828,0.245625,0.698725, 0.633877)
    #field_247wrhyq
    field = make_pose(3980311.20666, 388313.27916286973, 4951939.331979004, 0.22282832882015122,0.2456250227988286,0.6987253544229204, 0.6338759840354089)
    field_lat= 51.26349171056761
    field_lng = 5.571496497014907

    mytrans = Projections.transform_geopose_to_ENUframe(myenu0, field_lat, field_lng, 0.0)

    mytrans2 = Projections.transform_geopose_to_ENUframePOSE(myenu0, field)
    # TEST: transform_geopose_to_localframe vs Projections.gnss2ecef
    print(mytrans2)


    # Given ENU0 TF and list of lat,lng return a list of geometry_msgs/PoseStamped
    map_ref_ECEF = make_tf(3980358.46898, 388311.4009, 4952002.71428, 0.222828,0.245625,0.698725, 0.633877)
    
    latlng_array = [{"lat":51.26369227873253,"lng":5.571790251497127},{"lat":51.26355799938459,"lng":5.571406781673432},{"lat":51.26340023303474,"lng":5.5712860721224775},{"lat":51.26318372301355,"lng":5.571374427001571},{"lat":51.2630578446254,"lng":5.571672302005953},{"lat":51.263140085211376,"lng":5.572042465209961},{"lat":51.26341032382282,"lng":5.572409884067421}]

    class OrientationStrategy(enum.Enum):
        TOWARDS_NEXT = 1
        PAIRED = 2

    def generate_poses_from_latlng(latlng_array, map_ref_ECEF : Transform, squash_altitude : bool = True, orientation_strategy : OrientationStrategy = OrientationStrategy.TOWARDS_NEXT):
        """ Generate a list of PoseStamped from a list of lat,lng coordinates
            map_ref_ECEF is the ENU0 frame in ECEF coordinates (tf)
            squash_altitude: if True, the altitude of the poses will be set to 0.0
            orientation_strategy: OrientationStrategy enum value
        """
        _, _, altitude_ref = Projections.ecef2gnss(map_ref_ECEF.translation.x, map_ref_ECEF.translation.y, map_ref_ECEF.translation.z)
        poses_rel = list()        
        for loc in latlng_array:
            pose = Projections.transform_geopose_to_ENUframe(map_ref_ECEF, loc["lat"], loc["lng"], altitude_ref)
            poses_rel.append([pose.position.x, pose.position.y, pose.position.z])
        poses_stamped = []
        if len(poses_rel) == 0:
            return None
        if len(poses_rel) == 1:
            pose = poses_rel[0]
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = 0.0 if squash_altitude else pose[2]
            poses_stamped.append(pose_stamped)
            return poses_stamped
        for i, pose in enumerate(poses_rel):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = 0.0 if squash_altitude else pose[2]
            pose_stamped.pose.orientation.w = 1.0
            if orientation_strategy == OrientationStrategy.TOWARDS_NEXT:
                if i+1 < len(poses_rel):
                    next_pose = poses_rel[i+1] 
                    quat = determine_orientation(next_pose, pose)
                    pose_stamped.pose.orientation.x = quat[0]
                    pose_stamped.pose.orientation.y = quat[1]
                    pose_stamped.pose.orientation.z = quat[2]
                    pose_stamped.pose.orientation.w = quat[3]
                else:
                    # Last pose: copy previous orientation or set identity
                    if i > 0:
                        pose_stamped.pose.orientation = poses_stamped[-1].pose.orientation
                    else:
                        pose_stamped.pose.orientation.w = 1.0
            elif orientation_strategy == OrientationStrategy.PAIRED:
                # For PAIRED, orient in pairs (1->2, 3->4, etc.)
                if i % 2 == 0 and i+1 < len(poses_rel):
                    next_pose = poses_rel[i+1]
                    quat = determine_orientation(next_pose, pose)
                    pose_stamped.pose.orientation.x = quat[0]
                    pose_stamped.pose.orientation.y = quat[1]
                    pose_stamped.pose.orientation.z = quat[2]
                    pose_stamped.pose.orientation.w = quat[3]
                elif i % 2 == 1:
                    # Copy orientation from previous
                    pose_stamped.pose.orientation = poses_stamped[-1].pose.orientation
                else:
                    pose_stamped.pose.orientation.w = 1.0
            else:
                pose_stamped.pose.orientation.w = 1.0
            poses_stamped.append(pose_stamped)
        return poses_stamped
        
    pose_stamped = generate_poses_from_latlng(latlng_array, map_ref_ECEF, squash_altitude=False, orientation_strategy=OrientationStrategy.PAIRED)
    for pose in pose_stamped:
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        euler = R.from_quat(quat).as_euler('xyz', degrees=True)
        print(f"Pose: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}, "
              f"Euler angles (deg): roll={euler[0]:.3f}, pitch={euler[1]:.3f}, yaw={euler[2]:.3f}")
    print("Done generating poses from latlng array.")
