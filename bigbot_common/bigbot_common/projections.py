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
