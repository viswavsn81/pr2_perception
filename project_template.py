#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import pcl


#helper function to extract the data from a string
# def get_substring(data = , startStrSeq = '',endStrSeq = '',offset = 0)
#     if startStrSeq in data:               
#         total_data.append(data[(data.find(startStrSeq)+offset):data.find(endStrSeq)])               )


    #return extracted_string

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {'object_list': dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# function to load parameters and request PickPlace service
def pr2(object_list):
#------------------------------------------------------------------------------------------
    # TODO: Initialize variables

    object_name = String()
    #object_group = []
    
    test_scene_num = Int32()
    #object_name = [] #check: issue with string() as in notes
    object_group = []
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    dict_list = []
    test_scene_num.data = 1

    detected_objects = object_list
    #------------------------------------------------------------------------------------------
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    print "debug <start> ----------------------------------"

    #------------------------------------------------------------------------------------------
    # TODO: Parse parameters into individual variables
    #------------------------------------------------------------------------------------------
    for i in range(len(object_list_param)):
        #object_name.append(object_list_param[i]['name'])
        object_group.append(object_list_param[i]['group'])

        #------------------------------------------------------------------------------------------
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        #------------------------------------------------------------------------------------------        
    #try: appending centroids outside the loop to avoid the index out of range error
    for k in range(len(detected_objects)):
        labels.append(detected_objects[k].label) #check: not sure if this fits in to the logic                
        points_arr = ros_to_pcl(detected_objects[k].cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
        print "centroid[",k,"]:",centroids[k]


    # TODO: Rotate PR2 in place to capture side tables for the collision map
    #------------------------------------------------------------------------------------------
    # TODO: Loop through the pick list

    print "len(object_param):",len(object_list_param)," | len(detected_objects):", len(detected_objects)
    for i in range(len(object_list_param)):
        #if  object_name[i] in detected_objects:
        for j in range(len(detected_objects)):

            if  object_list_param[i]['name'] == detected_objects[j].label:

                #assign the test scene num
                test_scene_num.data = 1
                print "test_scene_num:", test_scene_num
                #------------------------------------------------------------------------------------------
                # TODO: Assign the arm to be used for pick_place
                #------------------------------------------------------------------------------------------
                #assign the arm name
                if object_group[i] == 'red':
                    #assign the task to the 
                    arm_name.data = 'left'
                    print "debug <start> arm_name: - object group red <end> "

                elif object_group[i] == 'green':
                    arm_name.data = 'right'
                    print "debug <start> arm_name: - object group green <end>"

                else :
                    print "debug <start> pick_pose: Invalid group color!! : ",object_group[i],"<end>"

                #assign the object name
                object_name.data = object_list_param[i]['name']
                print "debug: object_name.data:" , object_name.data

                #------------------------------------------------------------------------------------------
                        # TODO: Create 'place_pose' for the object        
                #------------------------------------------------------------------------------------------    
                print "pick_pose (j):", j
                pick_pose.position.x = np.asscalar(centroids[j][0])
                pick_pose.position.y = np.asscalar(centroids[j][1])
                pick_pose.position.z = np.asscalar(centroids[j][2])

                print "debug<start>pick_pose:",pick_pose, "<end>"

                #assign place pose
                if(object_group[i] == 'red'):
                    place_pose.position.x = dropbox_param[0]['position'][0]
                    place_pose.position.y = dropbox_param[0]['position'][1]
                    place_pose.position.z = dropbox_param[0]['position'][2]                    
                    print "debug <start> place_pose: (group red): ",place_pose, "<end>"

                elif object_group[i] == 'green':
                    place_pose.position.x = dropbox_param[1]['position'][0]
                    place_pose.position.y = dropbox_param[1]['position'][1]
                    place_pose.position.z = dropbox_param[1]['position'][2]
                    print "debug <start> place_pose: (group green): ",place_pose, "<end>"

                else :
                    print "debug <start> place_pose: Invalid group color!! : ",object_group[i],"<end>"

        #------------------------------------------------------------------------------------------
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        #------------------------------------------------------------------------------------------
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        print "yaml_dict:",yaml_dict
        dict_list.append(yaml_dict)
        print "-----------dict_list--------------------"
        print dict_list
        print "-----------dict_list <end>--------------"
               
    #------------------------------------------------------------------------------------------
    # TODO: Output your request parameters into output yaml file
    #------------------------------------------------------------------------------------------
    send_to_yaml('perception_out.yml',dict_list)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))


    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')
#------------------------------------------------------------------------------------------
    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # TODO: Insert your message variables to be sent as a service request
        resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

#------------------------------------------------------------------------------------------
# Outlier removal filter
#------------------------------------------------------------------------------------------
    # TODO: Convert ROS msg to PCL data
    cloudinc = ros_to_pcl(pcl_msg)

    # Outlier removal filter

    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloudinc.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10) #10 - cluster was successful but lot of noise

    # Set threshold scale factor
    x = 0.1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud = outlier_filter.filter()

#------------------------------------------------------------------------------------------
    # TODO: Statistical Outlier Filtering
#------------------------------------------------------------------------------------------
    # TODO: Voxel Grid Downsampling

    # Voxel Grid filter
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    #After experimentation, 0.01 was determined to be the best value
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
#------------------------------------------------------------------------------------------
    # TODO: PassThrough Filter

    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough_z = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough_z.set_filter_field_name (filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough_z.set_filter_limits (axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough_z.filter()

    #Adding another pass through filter to filter out the boxes on the y axis
    passthrough_y = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough_y.set_filter_field_name (filter_axis)
    axis_min = -0.5 # The distance between the camera center and the box edge is ~0.8 in rviz
    axis_max = 0.5

    passthrough_y.set_filter_limits (axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough_y.filter()
#------------------------------------------------------------------------------------------
    # TODO: RANSAC Plane Segmentation

    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

#------------------------------------------------------------------------------------------
    # TODO: Extract inliers and outliers

    cloud_table = cloud_filtered.extract(inliers, negative=False)   #extracted inliners
    cloud_objects = cloud_filtered.extract(inliers, negative=True)  #extracted outliers

#------------------------------------------------------------------------------------------



#------------------------------------------------------------------------------------------
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: Default values provided were  poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05) # The values here provided the best perfromance 
    ec.set_MinClusterSize(1)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
#------------------------------------------------------------------------------------------
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
#------------------------------------------------------------------------------------------
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects =  pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
#------------------------------------------------------------------------------------------
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_cloud_pub.publish(ros_cluster_cloud) #check: commented temp 
#------------------------------------------------------------------------------------------
# Exercise-3 TODOs:
#------------------------------------------------------------------------------------------
    
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
#------------------------------------------------------------------------------------------
        # Grab the points for the cluster
        #pcl_cluster = cluster_cloud.extract(pts_list) #check: changing cloud_out to cluster_cloud
        pcl_cluster = cloud_objects.extract(pts_list)    
#------------------------------------------------------------------------------------------
        # Compute the associated feature vector

        # Convert the cluster from pcl to ROS using helper function        
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True) #check: Improve performance by using_hsv = true
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
#------------------------------------------------------------------------------------------
        # Make the prediction
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list        
        #check: Doubt: how does th code cycle through the color and normal histogorams?

        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
#------------------------------------------------------------------------------------------
        # Publish a label into RViz
        
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos,index))
#------------------------------------------------------------------------------------------
        # Add th+ee detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        print"Detected objects <start>:","detected_objects_len:",len(detected_objects)
        for x in range(len(detected_objects)): 
            print detected_objects[x].label


    print "Detected object <end>"

    

#------------------------------------------------------------------------------------------
    # TODO: Parse parameters into individual variables
    

    #test_scene_num, object_name, arm_name, pick_pose, place_pose messages to be sent to yaml


    #yaml_filename = 'perception_out.yaml'
    
    
    

 


#------------------------------------------------------------------------------------------
    # Publish the list of detected objects

    #detected_objects_pub.publish(detected_objects) #check: commented temp 
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    print "pr2_mover"
    pr2_mover(detected_objects)
    detected_objects_pub.publish(detected_objects)
    
#------------------------------------------------------------------------------------------
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass




if __name__ == '__main__':
#------------------------------------------------------------------------------------------
    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
#------------------------------------------------------------------------------------------
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
#------------------------------------------------------------------------------------------
    #Check to see if we can publish 
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    # TODO: Create Publishers
    #check: commented temp 
    #pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    #pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_cloud_pub = rospy.Publisher("/pcl_cluster_cloud", PointCloud2, queue_size=1)

    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively

    #check: commented temp 
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", PointCloud2, queue_size=1)
#------------------------------------------------------------------------------------------
    # TODO: Load Model From disk
    
    #check : commented temp

    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
#------------------------------------------------------------------------------------------
    # Initialize color_list
    get_color_list.color_list = []
#------------------------------------------------------------------------------------------
    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

#------------------------------------------------------------------------------------------