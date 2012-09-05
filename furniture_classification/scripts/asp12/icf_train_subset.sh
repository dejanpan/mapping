# get features
rosrun icf_feature_extraction extract_pcl_features --normals -e subset.pcd -b $1 -f vfh -o vfh_subsets1to3.hdf5 -n x

# get labels
rosrun icf_feature_extraction extract_pcl_features -e subset.pcd -b $1 -f labels -o labels_subsets1to3.hdf5 -n y -l

# merge
rosrun icf_dataset join_datasets -a labels_subsets1to3.hdf5 -b vfh_subsets1to3.hdf5
# test: octave $(rospack find icf_dataset)/scripts/read_hdf5_octave.m vfh_subsets1to3.hdf5

# launch manager: roslaunch icf_core icf_service_node.launch

# Create classifier and parse out its ID
ID=`rosservice call /ias_classifier_manager/add_new_classifier -- "knn", "-m $2 -k $3 -w" | awk -F " " '{print $2; exit}'`
#ID=`rosservice call /ias_classifier_manager/add_new_classifier -- "svm", "" | awk -F " " '{print $2; exit}'`
echo "created classifier with ID $ID"

# Train SVM classifier (see LIBSVM)
rosrun icf_core upload_dataset -i vfh_subsets1to3.hdf5 -n dataset1
rosservice call /ias_classifier_manager/set_dataset -- "$ID" "train" "dataset1"
echo "training"
rosservice call /ias_classifier_manager/build_model -- "$ID" ""
#rosservice call /ias_classifier_manager/build_model -- "$ID" "-a 2 -t 0 -v 5 5:1:8 1:1:2"
echo "done"

# Evaluate classifier on the same data for now...
rosservice call /ias_classifier_manager/set_dataset -- "$ID" "eval" "dataset1"
echo "evaluating"
rosservice call /ias_classifier_manager/evaluate -- "$ID"
echo "done"

# Save model
rosservice call /ias_classifier_manager/save -- "$ID" "vfh_subsets1to3_with_evaluation_knn"
#rosservice call /ias_classifier_manager/save -- "$ID" "vfh_subsets1to3_with_evaluation_svm"
